#!/usr/bin/env python3
"""Software-only checks of differential decode, lifecycle and callback contracts.

Mocks do not validate the analog front end, silicon trigger timing or ADC DMA IRQ
latency. ARMCC compiles the module against the project's actual G474 HAL headers.
"""
from __future__ import annotations

import datetime as dt
import hashlib
import json
from pathlib import Path
import re
import shutil
import subprocess
import tempfile

from verify_module_target_build import DEFAULT_COMPILER, project_settings

ROOT = Path(__file__).resolve().parents[1]

STUB = r'''
#ifndef STM32G4XX_HAL_H
#define STM32G4XX_HAL_H
#include <stdint.h>
#include <stddef.h>
typedef enum {HAL_OK=0,HAL_ERROR=1,HAL_BUSY=2,HAL_TIMEOUT=3} HAL_StatusTypeDef;
typedef struct {uint32_t CR,CFGR,CFGR2,SQR1,DIFSEL,JSQR,OFR1,OFR2,OFR3,OFR4;} ADC_TypeDef;
typedef struct {uint32_t CCR;} ADC_Common_TypeDef;
extern ADC_TypeDef mock_adc[4];
extern ADC_Common_TypeDef mock_common[2];
#define ADC1 (&mock_adc[0])
#define ADC2 (&mock_adc[1])
#define ADC3 (&mock_adc[2])
#define ADC4 (&mock_adc[3])
#define __LL_ADC_COMMON_INSTANCE(x) (((x)==ADC1 || (x)==ADC2)?&mock_common[0]:&mock_common[1])
typedef struct {uint32_t Direction,PeriphInc,PeriphDataAlignment,MemDataAlignment,Mode,Request;} DMA_InitTypeDef;
typedef struct {DMA_InitTypeDef Init;void *Parent;void *Instance;uint32_t State;uint32_t ht;} DMA_HandleTypeDef;
typedef struct {uint32_t Resolution,DataAlign,NbrOfConversion,ContinuousConvMode,DiscontinuousConvMode,OversamplingMode,GainCompensation,DMAContinuousRequests,ExternalTrigConv,ExternalTrigConvEdge;} ADC_InitTypeDef;
typedef struct {ADC_TypeDef *Instance;ADC_InitTypeDef Init;DMA_HandleTypeDef *DMA_Handle;uint32_t ErrorCode;uint32_t ovr;} ADC_HandleTypeDef;
#define DISABLE 0U
#define ENABLE 1U
#define ADC_SQR1_SQ1_Pos 6U
#define ADC_SQR1_SQ1 (31U<<6)
#define ADC_SQR1_L 15U
#define ADC_CR_ADEN 1U
#define ADC_CR_ADSTART 4U
#define ADC_CR_JADSTART 8U
#define ADC_CR_ADCAL (1UL<<31)
#define ADC_CFGR_RES (3U<<3)
#define ADC_CFGR_ALIGN (1U<<15)
#define ADC_CFGR_CONT (1U<<13)
#define ADC_CFGR_DISCEN (1U<<16)
#define ADC_CFGR_JAUTO (1U<<25)
#define ADC_CFGR_AUTDLY (1U<<14)
#define ADC_CFGR2_ROVSE 1U
#define ADC_CFGR2_GCOMP (1U<<16)
#define ADC_JSQR_JEXTEN (3U<<7)
#define ADC_OFR1_OFFSET1_EN (1UL<<31)
#define ADC_OFR2_OFFSET2_EN (1UL<<31)
#define ADC_OFR3_OFFSET3_EN (1UL<<31)
#define ADC_OFR4_OFFSET4_EN (1UL<<31)
#define ADC_CFGR_EXTSEL (31U<<5)
#define ADC_CFGR_EXTEN (3U<<10)
#define ADC_CCR_DUAL 31U
#define ADC_CCR_MDMA (3U<<14)
#define ADC_CCR_DMACFG (1U<<13)
#define ADC_RESOLUTION_12B 0U
#define ADC_DATAALIGN_RIGHT 0U
#define ADC_DUALMODE_REGSIMULT 6U
#define ADC_DMAACCESSMODE_12_10_BITS (2U<<14)
#define ADC_EXTERNALTRIGCONVEDGE_NONE 0U
#define ADC_DIFFERENTIAL_ENDED 1U
#define DMA_PERIPH_TO_MEMORY 0U
#define DMA_PINC_DISABLE 0U
#define DMA_PDATAALIGN_WORD 2U
#define DMA_MDATAALIGN_WORD 2U
#define DMA_CIRCULAR 1U
#define DMA_REQUEST_ADC1 5U
#define DMA_REQUEST_ADC3 37U
#define HAL_DMA_STATE_BUSY 2U
#define HAL_DMA_STATE_READY 1U
#define DMA_IT_HT 4U
#define ADC_FLAG_OVR 16U
#define ADC_IT_OVR 16U
#define __HAL_ADC_CLEAR_FLAG(h,f) ((void)(h),(void)(f))
#define __HAL_ADC_ENABLE_IT(h,f) ((h)->ovr=1U,(void)(f))
#define __HAL_ADC_DISABLE_IT(h,f) ((h)->ovr=0U,(void)(f))
#define __HAL_DMA_DISABLE_IT(h,f) ((h)->ht=0U,(void)(f))
uint32_t __get_PRIMASK(void);
void __disable_irq(void);
void __set_PRIMASK(uint32_t value);
void __DMB(void);
HAL_StatusTypeDef HAL_ADCEx_Calibration_Start(ADC_HandleTypeDef*,uint32_t);
HAL_StatusTypeDef HAL_ADCEx_MultiModeStart_DMA(ADC_HandleTypeDef*,uint32_t*,uint32_t);
HAL_StatusTypeDef HAL_ADCEx_MultiModeStop_DMA(ADC_HandleTypeDef*);
HAL_StatusTypeDef HAL_ADC_Stop(ADC_HandleTypeDef*);
HAL_StatusTypeDef HAL_DMA_Abort(DMA_HandleTypeDef*);
#endif
'''

HARNESS = r'''
#include "adc_diff.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

ADC_TypeDef mock_adc[4];
ADC_Common_TypeDef mock_common[2];
static ADC_HandleTypeDef adc[4];
static DMA_HandleTypeDef dma[2];
static uint32_t primask, mask_count, trace[64], trace_count;
static uint32_t fail_operation, fail_stop, fail_abort;
static ADC_Diff_Context *error_on_lock;
uint32_t __get_PRIMASK(void) {return primask;}
void __disable_irq(void) {
    if(error_on_lock) {
        ADC_Diff_Context *c=error_on_lock;error_on_lock=NULL;
        c->config.master->ErrorCode=0x20;assert(ADC_Diff_OnError(c,c->config.master)==1);
    }
    primask=1;mask_count++;
}
void __set_PRIMASK(uint32_t value) {primask=value;}
void __DMB(void) {}
static void event(uint32_t e) {assert(trace_count<64);trace[trace_count++]=e;}
HAL_StatusTypeDef HAL_ADCEx_Calibration_Start(ADC_HandleTypeDef *h,uint32_t mode) {
    uint32_t n=(uint32_t)(h-adc)+1; assert(mode==ADC_DIFFERENTIAL_ENDED);
    event(n);return fail_operation==n?HAL_TIMEOUT:HAL_OK;
}
HAL_StatusTypeDef HAL_ADCEx_MultiModeStart_DMA(ADC_HandleTypeDef *h,uint32_t *word,uint32_t count) {
    uint32_t n=(uint32_t)(h-adc);assert(count==1 && word!=NULL);event(10+n);
    /* HAL实际实现即使DMA Start失败也可能设置ADSTART，测试此路径的清理。 */
    h->Instance->CR=ADC_CR_ADEN|ADC_CR_ADSTART;
    adc[n+1].Instance->CR=ADC_CR_ADEN;
    h->DMA_Handle->State=HAL_DMA_STATE_BUSY;h->DMA_Handle->ht=1;h->ovr=1;
    return fail_operation==10+n?HAL_ERROR:HAL_OK;
}
HAL_StatusTypeDef HAL_ADCEx_MultiModeStop_DMA(ADC_HandleTypeDef *h) {
    uint32_t n=(uint32_t)(h-adc);event(20+n);
    if(fail_stop==1) return HAL_ERROR;
    h->Instance->CR=0;adc[n+1].Instance->CR=0;h->DMA_Handle->State=HAL_DMA_STATE_READY;
    return HAL_OK;
}
HAL_StatusTypeDef HAL_ADC_Stop(ADC_HandleTypeDef *h) {
    event(30+(uint32_t)(h-adc));if(fail_stop==2)return HAL_ERROR;
    h->Instance->CR=0;return HAL_OK;
}
HAL_StatusTypeDef HAL_DMA_Abort(DMA_HandleTypeDef *h) {
    event(40);if(fail_abort)return HAL_ERROR;h->State=HAL_DMA_STATE_READY;return HAL_OK;
}
static void setup(void) {
    unsigned i;memset(mock_adc,0,sizeof(mock_adc));memset(mock_common,0,sizeof(mock_common));
    memset(adc,0,sizeof(adc));memset(dma,0,sizeof(dma));
    trace_count=0;fail_operation=0;fail_stop=0;fail_abort=0;mask_count=0;primask=0;error_on_lock=NULL;
    for(i=0;i<4;i++) {
        adc[i].Instance=&mock_adc[i];adc[i].Init.NbrOfConversion=1;
        mock_adc[i].SQR1=((i%2)?3U:1U)<<6;mock_adc[i].DIFSEL=0xAU;
    }
    for(i=0;i<2;i++) {
        unsigned a=i*2;dma[i].Parent=&adc[a];dma[i].Instance=&dma[i];
        dma[i].State=HAL_DMA_STATE_READY;adc[a].DMA_Handle=&dma[i];
        dma[i].Init.PeriphDataAlignment=DMA_PDATAALIGN_WORD;
        dma[i].Init.MemDataAlignment=DMA_MDATAALIGN_WORD;
        dma[i].Init.Mode=DMA_CIRCULAR;dma[i].Init.Request=i?DMA_REQUEST_ADC3:DMA_REQUEST_ADC1;
        adc[a].Init.DMAContinuousRequests=ENABLE;
        adc[a].Init.ExternalTrigConv=5U<<5;adc[a].Init.ExternalTrigConvEdge=2U<<10;
        mock_adc[a].CFGR=adc[a].Init.ExternalTrigConv|adc[a].Init.ExternalTrigConvEdge;
        mock_common[i].CCR=ADC_DUALMODE_REGSIMULT|ADC_DMAACCESSMODE_12_10_BITS|ADC_CCR_DMACFG;
    }
}
static ADC_Diff_Config config(unsigned pair) {
    ADC_Diff_Config c;
    c.master=&adc[pair*2];c.slave=&adc[pair*2+1];c.reference_v=3.0f;
    c.channel[0].zero_offset_code=pair?BUS_ZERO:VOLTAGE_ZERO;
    c.channel[1].zero_offset_code=pair?AUX_ZERO:CURRENT_ZERO;
    c.channel[0].polarity=pair?1:-1;c.channel[1].polarity=1;
    c.channel[0].physical_gain=pair?BUS_GAIN:VOLTAGE_GAIN;
    c.channel[1].physical_gain=pair?AUX_GAIN:CURRENT_GAIN;
    c.channel[0].physical_offset=0;c.channel[1].physical_offset=0;return c;
}
static void verify_decode_all_codes(void) {
    ADC_Diff_Context ctx[2];ADC_Diff_Config cfg[2];ADC_Diff_Snapshot s;
    unsigned p,r,ch;setup();memset(ctx,0,sizeof(ctx));
    for(p=0;p<2;p++) {cfg[p]=config(p);assert(ADC_Diff_Init(&ctx[p],&cfg[p])==HAL_OK);
        assert(ADC_Diff_Read(&ctx[p],&s)==HAL_BUSY);assert(ADC_Diff_Start(&ctx[p])==HAL_OK);}
    assert(trace_count==6 && trace[0]==1 && trace[1]==2 && trace[2]==10 &&
           trace[3]==3 && trace[4]==4 && trace[5]==12);
    for(r=0;r<4096;r++) for(p=0;p<2;p++) {
        unsigned codes[2]={r,4095-r};
        ctx[p].dma_word=(codes[0]|0xA000U)|((codes[1]|0xB000U)<<16);
        assert(ADC_Diff_OnConversionComplete(&ctx[p],cfg[p].master)==1);
        primask=r%2;assert(ADC_Diff_Read(&ctx[p],&s)==HAL_OK && primask==r%2);
        assert(s.sequence==r+1 && s.packed_word==ctx[p].dma_word);
        for(ch=0;ch<2;ch++) {
            int signed_code=(int)codes[ch]-2048;
            int corrected=signed_code-cfg[p].channel[ch].zero_offset_code;
            float volts=((float)corrected/2048.0f)*3.0f;
            float physical;
            if(cfg[p].channel[ch].polarity<0)volts=-volts;
            physical=volts*cfg[p].channel[ch].physical_gain;
            assert(s.channel[ch].raw==codes[ch]);assert(s.channel[ch].signed_code==signed_code);
            assert(s.channel[ch].corrected_code==corrected);
            assert(s.channel[ch].adc_voltage_v==volts);assert(s.channel[ch].value==physical);
        }
    }
    assert(mask_count==16386);
    printf("PASS all 4096 raw codes on four calibrated channels, paired decode and stable snapshots\n");
}
static void verify_lifecycle(void) {
    ADC_Diff_Context c={0};ADC_Diff_Config cfg;ADC_Diff_Snapshot out,before;
    unsigned stage;setup();cfg=config(0);
    assert(ADC_Diff_Start(NULL)==HAL_ERROR && ADC_Diff_Stop(NULL)==HAL_ERROR);
    assert(ADC_Diff_Start(&c)==HAL_ERROR && ADC_Diff_Stop(&c)==HAL_ERROR);
    assert(ADC_Diff_Init(&c,&cfg)==HAL_OK);assert(ADC_Diff_Stop(&c)==HAL_OK && trace_count==0);
    assert(ADC_Diff_Start(&c)==HAL_OK);assert(c.state==ADC_DIFF_RUNNING && adc[1].ovr==1 && dma[0].ht==0);
    assert(ADC_Diff_Start(&c)==HAL_BUSY && trace_count==3);
    {
        ADC_Diff_Config other=config(1);
        assert(ADC_Diff_Init(&c,&other)==HAL_BUSY && c.config.master==&adc[0]);
        assert(c.state==ADC_DIFF_RUNNING && mock_adc[0].CR!=0 && trace_count==3);
    }
    c.dma_word=2048U|(2048U<<16);assert(ADC_Diff_OnConversionComplete(&c,&adc[1])==0);
    assert(ADC_Diff_OnConversionComplete(&c,&adc[2])==0 && c.sample_count==0);
    assert(ADC_Diff_OnConversionComplete(&c,&adc[0])==1);
    memset(&out,0x5A,sizeof(out));before=out;c.snapshot_writing=1;
    assert(ADC_Diff_Read(&c,&out)==HAL_BUSY && memcmp(&out,&before,sizeof(out))==0);
    c.snapshot_writing=0;assert(ADC_Diff_Read(&c,&out)==HAL_OK);
    assert(ADC_Diff_Read(&c,NULL)==HAL_ERROR && ADC_Diff_Read(NULL,&out)==HAL_ERROR);
    assert(ADC_Diff_OnError(&c,&adc[2])==0);adc[1].ErrorCode=0x10;
    assert(ADC_Diff_OnError(&c,&adc[1])==1 && c.state==ADC_DIFF_FAULT && c.adc_error_code==0x10);
    assert(ADC_Diff_Read(&c,&out)==HAL_BUSY && ADC_Diff_Start(&c)==HAL_ERROR);
    assert(ADC_Diff_OnConversionComplete(&c,&adc[0])==0);
    assert(ADC_Diff_Stop(&c)==HAL_OK && c.state==ADC_DIFF_STOPPED);
    assert(mock_adc[0].CR==0 && mock_adc[1].CR==0 && dma[0].State==HAL_DMA_STATE_READY && adc[1].ovr==0);
    assert(ADC_Diff_Start(&c)==HAL_OK && c.sample_count==1);assert(ADC_Diff_Stop(&c)==HAL_OK);
    for(stage=0;stage<3;stage++) {
        setup();cfg=config(0);assert(ADC_Diff_Init(&c,&cfg)==HAL_OK);
        fail_operation=stage<2?stage+1:10;
        assert(ADC_Diff_Start(&c)!=HAL_OK && c.state==ADC_DIFF_STOPPED && c.error_count==1);
        assert(c.cleanup_status==HAL_OK && mock_adc[0].CR==0 && mock_adc[1].CR==0 && dma[0].State==HAL_DMA_STATE_READY);
        fail_operation=0;assert(ADC_Diff_Start(&c)==HAL_OK);assert(ADC_Diff_Stop(&c)==HAL_OK);
    }
    setup();cfg=config(0);assert(ADC_Diff_Init(&c,&cfg)==HAL_OK);
    fail_operation=10;fail_stop=1;
    assert(ADC_Diff_Start(&c)==HAL_ERROR && c.state==ADC_DIFF_FAULT && c.cleanup_status==HAL_ERROR);
    assert(mock_adc[0].CR==0 && mock_adc[1].CR==0 && dma[0].State==HAL_DMA_STATE_READY);
    assert(ADC_Diff_Start(&c)==HAL_ERROR);fail_operation=0;fail_stop=0;
    assert(ADC_Diff_Stop(&c)==HAL_OK && ADC_Diff_Start(&c)==HAL_OK);
    fail_stop=2;assert(ADC_Diff_Stop(&c)==HAL_ERROR && c.state==ADC_DIFF_FAULT);
    fail_stop=0;assert(ADC_Diff_Stop(&c)==HAL_OK);
    setup();cfg=config(0);assert(ADC_Diff_Init(&c,&cfg)==HAL_OK);
    fail_operation=10;fail_stop=1;fail_abort=1;
    assert(ADC_Diff_Start(&c)==HAL_ERROR && c.state==ADC_DIFF_FAULT && dma[0].State==HAL_DMA_STATE_BUSY);
    fail_stop=0;fail_abort=0;assert(ADC_Diff_Stop(&c)==HAL_OK);
    setup();cfg=config(0);assert(ADC_Diff_Init(&c,&cfg)==HAL_OK && ADC_Diff_Start(&c)==HAL_OK);
    c.dma_word=2048U|(2048U<<16);error_on_lock=&c;
    assert(ADC_Diff_OnConversionComplete(&c,&adc[0])==0);
    assert(c.state==ADC_DIFF_FAULT && c.snapshot_valid==0 && c.sample_count==0 && primask==0);
    assert(ADC_Diff_Init(&c,&c.config)==HAL_BUSY); /* FAULT必须先Stop，禁止清空上下文。 */
    assert(ADC_Diff_Stop(&c)==HAL_OK);
    printf("PASS callback routing, error invalidation, start failure rollback, stop retry and PRIMASK restoration\n");
}
static void verify_config(void) {
    ADC_Diff_Context c={0};ADC_Diff_Config cfg;
    setup();cfg=config(0);assert(ADC_Diff_Init(NULL,&cfg)==HAL_ERROR && ADC_Diff_Init(&c,NULL)==HAL_ERROR);
    cfg.reference_v=NAN;assert(ADC_Diff_Init(&c,&cfg)==HAL_ERROR);
    cfg=config(0);cfg.channel[0].physical_gain=INFINITY;assert(ADC_Diff_Init(&c,&cfg)==HAL_ERROR);
    cfg=config(0);cfg.channel[0].polarity=0;assert(ADC_Diff_Init(&c,&cfg)==HAL_ERROR);
    cfg=config(0);cfg.channel[0].zero_offset_code=-2049;assert(ADC_Diff_Init(&c,&cfg)==HAL_ERROR);
    cfg=config(0);cfg.slave=&adc[3];assert(ADC_Diff_Init(&c,&cfg)==HAL_ERROR);
    cfg=config(0);mock_adc[0].DIFSEL=0;assert(ADC_Diff_Init(&c,&cfg)==HAL_ERROR);
    setup();cfg=config(0);mock_adc[0].SQR1|=1;assert(ADC_Diff_Init(&c,&cfg)==HAL_ERROR);
    setup();cfg=config(0);dma[0].Init.MemDataAlignment=1;assert(ADC_Diff_Init(&c,&cfg)==HAL_ERROR);
    setup();cfg=config(0);mock_common[0].CCR=0;assert(ADC_Diff_Init(&c,&cfg)==HAL_ERROR);
    setup();cfg=config(0);mock_adc[0].CR=ADC_CR_ADEN;assert(ADC_Diff_Init(&c,&cfg)==HAL_ERROR);
    setup();cfg=config(0);mock_adc[0].CFGR=0;assert(ADC_Diff_Init(&c,&cfg)==HAL_ERROR);
    setup();cfg=config(0);mock_adc[0].OFR1=ADC_OFR1_OFFSET1_EN;assert(ADC_Diff_Init(&c,&cfg)==HAL_ERROR);
    setup();cfg=config(0);mock_adc[0].CFGR2=ADC_CFGR2_GCOMP;assert(ADC_Diff_Init(&c,&cfg)==HAL_ERROR);
    setup();cfg=config(0);mock_adc[0].CFGR|=ADC_CFGR_AUTDLY;assert(ADC_Diff_Init(&c,&cfg)==HAL_ERROR);
    setup();cfg=config(0);mock_adc[1].JSQR=ADC_JSQR_JEXTEN;assert(ADC_Diff_Init(&c,&cfg)==HAL_ERROR);
    setup();cfg=config(0);dma[0].State=HAL_DMA_STATE_BUSY;assert(ADC_Diff_Init(&c,&cfg)==HAL_ERROR);
    setup();cfg=config(0);assert(ADC_Diff_Init(&c,&cfg)==HAL_OK);
    assert(ADC_Diff_Init(&c,&c.config)==HAL_OK && c.config.master==&adc[0]);
    setup();cfg=config(0);cfg.channel[0].physical_offset=16.0f;assert(ADC_Diff_Init(&c,&cfg)==HAL_OK);
    assert(ADC_Diff_Start(&c)==HAL_OK);c.dma_word=2048U|(2048U<<16);
    assert(ADC_Diff_OnConversionComplete(&c,&adc[0])==1);
    assert(c.latest.channel[0].value==(-3.0f/2048.0f)*3.0f*VOLTAGE_GAIN-16.0f);
    assert(ADC_Diff_Stop(&c)==HAL_OK);
    printf("PASS invalid parameters/configuration and physical offset ordering\n");
}
int main(void) {verify_decode_all_codes();verify_lifecycle();verify_config();return 0;}
'''


def sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main() -> int:
    source = (ROOT / "Core/Src/main.c").read_text(encoding="utf-8-sig")
    names = {"VOLTAGE_ZERO": "VOLTAGE_ZERO_CODE", "CURRENT_ZERO": "CURRENT_ZERO_CODE",
             "BUS_ZERO": "DC_BUS_ZERO_CODE", "AUX_ZERO": "AUX_CURRENT_ZERO_CODE"}
    defines = []
    for key, name in names.items():
        match = re.search(r"^#define\s+" + name + r"\s+\(([-\d]+)\)", source, re.M)
        assert match, name
        defines.append(f"#define {key} ({match.group(1)})")
    gains = {"VOLTAGE_GAIN": "voltage_gain", "CURRENT_GAIN": "cksr6_current_gain",
             "BUS_GAIN": "dc_bus_voltage_gain", "AUX_GAIN": "aux_current_gain"}
    for key, name in gains.items():
        match = re.search(r"volatile float\s+" + name + r"\s*=\s*([\d.]+f)", source)
        assert match, name
        defines.append(f"#define {key} {match.group(1)}")
    files = [ROOT / n for n in ("adc_diff.c", "adc_diff.h", "Core/Src/main.c", "Core/Src/adc.c",
                                "Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_adc_ex.c")]
    hashes = {str(p.relative_to(ROOT)): sha(p) for p in files}
    result = {"timestamp_utc": dt.datetime.now(dt.timezone.utc).isoformat(),
              "scope": "Host mocked HAL and ARM translation-unit compile; no hardware validation",
              "source_sha256": hashes, "baseline_calibration": defines, "checks": []}
    gcc = shutil.which("gcc")
    if not gcc:
        raise RuntimeError("gcc not found")
    with tempfile.TemporaryDirectory(prefix="dcac_adc_diff_") as directory:
        temp = Path(directory)
        (temp / "stm32g4xx_hal.h").write_text(STUB, encoding="utf-8")
        (temp / "harness.c").write_text("\n".join(defines) + "\n" + HARNESS, encoding="utf-8")
        binary = temp / "verify_adc_diff.exe"
        command = [gcc, "-std=c11", "-Wall", "-Wextra", "-Werror", "-O2", "-I" + str(temp),
                   "-I" + str(ROOT), str(ROOT / "adc_diff.c"), str(temp / "harness.c"), "-lm", "-o", str(binary)]
        build = subprocess.run(command, capture_output=True, text=True)
        if build.returncode:
            raise RuntimeError(build.stdout + build.stderr)
        run = subprocess.run([str(binary)], capture_output=True, text=True)
        print(run.stdout + run.stderr, end="")
        result["checks"].append({"name": "host_exhaustive_and_lifecycle", "passed": run.returncode == 0,
                                 "output": run.stdout + run.stderr})
        settings = project_settings(ROOT / "MDK-ARM/dcac.uvprojx", "dcac")
        arm_command = [str(DEFAULT_COMPILER), "--cpu", "Cortex-M4.fp", "--c99", "-O2"]
        arm_command += ["-D" + value for value in settings["defines"]]
        arm_command += ["-I" + value for value in settings["include_paths"]]
        arm_command += ["-c", str(ROOT / "adc_diff.c"), "-o", str(temp / "adc_diff.o")]
        arm = subprocess.run(arm_command, cwd=temp, capture_output=True, text=True)
        result["checks"].append({"name": "actual_hal_armcc_compile", "passed": arm.returncode == 0,
                                 "diagnostics": arm.stdout + arm.stderr, "command": arm_command})
        print("PASS ARMCC actual HAL compilation" if arm.returncode == 0 else arm.stdout + arm.stderr)
    result["source_unchanged"] = hashes == {str(p.relative_to(ROOT)): sha(p) for p in files}
    result["temporary_files_removed"] = not temp.exists()
    result["checks_passed"] = (all(c["passed"] for c in result["checks"])
                               and result["source_unchanged"] and result["temporary_files_removed"])
    output = ROOT / "Tests/results/adc_diff_module_verification.json"
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(f"Result: {output}")
    return 0 if result["checks_passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
