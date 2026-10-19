#!/usr/bin/env python3
"""Check the standalone HRTIM module against the current formula and local HAL.

Host mocks validate software decisions, ordering and ownership only. ARMCC checks
the actual STM32G474 translation unit. Neither proves hardware waveforms/timing.
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
HAL = ROOT / "Drivers/STM32G4xx_HAL_Driver/Inc/stm32g4xx_hal_hrtim.h"
DEVICE = ROOT / "Drivers/CMSIS/Device/ST/STM32G4xx/Include/stm32g474xx.h"
NAMES = ["TimeBaseConfig", "WaveformTimerConfig", "WaveformTimerControl",
         "RollOverModeConfig", "WaveformCompareConfig", "DeadTimeConfig",
         "WaveformOutputConfig", "WaveformSetOutputLevel", "ADCTriggerConfig",
         "ADCPostScalerConfig", "SoftwareUpdate", "WaveformCountStop"]


def sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def function(source: str, signature: str) -> str:
    start = source.index(signature)
    brace = source.index("{", start)
    depth = 1
    end = brace + 1
    while depth:
        depth += (source[end] == "{") - (source[end] == "}")
        end += 1
    return source[start:end]


def extract_type(source: str, name: str) -> str:
    pattern = r"typedef\s+struct\s*\{[^{}]*\}\s*" + name + r"\s*;"
    match = re.search(pattern, source)
    if not match:
        raise ValueError(f"Missing actual HAL/CMSIS type: {name}")
    return match.group()


def stub_header(module: str, harness: str) -> str:
    hal = HAL.read_text(encoding="utf-8-sig")
    device = DEVICE.read_text(encoding="utf-8-sig")
    definitions = {}
    joined = re.sub(r"\\\r?\n", "", hal + "\n" + device)
    for name, body in re.findall(r"^#define\s+(\w+)\s+([^\n]+)", joined, re.M):
        if name.startswith("HRTIM_"):
            definitions[name] = re.sub(r"/\*.*?\*/", "", body).strip()
    wanted = set(re.findall(r"\bHRTIM_[A-Z][A-Z0-9_]+\b", module + harness))
    wanted.discard("HRTIM_SPWM_COMPARE_MARGIN_TICKS")
    wanted.discard("HRTIM_COMPARE_MARGIN_TICKS")
    resolved = {}

    def resolve(name: str) -> None:
        if name in resolved:
            return
        if name not in definitions:
            raise ValueError(f"HAL constant not found: {name}")
        body = definitions[name]
        for dependency in re.findall(r"\bHRTIM_[A-Za-z0-9_]+\b", body):
            resolve(dependency)
        resolved[name] = body

    for name in sorted(wanted):
        resolve(name)
    types = [extract_type(device, name) for name in
             ("HRTIM_Master_TypeDef", "HRTIM_Timerx_TypeDef", "HRTIM_Common_TypeDef", "HRTIM_TypeDef")]
    types += [extract_type(hal, name) for name in
              ("HRTIM_TimeBaseCfgTypeDef", "HRTIM_TimerCfgTypeDef", "HRTIM_TimerCtlTypeDef",
               "HRTIM_CompareCfgTypeDef", "HRTIM_DeadTimeCfgTypeDef", "HRTIM_OutputCfgTypeDef",
               "HRTIM_ADCTriggerCfgTypeDef")]
    prototypes = [re.search(r"HAL_StatusTypeDef\s+HAL_HRTIM_" + name + r"\([^;]+;", hal).group()
                  for name in NAMES]
    return """#ifndef MOCK_HAL_H
#define MOCK_HAL_H
#include <stdint.h>
#include <stddef.h>
#define __IO volatile
typedef enum { HAL_OK=0, HAL_ERROR=1, HAL_BUSY=2, HAL_TIMEOUT=3 } HAL_StatusTypeDef;
#define HAL_HRTIM_STATE_READY 1U
#define HAL_UNLOCKED 0U
""" + "\n".join(f"#define {name} {body}" for name, body in resolved.items()) + "\n" + "\n".join(types) + """
typedef struct { HRTIM_TypeDef *Instance; uint32_t State, Lock; } HRTIM_HandleTypeDef;
uint32_t __get_PRIMASK(void);
void __disable_irq(void);
void __set_PRIMASK(uint32_t value);
void MockCompare(HRTIM_HandleTypeDef *handle, uint32_t timer, uint32_t unit, uint32_t value);
void MockClear(HRTIM_HandleTypeDef *handle, uint32_t flag);
#define __HAL_HRTIM_SETCOMPARE(h,t,u,v) MockCompare(h,t,u,v)
#define __HAL_HRTIM_MASTER_CLEAR_FLAG(h,f) MockClear(h,f)
""" + "\n".join(prototypes) + "\n#endif\n"


HARNESS = r'''
#include "hrtim_spwm.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#define ALL_TIMERS (HRTIM_TIMERID_MASTER | HRTIM_TIMERID_TIMER_A | HRTIM_TIMERID_TIMER_B)
#define ALL_OUTPUTS (HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TA2 | HRTIM_OUTPUT_TB1 | HRTIM_OUTPUT_TB2)
#define AB_UPDATES (HRTIM_TIMERUPDATE_A | HRTIM_TIMERUPDATE_B)
static HRTIM_TypeDef registers;
static HRTIM_SPWM_Context context;
static HRTIM_SPWM_Config config;
HRTIM_HandleTypeDef hhrtim1;
static unsigned primask, calls, fail_at, guarded_writes, compare_calls, clear_calls;
static uint32_t active_a, active_b;
static uint8_t inject_valley;
uint16_t Period;
float modulation_command;
#define HRTIM_COMPARE_MARGIN_TICKS 6U
#define SPWM_MODULATION_LIMIT 0.95f

uint32_t __get_PRIMASK(void) { return primask; }
void __disable_irq(void) { primask=1; }
void __set_PRIMASK(uint32_t value) { assert(value<=1); primask=value; }
static HAL_StatusTypeDef result(void) { calls++; return calls==fail_at ? HAL_ERROR : HAL_OK; }
static void timer_index(uint32_t index) { assert(index==HRTIM_TIMERINDEX_MASTER || index<=HRTIM_TIMERINDEX_TIMER_B); }
static void valley(void) {
    if (!(registers.sCommonRegs.CR1 & HRTIM_TIMERUPDATE_A)) active_a=registers.sTimerxRegs[0].CMP1xR;
    if (!(registers.sCommonRegs.CR1 & HRTIM_TIMERUPDATE_B)) active_b=registers.sTimerxRegs[1].CMP1xR;
}
void MockCompare(HRTIM_HandleTypeDef *h, uint32_t timer, uint32_t unit, uint32_t value) {
    assert(h==&hhrtim1 && timer<=HRTIM_TIMERINDEX_TIMER_B && unit==HRTIM_COMPAREUNIT_1);
    if (guarded_writes) assert(primask && (registers.sCommonRegs.CR1 & AB_UPDATES)==AB_UPDATES);
    h->Instance->sTimerxRegs[timer].CMP1xR=value;
    compare_calls++;
    if (inject_valley && timer==0) {
        uint32_t old_a=active_a, old_b=active_b;
        valley();
        assert(active_a==old_a && active_b==old_b);
    }
}
void MockClear(HRTIM_HandleTypeDef *h, uint32_t flag) {
    assert(h==&hhrtim1 && flag==HRTIM_MASTER_FLAG_MREP && primask==1);
    assert((registers.sMasterRegs.MCR & ALL_TIMERS)==0);
    assert(registers.sCommonRegs.CR2==0 && registers.sCommonRegs.OENR==0);
    clear_calls++;
    h->Instance->sMasterRegs.MICR=flag;
}
HAL_StatusTypeDef HAL_HRTIM_TimeBaseConfig(HRTIM_HandleTypeDef *h, uint32_t index, const HRTIM_TimeBaseCfgTypeDef *cfg) {
    HAL_StatusTypeDef s=result(); timer_index(index); assert(h==&hhrtim1);
    assert(cfg->Mode==HRTIM_MODE_CONTINUOUS && cfg->PrescalerRatio==HRTIM_PRESCALERRATIO_DIV1 && cfg->RepetitionCounter==0);
    assert(cfg->Period==(index==HRTIM_TIMERINDEX_MASTER ? 2*config.carrier_peak_ticks : config.carrier_peak_ticks));
    if(s==HAL_OK) { if(index==HRTIM_TIMERINDEX_MASTER) registers.sMasterRegs.MPER=cfg->Period;
    else registers.sTimerxRegs[index].PERxR=cfg->Period; } return s;
}
HAL_StatusTypeDef HAL_HRTIM_WaveformTimerConfig(HRTIM_HandleTypeDef *h, uint32_t index, const HRTIM_TimerCfgTypeDef *cfg) {
    assert(h==&hhrtim1); timer_index(index);
    assert(cfg->PreloadEnable==HRTIM_PRELOAD_ENABLED && cfg->RepetitionUpdate==HRTIM_UPDATEONREPETITION_ENABLED);
    assert(cfg->StartOnSync==HRTIM_SYNCSTART_DISABLED && cfg->ResetOnSync==HRTIM_SYNCRESET_DISABLED);
    assert(cfg->InterruptRequests==0 && cfg->DMARequests==0 && cfg->UpdateGating==HRTIM_UPDATEGATING_INDEPENDENT);
    if(index!=HRTIM_TIMERINDEX_MASTER) assert(cfg->ResetTrigger==HRTIM_TIMRESETTRIGGER_NONE && cfg->UpdateTrigger==HRTIM_TIMUPDATETRIGGER_NONE && cfg->DeadTimeInsertion==HRTIM_TIMDEADTIMEINSERTION_ENABLED);
    return result();
}
HAL_StatusTypeDef HAL_HRTIM_WaveformTimerControl(HRTIM_HandleTypeDef *h,uint32_t index,const HRTIM_TimerCtlTypeDef *cfg) {
    assert(h==&hhrtim1 && index<=1 && cfg->UpDownMode==HRTIM_TIMERUPDOWNMODE_UPDOWN && cfg->GreaterCMP1==HRTIM_TIMERGTCMP1_EQUAL); return result();
}
HAL_StatusTypeDef HAL_HRTIM_RollOverModeConfig(HRTIM_HandleTypeDef *h,uint32_t index,uint32_t mode) {
    assert(h==&hhrtim1 && index<=1 && mode==(HRTIM_TIM_FEROM_VALLEY|HRTIM_TIM_BMROM_VALLEY|HRTIM_TIM_ADROM_VALLEY|HRTIM_TIM_OUTROM_VALLEY|HRTIM_TIM_ROM_VALLEY)); return result();
}
HAL_StatusTypeDef HAL_HRTIM_WaveformCompareConfig(HRTIM_HandleTypeDef *h,uint32_t index,uint32_t unit,const HRTIM_CompareCfgTypeDef *cfg) {
    HAL_StatusTypeDef s=result(); assert(h==&hhrtim1 && index<=1 && unit==HRTIM_COMPAREUNIT_1);
    assert(cfg->CompareValue==config.carrier_peak_ticks/2+config.carrier_peak_ticks%2);
    if(s==HAL_OK) { registers.sTimerxRegs[index].CMP1xR=cfg->CompareValue; } return s;
}
HAL_StatusTypeDef HAL_HRTIM_DeadTimeConfig(HRTIM_HandleTypeDef *h,uint32_t index,const HRTIM_DeadTimeCfgTypeDef *cfg) {
    assert(h==&hhrtim1 && index<=1 && cfg->Prescaler==HRTIM_TIMDEADTIME_PRESCALERRATIO_DIV1);
    assert(cfg->RisingValue==(index==0 ? config.deadtime_a_rising_ticks : config.deadtime_b_rising_ticks));
    assert(cfg->FallingValue==(index==0 ? config.deadtime_a_falling_ticks : config.deadtime_b_falling_ticks)); return result();
}
HAL_StatusTypeDef HAL_HRTIM_WaveformOutputConfig(HRTIM_HandleTypeDef *h,uint32_t index,uint32_t output,const HRTIM_OutputCfgTypeDef *cfg) {
    assert(h==&hhrtim1 && index<=1 && cfg->Polarity==HRTIM_OUTPUTPOLARITY_HIGH && cfg->SetSource==HRTIM_OUTPUTSET_NONE);
    assert(output==(index==0 ? HRTIM_OUTPUT_TA1 : HRTIM_OUTPUT_TB1) || output==(index==0 ? HRTIM_OUTPUT_TA2 : HRTIM_OUTPUT_TB2));
    assert(cfg->ResetSource==((output==(index==0 ? HRTIM_OUTPUT_TA1 : HRTIM_OUTPUT_TB1)) ? HRTIM_OUTPUTRESET_TIMCMP1 : HRTIM_OUTPUTRESET_NONE)); return result();
}
HAL_StatusTypeDef HAL_HRTIM_WaveformSetOutputLevel(HRTIM_HandleTypeDef *h,uint32_t index,uint32_t output,uint32_t level) {
    assert(h==&hhrtim1 && index<=1 && output==(index==0 ? HRTIM_OUTPUT_TA1 : HRTIM_OUTPUT_TB1) && level==HRTIM_OUTPUTLEVEL_ACTIVE); return result();
}
HAL_StatusTypeDef HAL_HRTIM_ADCTriggerConfig(HRTIM_HandleTypeDef *h,uint32_t trigger,const HRTIM_ADCTriggerCfgTypeDef *cfg) {
    assert(h==&hhrtim1 && trigger==HRTIM_ADCTRIGGER_1 && cfg->UpdateSource==HRTIM_ADCTRIGGERUPDATE_MASTER && cfg->Trigger==HRTIM_ADCTRIGGEREVENT13_MASTER_PERIOD); return result();
}
HAL_StatusTypeDef HAL_HRTIM_ADCPostScalerConfig(HRTIM_HandleTypeDef *h,uint32_t trigger,uint32_t scale) {
    assert(h==&hhrtim1 && trigger==HRTIM_ADCTRIGGER_1 && scale==0); return result();
}
HAL_StatusTypeDef HAL_HRTIM_SoftwareUpdate(HRTIM_HandleTypeDef *h,uint32_t timers) {
    assert(h==&hhrtim1 && timers==(HRTIM_TIMERUPDATE_MASTER|AB_UPDATES)); return result();
}
HAL_StatusTypeDef HAL_HRTIM_WaveformCountStop(HRTIM_HandleTypeDef *h,uint32_t timers) {
    HAL_StatusTypeDef s=result(); assert(h==&hhrtim1 && timers==ALL_TIMERS && registers.sCommonRegs.ODISR==ALL_OUTPUTS);
    if(s==HAL_OK) { registers.sMasterRegs.MCR &= ~timers; } return s;
}

@ORIGINAL@

static void fresh(void) {
    memset(&registers,0,sizeof(registers)); memset(&context,0,sizeof(context));
    memset(&registers.sTimerxRegs[2],0xA5,4*sizeof(HRTIM_Timerx_TypeDef));
    hhrtim1.Instance=&registers; hhrtim1.State=HAL_HRTIM_STATE_READY; hhrtim1.Lock=HAL_UNLOCKED;
    calls=fail_at=guarded_writes=compare_calls=clear_calls=primask=inject_valley=0;
    HRTIM_SPWM_DefaultConfig(&config);
}
static void check_others(void) {
    unsigned char expected[4*sizeof(HRTIM_Timerx_TypeDef)]; memset(expected,0xA5,sizeof(expected));
    assert(memcmp(expected,&registers.sTimerxRegs[2],sizeof(expected))==0);
}
static void initialization(void) {
    unsigned count, fail; fresh();
    assert(HRTIM_SPWM_Init(&context,&hhrtim1,&config)==HAL_OK); count=calls;
    assert(context.initialized && context.update_hz==20000.0f && !context.counters_started && !context.outputs_enabled);
    assert(registers.sMasterRegs.MCR==0 && registers.sCommonRegs.OENR==0 && registers.sCommonRegs.ODISR==ALL_OUTPUTS);
    assert(context.compare_a==2125 && context.compare_b==2125); check_others();
    for(fail=1;fail<=count;fail++) {
        fresh(); fail_at=fail;
        assert(HRTIM_SPWM_Init(&context,&hhrtim1,&config)==HAL_ERROR && !context.initialized);
        assert(calls==fail && registers.sMasterRegs.MCR==0 && registers.sCommonRegs.OENR==0);
        check_others();
    }
    printf("PASS initialization: %u HAL failure positions, M/A/B only, no automatic outputs\n",count);
}
static void formula(void) {
    uint32_t peaks[]={13,4250,32767}, p, n, old_a, old_b;
    for(p=0;p<3;p++) {
        fresh(); config.carrier_peak_ticks=peaks[p];
        assert(HRTIM_SPWM_Init(&context,&hhrtim1,&config)==HAL_OK); Period=(uint16_t)peaks[p];
        for(n=0;n<=20000;n++) {
            float m=-1.5f+(3.0f*(float)n/20000.0f), old_m;
            guarded_writes=inject_valley=0; OriginalSetModulation(m);
            old_a=registers.sTimerxRegs[0].CMP1xR; old_b=registers.sTimerxRegs[1].CMP1xR; old_m=modulation_command;
            active_a=999; active_b=888; registers.sCommonRegs.CR1=HRTIM_TIMERUPDATE_C;
            primask=n%2; guarded_writes=inject_valley=1;
            assert(HRTIM_SetModulation(&context,m)==HAL_OK);
            assert(primask==n%2 && registers.sCommonRegs.CR1==HRTIM_TIMERUPDATE_C);
            assert(context.compare_a==old_a && context.compare_b==old_b && context.modulation==old_m);
            assert(active_a==999 && active_b==888); valley(); assert(active_a==old_a && active_b==old_b);
            check_others();
        }
    }
    /* 把m限值开放到1时，仍严格保护距离上下边界6 tick。 */
    context.config.modulation_limit=1.0f;
    assert(HRTIM_SetModulation(&context,1)==HAL_OK && context.compare_b==6 && context.compare_a==32761);
    assert(HRTIM_SetModulation(&context,-1)==HAL_OK && context.compare_a==6 && context.compare_b==32761);
    printf("PASS 60003 original-formula comparisons; injected valley between writes; PRIMASK and unrelated update bits\n");
}
static void lifecycle(void) {
    fresh(); assert(HRTIM_SPWM_Init(&context,&hhrtim1,&config)==HAL_OK);
    assert(HRTIM_SPWM_EnableOutputs(&context)==HAL_ERROR);
    registers.sMasterRegs.MCR=HRTIM_TIMERID_TIMER_C;
    registers.sMasterRegs.MCNTR=243;
    assert(HRTIM_SPWM_StartCounters(&context)==HAL_BUSY && !context.ever_started);
    registers.sMasterRegs.MCNTR=0;
    primask=1; assert(HRTIM_SPWM_StartCounters(&context)==HAL_OK && primask==1 && clear_calls==1);
    assert((registers.sMasterRegs.MCR & HRTIM_TIMERID_TIMER_C)!=0 && registers.sCommonRegs.OENR==0);
    assert(context.counters_started && !context.outputs_enabled && HRTIM_SPWM_StartCounters(&context)==HAL_BUSY);
    assert(HRTIM_SPWM_EnableOutputs(&context)==HAL_OK && registers.sCommonRegs.OENR==ALL_OUTPUTS);
    HRTIM_SPWM_DisableOutputs(&context); assert(registers.sCommonRegs.ODISR==ALL_OUTPUTS && !context.outputs_enabled);
    /* 模拟ODISR对硬件OENR使能状态的清除（普通主机内存没有寄存器副作用）。 */
    registers.sCommonRegs.OENR=0;
    fail_at=calls+1; assert(HRTIM_SPWM_Stop(&context)==HAL_ERROR && context.counters_started && !context.outputs_enabled);
    fail_at=0; assert(HRTIM_SPWM_Stop(&context)==HAL_OK && !context.counters_started);
    assert(registers.sMasterRegs.MCR==HRTIM_TIMERID_TIMER_C);
    primask=0; assert(HRTIM_SPWM_StartCounters(&context)==HAL_BUSY && primask==0 && clear_calls==1);
    assert(context.ever_started && HRTIM_SPWM_Init(&context,&hhrtim1,&config)==HAL_BUSY);
    check_others();
    puts("PASS first-zero-count start, separate output control, stop prevents restart, unrelated timer preserved");
}
static void bad_inputs(void) {
    uint32_t before; fresh(); HRTIM_SPWM_DefaultConfig(NULL);
    assert(HRTIM_SPWM_Init(NULL,&hhrtim1,&config)==HAL_ERROR);
    assert(HRTIM_SPWM_Init(&context,NULL,&config)==HAL_ERROR);
    assert(HRTIM_SPWM_Init(&context,&hhrtim1,NULL)==HAL_ERROR);
    assert(HRTIM_SPWM_StartCounters(NULL)==HAL_ERROR && HRTIM_SPWM_Stop(NULL)==HAL_ERROR);
    assert(HRTIM_SPWM_EnableOutputs(NULL)==HAL_ERROR && HRTIM_SetModulation(NULL,0)==HAL_ERROR);
    HRTIM_SPWM_DisableOutputs(NULL);
    config.carrier_peak_ticks=12; assert(HRTIM_SPWM_Init(&context,&hhrtim1,&config)==HAL_ERROR);
    config.carrier_peak_ticks=32768; assert(HRTIM_SPWM_Init(&context,&hhrtim1,&config)==HAL_ERROR);
    HRTIM_SPWM_DefaultConfig(&config); config.modulation_limit=NAN;
    assert(HRTIM_SPWM_Init(&context,&hhrtim1,&config)==HAL_ERROR);
    config.modulation_limit=INFINITY; assert(HRTIM_SPWM_Init(&context,&hhrtim1,&config)==HAL_ERROR);
    HRTIM_SPWM_DefaultConfig(&config); config.deadtime_b_rising_ticks=512;
    assert(HRTIM_SPWM_Init(&context,&hhrtim1,&config)==HAL_ERROR);
    HRTIM_SPWM_DefaultConfig(&config); config.timer_clock_hz=0;
    assert(HRTIM_SPWM_Init(&context,&hhrtim1,&config)==HAL_ERROR);
    HRTIM_SPWM_DefaultConfig(&config); registers.sMasterRegs.MCR=HRTIM_TIMERID_TIMER_A;
    assert(HRTIM_SPWM_Init(&context,&hhrtim1,&config)==HAL_BUSY && calls==0);
    registers.sMasterRegs.MCR=0; hhrtim1.Lock=1;
    assert(HRTIM_SPWM_Init(&context,&hhrtim1,&config)==HAL_BUSY && calls==0);
    hhrtim1.Lock=HAL_UNLOCKED; assert(HRTIM_SPWM_Init(&context,&hhrtim1,&config)==HAL_OK);
    before=compare_calls;
    assert(HRTIM_SetModulation(&context,NAN)==HAL_ERROR && HRTIM_SetModulation(&context,INFINITY)==HAL_ERROR);
    assert(HRTIM_SetModulation(&context,-INFINITY)==HAL_ERROR && compare_calls==before);
    registers.sCommonRegs.CR1=HRTIM_TIMERUPDATE_A; primask=0;
    assert(HRTIM_SetModulation(&context,.1f)==HAL_BUSY && compare_calls==before && primask==0);
    assert(registers.sCommonRegs.CR1==HRTIM_TIMERUPDATE_A);
    puts("PASS null/uninitialized/busy/invalid inputs; rejected requests do not write compares");
}
int main(void) { initialization(); formula(); lifecycle(); bad_inputs(); return 0; }
'''


def main() -> int:
    gcc = shutil.which("gcc")
    if not gcc:
        raise FileNotFoundError("gcc is required for the host model")
    module = (ROOT / "hrtim_spwm.c").read_text(encoding="utf-8")
    original = (ROOT / "Core/Src/main.c").read_text(encoding="utf-8-sig")
    qpr_header = (ROOT / "Core/Inc/qpr_control.h").read_text(encoding="utf-8-sig")
    extracted = function(original, "static void HRTIM_SetModulation(float modulation)")
    extracted = extracted.replace("static void HRTIM_SetModulation", "static void OriginalSetModulation", 1)
    harness = HARNESS.replace("@ORIGINAL@", extracted)
    margin = re.search(r"^#define\s+HRTIM_COMPARE_MARGIN_TICKS\s+(\S+)", original, re.M).group(1)
    limit = re.search(r"^#define\s+QPR_MODULATION_LIMIT\s+(\S+)", qpr_header, re.M).group(1)
    assert re.search(r"^#define\s+SPWM_MODULATION_LIMIT\s+QPR_MODULATION_LIMIT\b", original, re.M)
    harness = harness.replace("#define HRTIM_COMPARE_MARGIN_TICKS 6U", "#define HRTIM_COMPARE_MARGIN_TICKS " + margin)
    harness = harness.replace("#define SPWM_MODULATION_LIMIT 0.95f", "#define SPWM_MODULATION_LIMIT " + limit)
    start = function(module, "HAL_StatusTypeDef HRTIM_SPWM_StartCounters(")
    assert start.count("instance->sMasterRegs.MCR |= SPWM_TIMERS")==1
    assert "instance->sCommonRegs.CR2 =" not in start and "SETCOUNTER" not in start
    setting = function(module, "HAL_StatusTypeDef HRTIM_SetModulation(")
    stripped = re.sub(r"/\*.*?\*/", "", setting, flags=re.S)
    assert "while" not in stripped and not re.search(r"\bHAL_HRTIM_\w+\s*\(", stripped)
    assert "HRTIM_TIMERINDEX_TIMER_C" not in module and "HRTIM_TIMERINDEX_TIMER_D" not in module
    files = [ROOT / "hrtim_spwm.c", ROOT / "hrtim_spwm.h", ROOT / "Core/Src/main.c",
             ROOT / "Core/Inc/qpr_control.h", HAL, DEVICE]
    before = {str(path.relative_to(ROOT)): sha(path) for path in files}
    result = {
        "timestamp_utc": dt.datetime.now(dt.timezone.utc).isoformat(),
        "scope": "Software mock/model and ARM translation-unit checks; no hardware waveform, dead-time, ADC aperture, timing or power-stage validation.",
        "sources_sha256": before,
        "checks": [],
    }
    with tempfile.TemporaryDirectory(prefix="dcac_hrtim_spwm_") as directory:
        work = Path(directory)
        (work / "stm32g4xx_hal.h").write_text(stub_header(module, harness), encoding="utf-8")
        (work / "harness.c").write_text(harness, encoding="utf-8")
        executable = work / "verify_hrtim.exe"
        command = [gcc, "-std=c11", "-Wall", "-Wextra", "-Werror", "-O0", "-I"+str(work),
                   "-I"+str(ROOT), str(ROOT/"hrtim_spwm.c"), str(work/"harness.c"), "-lm", "-o", str(executable)]
        compiled = subprocess.run(command, cwd=work, capture_output=True, text=True)
        if compiled.returncode:
            raise RuntimeError(compiled.stdout + compiled.stderr)
        run = subprocess.run([str(executable)], cwd=work, capture_output=True, text=True)
        print(run.stdout + run.stderr, end="")
        result["checks"].append({"name": "host_model", "passed": run.returncode==0, "output": run.stdout+run.stderr})
        if run.returncode:
            raise RuntimeError("HRTIM host assertions failed")
        settings = project_settings(ROOT / "MDK-ARM/dcac.uvprojx", "dcac")
        object_file = work / "hrtim_spwm.o"
        arm_cmd = [str(DEFAULT_COMPILER), "--cpu", "Cortex-M4.fp", "--c99", "-O2"]
        arm_cmd += ["-D"+value for value in settings["defines"]]
        arm_cmd += ["-I"+value for value in settings["include_paths"]]
        arm_cmd += ["-c", str(ROOT/"hrtim_spwm.c"), "-o", str(object_file)]
        # ARM compilation must use actual HAL, so keep the mock header directory out of include search.
        arm = subprocess.run(arm_cmd, cwd=ROOT, capture_output=True, text=True)
        passed = arm.returncode==0 and object_file.is_file() and object_file.stat().st_size>0
        result["checks"].append({"name": "armcc_actual_hal", "passed": passed, "command": arm_cmd,
                                 "output": arm.stdout+arm.stderr,
                                 "object_bytes": object_file.stat().st_size if object_file.is_file() else 0})
        print("PASS ARMCC actual STM32G474 HAL" if passed else arm.stdout+arm.stderr)
    result["temporary_intermediates_removed"] = not work.exists()
    result["sources_unchanged_during_checks"] = before == {str(path.relative_to(ROOT)): sha(path) for path in files}
    result["checks_passed"] = all(item["passed"] for item in result["checks"]) and result["sources_unchanged_during_checks"]
    output = ROOT/"Tests/results/hrtim_spwm_module_verification.json"
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(result, ensure_ascii=False, indent=2)+"\n", encoding="utf-8")
    print(f"Result: {output}")
    return 0 if result["checks_passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
