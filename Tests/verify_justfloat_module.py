"""Compile real root waveform-only JustFloat; compare against extracted main.c.

Uses HAL/CMSIS host stubs, not hardware, ADC timing or physical UART validation.
Only temporary build files and results/justfloat_module_verification.json are written.
"""
from pathlib import Path
import datetime as dt
import hashlib
import json
import re
import shutil
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
SOURCE = (ROOT / "Core/Src/main.c").read_text(encoding="utf-8-sig")


def extract_function(name):
    start = SOURCE.index(f"static void {name}(")
    brace = SOURCE.index("{", start)
    end, depth = brace + 1, 1
    while depth:
        depth += (SOURCE[end] == "{") - (SOURCE[end] == "}")
        end += 1
    return SOURCE[start:end]


ORIGINAL_FUNCTIONS = "\n".join(extract_function(name) for name in (
    "VOFA_AccumulateVoltageRms", "VOFA_AccumulateMean", "VOFA_SendJustFloat", "VOFA_Process"))
callback = SOURCE[SOURCE.index("void HAL_ADC_ConvCpltCallback("):
                  SOURCE.index("void HAL_ADC_ErrorCallback(")]
begin = callback.index("    VOFA_AccumulateVoltageRms(closed_loop);")
ORIGINAL_TELEMETRY = callback[begin:callback.index("#endif", begin)]
ORIGINAL_DEFINES = "\n".join(re.search(r"^#define " + name + r"\s+\S+", SOURCE, re.M).group()
                              for name in ("VOFA_SAMPLE_DIV", "VOFA_MEAN_SAMPLE_COUNT", "VOFA_UART_TIMEOUT_MS"))

HAL = r'''
#ifndef TEST_STM32_HAL_H
#define TEST_STM32_HAL_H
#include <stdint.h>
#include <stddef.h>
typedef enum { HAL_OK=0, HAL_ERROR=1, HAL_BUSY=2, HAL_TIMEOUT=3 } HAL_StatusTypeDef;
typedef struct { int identity; } UART_HandleTypeDef;
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *, uint8_t *, uint16_t, uint32_t);
uint32_t __get_PRIMASK(void);
void __disable_irq(void);
void __set_PRIMASK(uint32_t);
#endif
'''
CMSIS = r'''
#ifndef TEST_ARM_MATH_H
#define TEST_ARM_MATH_H
#include <math.h>
static inline int arm_sqrt_f32(float value, float *out) {
    *out=value<0.0f?0.0f:sqrtf(value);return value<0.0f?-1:0;
}
#endif
'''

HARNESS = r'''
#include "justfloat.h"
#include "arm_math.h"
#include <assert.h>
#include <float.h>
#include <stdio.h>
#include <string.h>
#define PI 3.14159265358979323846
#define VOFA_MEAN_OUTPUT_ENABLE 0U
__ORIGINAL_DEFINES__
static UART_HandleTypeDef huart4;
static uint32_t irq_mask, transmit_calls, irq_disable_calls, irq_restore_calls;
static uint32_t expected_tx_mask;
static HAL_StatusTypeDef next_status = HAL_OK;
static uint8_t captured[36];
static JustFloat_Context *inject_context;
static JustFloat_SampleData inject_sample;
static uint32_t inject_count;
uint32_t __get_PRIMASK(void) { return irq_mask; }
void __disable_irq(void) { irq_disable_calls++;irq_mask=1; }
void __set_PRIMASK(uint32_t value) { irq_restore_calls++;irq_mask=value; }
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *uart,uint8_t *data,uint16_t size,uint32_t timeout) {
    assert(uart==&huart4 && size==36 && timeout==JUSTFLOAT_UART_TIMEOUT_MS);
    assert(irq_mask==expected_tx_mask);
    transmit_calls++;memcpy(captured,data,36);
    if(inject_context) {
        JustFloat_Context *context=inject_context;inject_context=NULL;
        assert(!irq_mask);
        for(unsigned i=0;i<inject_count;i++) JustFloat_Sample(context,&inject_sample,0);
    }
    return next_status;
}
static float true_voltage,true_current,true_aux_current,dc_bus_voltage_v,voltage_reference_v;
static float output_voltage_rms_v,output_voltage_mean_v,output_voltage_ac_rms_v;
static float voltage_rms_square_sum,voltage_rms_sum,voltage_rms_mean_scale;
static uint32_t voltage_rms_count,output_voltage_rms_window_samples;
static uint32_t output_voltage_rms_window_count;
static uint8_t voltage_rms_mode,output_voltage_rms_valid;
static float vofa_ch1,vofa_ch2,vofa_ch3,vofa_ch4,vofa_ch5,vofa_ch6,vofa_ch7,vofa_ch8;
static float vofa_mean_sum[6];
static uint32_t vofa_mean_count,vofa_mean_window_count,vofa_overwrite_count;
static uint8_t vofa_mean_mode,vofa_frame_ready;
static uint32_t adc_sample_div_count,vofa_sent_count,vofa_tx_error_count;
static uint8_t vofa_frame[36];
static struct {float reference_v,current_reference_a,resonant_r,qpr_unlimited_v,qpr_correction_v,bridge_reference_v;} qpr_control;
__ORIGINAL_FUNCTIONS__
static void OriginalSample(const JustFloat_SampleData *data,uint8_t closed_loop) {
    true_voltage=data->voltage_v;true_current=data->current_a;true_aux_current=data->auxiliary_current_a;
    dc_bus_voltage_v=data->dc_bus_v;voltage_reference_v=data->reference_v;
    qpr_control.reference_v=data->reference_v;qpr_control.current_reference_a=data->current_reference_a;
    qpr_control.qpr_correction_v=data->controller_limited_output_v;
    /* Keep feedforward + correction distinct to detect a CH5 source regression. */
    qpr_control.bridge_reference_v=data->reference_v+data->controller_limited_output_v;
__ORIGINAL_TELEMETRY__
}
static void compare(const JustFloat_Context *context) {
    float expected[8]={vofa_ch1,vofa_ch2,vofa_ch3,vofa_ch4,vofa_ch5,vofa_ch6,vofa_ch7,vofa_ch8};
    for(unsigned i=0;i<8;i++) assert(context->channels[i]==expected[i]);
    assert(context->frame_ready==vofa_frame_ready);
    assert(context->sample_div_count==adc_sample_div_count);
    assert(context->overwrite_count==vofa_overwrite_count);
    assert(context->sent_count==vofa_sent_count && context->tx_error_count==vofa_tx_error_count);
    assert(context->rms_count==voltage_rms_count && context->rms_square_sum==voltage_rms_square_sum);
    assert(context->rms_mode==voltage_rms_mode);
    assert(context->output_voltage_rms_v==output_voltage_rms_v);
    assert(context->output_voltage_rms_valid==output_voltage_rms_valid);
    assert(context->output_voltage_rms_window_count==output_voltage_rms_window_count);
}
static void run_differential(void) {
    JustFloat_Context context={0};JustFloat_SampleData data;
    assert(JustFloat_Init(&context,&huart4,20000,50));
    output_voltage_rms_window_samples=context.output_voltage_rms_window_samples;
    voltage_rms_mean_scale=1.0f/output_voltage_rms_window_samples;
    for(unsigned n=0;n<7500;n++) {
        uint8_t mode=(n>=173 && n<1901)||(n>=2455 && n<5103);
        float p=(float)(2*PI*n/400);
        data.reference_v=30*sinf(p);data.voltage_v=2+25*sinf(p)+3*cosf(3*p);
        data.current_reference_a=1.23f*cosf(p);data.current_a=0.731f*sinf(p);
        data.controller_limited_output_v=4.97f*cosf(p);
        data.auxiliary_current_a=0.6f*sinf(p);data.dc_bus_v=48+0.3f*cosf(2*p);
        JustFloat_Sample(&context,&data,mode);OriginalSample(&data,mode);compare(&context);
        if(n%37==0 || n%103==0) {
            uint8_t actual_frame[36];uint32_t before=transmit_calls;
            uint32_t ready=context.frame_ready;
            expected_tx_mask=irq_mask=n%2;next_status=(HAL_StatusTypeDef)(n%4);
            JustFloat_Process(&context);memcpy(actual_frame,captured,36);
            assert(irq_mask==expected_tx_mask && transmit_calls==before+ready);
            VOFA_Process();assert(irq_mask==expected_tx_mask);
            if(ready) assert(memcmp(actual_frame,captured,36)==0);
            compare(&context);
        }
    }
    irq_mask=expected_tx_mask=0;next_status=HAL_OK;
    printf("PASS 7500 per-sample state and transmitted-byte comparisons with original main.c\n");
}
static void run_safety_and_frame(void) {
    JustFloat_Context context={0};JustFloat_SampleData data={0};
    uint32_t before=transmit_calls;
    JustFloat_Sample(NULL,&data,0);JustFloat_Sample(&context,&data,0);JustFloat_Process(NULL);JustFloat_Process(&context);
    assert(JustFloat_Send8(NULL,0,0,0,0,0,0,0,0)==HAL_ERROR);
    assert(JustFloat_Send8(&context,0,0,0,0,0,0,0,0)==HAL_ERROR && transmit_calls==before);
    assert(!JustFloat_Init(NULL,&huart4,20000,50));
    assert(!JustFloat_Init(&context,NULL,20000,50));
    assert(!JustFloat_Init(&context,&huart4,0,50));
    assert(!JustFloat_Init(&context,&huart4,20000,0));
    assert(!JustFloat_Init(&context,&huart4,NAN,50));
    assert(!JustFloat_Init(&context,&huart4,20000,INFINITY));
    assert(!JustFloat_Init(&context,&huart4,1,2));
    assert(!JustFloat_Init(&context,&huart4,FLT_MAX,FLT_MIN));
    assert(!context.initialized && !context.sent_count);
    assert(JustFloat_Init(&context,&huart4,20000,50));
    JustFloat_Sample(&context,NULL,0);
    assert(context.output_voltage_rms_window_samples==400);
    const float expected[8]={1.25f,-2.5f,3.75f,-4,5.5f,-6.25f,7.125f,24};
    const uint8_t tail[4]={0,0,0x80,0x7f};
    for(unsigned status=0;status<4;status++) {
        next_status=(HAL_StatusTypeDef)status;
        assert(JustFloat_Send8(&context,expected[0],expected[1],expected[2],expected[3],expected[4],expected[5],expected[6],expected[7])==next_status);
        assert(memcmp(captured,expected,32)==0 && memcmp(captured+32,tail,4)==0);
    }
    assert(context.sent_count==1 && context.tx_error_count==3 && !context.frame_ready);
    next_status=HAL_OK;
    printf("PASS initialization guards, 36-byte framing and all HAL status counters\n");
}
static void near(float value,double expected) { assert(fabs(value-expected)<0.0002); }
static void run_rms_and_mapping(void) {
    JustFloat_Context context={0};JustFloat_SampleData data={10,2,3,4,7,8,48};
    assert(JustFloat_Init(&context,&huart4,20000,50));
    const unsigned publish=JUSTFLOAT_SAMPLE_DIV;
    for(unsigned i=0;i<publish-1;i++) JustFloat_Sample(&context,&data,0);
    assert(!context.frame_ready);JustFloat_Sample(&context,&data,0);assert(context.frame_ready);
    const float expected[7]={10,2,3,4,7,8,48};
    for(unsigned i=0;i<7;i++) near(context.channels[i],expected[i]);
    assert(JustFloat_Init(&context,&huart4,20000,50));
    for(unsigned n=0;n<400;n++) {
        double p=2*PI*n/400;data.voltage_v=(float)(2+24*sqrt(2.0)*sin(p)+3*sqrt(2.0)*sin(3*p));
        JustFloat_Sample(&context,&data,0);
        if(n<399) assert(!context.output_voltage_rms_valid);
    }
    near(context.output_voltage_rms_v,sqrt(589));
    assert(context.output_voltage_rms_window_count==1);
    for(unsigned n=0;n<173;n++) { data.voltage_v=100;JustFloat_Sample(&context,&data,0); }
    for(unsigned n=0;n<400;n++) {
        data.voltage_v=(n%10==0)?9:-1;JustFloat_Sample(&context,&data,1);
        if(n<399) assert(!context.output_voltage_rms_valid && context.output_voltage_rms_v==0);
    }
    near(context.output_voltage_rms_v,3);
    assert(context.output_voltage_rms_window_count==2);
    for(unsigned n=0;n<400;n++) { data.voltage_v=-5;JustFloat_Sample(&context,&data,1); }
    near(context.output_voltage_rms_v,5);
    assert(JustFloat_Init(&context,&huart4,16000,50) && context.output_voltage_rms_window_samples==320);
    for(unsigned n=0;n<320;n++) { data.voltage_v=25;JustFloat_Sample(&context,&data,0); }
    near(context.output_voltage_rms_v,25);
    printf("PASS channel mappings, decimation, all-sample RMS/DC/harmonics and mode reset\n");
}
static void run_mailbox(void) {
    JustFloat_Context context={0};JustFloat_SampleData data={10,2,3,4,7,8,48};
    assert(JustFloat_Init(&context,&huart4,20000,50));
    const unsigned publish=JUSTFLOAT_SAMPLE_DIV;
    for(unsigned n=0;n<3*publish;n++) JustFloat_Sample(&context,&data,0);
    assert(context.frame_ready && context.overwrite_count==2);
    for(unsigned mask=0;mask<2;mask++) {
        uint32_t before=transmit_calls,disables=irq_disable_calls,restores=irq_restore_calls;
        irq_mask=expected_tx_mask=mask;JustFloat_Process(&context);
        assert(!context.frame_ready && irq_mask==mask && transmit_calls==before+1);
        assert(irq_disable_calls==disables+1 && irq_restore_calls==restores+1);
        JustFloat_Process(&context);assert(transmit_calls==before+1);
        for(unsigned n=0;n<publish;n++) JustFloat_Sample(&context,&data,0);
    }
    irq_mask=expected_tx_mask=0;next_status=HAL_TIMEOUT;
    inject_context=&context;inject_sample=data;inject_sample.reference_v=90;inject_count=2*publish;
    uint32_t before=transmit_calls,errors=context.tx_error_count,overwrites=context.overwrite_count;
    JustFloat_Process(&context);
    assert(transmit_calls==before+1 && context.tx_error_count==errors+1);
    assert(context.frame_ready && context.overwrite_count==overwrites+1);
    /* The just-failed frame is discarded; two incoming frames leave the latest pending. */
    next_status=HAL_OK;JustFloat_Process(&context);
    float first;memcpy(&first,captured,4);near(first,90);
    assert(!context.frame_ready && transmit_calls==before+2);
    JustFloat_Process(&context);assert(transmit_calls==before+2);
    printf("PASS mailbox overwrite, PRIMASK restoration, failure discard and ADC arrival during send\n");
}
int main(void) {
    run_differential();run_safety_and_frame();run_rms_and_mapping();run_mailbox();
    printf("PASS waveform-only mode\n");return 0;
}
'''.replace("__ORIGINAL_DEFINES__", ORIGINAL_DEFINES).replace(
    "__ORIGINAL_FUNCTIONS__", ORIGINAL_FUNCTIONS).replace("__ORIGINAL_TELEMETRY__", ORIGINAL_TELEMETRY)


def main():
    gcc = shutil.which("gcc")
    if not gcc:
        raise SystemExit("Host GCC is required")
    result = {
        "created_utc": dt.datetime.now(dt.timezone.utc).isoformat(),
        "scope": "Real root module, host HAL/CMSIS stubs, extracted original main.c differential checks",
        "limitations": "No physical UART/VOFA, target interrupt timing or hardware validation",
        "source_sha256": {str(path.relative_to(ROOT)): hashlib.sha256(path.read_bytes()).hexdigest()
                          for path in (ROOT / "justfloat.c", ROOT / "justfloat.h", ROOT / "Core/Src/main.c")},
        "compiler": gcc,
        "mode": "waveform",
    }
    with tempfile.TemporaryDirectory(prefix="dcac_justfloat_module_") as tmp:
        tmp = Path(tmp)
        (tmp / "stm32g4xx_hal.h").write_text(HAL, encoding="utf-8")
        (tmp / "arm_math.h").write_text(CMSIS, encoding="utf-8")
        harness = tmp / "test.c"
        harness.write_text(HARNESS, encoding="utf-8")
        exe = tmp / "waveform.exe"
        command = [gcc, "-std=c11", "-Wall", "-Wextra", "-Werror", "-O0",
                   "-I", str(tmp), "-I", str(ROOT),
                   str(ROOT / "justfloat.c"), str(harness), "-lm", "-o", str(exe)]
        subprocess.run(command, check=True, capture_output=True, text=True)
        completed = subprocess.run([str(exe)], check=True, capture_output=True, text=True)
        print(completed.stdout, end="")
        result["differential_samples"] = 7500
        result["output"] = completed.stdout.splitlines()
    result["passed"] = True
    output = ROOT / "Tests/results/justfloat_module_verification.json"
    output.parent.mkdir(exist_ok=True)
    output.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(f"Results: {output}")


if __name__ == "__main__":
    main()
