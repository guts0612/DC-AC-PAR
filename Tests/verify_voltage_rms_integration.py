"""Run the actual ADC callback, RMS, mailbox and sender in both VOFA modes.

Extract production main.c functions and link the real qpr_control.c; only the
ADC/register/UART/CMSIS boundaries are mocked. This verifies callback routing
and numeric telemetry, not physical ADC accuracy, interrupt latency or UART
throughput. Requires host GCC (PATH or the project's MinGW installation).
"""
from pathlib import Path
import re
import shutil
import subprocess
import tempfile


ROOT = Path(__file__).resolve().parents[1]
SOURCE = (ROOT / "Core/Src/main.c").read_text(encoding="utf-8-sig")


def function(signature):
    start = SOURCE.index(signature)
    brace = SOURCE.index("{", start)
    end, depth = brace + 1, 1
    while depth:
        depth += (SOURCE[end] == "{") - (SOURCE[end] == "}")
        end += 1
    return SOURCE[start:end]


def user_block(name):
    return SOURCE.split(f"/* USER CODE BEGIN {name} */", 1)[1].split(
        f"/* USER CODE END {name} */", 1
    )[0]


HARNESS = r'''
#include <stdint.h>
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "qpr_control.h"
#define PI 3.14159265358979323846f
#define ADC1 1U
#define ADC2 2U
#define ADC3 3U
#define ADC4 4U
#define HRTIM_OUTPUT_TA1 1U
#define HRTIM_OUTPUT_TA2 2U
#define HRTIM_OUTPUT_TB1 4U
#define HRTIM_OUTPUT_TB2 8U
#define HRTIM_OUTPUT_TC1 16U
#define HRTIM_OUTPUT_TC2 32U
#define HRTIM_OUTPUT_TD1 64U
#define HRTIM_OUTPUT_TD2 128U
#define HRTIM_TIMERUPDATE_A 1U
#define HRTIM_TIMERUPDATE_B 2U
typedef struct { unsigned Instance; } ADC_HandleTypeDef;
typedef enum { HAL_OK=0, HAL_ERROR=1, HAL_BUSY=2, HAL_TIMEOUT=3 } HAL_StatusTypeDef;
static struct { struct { uint32_t ODISR; } sCommonRegs; } registers;
#define HRTIM1 (&registers)
static int hhrtim1, huart4;
static unsigned errors, slave_calls, starts;
static unsigned irq_mask, expected_irq_mask, tx_calls, inject_samples;
static uint8_t captured[36];
__DEFINES__
__VARIABLES__
static void on_uart(void);
static uint32_t __get_PRIMASK(void) { return irq_mask; }
static void __disable_irq(void) { irq_mask=1; }
static void __set_PRIMASK(uint32_t p) { irq_mask=p; }
static void Error_Handler(void) { errors++; registers.sCommonRegs.ODISR=255; }
static void Slave_Stop(uint32_t fault) {
    slave_fault_flags |= fault; slave_enabled=slave_enable_request=0;
}
static void Slave_ControlUpdate(void) {
    assert(adc_sample_count==adc4_sample_count); slave_calls++;
}
static void HRTIM_SetModulation(float m) { modulation_command=m; }
static int HAL_HRTIM_SoftwareUpdate(int *h, uint32_t mask) {
    assert(h==&hhrtim1 && mask==3); return HAL_OK;
}
static int HAL_HRTIM_WaveformOutputStart(int *h, uint32_t mask) {
    assert(h==&hhrtim1 && mask==15); starts++; return HAL_OK;
}
static float arm_sin_f32(float x) { return sinf(x); }
static int arm_sqrt_f32(float value, float *out) { *out=sqrtf(value); return 0; }
static HAL_StatusTypeDef HAL_UART_Transmit(int *port, uint8_t *data,
                                         uint16_t count, uint32_t timeout) {
    assert(port==&huart4 && count==36 && timeout==5);
    assert(irq_mask==expected_irq_mask);
    memcpy(captured,data,count); tx_calls++; on_uart(); return HAL_OK;
}
__RMS__
__MEAN__
__SENDER__
__PROCESS__
__DECODE__
__CALLBACK__
static ADC_HandleTypeDef a1={ADC1}, a3={ADC3};
static void reset(void) {
    assert(QPR_ControlInit(&qpr_control,20000));
    assert(QPR_ControlInit(&slave_qpr_control,20000));
    assert(SPWM_OUTPUT_FREQUENCY_HZ==50 && VOFA_MEAN_SAMPLE_COUNT==400);
    adc12_ready=adc34_ready=dc_bus_ready=0;
    adc_sample_count=adc4_sample_count=adc_pair_error_count=0;
    errors=slave_calls=starts=0; slave_fault_flags=0;
    adc_sample_div_count=0; vofa_frame_ready=0;
    voltage_closed_loop_enable=0; voltage_reference_peak_v=QPR_DEFAULT_REFERENCE_PEAK_V;
    spwm_update_count=0; spwm_phase_rad=0; phase_step_rad=2*PI/400;
    vref_voltage=3; voltage_gain=12.8f; dc_bus_voltage_gain=24;
    voltage_rms_count=0; voltage_rms_sum=voltage_rms_square_sum=0; voltage_rms_mode=0;
    output_voltage_rms_valid=0; output_voltage_rms_window_count=0;
    output_voltage_rms_v=output_voltage_mean_v=output_voltage_ac_rms_v=0;
    /* HRTIM is mocked: this test supplies its 20 kHz / 50 Hz window. */
    output_voltage_rms_window_samples=400; voltage_rms_mean_scale=1.0f/400;
    memset(vofa_mean_sum,0,sizeof(vofa_mean_sum));
    vofa_mean_count=vofa_mean_window_count=0; vofa_mean_mode=0;
    vofa_sent_count=vofa_overwrite_count=vofa_tx_error_count=0;
    irq_mask=expected_irq_mask=tx_calls=inject_samples=0;
}
static void pair(float u, int reverse) {
    int raw=(int)lroundf(2048+VOLTAGE_ZERO_CODE-u*2048/(3*12.8f));
    assert(raw>=0 && raw<=4095);
    adc12_word=(uint32_t)raw | ((uint32_t)(2048+CURRENT_ZERO_CODE)<<16);
    adc34_word=3413U | ((uint32_t)(2048+AUX_CURRENT_ZERO_CODE)<<16);
    unsigned before=spwm_update_count, before_count=voltage_rms_count;
    unsigned before_windows=output_voltage_rms_window_count;
    HAL_ADC_ConvCpltCallback(reverse?&a3:&a1);
    assert(spwm_update_count==before && voltage_rms_count==before_count);
    assert(output_voltage_rms_window_count==before_windows);
    HAL_ADC_ConvCpltCallback(reverse?&a1:&a3);
    assert(spwm_update_count==before+1);
}
static void on_uart(void) {
    unsigned count=inject_samples; inject_samples=0;
    /* Publish a later mailbox frame while the previous snapshot is sent. */
    for(unsigned n=0;n<count;n++) pair(7,n%2);
}
static float captured_channel(unsigned n) {
    float value; memcpy(&value,captured+4*n,4); return value;
}
static void verify_mode_switch(void) {
    reset();
    for(unsigned n=0;n<400;n++) pair(15,n%2);
    for(unsigned n=0;n<173;n++) pair(15,n%2);
    voltage_closed_loop_enable=1; pair(-5,1);
    assert(voltage_rms_count==1 && !output_voltage_rms_valid && output_voltage_rms_v==0);
    if(VOFA_MEAN_OUTPUT_ENABLE) assert(!vofa_frame_ready && vofa_mean_count==1);
    for(unsigned n=1;n<400;n++) pair(-5,n%2);
    assert(output_voltage_rms_valid && output_voltage_rms_window_count==2);
    assert(fabsf(output_voltage_rms_v-5)<0.02f);
    if(VOFA_MEAN_OUTPUT_ENABLE) {
        assert(vofa_mean_window_count==2 && vofa_frame_ready);
    } else {
        /* Mode changes need not align the RMS boundary with the /10 publisher. */
        unsigned until_publish=VOFA_SAMPLE_DIV-adc_sample_div_count;
        for(unsigned n=0;n<until_publish;n++) pair(-5,n%2);
    }
    assert(vofa_ch8==output_voltage_rms_v && errors==0);
}
int main(void) {
    reset(); double square_sum=0, mean_sums[6]={0};
    for(unsigned n=0;n<400;n++) {
        pair((float)(24*sqrt(2.0)*sin(2*3.141592653589793*n/400)),n%2);
        square_sum+=(double)true_voltage*true_voltage;
        mean_sums[0]+=qpr_control.reference_v-true_voltage;
        mean_sums[1]+=qpr_control.current_reference_a;
        mean_sums[2]+=true_current;
        mean_sums[3]+=qpr_control.resonant_r;
        mean_sums[4]+=qpr_control.qpr_unlimited_v;
        mean_sums[5]+=dc_bus_voltage_v;
        assert(voltage_rms_count==(n+1)%400);
        assert(output_voltage_rms_valid==(n==399));
        if(n<399) {
            assert(vofa_ch8==0);
            if(VOFA_MEAN_OUTPUT_ENABLE) assert(!vofa_frame_ready);
        }
    }
    float expected=(float)sqrt(square_sum/400);
    assert(fabsf(output_voltage_rms_v-expected)<0.0001f && fabsf(expected-24)<0.003f);
    assert(output_voltage_rms_window_count==1 && vofa_ch8==output_voltage_rms_v);
    if(VOFA_MEAN_OUTPUT_ENABLE) {
        float actual[6]={vofa_ch1,vofa_ch2,vofa_ch3,vofa_ch4,vofa_ch5,vofa_ch6};
        for(unsigned n=0;n<6;n++) assert(fabs(actual[n]-mean_sums[n]/400)<0.0001);
        assert(vofa_mean_window_count==1 && vofa_ch7==true_aux_current);
    } else {
        assert(vofa_ch1==voltage_reference_v && vofa_ch2==true_voltage);
        assert(vofa_ch3==qpr_control.current_reference_a && vofa_ch4==true_current);
        assert(vofa_ch5==qpr_control.qpr_correction_v);
        assert(vofa_ch6==true_aux_current && vofa_ch7==dc_bus_voltage_v);
    }
    float snapshot[8]={vofa_ch1,vofa_ch2,vofa_ch3,vofa_ch4,vofa_ch5,vofa_ch6,vofa_ch7,vofa_ch8};
    inject_samples=VOFA_MEAN_OUTPUT_ENABLE?VOFA_MEAN_SAMPLE_COUNT:VOFA_SAMPLE_DIV;
    VOFA_Process();
    assert(tx_calls==1 && vofa_sent_count==1 && vofa_frame_ready && irq_mask==0);
    for(unsigned n=0;n<8;n++) assert(captured_channel(n)==snapshot[n]);
    assert(memcmp(captured+32,"\x00\x00\x80\x7f",4)==0);
    /* Mean-mode CH2 is the current reference and remains zero in open loop. */
    if(VOFA_MEAN_OUTPUT_ENABLE) assert(vofa_ch8!=snapshot[7]);
    else assert(vofa_ch2!=snapshot[1]);
    VOFA_Process(); assert(tx_calls==2 && !vofa_frame_ready);
    VOFA_Process(); assert(tx_calls==2); /* Empty mailbox sends nothing. */
    vofa_frame_ready=1; irq_mask=expected_irq_mask=1;
    VOFA_Process(); assert(irq_mask==1); irq_mask=expected_irq_mask=0;
    assert(errors==0 && adc_sample_count==adc4_sample_count && slave_calls==adc_sample_count);
    verify_mode_switch();
    reset(); HAL_ADC_ConvCpltCallback(&a1); assert(voltage_rms_count==0);
    HAL_ADC_ConvCpltCallback(&a1);
    assert(errors==1 && adc_pair_error_count==1 && voltage_rms_count==0 && !output_voltage_rms_valid);
    reset(); HAL_ADC_ConvCpltCallback(&a3); HAL_ADC_ConvCpltCallback(&a3);
    assert(errors==1 && adc_pair_error_count==1 && voltage_rms_count==0 && !output_voltage_rms_valid);
    printf("PASS actual ADC/VOFA integration mean_mode=%u quantized_24V_rms=%.7f V "
           "frame_bytes=36: callback pairing, CH1-8, mode reset, UART mailbox snapshot, PRIMASK, duplicate ADC\n",
           (unsigned)VOFA_MEAN_OUTPUT_ENABLE,expected);
    return 0;
}
'''


def main():
    compiler = shutil.which("gcc")
    if not compiler:
        fallback = Path("D:/tools/mingw64/bin/gcc.exe")
        if not fallback.is_file():
            raise SystemExit("Host GCC is required")
        compiler = str(fallback)
    replacements = {
        "__VARIABLES__": user_block("PV"),
        "__RMS__": function("static void VOFA_AccumulateVoltageRms("),
        "__MEAN__": function("static void VOFA_AccumulateMean("),
        "__SENDER__": function("static void VOFA_SendJustFloat("),
        "__PROCESS__": function("static void VOFA_Process("),
        "__DECODE__": function("static int16_t ADC_DiffRawToSigned("),
        "__CALLBACK__": function("void HAL_ADC_ConvCpltCallback("),
    }
    with tempfile.TemporaryDirectory(prefix="dcac_rms_integration_") as directory:
        for mode in (0, 1):
            defines, count = re.subn(
                r"(?m)^(#define\s+VOFA_MEAN_OUTPUT_ENABLE\s+)\S+",
                lambda match: match.group(1) + f"{mode}U",
                user_block("PD"),
            )
            if count != 1:
                raise SystemExit("Expected exactly one VOFA_MEAN_OUTPUT_ENABLE definition")
            harness = HARNESS.replace("__DEFINES__", defines)
            for marker, code in replacements.items():
                harness = harness.replace(marker, code)
            path = Path(directory) / f"mean_{mode}.c"
            executable = Path(directory) / f"mean_{mode}.exe"
            path.write_text(harness, encoding="utf-8")
            subprocess.run(
                [compiler, "-std=c11", "-Wall", "-Wextra", "-Werror",
                 "-Wno-unused-variable", "-I" + str(ROOT / "Core/Inc"),
                 str(path), str(ROOT / "Core/Src/qpr_control.c"),
                 str(ROOT / "Core/Src/PR.c"), "-o", str(executable)],
                check=True,
            )
            subprocess.run([str(executable)], check=True)


if __name__ == "__main__":
    main()
