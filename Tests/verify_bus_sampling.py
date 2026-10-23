"""Run the real ADC callback and QPR C with DMA/register stubs; no hardware timing proof."""
from pathlib import Path
import subprocess, tempfile

root = Path(__file__).resolve().parents[1]
source = (root/'Core/Src/main.c').read_text(encoding='utf-8-sig')
def function(name):
    start = source.index(name)
    brace = source.index('{', start)
    end, depth = brace+1, 1
    while depth:
        depth += (source[end] == '{') - (source[end] == '}')
        end += 1
    return source[start:end]
defines = source.split('/* USER CODE BEGIN PD */')[1].split('/* USER CODE END PD */')[0]
variables = source.split('/* USER CODE BEGIN PV */')[1].split('/* USER CODE END PV */')[0]
harness = r'''
#include <stdint.h>
#include <assert.h>
#include <stdio.h>
#include <math.h>
#include "qpr_control.h"
#define PI 3.14159265359f
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
#define HAL_OK 0
typedef struct {unsigned Instance;} ADC_HandleTypeDef;
static struct {struct {uint32_t ODISR;} sCommonRegs;} registers;
#define HRTIM1 (&registers)
static int hhrtim1;
static unsigned errors, slave_calls, starts;
__DEFINES__
__VARIABLES__
static void Error_Handler(void){errors++;registers.sCommonRegs.ODISR=255;}
static void Slave_Stop(uint32_t fault){slave_fault_flags |= fault;slave_enabled=slave_enable_request=0;}
static void Slave_ControlUpdate(void){assert(adc_sample_count==adc4_sample_count);slave_calls++;}
static void HRTIM_SetModulation(float m){modulation_command=m;}
static int HAL_HRTIM_SoftwareUpdate(int *h,uint32_t mask){assert(h==&hhrtim1 && mask==3);return HAL_OK;}
static int HAL_HRTIM_WaveformOutputStart(int *h,uint32_t mask){assert(h==&hhrtim1 && mask==15);starts++;return HAL_OK;}
static float arm_sin_f32(float x){return sinf(x);}
static int arm_sqrt_f32(float value,float *out){*out=sqrtf(value);return 0;}
__RMS__
__MEAN__
__DECODE__
__CALLBACK__
__ERROR_CALLBACK__
static ADC_HandleTypeDef a1={ADC1}, a3={ADC3};
static void reset(void){
    assert(QPR_ControlInit(&qpr_control,20000));
    assert(QPR_ControlInit(&slave_qpr_control,20000));
    adc12_ready=adc34_ready=dc_bus_ready=0;
    adc_sample_count=adc4_sample_count=adc_pair_error_count=0;
    errors=slave_calls=starts=0;slave_fault_flags=0;
    adc_sample_div_count=0;vofa_frame_ready=0;
    voltage_closed_loop_enable=0;voltage_reference_peak_v=24;
    spwm_update_count=0;spwm_phase_rad=0.7f;phase_step_rad=0.015707963f;
    vref_voltage=3;dc_bus_voltage_gain=24;
    output_voltage_rms_window_samples=400;voltage_rms_mean_scale=1.0f/400;
}
static void pair(uint16_t bus, uint16_t current, int reverse){
    adc12_word=2048U | (2048U<<16);
    adc34_word=(uint32_t)bus | ((uint32_t)current<<16);
    unsigned before=spwm_update_count;
    HAL_ADC_ConvCpltCallback(reverse?&a3:&a1);
    assert(spwm_update_count==before);
    HAL_ADC_ConvCpltCallback(reverse?&a1:&a3);
    assert(spwm_update_count==before+1);
}
int main(void){
    reset();
    assert(ADC_DiffRawToSigned(2048)==0);
    assert(ADC_DiffRawToSigned(0)==-2048);
    assert(ADC_DiffRawToSigned(4095)==2047);
    for(unsigned n=0;n<20;n++){
        /* Alternate order AND bus level each cycle: a previous-cycle bus fails this oracle. */
        uint16_t raw=(n%2)?3000:3413;
        float bus=(raw-2048-DC_BUS_ZERO_CODE)*3.0f/2048.0f*24.0f;
        QPR_Control expected=qpr_control;
        float modulation=QPR_ControlOpenLoopStep(&expected,sinf(spwm_phase_rad),24,bus);
        pair(raw,2300,n%2);
        assert(fabsf(dc_bus_voltage_v-bus)<1e-5f && slave_dc_bus_voltage_v==dc_bus_voltage_v);
        assert(fabsf(modulation_command-modulation)<1e-6f);
        assert(adc_aux_current_raw==2300 && adc_bus_voltage_raw==raw);
    }
    assert(errors==0 && starts==1 && slave_calls==20);
    assert(vofa_frame_ready && vofa_ch7==dc_bus_voltage_v && vofa_ch6==true_aux_current);
    assert(adc_sample_count==20 && adc4_sample_count==20);
    pair(2048+DC_BUS_ZERO_CODE,2048,1);
    assert(dc_bus_voltage_v==0 && dc_bus_ready==0 && modulation_command==0);
    assert(registers.sCommonRegs.ODISR==255 && qpr_control.fault_flags==0);
    pair(3413,2048,0);
    assert(starts==2 && dc_bus_ready && qpr_control.reference_peak_v<0.01f);
    voltage_closed_loop_enable=1;
    QPR_Control expected=qpr_control;
    float bus=(3413-2048-DC_BUS_ZERO_CODE)*3.0f/2048.0f*24;
    float measured_v=(VOLTAGE_ZERO_CODE/2048.0f)*3.0f*voltage_gain;
    float measured_i=(-CURRENT_ZERO_CODE/2048.0f)*3.0f*cksr6_current_gain;
    float m=QPR_ControlStep(&expected,sinf(spwm_phase_rad),24,measured_v,measured_i,bus);
    pair(3413,2048,1);
    assert(fabsf(modulation_command-m)<1e-6f);
    reset();HAL_ADC_ConvCpltCallback(&a1);HAL_ADC_ConvCpltCallback(&a1);
    assert(errors==1 && adc_pair_error_count==1 && spwm_update_count==0);
    reset();HAL_ADC_ConvCpltCallback(&a3);HAL_ADC_ConvCpltCallback(&a3);
    assert(errors==1 && adc_pair_error_count==1 && spwm_update_count==0);
    reset();HAL_ADC_ErrorCallback(&a3);assert(errors==1 && registers.sCommonRegs.ODISR==255);
    puts("PASS: offset-binary decode, both callback orders, same-cycle varying bus feedforward, slave bus, CH6/7, zero-bus inhibit/restart, closed loop, missing pair and ADC error shutdown");
}
'''
for marker, code in {
    '__DEFINES__':defines, '__VARIABLES__':variables,
    '__RMS__':function('static void VOFA_AccumulateVoltageRms('),
    '__MEAN__':function('static void VOFA_AccumulateMean('),
    '__DECODE__':function('static int16_t ADC_DiffRawToSigned('),
    '__CALLBACK__':function('void HAL_ADC_ConvCpltCallback('),
    '__ERROR_CALLBACK__':function('void HAL_ADC_ErrorCallback('),
}.items(): harness=harness.replace(marker,code)
with tempfile.TemporaryDirectory(prefix='bus_sampling_') as d:
    path=Path(d)/'test.c';exe=Path(d)/'test.exe'
    path.write_text(harness,encoding='utf-8')
    subprocess.run(['D:/tools/mingw64/bin/gcc.exe','-std=c11','-Wall','-Wextra','-Werror',
                    '-Wno-unused-variable','-I'+str(root/'Core/Inc'),str(path),
                    str(root/'Core/Src/qpr_control.c'),str(root/'Core/Src/PR.c'),'-o',str(exe)],check=True)
    subprocess.run([str(exe)],check=True)
