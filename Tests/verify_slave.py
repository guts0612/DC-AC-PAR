"""Exercise actual slave helpers/PR using mocked registers, not hardware timing."""
from pathlib import Path
import subprocess, tempfile
root = Path(__file__).resolve().parents[1]
s = (root/'Core/Src/main.c').read_text(encoding='utf-8-sig')
helpers = s[s.index('static void HRTIM_SetSlaveModulation('):s.index('static void HRTIM_SPWM_Init(')]
globals_ = s[s.index('QPR_Control slave_qpr_control;'):s.index('volatile float modulation_command;')]
key = s[s.index('    if (key_number == 1U)'):s.index('#if VOFA_OUTPUT_ENABLE', s.index('    if (key_number == 1U)'))]
harness = r'''
#include <stdint.h>
#include <stdio.h>
#include <assert.h>
#include <math.h>
#include "qpr_control.h"
#define HRTIM_COMPARE_MARGIN_TICKS 6U
#define SPWM_MODULATION_LIMIT QPR_MODULATION_LIMIT
#define HRTIM_OUTPUT_TC1 16U
#define HRTIM_OUTPUT_TC2 32U
#define HRTIM_OUTPUT_TD1 64U
#define HRTIM_OUTPUT_TD2 128U
#define HRTIM_TIMERINDEX_TIMER_C 2U
#define HRTIM_TIMERINDEX_TIMER_D 3U
#define HRTIM_COMPAREUNIT_1 0U
#define HRTIM_TIMERUPDATE_C 4U
#define HRTIM_TIMERUPDATE_D 8U
#define HRTIM_OUTPUTLEVEL_ACTIVE 1U
#define HAL_OK 0
static struct { struct {uint32_t ODISR,OENR;} sCommonRegs; } registers;
#define HRTIM1 (&registers)
static int hhrtim1;
static uint16_t Period=4250;
static uint32_t compares[4]={111,222,0,0},adc_sample_count,adc4_sample_count,irq_mask;
static float true_current,true_aux_current,true_voltage,voltage_reference_v;
static uint8_t voltage_closed_loop_enable;
static uint32_t __get_PRIMASK(void){return irq_mask;}
static void __disable_irq(void){irq_mask=1;}
static void __set_PRIMASK(uint32_t m){irq_mask=m;}
#define __HAL_HRTIM_SETCOMPARE(h,t,u,v) (compares[t]=(v))
static int HAL_HRTIM_SoftwareUpdate(int *h,uint32_t mask){assert(h==&hhrtim1 && mask==12);return HAL_OK;}
static int HAL_HRTIM_WaveformSetOutputLevel(int *h,uint32_t t,uint32_t o,uint32_t l){assert(h==&hhrtim1 && t>=2 && o>=16 && l==1);return HAL_OK;}
__GLOBALS__
__HELPERS__
static void key_press(uint8_t key_number){__KEY__}
static void reset_test(void){
 assert(QPR_ControlInit(&slave_qpr_control,20000));
 slave_fault_flags=0;slave_enable_request=0;slave_enabled=0;
 slave_current_ratio=1;slave_dc_bus_voltage_v=48;
 adc_sample_count=adc4_sample_count=0;
 true_current=1;true_aux_current=0;true_voltage=voltage_reference_v=0;
 voltage_closed_loop_enable=0;registers.sCommonRegs.OENR=0;
}
int main(void){
 reset_test();key_press(2);assert(!slave_enable_request);Slave_ControlUpdate();assert(!slave_enabled);
 key_press(1);assert(voltage_closed_loop_enable);key_press(2);assert(slave_enable_request);
 voltage_reference_v=10;Slave_ControlUpdate();assert(!slave_enabled);
 voltage_reference_v=0;Slave_ControlUpdate();assert(slave_enabled && registers.sCommonRegs.OENR==SLAVE_OUTPUTS);
 assert(slave_ratio_applied>0 && slave_ratio_applied<0.001f);
 assert(compares[0]==111 && compares[1]==222);
 for(unsigned i=0;i<21000;i++){
  adc_sample_count++;adc4_sample_count++;true_aux_current=slave_qpr_control.current_reference_a;Slave_ControlUpdate();
 }
 assert(slave_enabled && fabsf(slave_ratio_applied-1)<1e-4f);
 /* With sharing established, a master-current step reaches the reference
  * in this call, rather than through a low-pass state. Test both polarities. */
 true_current=2.0f;Slave_ControlUpdate();
 assert(fabsf(slave_qpr_control.current_reference_a-2.0f)<1e-4f);
 true_current=-1.5f;Slave_ControlUpdate();
 assert(fabsf(slave_qpr_control.current_reference_a+1.5f)<1e-4f);
 key_press(2);assert(!slave_enabled && !slave_enable_request);
 key_press(2);Slave_ControlUpdate();assert(slave_enabled);
 key_press(1);assert(!voltage_closed_loop_enable && !slave_enabled && !slave_enable_request);
 reset_test();voltage_closed_loop_enable=1;slave_enable_request=1;adc_sample_count=1;Slave_ControlUpdate();assert(slave_fault_flags==SLAVE_FAULT_TIMING && !slave_enabled);
 reset_test();voltage_closed_loop_enable=1;slave_enable_request=1;true_aux_current=5.1f;Slave_ControlUpdate();assert(slave_fault_flags==SLAVE_FAULT_CURRENT);
 reset_test();voltage_closed_loop_enable=1;slave_enable_request=1;true_aux_current=NAN;Slave_ControlUpdate();assert(slave_fault_flags==SLAVE_FAULT_CONTROL && !slave_enabled);
 reset_test();voltage_closed_loop_enable=1;slave_enable_request=1;slave_current_ratio=3;Slave_ControlUpdate();assert(slave_fault_flags==SLAVE_FAULT_CONTROL);
 assert(compares[0]==111 && compares[1]==222 && irq_mask==0);
 QPR_Control c;assert(QPR_ControlInit(&c,20000));
 assert(fabsf(QPR_CurrentStep(&c,0,0,24,48)-0.5f)<1e-6f);
 assert(c.voltage_error_previous_v==0 && c.voltage_resonant_g>0);
 for(unsigned i=0;i<1000;i++)QPR_CurrentStep(&c,100,0,0,48);
 assert(c.resonant_r==0 && c.modulation<=QPR_MODULATION_LIMIT);
 puts("PASS: startup/off, PB9/open interlock, zero-cross admission, soft sharing, toggle/disconnect, unchanged A/B writes, timing/overcurrent/NaN faults, PR feedforward and antiwindup");
}
'''.replace('__GLOBALS__',globals_).replace('__HELPERS__',helpers).replace('__KEY__',key)
with tempfile.TemporaryDirectory(prefix='slave_test_') as d:
    c=Path(d)/'test.c'; exe=Path(d)/'test.exe'; c.write_text(harness,encoding='utf-8')
    subprocess.run(['D:/tools/mingw64/bin/gcc.exe','-std=c11','-Wall','-Wextra','-Werror','-I'+str(root/'Core/Inc'),str(c),str(root/'Core/Src/qpr_control.c'),'-o',str(exe)],check=True)
    subprocess.run([str(exe)],check=True)
