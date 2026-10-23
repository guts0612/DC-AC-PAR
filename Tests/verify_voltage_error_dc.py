"""Exercise the actual C controller: DC rejection, 50 Hz response and reset."""
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
HARNESS = r'''
#include "qpr_control.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#define PI 3.14159265358979323846
static void require(int ok, const char *message) {
    if (!ok) { fprintf(stderr,"FAIL %s\n",message);exit(1); }
}
static void response(float bias, float amplitude) {
    QPR_Control c;
    double dc=0,ys=0,yc=0;
    require(QPR_ControlInit(&c,20000),"initialize");
    for(unsigned n=0;n<60000;n++) {
        double p=2*PI*50*n/20000;
        float e=bias+amplitude*(float)sin(p);
        QPR_ControlStep(&c,0,0,-e,0,48);
        require(c.fault_flags==0,"finite controller state");
        require(fabsf(c.voltage_error_v-e)<1e-6f,"preserve raw diagnostic error");
        if(n>=40000) {dc+=c.current_reference_a;ys+=c.current_reference_a*sin(p);yc+=c.current_reference_a*cos(p);}
    }
    dc/=20000;ys/=10000;yc/=10000;
    printf("bias=%+.3f V: iref DC=%+.7f A, fundamental peak=%.7f A\n",bias,dc,hypot(ys,yc));
    require(fabs(dc)<.002,"reject voltage-error DC before virtual impedance");
    if(amplitude>0) {
        double wd=40000*tan(PI*50/20000);
        double expected=amplitude/hypot(QPR_VIRTUAL_R_OHM,QPR_VIRTUAL_L_H*wd);
        require(fabs(hypot(ys,yc)-expected)<.0002,"preserve 50 Hz admittance gain");
    }
    QPR_ControlReset(&c);
    for(unsigned n=0;n<1000;n++) QPR_ControlStep(&c,0,0,0,0,48);
    require(c.current_reference_a==0,"reset clears DC compensation history");
    QPR_ControlStep(&c,0,0,-.1f,0,48);
    QPR_ControlOpenLoopStep(&c,0,0,48);
    QPR_ControlStep(&c,0,0,0,0,48);
    require(c.current_reference_a==0,"open loop clears closed-loop error history");
}
static void changing_bias_and_ramp(void) {
    QPR_Control c;
    require(QPR_ControlInit(&c,20000),"initialize tracker");
    require(c.voltage_error_dc_window_samples==400,"one 50 Hz period at full control rate");
    for(unsigned n=0;n<2000;n++) QPR_ControlStep(&c,0,10,-.05f,0,48);
    require(c.voltage_error_dc_v==0 && c.voltage_error_dc_count==0,
            "amplitude ramp must not enter DC estimate");
    QPR_ControlReset(&c);
    for(unsigned n=0;n<60000;n++) QPR_ControlStep(&c,0,0,-.05f,0,48);
    require(fabsf(c.voltage_error_dc_v-.05f)<.00001f,"track positive bias");
    for(unsigned n=0;n<60000;n++) QPR_ControlStep(&c,0,0,.075f,0,48);
    require(fabsf(c.voltage_error_dc_v+.075f)<.00001f,"follow bias reversal");
    require(fabsf(c.current_reference_a)<.002f,"remove DC after bias reversal");
    require(fabsf(c.voltage_error_v+.075f)<.00001f,"raw error survives compensation");
    require(fabsf(c.voltage_error_compensated_v)<.00001f,"expose compensated error");
    QPR_ControlOpenLoopStep(&c,0,0,48);
    require(c.voltage_error_dc_v==0 && c.voltage_error_dc_count==0,
            "clear estimator on open-loop entry");
    puts("PASS window size, ramp freeze, positive-to-negative bias and observable state");
}
int main(void) {
    response(.05f,0);response(-.075f,0);
    response(0,.3f);response(.05f,.3f);response(-.075f,.3f);
    changing_bias_and_ramp();
    puts("PASS DC rejection, 50 Hz gain, raw telemetry, reset and open-loop clearing");
}
'''
with tempfile.TemporaryDirectory(prefix='voltage_error_dc_') as temp:
    src=Path(temp)/'test.c'
    exe=Path(temp)/'test.exe'
    src.write_text(HARNESS,encoding='utf-8')
    subprocess.run(['D:/tools/mingw64/bin/gcc.exe','-std=c11','-Wall','-Wextra','-Werror',
                    '-I'+str(ROOT/'Core/Inc'),str(src),str(ROOT/'Core/Src/qpr_control.c'),str(ROOT/'Core/Src/PR.c'),
                    '-o',str(exe)],check=True)
    subprocess.run([str(exe)],check=True)
