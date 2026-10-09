"""Execute real C; compare both QPR frequency responses with analytic Tustin response."""
from pathlib import Path
import subprocess
import tempfile

root = Path(__file__).resolve().parents[1]
harness = r'''
#include "qpr_control.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#define PI 3.14159265358979323846
static void response(double f, int outer) {
    QPR_Control c;
    double ys=0, yc=0, omega=2*PI*f, ts=1.0/20000;
    double kp=outer?QPR_VOLTAGE_KP_A_PER_V:QPR_KP;
    double kr=outer?QPR_VOLTAGE_KR_A_PER_V:QPR_KR;
    double wc=outer?QPR_VOLTAGE_WC_RAD_S:QPR_WC_RAD_S;
    double wd=2/ts*tan(omega*ts/2), w0=2*PI*QPR_FREQUENCY_HZ;
    double a=w0*w0-wd*wd, b=2*wc*wd;
    double re=kp+kr*b*b/(a*a+b*b), im=kr*b*a/(a*a+b*b);
    assert(QPR_ControlInit(&c,20000));
    for (unsigned n=0;n<120000;n++) {
        float e=(float)(0.2*sin(omega*n*ts));
        double y;
        if (outer) {
            QPR_ControlStep(&c,0,0,-e,0,48);
            y=c.current_reference_a;
        } else {
            QPR_CurrentStep(&c,e,0,0,48);
            y=c.qpr_unlimited_v;
        }
        assert(c.fault_flags==0);
        if(n>=100000) {ys+=y*sin(omega*n*ts);yc+=y*cos(omega*n*ts);}
    }
    ys/=2000;yc/=2000;
    if(fabs(ys-re)>=0.0003 || fabs(yc-im)>=0.0003) {
        fprintf(stderr,"response mismatch outer=%d f=%.0f actual %.8f %.8f expected %.8f %.8f\n",outer,f,ys,yc,re,im);
        assert(0);
    }
    assert(c.qpr_saturation_count==0 && c.voltage_qpr_saturation_count==0);
    printf("PASS %s %.0f Hz: gain components %.6f %.6f\n",outer?"voltage":"current",f,ys,yc);
}
int main(void) {
    QPR_Control c;
    for(int outer=0;outer<2;outer++) for(int f=49;f<=51;f++) response(f,outer);
    assert(!QPR_ControlInit(&c,100));
    assert(!QPR_ControlInit(&c,NAN));
    assert(QPR_ControlInit(&c,20000));
    for(unsigned n=0;n<20000;n++) {
        QPR_ControlStep(&c,0,0,-1000,0,48);
        assert(c.current_reference_a==QPR_CURRENT_REFERENCE_LIMIT_A);
        assert(fabsf(c.modulation)<=QPR_MODULATION_LIMIT);
        assert(c.voltage_resonant_r==0 && c.resonant_r==0);
    }
    assert(c.voltage_qpr_saturation_count==20000 && c.qpr_saturation_count==20000);
    QPR_ControlReset(&c);
    QPR_ControlStep(&c,0,0,0,0,48);
    assert(c.modulation==0 && c.voltage_qpr_saturation_count==0);
    QPR_ControlStep(&c,0,0,-0.2f,0,48);
    assert(c.voltage_resonant_r!=0 && c.resonant_r!=0);
    QPR_ControlOpenLoopStep(&c,0.5f,10,48);
    assert(c.voltage_resonant_r==0 && c.voltage_resonant_q==0);
    assert(c.resonant_r==0 && c.resonant_q==0 && c.current_reference_a==0);
    assert(fabsf(c.modulation-c.reference_v/48)<1e-7f);
    QPR_ControlStep(&c,0,0,NAN,0,48);
    assert(c.fault_flags==QPR_FAULT_INPUT && c.modulation==0);
    assert(QPR_ControlStep(&c,0,0,0,0,48)==0);
    QPR_ControlReset(&c);
    assert(c.fault_flags==0);
    puts("PASS limits, antiwindup, reset, independent states, open-loop clearing, fault latch");
}
'''
with tempfile.TemporaryDirectory(prefix='dual_qpr_') as directory:
    source = Path(directory) / 'test.c'
    exe = Path(directory) / 'test.exe'
    source.write_text(harness, encoding='utf-8')
    subprocess.run(['D:/tools/mingw64/bin/gcc.exe', '-std=c11', '-Wall', '-Wextra', '-Werror',
                    '-I'+str(root/'Core/Inc'), str(source), str(root/'Core/Src/qpr_control.c'),
                    '-o', str(exe)], check=True)
    subprocess.run([str(exe)], check=True)
