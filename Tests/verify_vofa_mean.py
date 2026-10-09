"""Host-test the actual full-rate accumulator, including mode transitions."""
from pathlib import Path
import re, subprocess, tempfile
source = (Path(__file__).resolve().parents[1] / 'Core/Src/main.c').read_text(encoding='utf-8-sig')
start = source.index('static void VOFA_AccumulateMean(')
brace = source.index('{', start)
depth, end = 1, brace + 1
while depth:
    depth += (source[end] == '{') - (source[end] == '}')
    end += 1
function = source[start:end]
count = re.search(r'^#define VOFA_MEAN_SAMPLE_COUNT\s+\S+', source, re.M).group()
harness = r"""
#include <stdint.h>
#include <assert.h>
#include <math.h>
#include <stdio.h>
__COUNT__
static struct {float reference_v,current_reference_a,resonant_r,qpr_unlimited_v;} qpr_control;
static float true_voltage,true_current,true_aux_current,vofa_mean_sum[5];
static float vofa_ch1,vofa_ch2,vofa_ch3,vofa_ch4,vofa_ch5,vofa_ch6,vofa_ch7;
static uint32_t vofa_mean_count,vofa_mean_window_count,vofa_overwrite_count;
static uint8_t vofa_mean_mode,vofa_frame_ready;
__FUNCTION__
static void sample(unsigned n, uint8_t mode) {
    /* A zero-mean ripple deliberately aligned with the old /10 decimator. */
    qpr_control.reference_v = (n % 10 == 0) ? 9.0f : -1.0f;
    true_voltage = -0.1f;
    qpr_control.current_reference_a = 2.0f;
    true_current = 0.25f;
    true_aux_current = (float)n * 0.01f;
    qpr_control.resonant_r = -0.5f;
    qpr_control.qpr_unlimited_v = 1.25f;
    VOFA_AccumulateMean(mode);
}
int main(void) {
    for(unsigned n=0;n<399;n++) sample(n,1);
    assert(!vofa_frame_ready && vofa_mean_count==399);
    sample(399,1);
    assert(vofa_frame_ready && vofa_mean_window_count==1 && vofa_mean_count==0);
    assert(fabsf(vofa_ch1-0.1f)<0.00001f);
    assert(fabsf(vofa_ch2-2.0f)<0.00001f);
    assert(fabsf(vofa_ch3-0.25f)<0.00001f);
    assert(fabsf(vofa_ch4+0.5f)<0.00001f);
    assert(fabsf(vofa_ch5-1.25f)<0.00001f);
    assert(fabsf(vofa_ch6-1.75f)<0.00001f);
    assert(fabsf(vofa_ch7-3.99f)<0.00001f); /* CH7 is latest, not averaged. */
    for(unsigned n=0;n<400;n++) sample(n,1);
    assert(vofa_overwrite_count==1 && vofa_mean_window_count==2);
    for(unsigned n=0;n<173;n++) sample(n,1);
    sample(0,0);
    assert(!vofa_frame_ready && vofa_mean_count==1);
    for(unsigned n=1;n<400;n++) sample(n,0);
    assert(vofa_frame_ready && vofa_mean_window_count==3);
    assert(fabsf(vofa_ch1-0.1f)<0.00001f);
    puts("PASS: 400-sample means, all channels, decimation bias rejection, window reset, mode switch, mailbox overwrite");
}
""".replace('__COUNT__',count).replace('__FUNCTION__',function)
with tempfile.TemporaryDirectory(prefix='dcac_mean_') as d:
    c=Path(d)/'test.c';exe=Path(d)/'test.exe';c.write_text(harness,encoding='utf-8')
    subprocess.run(['D:/tools/mingw64/bin/gcc.exe','-std=c11','-Wall','-Wextra','-Werror',str(c),'-o',str(exe)],check=True)
    subprocess.run([str(exe)],check=True)
