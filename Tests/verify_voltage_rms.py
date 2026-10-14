"""Run the actual full-rate RMS function and telemetry publication with host stubs.

Known signals check signed samples, DC, harmonics, window/mode boundaries and
the /10 decimation trap. This is numeric verification, not ADC/hardware proof.
"""
from pathlib import Path
import re
import shutil
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
source = (ROOT / "Core/Src/main.c").read_text(encoding="utf-8-sig")


def extract_function(name):
    start = source.find(f"static void {name}(")
    if start < 0:
        raise SystemExit(f"FAIL: missing full-rate function {name}")
    brace = source.index("{", start)
    depth, end = 1, brace + 1
    while depth:
        depth += (source[end] == "{") - (source[end] == "}")
        end += 1
    return source[start:end]


function = extract_function("VOFA_AccumulateVoltageRms")
mean_function = extract_function("VOFA_AccumulateMean")
count = re.search(r"^#define VOFA_MEAN_SAMPLE_COUNT\s+\S+", source, re.M).group()
callback = source[source.index("void HAL_ADC_ConvCpltCallback("):
                  source.index("void HAL_ADC_ErrorCallback(")]
assert callback.count("VOFA_AccumulateVoltageRms(closed_loop);") == 1
assert callback.index("VOFA_AccumulateVoltageRms(closed_loop);") < callback.index("if (VOFA_MEAN_OUTPUT_ENABLE")
assert "vofa_ch8 = output_voltage_rms_v;" in callback
assert "output_voltage_rms_window_samples =" in source
assert "spwm_update_hz / SPWM_OUTPUT_FREQUENCY_HZ" in source

harness = r'''
#include <stdint.h>
#include <assert.h>
#include <math.h>
#include <float.h>
#include <stdio.h>
#define PI 3.14159265358979323846
__COUNT__
static float true_voltage, true_current, true_aux_current, dc_bus_voltage_v;
static float output_voltage_rms_v, output_voltage_mean_v, output_voltage_ac_rms_v;
static float voltage_rms_square_sum, voltage_rms_sum, voltage_rms_mean_scale;
static uint32_t voltage_rms_count, output_voltage_rms_window_samples;
static uint32_t output_voltage_rms_window_count;
static uint8_t voltage_rms_mode, output_voltage_rms_valid;
static float vofa_ch1,vofa_ch2,vofa_ch3,vofa_ch4,vofa_ch5,vofa_ch6,vofa_ch7,vofa_ch8;
static float vofa_mean_sum[6];
static uint32_t vofa_mean_count,vofa_mean_window_count,vofa_overwrite_count;
static uint8_t vofa_mean_mode,vofa_frame_ready;
static struct {float reference_v,current_reference_a,resonant_r,qpr_unlimited_v;} qpr_control;
static int arm_sqrt_f32(float value, float *out) { *out=sqrtf(value); return 0; }
__FUNCTION__
__MEAN_FUNCTION__
static void near(float actual, double expected) { assert(fabs(actual-expected)<0.00015); }
static void configure(unsigned count) {
    output_voltage_rms_window_samples=count;
    voltage_rms_mean_scale=1.0f/count;
    voltage_rms_count=0;voltage_rms_sum=0;voltage_rms_square_sum=0;
    output_voltage_rms_valid=0;voltage_rms_mode=0;
}
int main(void) {
    configure(400);
    for(unsigned n=0;n<400;n++) {
        true_voltage=(float)(24*sqrt(2.0)*sin(2*PI*n/400));
        VOFA_AccumulateVoltageRms(0);
        if(n<399) assert(!output_voltage_rms_valid);
    }
    assert(output_voltage_rms_valid && output_voltage_rms_window_count==1);
    near(output_voltage_rms_v,24);near(output_voltage_ac_rms_v,24);near(output_voltage_mean_v,0);
    for(unsigned n=0;n<173;n++) { true_voltage=100;VOFA_AccumulateVoltageRms(0); }
    for(unsigned n=0;n<400;n++) {
        double p=2*PI*n/400;
        true_voltage=(float)(2+24*sqrt(2.0)*sin(p)+3*sqrt(2.0)*sin(3*p));
        VOFA_AccumulateVoltageRms(1);
        if(n<399) assert(!output_voltage_rms_valid && output_voltage_rms_v==0);
    }
    near(output_voltage_rms_v,sqrt(4+576+9));
    near(output_voltage_ac_rms_v,sqrt(576+9));near(output_voltage_mean_v,2);
    /* A /10 waveform sampler would see only 9 V and miss the -1 V samples. */
    for(unsigned n=0;n<400;n++) {
        true_voltage=(n%10==0)?9.0f:-1.0f;VOFA_AccumulateVoltageRms(1);
    }
    near(output_voltage_rms_v,3);near(output_voltage_mean_v,0);
    for(unsigned n=0;n<400;n++) { true_voltage=-5;VOFA_AccumulateVoltageRms(1); }
    near(output_voltage_rms_v,5);near(output_voltage_ac_rms_v,0);near(output_voltage_mean_v,-5);
    configure(320); /* Derived window also works if control rate changes to 16 kHz. */
    for(unsigned n=0;n<320;n++) {
        true_voltage=(float)(25*sqrt(2.0)*cos(2*PI*n/320));VOFA_AccumulateVoltageRms(0);
    }
    near(output_voltage_rms_v,25);
    /* Mean telemetry keeps CH8 as RMS instead of averaging signed voltage. */
    for(unsigned n=0;n<400;n++) VOFA_AccumulateMean(0);
    near(vofa_ch8,25);
    puts("PASS full-rate RMS: 24/25 V sine, DC, harmonics, /10 aliasing, mode reset, dynamic window and CH8 mean-mode publication");
}
'''.replace("__COUNT__", count).replace("__FUNCTION__", function).replace("__MEAN_FUNCTION__", mean_function)

compiler = shutil.which("gcc") or "D:/tools/mingw64/bin/gcc.exe"
with tempfile.TemporaryDirectory(prefix="dcac_voltage_rms_") as directory:
    path = Path(directory) / "test.c"
    exe = Path(directory) / "test.exe"
    path.write_text(harness, encoding="utf-8")
    subprocess.run([compiler, "-std=c11", "-Wall", "-Wextra", "-Werror", str(path), "-o", str(exe)], check=True)
    subprocess.run([str(exe)], check=True)
