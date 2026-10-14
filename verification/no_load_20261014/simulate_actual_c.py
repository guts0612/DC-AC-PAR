"""Read-only numerical audit of actual QPR C on an ideal averaged LC plant.

The plant is exact zero-order hold; current is measured before the capacitor.
The capacitor ESR is in series with C, so the sensed load-node voltage differs
from the capacitor state voltage. This is not switching/hardware validation.
"""
from pathlib import Path
import argparse
from collections import deque
import ctypes
import _ctypes
import hashlib
import json
import math
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]
FS = 20000.0
TS = 1.0 / FS
F0 = 50.0
SHIM = r'''
#include "qpr_control.h"
static QPR_Control c;
int audit_init(float fs) { return QPR_ControlInit(&c, fs); }
float audit_step(float sine, float peak, float voltage, float current, float bus) {
    return QPR_ControlStep(&c, sine, peak, voltage, current, bus);
}
float audit_open(float sine, float peak, float bus) {
    return QPR_ControlOpenLoopStep(&c, sine, peak, bus);
}
void audit_observe(float *v) {
    v[0]=c.reference_v; v[1]=c.current_reference_a;
    v[2]=c.qpr_unlimited_v; v[3]=c.qpr_correction_v;
    v[4]=c.voltage_error_dc_v; v[5]=c.voltage_error_compensated_v;
    v[6]=(float)c.qpr_saturation_count; v[7]=(float)c.modulation_saturation_count;
    v[8]=(float)c.fault_flags;
}
'''


def exact_plant(inductance, capacitance, load_ohm, capacitor_esr, sample_s=TS):
    if math.isinf(load_ohm):
        alpha, beta, gamma = 1.0, capacitor_esr, 0.0
    else:
        alpha = load_ohm / (load_ohm + capacitor_esr)
        beta = capacitor_esr * alpha
        gamma = 1.0 / (load_ohm + capacitor_esr)
    # x=(iL,vC), sensed vOut=beta*iL+alpha*vC.
    a, b = -beta / inductance, -alpha / inductance
    c, d = alpha / capacitance, -gamma / capacitance
    sigma = (a + d) / 2.0
    discriminant = ((a - d) / 2.0) ** 2 + b * c
    if discriminant < 0:
        omega = math.sqrt(-discriminant)
        co, si = math.cos(omega * sample_s), math.sin(omega * sample_s) / omega
    elif discriminant > 0:
        omega = math.sqrt(discriminant)
        co, si = math.cosh(omega * sample_s), math.sinh(omega * sample_s) / omega
    else:
        co, si = 1.0, sample_s
    scale = math.exp(sigma * sample_s)
    e11 = scale * (co + si * (a - sigma))
    e12 = scale * si * b
    e21 = scale * si * c
    e22 = scale * (co + si * (d - sigma))
    determinant = a * d - b * c
    r1, r2 = (e11 - 1.0) / inductance, e21 / inductance
    g1 = (d * r1 - b * r2) / determinant
    g2 = (-c * r1 + a * r2) / determinant
    return (e11, e12, e21, e22, g1, g2, alpha, beta)


def run_case(lib, inductance, load, esr, delay_samples, duration=3.0,
             voltage_bias=0.0, current_bias=0.0, retain_trace=False,
             capacitance=2.2e-6, closed_from_s=0.0):
    assert lib.audit_init(FS) == 1
    e11, e12, e21, e22, g1, g2, alpha, beta = exact_plant(
        inductance, capacitance, load, esr)
    i, vc, previous_u = 0.0, 0.0, 0.0
    observation = (ctypes.c_float * 9)()
    rows, tail = [], deque(maxlen=round(0.4 * FS))
    max_v = max_i = max_m = 0.0
    first_45v = None
    stopped_at = None
    for n in range(round(duration * FS)):
        t = n * TS
        phase = 2.0 * math.pi * F0 * t
        v = alpha * vc + beta * i
        if t < closed_from_s:
            m = float(lib.audit_open(math.sin(phase), 24.0 * math.sqrt(2), 48.0))
        else:
            m = float(lib.audit_step(math.sin(phase), 24.0 * math.sqrt(2),
                                     v + voltage_bias, i + current_bias, 48.0))
        lib.audit_observe(observation)
        max_v, max_i, max_m = max(max_v, abs(v)), max(max_i, abs(i)), max(max_m, abs(m))
        if abs(v) > 45.0 and first_45v is None:
            first_45v = t
        tail.append((phase, v, i, observation[0], observation[2], m))
        if retain_trace and n % 10 == 0:
            rows.append([t, v, i, observation[0], observation[1],
                         observation[2], observation[3], m, observation[4]])
        assert observation[8] == 0.0, f"C fault {observation[8]}"
        if abs(v) >= 1000.0 or abs(i) >= 1000.0:
            stopped_at = t
            break
        applied_u = previous_u if delay_samples else 48.0 * m
        previous_u = 48.0 * m
        i, vc = e11 * i + e12 * vc + g1 * applied_u, e21 * i + e22 * vc + g2 * applied_u
    # Keep integer 50 Hz periods even when a divergent case terminates early.
    count = (len(tail) // 400) * 400
    tail = list(tail)[-count:]
    mean = sum(x[1] for x in tail) / count
    rms = math.sqrt(sum(x[1] ** 2 for x in tail) / count)
    fundamental_s = 2.0 * sum(x[1] * math.sin(x[0]) for x in tail) / count
    fundamental_c = 2.0 * sum(x[1] * math.cos(x[0]) for x in tail) / count
    fundamental_rms = math.hypot(fundamental_s, fundamental_c) / math.sqrt(2.0)
    residual_rms = math.sqrt(max(0.0, rms ** 2 - mean ** 2 - fundamental_rms ** 2))
    cycle_rms = [math.sqrt(sum(x[1] ** 2 for x in tail[k:k+400]) / 400.0)
                 for k in range(0, count - 399, 400)]
    result = dict(inductance_h=inductance, capacitance_f=capacitance,
                  load_ohm=None if math.isinf(load) else load,
                  capacitor_series_r_ohm=esr, delay_samples=delay_samples,
                  closed_loop_from_s=closed_from_s,
                  duration_s=duration, voltage_sensor_bias_v=voltage_bias,
                  current_sensor_bias_a=current_bias, steady_window_s=count/FS,
                  status='diverged_above_audit_bound' if stopped_at is not None else 'completed',
                  stopped_at_s=stopped_at, steady_statistics_valid=(stopped_at is None),
                  steady_rms_v=rms, steady_fundamental_rms_v=fundamental_rms,
                  steady_nonfundamental_rms_v=residual_rms, steady_dc_v=mean,
                  steady_fundamental_phase_deg=math.degrees(math.atan2(fundamental_c, fundamental_s)),
                  steady_current_rms_a=math.sqrt(sum(x[2] ** 2 for x in tail) / count),
                  cycle_rms_range_v=[min(cycle_rms), max(cycle_rms)],
                  max_voltage_abs_v=max_v, max_current_abs_a=max_i,
                  max_modulation_abs=max_m, qpr_saturation_count=int(observation[6]),
                  modulation_saturation_count=int(observation[7]),
                  first_above_45v_s=first_45v)
    return result, rows


def check_plant():
    # Independent analytic checks avoid a numerical integration artifact being
    # mistaken for control instability. Exact lossless LC energy is conserved.
    inductance, capacitance = 440e-6, 2.2e-6
    e11, e12, e21, e22, g1, g2, _, _ = exact_plant(inductance, capacitance, math.inf, 0.0)
    omega = 1.0 / math.sqrt(inductance * capacitance)
    z0 = math.sqrt(inductance / capacitance)
    expected = [math.cos(omega*TS), -math.sin(omega*TS)/z0,
                math.sin(omega*TS)*z0, math.cos(omega*TS),
                math.sin(omega*TS)/z0, 1.0-math.cos(omega*TS)]
    error = max(abs(a-b) for a,b in zip([e11,e12,e21,e22,g1,g2],expected))
    assert error < 1e-13
    i, vc = 0.123, 4.56
    initial_energy = 0.5*inductance*i*i + 0.5*capacitance*vc*vc
    for _ in range(60000):
        i, vc = e11*i+e12*vc, e21*i+e22*vc
    energy_relative_error = abs((0.5*inductance*i*i+0.5*capacitance*vc*vc)/initial_energy-1.0)
    assert energy_relative_error < 1e-9
    return dict(lossless_matrix_max_error=error,
                lossless_energy_relative_error_3s=energy_relative_error)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--compiler', default='D:/tools/mingw64/bin/gcc.exe')
    parser.add_argument('--output', type=Path, default=Path(__file__).with_name('actual_c_lc_results.json'))
    args = parser.parse_args()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    result = dict(scope='Actual current C controller; exact ZOH ideal averaged single-bridge LC. Not Simulink execution, not switched model, not hardware proof.',
                  sample_hz=FS, bus_v=48.0, reference_rms_v=24.0,
                  source_sha256=hashlib.sha256((ROOT/'Core/Src/qpr_control.c').read_bytes()).hexdigest(),
                  header_sha256=hashlib.sha256((ROOT/'Core/Inc/qpr_control.h').read_bytes()).hexdigest(),
                  script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(), cases=[])
    result['plant_checks'] = check_plant()
    with tempfile.TemporaryDirectory(prefix='no_load_actual_c_') as tmp:
        src, dll = Path(tmp)/'bridge.c', Path(tmp)/'qpr_audit.dll'
        src.write_text(SHIM, encoding='utf-8')
        subprocess.run([args.compiler, '-std=c99', '-O2', '-Wall', '-Wextra', '-Werror',
                        '-shared', '-I'+str(ROOT/'Core/Inc'), str(src),
                        str(ROOT/'Core/Src/qpr_control.c'), '-o', str(dll)], check=True)
        lib = ctypes.CDLL(str(dll))
        lib.audit_init.argtypes = [ctypes.c_float]
        lib.audit_init.restype = ctypes.c_int
        lib.audit_step.argtypes = [ctypes.c_float]*5
        lib.audit_step.restype = ctypes.c_float
        lib.audit_open.argtypes = [ctypes.c_float]*3
        lib.audit_open.restype = ctypes.c_float
        lib.audit_observe.argtypes = [ctypes.POINTER(ctypes.c_float)]
        for inductance in [440e-6, 600e-6]:
            for load in [12.0, math.inf]:
                for esr in [0.0, 1.0]:
                    for delay in [0, 1]:
                        case, rows = run_case(lib, inductance, load, esr, delay,
                                              retain_trace=(inductance==440e-6 and math.isinf(load) and delay==1))
                        result['cases'].append(case)
                        print(f"L={inductance*1e6:.0f}uH load={load:g} ESR={esr:g} delay={delay}: "
                              f"RMS={case['steady_rms_v']:.6f}V non50={case['steady_nonfundamental_rms_v']:.6f}V "
                              f"peak={case['max_voltage_abs_v']:.3f}V sat={case['qpr_saturation_count']}")
                        if rows:
                            csv = args.output.with_name(f'trace_440uH_no_load_esr{esr:g}_delay1.csv')
                            csv.write_text('time_s,vout_v,il_a,vref_v,iref_a,qpr_raw_v,qpr_clamped_v,m,error_dc_v\n'+
                                           '\n'.join(','.join(f'{v:.9g}' for v in row) for row in rows)+'\n', encoding='utf-8')
        for load, esr, cap, closed_from in [(12.0,0.0,4.4e-6,0.0),
                                          (math.inf,0.0,4.4e-6,0.0),
                                          (math.inf,1.0,4.4e-6,0.0),
                                          (math.inf,0.0,2.2e-6,1.0),
                                          (math.inf,1.0,2.2e-6,1.0)]:
            case, _ = run_case(lib, 440e-6, load, esr, 1,
                               capacitance=cap, closed_from_s=closed_from)
            result['cases'].append(case)
            print(f"C={cap*1e6:g}uF load={load:g} ESR={esr:g} delay=1 closed_from={closed_from:g}s: "
                  f"RMS={case['steady_rms_v']:.6f}V peak={case['max_voltage_abs_v']:.3f}V "
                  f"status={case['status']} stop={case['stopped_at_s']}")
        # Bias sensitivity is deliberately separate from the zero-bias baseline.
        for vb, ib in [(0.21941, 0.0), (0.0, 0.032844), (0.21941, 0.032844)]:
            case, _ = run_case(lib, 440e-6, math.inf, 1.0, 1,
                               voltage_bias=vb, current_bias=ib)
            result['cases'].append(case)
            print(f"ESR1 delay1 biasV={vb:g} biasI={ib:g}: RMS={case['steady_rms_v']:.6f}V "
                  f"non50={case['steady_nonfundamental_rms_v']:.6f}V peak={case['max_voltage_abs_v']:.3f}V")
        _ctypes.FreeLibrary(lib._handle)
        del lib
    args.output.write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
    print(args.output)


if __name__ == '__main__':
    main()
