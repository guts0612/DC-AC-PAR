"""Verify the port against unmodified archive C and the analytic QPR response.

Executes actual controller C with host GCC, not a plant/MCU simulation. Uses
immutable pre-port source for a numerical comparison; never changes live gains.
"""
from pathlib import Path
import hashlib
import json
import math
import re
import shutil
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
EVIDENCE = ROOT / "verification/reg_pfc_pr_20261019"
GCC = shutil.which("gcc") or "D:/tools/mingw64/bin/gcc.exe"

HARNESS = r'''
#include "qpr_control.h"
#include "PR.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#define PI 3.14159265358979323846
void Archive_PR_init(struct PR_PATH *, float, float, float, float, float);
void Archive_PR_update(struct PR_PATH *, float);

static void response(double frequency)
{
    const int fs = 20000;
    const double amplitude = 0.1;
    QPR_Control master, slave;
    struct PR_PATH archive;
    double sine_sum = 0.0, cosine_sum = 0.0, dc_sum = 0.0;
    double archive_max_error = 0.0;
    assert(QPR_ControlInit(&master, (float)fs));
    assert(QPR_ControlInit(&slave, (float)fs));
    Archive_PR_init(&archive, 1.0f/fs, QPR_KP, QPR_KR, QPR_WC_RAD_S,
                    (float)(2.0*PI*QPR_FREQUENCY_HZ));
#ifndef PREVIOUS_IMPLEMENTATION
    assert(master.pr.A0 == archive.A0 && master.pr.A1 == archive.A1 &&
           master.pr.A2 == archive.A2 && master.pr.B0 == archive.B0 &&
           master.pr.B1 == archive.B1 && master.pr.B2 == archive.B2);
#endif
    for (int n = 0; n < 5*fs; ++n)
    {
        const double phase = 2.0*PI*frequency*n/fs;
        const float input = (float)(amplitude*(frequency == 0 ? 1.0 : sin(phase)));
        double difference;
        Archive_PR_update(&archive, input);
        /* Zero voltage error isolates the master's actual inner-loop path. */
        QPR_ControlStep(&master, 0.0f, 0.0f, 0.0f, -input, 48.0f);
        QPR_CurrentStep(&slave, input, 0.0f, 0.0f, 48.0f);
        assert(master.fault_flags == 0 && slave.fault_flags == 0);
        assert(master.qpr_saturation_count == 0 && slave.qpr_saturation_count == 0);
#ifndef PREVIOUS_IMPLEMENTATION
        assert(master.qpr_unlimited_v == slave.qpr_unlimited_v);
#endif
        difference = fabs((double)master.qpr_unlimited_v - archive.Vo);
        if (difference > archive_max_error) archive_max_error = difference;
#ifndef PREVIOUS_IMPLEMENTATION
        assert(difference == 0.0);
#endif
        if (n >= 4*fs)
        {
            sine_sum += master.qpr_unlimited_v*sin(phase);
            cosine_sum += master.qpr_unlimited_v*cos(phase);
            dc_sum += master.qpr_unlimited_v;
        }
    }
    printf("{\"frequency_hz\":%.9g,\"gain\":%.12g,\"phase_deg\":%.12g,"
           "\"archive_max_error_v\":%.12g}\n", frequency,
           frequency == 0 ? dc_sum/fs/amplitude : 2.0*hypot(sine_sum,cosine_sum)/fs/amplitude,
           frequency == 0 ? 0.0 : atan2(cosine_sum,sine_sum)*180.0/PI,
           archive_max_error);
}

#ifndef PREVIOUS_IMPLEMENTATION
static void boundary_checks(void)
{
    QPR_Control master, slave;
    struct PR_PATH before, reference;
    assert(QPR_ControlInit(&master, 20000));
    assert(QPR_ControlInit(&slave, 20000));
    before = slave.pr;
    /* Impulse/steps and a deterministic mixed signal, beyond pure sinusoids. */
    Archive_PR_init(&reference, master.sample_period_s, QPR_KP, QPR_KR,
                    QPR_WC_RAD_S, (float)(2.0*PI*QPR_FREQUENCY_HZ));
    for (int n = 0; n < 10000; ++n)
    {
        float error = n == 0 ? 0.03f : n < 500 ? 0.0f :
                      n < 2000 ? -0.01f : (float)(0.02*sin(n*0.013)+0.01*cos(n*0.17));
        Archive_PR_update(&reference, error);
        QPR_ControlStep(&master, 0, 0, 0, -error, 48);
        assert(master.qpr_unlimited_v == reference.Vo);
    }
    assert(memcmp(&slave.pr, &before, sizeof(before)) == 0); /* independent states */
    QPR_ControlOpenLoopStep(&master, 0, 0, 48);
    assert(master.pr.Vi1 == 0 && master.pr.Vi2 == 0 &&
           master.pr.Vo1 == 0 && master.pr.Vo2 == 0);
    assert(master.pr.A0 == before.A0 && master.pr.B2 == before.B2);
    assert(QPR_ControlStep(&master, 0, 0, 0, 0, 48) == 0);
    /* Both saturation directions and both voltage/modulation limiters. */
    for (int sign = -1; sign <= 1; sign += 2)
    {
        QPR_ControlReset(&master);
        QPR_ControlReset(&slave);
        before = slave.pr;
        for (int n = 0; n < 100; ++n)
        {
            QPR_ControlStep(&master, 0, 0, 0, -sign*100.0f, 48);
            QPR_CurrentStep(&slave, sign*100.0f, 0, 0, 48);
            assert(master.qpr_correction_v == sign*QPR_CORRECTION_LIMIT_V);
            assert(slave.qpr_correction_v == sign*QPR_CORRECTION_LIMIT_V);
        }
        assert(master.qpr_saturation_count == 100 && slave.qpr_saturation_count == 100);
        assert(master.pr.Vi1 == sign*100.0f); /* master retains unlimited history */
        assert(memcmp(&slave.pr, &before, sizeof(before)) == 0); /* slave freezes */
        QPR_CurrentStep(&slave, sign*0.01f, 0, 0, 48);
        assert(slave.pr.Vi1 == sign*0.01f); /* accepts a nonsaturating sample */
        QPR_ControlReset(&slave);
        before = slave.pr;
        assert(QPR_CurrentStep(&slave, sign*0.01f, 0, sign*60.0f, 48) ==
               sign*QPR_MODULATION_LIMIT);
        assert(memcmp(&slave.pr, &before, sizeof(before)) == 0);
        QPR_CurrentStep(&slave, -sign*0.01f, 0, sign*60.0f, 48);
        assert(slave.pr.Vi1 == -sign*0.01f); /* reversal reduces saturation */
    }
    QPR_ControlReset(&master);
    assert(QPR_ControlStep(&master, 0, 0, 0, NAN, 48) == 0);
    assert(master.fault_flags & QPR_FAULT_INPUT);
    QPR_ControlReset(&master);
    master.pr.Vo1 = INFINITY;
    assert(QPR_ControlStep(&master, 0, 0, 0, 0, 48) == 0);
    assert(master.fault_flags & QPR_FAULT_NUMERIC);
    QPR_ControlReset(&slave);
    assert(QPR_CurrentStep(&slave, NAN, 0, 0, 48) == 0);
    assert(slave.fault_flags & QPR_FAULT_INPUT);
    QPR_ControlReset(&slave);
    slave.pr.Vo2 = INFINITY;
    assert(QPR_CurrentStep(&slave, 0, 0, 0, 48) == 0);
    assert(slave.fault_flags & QPR_FAULT_NUMERIC);
    assert(!QPR_ControlInit(&master, 0));
    assert(!QPR_ControlInit(&master, NAN));
}
#endif

int main(void)
{
    const double frequencies[] = {0,49,50,51,150,1000,5000};
    for (unsigned int i = 0; i < sizeof(frequencies)/sizeof(frequencies[0]); ++i)
        response(frequencies[i]);
#ifndef PREVIOUS_IMPLEMENTATION
    boundary_checks();
#endif
    return 0;
}
'''


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    manifest = json.loads((EVIDENCE / "source_manifest.json").read_text(encoding="utf-8"))
    for name, digest in manifest["archive_members"].items():
        assert sha(EVIDENCE / "original" / name) == digest, f"Archive fixture changed: {name}"
    for kind, name in (("Src", "qpr_control.c"), ("Inc", "qpr_control.h")):
        assert sha(EVIDENCE / "previous" / name) == manifest["original_files"][f"Core/{kind}/{name}"]
    live_header = (ROOT / "Core/Inc/qpr_control.h").read_text(encoding="utf-8-sig")
    live_gains = [float(re.search(rf"#define\s+QPR_K{letter}\s+([\d.]+)", live_header)[1])
                  for letter in ("P", "R")]
    results = []
    flags = ["-std=c11", "-O2", "-ffp-contract=off", "-Wall", "-Wextra", "-Werror"]
    with tempfile.TemporaryDirectory(prefix="archive_pr_") as directory:
        temp = Path(directory)
        archive_obj = temp / "archive.o"
        subprocess.run([GCC, *flags, "-DPR_init=Archive_PR_init", "-DPR_update=Archive_PR_update",
                        "-c", str(EVIDENCE / "original/PR.c"), "-o", str(archive_obj)], check=True)
        for label, kp, kr in (("saved", *live_gains), ("user_trial", 1.0, 1.6)):
            for implementation in ("ported", "previous"):
                folder = temp / f"{label}_{implementation}"
                folder.mkdir()
                if implementation == "ported":
                    source = ROOT / "Core/Src/qpr_control.c"
                    header = live_header
                else:
                    source = EVIDENCE / "previous/qpr_control.c"
                    header = (EVIDENCE / "previous/qpr_control.h").read_text(encoding="utf-8-sig")
                for macro, value in (("QPR_KP", kp), ("QPR_KR", kr)):
                    header, count = re.subn(rf"(?m)^(#define\s+{macro}\s+)\S+",
                                           lambda match: match[1]+f"{value}f", header)
                    assert count == 1
                (folder / "qpr_control.h").write_text(header, encoding="utf-8")
                shutil.copyfile(source, folder / "qpr_control.c")
                (folder / "harness.c").write_text(HARNESS, encoding="utf-8")
                exe = folder / "verify.exe"
                extra = ["-DPREVIOUS_IMPLEMENTATION"] if implementation == "previous" else []
                subprocess.run([GCC, *flags, *extra, "-I"+str(folder), "-I"+str(ROOT / "Core/Inc"),
                                str(folder / "harness.c"), str(folder / "qpr_control.c"),
                                str(ROOT / "Core/Src/PR.c"), str(archive_obj), "-o", str(exe)], check=True)
                run = subprocess.run([str(exe)], text=True, capture_output=True)
                if run.returncode:
                    raise RuntimeError(f"{label}/{implementation}: {run.stdout}\n{run.stderr}")
                output = run.stdout
                responses = [json.loads(line) for line in output.splitlines()]
                for response in responses:
                    frequency = response["frequency_hz"]
                    wd = 40000*math.tan(math.pi*frequency/20000)
                    w0 = 2*math.pi*50
                    h = kp + kr*(10j*wd)/(w0*w0-wd*wd+10j*wd)
                    reference_phase = math.degrees(math.atan2(h.imag, h.real))
                    response["analytic_gain"] = abs(h)
                    response["analytic_phase_deg"] = reference_phase
                    response["gain_error_percent"] = (response["gain"]/abs(h)-1)*100
                    response["phase_error_deg"] = response["phase_deg"]-reference_phase
                    assert abs(response["gain_error_percent"]) < 1, response
                    assert abs(response["phase_error_deg"]) < 1, response
                results.append(dict(parameters=label, kp=kp, kr=kr, implementation=implementation,
                                    responses=responses))
    files = ["Core/Src/PR.c", "Core/Inc/PR.h", "Core/Src/qpr_control.c", "Core/Inc/qpr_control.h"]
    report = dict(scope="Host GCC numeric/API checks only; no plant, ADC/PWM or MCU timing validation",
                  compiler=GCC, compiler_flags=flags, sample_frequency_hz=20000,
                  source_sha256={name: sha(ROOT / name) for name in files},
                  boundary_checks="PASS: independent states, reset/open loop, both saturation directions, "
                                  "slave conditional freeze/reversal and invalid input/state faults",
                  results=results)
    (EVIDENCE / "pr_port_results.json").write_text(json.dumps(report, indent=2)+"\n", encoding="utf-8")
    print("PASS: exact archive agreement in master/slave; 28 response cases; reset/saturation/fault checks")
    for result in results:
        response = next(row for row in result["responses"] if row["frequency_hz"] == 50)
        print(f'{result["parameters"]} {result["implementation"]}: '
              f'50 Hz gain={response["gain"]:.8f}, phase={response["phase_deg"]:.6f} deg')


if __name__ == "__main__":
    main()
