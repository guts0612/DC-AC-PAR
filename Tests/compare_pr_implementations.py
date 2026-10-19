#!/usr/bin/env python3
"""Compare the actual, unmodified C implementations at matched QPR parameters.

Host numerical verification only: no plant, PWM, ADC or MCU timing simulation.
The DSPLibrary tree is hashed before and after and is only read by this script.
All compiler intermediates are created in a temporary directory.
"""
from __future__ import annotations

import argparse
import datetime as dt
import hashlib
import json
import math
from pathlib import Path
import shutil
import subprocess
import tempfile


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_LIBRARY = Path(r"D:\study\电赛\开源方案\2021年B题国一\DSPLibrary-main")

HARNESS = r'''
#include "qpr_control.h"
#include "pr.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

#define PI 3.14159265358979323846
#ifndef REQUIRE_EQUIVALENCE
#define REQUIRE_EQUIVALENCE 0
#endif

static void coefficients(double fs, double scale, double *sos)
{
    const double theta = 2.0 * PI * QPR_FREQUENCY_HZ / fs;
    const double d = QPR_WC_RAD_S / fs;
    const double D = 1.0 + d + theta * theta * 0.25;
    /* The library's SOS contains the resonant branch without Kr. */
    sos[0] = scale * d / D;
    sos[1] = 0.0;
    sos[2] = -sos[0];
    sos[3] = scale;
    sos[4] = scale * (-2.0 + theta * theta * 0.5) / D;
    sos[5] = scale * (1.0 - d + theta * theta * 0.25) / D;
}

static void response(int fs, double frequency, double scale)
{
    QPR_Control current, master;
    ButterQueue library;
    double sos[6];
    float excess = 0.0f;
    const double amplitude = 0.1;
    double current_s = 0.0, current_c = 0.0;
    double master_s = 0.0, master_c = 0.0;
    double library_s = 0.0, library_c = 0.0;
    double current_error_sq = 0.0, master_error_sq = 0.0, library_error_sq = 0.0;
    const double wd = 2.0 * fs * tan(PI * frequency / fs);
    const double w0 = 2.0 * PI * QPR_FREQUENCY_HZ;
    const double a = w0 * w0 - wd * wd;
    const double b = 2.0 * QPR_WC_RAD_S * wd;
    const double re = QPR_KP + QPR_KR * b * b / (a * a + b * b);
    const double im = QPR_KR * b * a / (a * a + b * b);
    assert(QPR_ControlInit(&current, (float)fs));
    assert(QPR_ControlInit(&master, (float)fs));
    InitButterQueue(&library);
    coefficients(fs, scale, sos);
    for (int n = 0; n < 5 * fs; ++n)
    {
        const double phase = 2.0 * PI * frequency * n / fs;
        const double s = sin(phase), c = cos(phase);
        const float input = (float)(amplitude * s);
        double y_current, y_master, y_library;
        /* Slave public API, unsaturated current loop. */
        (void)QPR_CurrentStep(&current, input, 0.0f, 0.0f, 48.0f);
        y_current = current.qpr_unlimited_v;
        /* Master public API: ev=0 => virtual-impedance iref=0, iL=-input. */
        (void)QPR_ControlStep(&master, 0.0f, 0.0f, 0.0f, -input, 48.0f);
        y_master = master.qpr_unlimited_v;
        y_library = PRCalc(&library, input, sos, 1.0f, QPR_KP, QPR_KR,
                           QPR_CORRECTION_LIMIT_V, 0.0f, &excess);
        assert(isfinite(y_current) && isfinite(y_master) && isfinite(y_library));
        assert(current.fault_flags == 0 && master.fault_flags == 0);
        if (n >= 4 * fs)
        {
            const double expected = amplitude * (re * s + im * c);
            current_s += y_current * s; current_c += y_current * c;
            master_s += y_master * s; master_c += y_master * c;
            library_s += y_library * s; library_c += y_library * c;
            current_error_sq += (y_current - expected) * (y_current - expected);
            master_error_sq += (y_master - expected) * (y_master - expected);
            library_error_sq += (y_library - expected) * (y_library - expected);
        }
    }
    assert(current.qpr_saturation_count == 0 && master.qpr_saturation_count == 0);
    assert(current.modulation_saturation_count == 0 && master.modulation_saturation_count == 0);
    current_s *= 2.0 / (fs * amplitude); current_c *= 2.0 / (fs * amplitude);
    master_s *= 2.0 / (fs * amplitude); master_c *= 2.0 / (fs * amplitude);
    library_s *= 2.0 / (fs * amplitude); library_c *= 2.0 / (fs * amplitude);
    /* Current code must agree with the analytical Tustin response. */
    if (REQUIRE_EQUIVALENCE &&
        (hypot(current_s - re, current_c - im) / hypot(re, im) >= 0.0001 ||
         hypot(master_s - re, master_c - im) / hypot(re, im) >= 0.0001))
    {
        fprintf(stderr, "Unexpected error: fs=%d f=%.0f scale=%.0f expected %.12g %.12g current %.12g %.12g master %.12g %.12g\n",
                fs, frequency, scale, re, im, current_s, current_c, master_s, master_c);
        assert(0);
    }
    printf("{\"sample_hz\":%d,\"signal_hz\":%.0f,\"sos_scale\":%.0f,"
           "\"analytic_re\":%.12g,\"analytic_im\":%.12g,"
           "\"current_re\":%.12g,\"current_im\":%.12g,"
           "\"master_re\":%.12g,\"master_im\":%.12g,"
           "\"library_re\":%.12g,\"library_im\":%.12g,"
           "\"current_rms_error_v\":%.12g,\"master_rms_error_v\":%.12g,"
           "\"library_rms_error_v\":%.12g}\n",
           fs, frequency, scale, re, im, current_s, current_c, master_s, master_c,
           library_s, library_c, sqrt(current_error_sq/fs), sqrt(master_error_sq/fs),
           sqrt(library_error_sq/fs));
}

static void saturation(void)
{
    QPR_Control current, master;
    ButterQueue library;
    double sos[6], last_library = 0.0;
    float excess = 0.0f;
    assert(QPR_ControlInit(&current, 20000.0f));
    assert(QPR_ControlInit(&master, 20000.0f));
    InitButterQueue(&library);
    coefficients(20000.0, 10000.0, sos);
    for (int n = 0; n < 20000; ++n)
    {
        (void)QPR_CurrentStep(&current, 100.0f, 0.0f, 0.0f, 48.0f);
        (void)QPR_ControlStep(&master, 0.0f, 0.0f, 0.0f, -100.0f, 48.0f);
        last_library = PRCalc(&library, 100.0f, sos, 1.0f, QPR_KP, QPR_KR,
                              QPR_CORRECTION_LIMIT_V, 10.0f, &excess);
    }
    assert(excess == 0.0f && fabs(last_library) > QPR_CORRECTION_LIMIT_V);
    assert(fabsf(current.qpr_correction_v) <= QPR_CORRECTION_LIMIT_V);
    assert(fabsf(master.qpr_correction_v) <= QPR_CORRECTION_LIMIT_V);
    assert(current.resonant_r == 0.0f && current.resonant_q == 0.0f);
    printf("{\"test\":\"saturation\",\"library_return_v\":%.12g,"
           "\"library_excess_v\":%.12g,\"current_correction_v\":%.12g,"
           "\"master_correction_v\":%.12g,\"current_resonant_r_v\":%.12g,"
           "\"master_resonant_r_v\":%.12g}\n", last_library, excess,
           (double)current.qpr_correction_v, (double)master.qpr_correction_v,
           (double)current.resonant_r, (double)master.resonant_r);
}

static void candidate_regression(void)
{
    QPR_Control c;
    const float signs[] = {-1.0f, 1.0f};
    if (!REQUIRE_EQUIVALENCE) return;
    assert(!QPR_ControlInit(&c, NAN));
    assert(!QPR_ControlInit(&c, 100.0f));
    for (unsigned i = 0; i < 2; ++i)
    {
        const float sign = signs[i];
        /* Correction limit alone: freeze when the error pushes further. */
        assert(QPR_ControlInit(&c, 20000.0f));
        (void)QPR_CurrentStep(&c, sign * 100.0f, 0.0f, 0.0f, 48.0f);
        assert(c.qpr_saturation_count == 1 && c.modulation_saturation_count == 0);
        assert(c.resonant_r == 0.0f && c.resonant_q == 0.0f);
        assert(c.current_error_previous_a == sign * 100.0f);
        /* Error reversal must release the held states. */
        (void)QPR_CurrentStep(&c, -sign * 0.01f, 0.0f, 0.0f, 48.0f);
        assert(c.resonant_r != 0.0f && c.resonant_q != 0.0f);
        for (int n = 0; n < 40000; ++n)
            (void)QPR_CurrentStep(&c, 0.0f, 0.0f, 0.0f, 48.0f);
        assert(fabsf(c.resonant_r) < 0.001f && fabsf(c.resonant_q) < 0.001f);

        /* Total modulation limit alone, caused by voltage feedforward. */
        assert(QPR_ControlInit(&c, 20000.0f));
        (void)QPR_CurrentStep(&c, sign * 0.01f, 0.0f, sign * 48.0f, 48.0f);
        assert(c.qpr_saturation_count == 0 && c.modulation_saturation_count == 1);
        assert(c.resonant_r == 0.0f && c.resonant_q == 0.0f);
        QPR_ControlReset(&c);
        /* Still saturated, but the reversed error helps leave the limit. */
        (void)QPR_CurrentStep(&c, -sign * 0.01f, 0.0f, sign * 48.0f, 48.0f);
        assert(c.modulation_saturation_count == 1);
        assert(c.resonant_r != 0.0f && c.resonant_q != 0.0f);
    }
    assert(QPR_ControlInit(&c, 20000.0f));
    (void)QPR_CurrentStep(&c, 0.1f, 0.0f, 0.0f, 48.0f);
    assert(c.modulation != 0.0f);
    assert(QPR_CurrentStep(&c, NAN, 0.0f, 0.0f, 48.0f) == 0.0f);
    assert(c.modulation == 0.0f && c.fault_flags == QPR_FAULT_INPUT);
    assert(QPR_CurrentStep(&c, 0.1f, 0.0f, 0.0f, 48.0f) == 0.0f);
    assert(c.modulation == 0.0f);
    QPR_ControlReset(&c);
    assert(c.fault_flags == 0);
    c.resonant_q = INFINITY;
    assert(QPR_CurrentStep(&c, 0.1f, 0.0f, 0.0f, 48.0f) == 0.0f);
    assert(c.modulation == 0.0f && c.fault_flags == QPR_FAULT_NUMERIC);
    puts("{\"test\":\"candidate_regression\",\"positive_negative_correction_limits\":true,"
         "\"feedforward_modulation_limits\":true,\"error_reversal_and_decay\":true,"
         "\"fault_latch_reset_and_numeric_failure\":true}");
}

int main(void)
{
    const int rates[] = {20000, 100000};
    const double frequencies[] = {10, 49, 50, 51, 150, 250, 1000};
    const double scales[] = {1, 10000};
    for (unsigned r = 0; r < sizeof(rates)/sizeof(rates[0]); ++r)
        for (unsigned k = 0; k < sizeof(scales)/sizeof(scales[0]); ++k)
            for (unsigned f = 0; f < sizeof(frequencies)/sizeof(frequencies[0]); ++f)
                response(rates[r], frequencies[f], scales[k]);
    saturation();
    candidate_regression();
    return 0;
}
'''


def tree_hashes(directory: Path) -> dict[str, str]:
    return {
        path.relative_to(directory).as_posix(): file_hash(path)
        for path in sorted(directory.rglob("*")) if path.is_file()
    }


def file_hash(path: Path) -> str:
    with path.open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--library", type=Path, default=DEFAULT_LIBRARY)
    parser.add_argument("--compiler", default=shutil.which("gcc") or r"D:\tools\mingw64\bin\gcc.exe")
    parser.add_argument("--output", type=Path, default=ROOT / "Tests/results/pr_comparison.json")
    args = parser.parse_args()
    library = args.library.resolve()
    library_before = tree_hashes(library)
    assert library_before, "Library directory must exist and contain files"
    sources = [ROOT / "Core/Src/qpr_control.c", ROOT / "Core/Inc/qpr_control.h"]
    candidates = [ROOT / "qpr_control.c", ROOT / "qpr_control.h"]
    original_hashes = {str(path): file_hash(path) for path in sources}
    candidate_hashes = {str(path): file_hash(path) for path in candidates}
    pr_dir = library / "DSPTools/PR_Controller"
    filter_dir = library / "DSPTools/ButterWorth_Filter"
    rows = []
    target_compile = {"status": "not_available", "scope": "Candidate translation unit only; no firmware link"}
    with tempfile.TemporaryDirectory(prefix="pr_comparison_") as temporary:
        temporary = Path(temporary)
        harness = temporary / "compare.c"
        executable = temporary / "compare.exe"
        harness.write_text(HARNESS, encoding="utf-8")
        for name, source, include, require in (
            ("original", sources[0], ROOT / "Core/Inc", 0),
            ("candidate", candidates[0], ROOT, 1),
        ):
            command = [args.compiler, "-std=c11", "-O2", "-ffp-contract=off",
                       "-Wall", "-Wextra", "-Werror", "-Wno-unused-parameter",
                       f"-DREQUIRE_EQUIVALENCE={require}",
                       "-I" + str(include), "-I" + str(pr_dir),
                       "-I" + str(filter_dir), str(harness), str(source),
                       str(pr_dir / "pr.c"), str(filter_dir / "filter.c"),
                       "-lm", "-o", str(executable)]
            compiled = subprocess.run(command, capture_output=True, text=True)
            if compiled.returncode:
                raise RuntimeError(compiled.stderr)
            run = subprocess.run([str(executable)], capture_output=True, text=True)
            if run.returncode:
                raise RuntimeError(run.stderr + run.stdout)
            rows.extend(dict(json.loads(line), variant=name) for line in run.stdout.splitlines())
        armcc = Path(r"D:\tools\keil\core\ARM\ARMCC\Bin\armcc.exe")
        if armcc.is_file():
            arm_command = [str(armcc), "--cpu", "Cortex-M4.fp", "--c99", "-O2",
                           "-I" + str(ROOT), "-c", str(candidates[0]),
                           "-o", str(temporary / "qpr_candidate.o")]
            compiled = subprocess.run(arm_command, capture_output=True, text=True)
            if compiled.returncode:
                raise RuntimeError(compiled.stderr + compiled.stdout)
            target_compile.update(status="passed", compiler=str(armcc),
                                  diagnostics=compiled.stderr + compiled.stdout)
    for row in rows:
        if "sample_hz" not in row:
            continue
        expected = complex(row["analytic_re"], row["analytic_im"])
        for name in ("current", "master", "library"):
            actual = complex(row[f"{name}_re"], row[f"{name}_im"])
            row[f"{name}_relative_complex_error_percent"] = abs(actual - expected) / abs(expected) * 100
            row[f"{name}_gain"] = abs(actual)
            row[f"{name}_phase_degrees"] = math.degrees(math.atan2(actual.imag, actual.real))
    library_after = tree_hashes(library)
    assert library_before == library_after, "Library contents changed during verification"
    assert original_hashes == {str(path): file_hash(path) for path in sources}
    result = {
        "verified_at": dt.datetime.now(dt.timezone.utc).isoformat(),
        "scope": "Host C numerical comparison only; no plant, hardware or target cycle-time validation",
        "compiler": subprocess.run([args.compiler, "--version"], check=True,
                                   capture_output=True, text=True).stdout.splitlines()[0],
        "compiler_options": ["-std=c11", "-O2", "-ffp-contract=off", "no fast-math"],
        "parameters": {"kp": 1.3, "kr": 10, "wc_rad_s": 5, "f0_hz": 50,
                       "input_peak_a": 0.1, "settle_s": 4, "measure_s": 1},
        "original_qpr_sha256": original_hashes,
        "candidate_qpr_sha256": candidate_hashes,
        "target_object_compile": target_compile,
        "library_root": str(library),
        "library_files_hashed": len(library_before),
        "library_tree_unchanged": True,
        "library_pr_sha256": file_hash(pr_dir / "pr.c"),
        "cases": rows,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(f"PASS: {sum('sample_hz' in row for row in rows)} frequency comparisons; original and candidate saturation checks")
    print(f"PASS: {len(library_before)} library files and both original QPR files unchanged")
    print(f"ARM candidate object compilation: {target_compile['status']}")
    for row in rows:
        if row.get("sample_hz") == 20000 and row.get("signal_hz") == 50:
            print(f"{row['variant']} 20 kHz / 50 Hz / SOS scale {row['sos_scale']}: relative complex error "
                  f"current={row['current_relative_complex_error_percent']:.6g}%, "
                  f"master={row['master_relative_complex_error_percent']:.6g}%, "
                  f"library={row['library_relative_complex_error_percent']:.6g}%")
    for row in rows:
        if "test" in row:
            print(json.dumps(row, ensure_ascii=False))
    print(args.output)


if __name__ == "__main__":
    main()
