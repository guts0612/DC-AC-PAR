#!/usr/bin/env python3
"""Compile main.c and extracted modules for the configured ARM target without linking.

The include paths and target defines are read from the actual Keil project.
All compiler intermediates are placed in a TemporaryDirectory and removed.
This verifies translation units only, not linkage, timing, UART or hardware.
"""
from __future__ import annotations

import argparse
import datetime as dt
import hashlib
import json
from pathlib import Path
import subprocess
import tempfile
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_COMPILER = Path(r"D:\tools\keil\core\ARM\ARMCC\Bin\armcc.exe")


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def project_settings(project: Path, target_name: str) -> dict:
    tree = ET.parse(project)
    target = next((item for item in tree.findall("./Targets/Target")
                   if item.findtext("TargetName") == target_name), None)
    if target is None:
        raise ValueError(f"Keil target not found: {target_name}")
    controls = target.find("./TargetOption/TargetArmAds/Cads/VariousControls")
    if controls is None:
        raise ValueError("Keil target has no C compiler controls")
    include_paths = [str((project.parent / value.strip()).resolve())
                     for value in controls.findtext("IncludePath", "").split(";")
                     if value.strip()]
    defines = [value.strip() for value in controls.findtext("Define", "").split(",")
               if value.strip()]
    return {
        "target": target_name,
        "configured_compiler": target.findtext("pCCUsed", ""),
        "configured_c99": target.findtext("./TargetOption/TargetArmAds/Cads/uC99", ""),
        "include_paths": include_paths,
        "defines": defines,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--project", type=Path, default=ROOT / "MDK-ARM/dcac.uvprojx")
    parser.add_argument("--target", default="dcac")
    parser.add_argument("--compiler", type=Path, default=DEFAULT_COMPILER)
    parser.add_argument("--output", type=Path,
                        default=ROOT / "Tests/results/module_target_build.json")
    args = parser.parse_args()
    project = args.project.resolve()
    compiler = args.compiler.resolve()
    settings = project_settings(project, args.target)
    if not compiler.is_file():
        raise FileNotFoundError(f"ARMCC is unavailable: {compiler}")
    if settings["configured_c99"] != "1":
        raise ValueError("The selected Keil target is not configured for C99")
    files = [ROOT / name for name in
             ("justfloat.c", "justfloat.h", "key_control.c", "key_control.h",
              "hrtim_spwm.c", "hrtim_spwm.h", "adc_diff.c", "adc_diff.h")]
    files.append(ROOT / "Core/Src/main.c")
    before = {path.name: sha256(path) for path in files}
    version = subprocess.run([str(compiler), "--vsn"], check=True,
                             capture_output=True, text=True)
    result = {
        "timestamp_utc": dt.datetime.now(dt.timezone.utc).isoformat(),
        "scope": ("ARMCC Cortex-M4 floating-point target translation-unit compilation "
                  "of main.c and the extracted modules. No full firmware link, execution, "
                  "UART capture, timing test or hardware validation."),
        "project": str(project),
        "project_sha256": sha256(project),
        "compiler": str(compiler),
        "compiler_version": (version.stdout + version.stderr).strip(),
        "project_settings": settings,
        "source_sha256": before,
        "cases": [],
    }
    common = [str(compiler), "--cpu", "Cortex-M4.fp", "--c99", "-O2"]
    common += ["-D" + value for value in settings["defines"]]
    common += ["-I" + value for value in settings["include_paths"]]
    cases = [
        ("main", "Core/Src/main.c", []),
        ("justfloat_waveform", "justfloat.c", []),
        ("key_control", "key_control.c", []),
        ("hrtim_spwm", "hrtim_spwm.c", []),
        ("adc_diff", "adc_diff.c", []),
    ]
    with tempfile.TemporaryDirectory(prefix="dcac_module_armcc_") as directory:
        temporary = Path(directory)
        for name, filename, extra_defines in cases:
            output = temporary / (name + ".o")
            command = common + ["-D" + value for value in extra_defines]
            command += ["-c", str(ROOT / filename), "-o", str(output)]
            build = subprocess.run(command, cwd=temporary, capture_output=True, text=True)
            object_bytes = output.stat().st_size if output.is_file() else 0
            passed = build.returncode == 0 and object_bytes > 0
            result["cases"].append({
                "name": name,
                "source": str(ROOT / filename),
                "extra_defines": extra_defines,
                "command": command,
                "returncode": build.returncode,
                "object_bytes": object_bytes,
                "diagnostics": (build.stdout + build.stderr).strip(),
                "passed": passed,
            })
            print(f"{'PASS' if passed else 'FAIL'} {name}", flush=True)
    result["temporary_intermediates_removed"] = not temporary.exists()
    after = {path.name: sha256(path) for path in files}
    result["sources_unchanged"] = before == after
    result["checks_passed"] = (all(case["passed"] for case in result["cases"])
                               and result["sources_unchanged"]
                               and result["temporary_intermediates_removed"])
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n",
                           encoding="utf-8")
    print(f"Result: {args.output.resolve()}", flush=True)
    return 0 if result["checks_passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
