#!/usr/bin/env python3
"""Execute real QPR/inverter C and compare it with SLX-derived equations.

Requires Python 3 and GCC, no third-party Python packages. The bridge and DLL
are host-only test artifacts. This does NOT run Simulink or validate hardware,
ADC polarity, PWM timing, switching ripple, protection, or target execution time.
"""
from __future__ import annotations

import argparse
import ast
import ctypes as ct
import datetime as dt
import hashlib
import json
import math
import os
from pathlib import Path
import re
import shutil
import struct
import subprocess
import tempfile
import xml.etree.ElementTree as ET
import zipfile

ROOT = Path(__file__).resolve().parents[1]
F32 = lambda value: struct.unpack("<f", struct.pack("<f", value))[0]


def arithmetic(expression, variables=None):
    """Read numeric model data; never execute model callbacks or other code."""
    variables = variables or {}
    node = ast.parse(expression.strip().replace("^", "**"), mode="eval").body

    def evaluate(n):
        if isinstance(n, ast.Constant) and type(n.value) in (int, float):
            return float(n.value)
        if isinstance(n, ast.Name) and n.id in variables:
            return variables[n.id]
        if isinstance(n, ast.UnaryOp) and isinstance(n.op, (ast.UAdd, ast.USub)):
            return evaluate(n.operand) * (-1 if isinstance(n.op, ast.USub) else 1)
        if isinstance(n, ast.BinOp):
            a, b = evaluate(n.left), evaluate(n.right)
            if isinstance(n.op, ast.Add): return a + b
            if isinstance(n.op, ast.Sub): return a - b
            if isinstance(n.op, ast.Mult): return a * b
            if isinstance(n.op, ast.Div): return a / b
            if isinstance(n.op, ast.Pow): return a ** b
        raise ValueError(f"Unsupported model arithmetic: {expression}")
    return evaluate(node)


def read_model(path):
    with zipfile.ZipFile(path) as archive:
        root = ET.fromstring(archive.read("simulink/systems/system_root.xml"))
        blocks = {b.get("SID"): b for b in root.findall("./Block")}
        pr = next(b for b in blocks.values() if b.get("Name") == "Digital PR")
        values = {m.get("Name"): arithmetic(m.findtext("Value"))
                  for m in pr.findall("./Mask/MaskParameter")}
        scripts = []
        for name in archive.namelist():
            if name.startswith("simulink/stateflow/chart_") and name.endswith(".xml"):
                chart = ET.fromstring(archive.read(name))
                scripts += [p.text for p in chart.findall(".//P[@Name='script']")
                            if p.text and "out_A = b0*err_A" in p.text]
        assert len(scripts) == 1, "Expected one matching original Digital PR function"
        script = scripts[0]
        env = dict(values, pi=math.pi)
        for name in ("W0", "t1", "t2", "t3", "b0", "b1", "b2", "a1", "a2"):
            match = re.search(r"^\s*" + name + r"\s*=\s*([^;\r\n]+);", script, re.M)
            assert match, f"Missing model assignment {name}"
            env[name] = arithmetic(match.group(1), env)
        assert abs(env["b0"]) <= 3000., "Model b0 guard would activate; extend the reference first"
        recurrence = re.search(r"^\s*out_A\s*=\s*([^;\r\n]+);", script, re.M)
        expected = "b0*err_A+b1*err_A1+b2*err_A2-a1*out_A1-a2*out_A2"
        assert re.sub(r"\s+", "", recurrence.group(1)) == expected
        virtual = next(b for b in blocks.values() if b.get("BlockType") == "TransferFcn")
        denominator = virtual.findtext("./P[@Name='Denominator']")
        lv, rv = map(arithmetic, denominator.strip("[]").split())
        component = lambda sid, field: arithmetic(blocks[sid].findtext(
            "./InstanceData/P[@Name='" + field + "']"))
        return dict(parameters=values, w0=env["W0"],
                    coefficients=[env[k] for k in ("b0", "b1", "b2", "a1", "a2")],
                    virtual_l_h=lv, virtual_r_ohm=rv,
                    physical_l_h=component("14", "Inductance") + component("110", "Inductance"),
                    physical_c_f=component("15", "Capacitance"),
                    model_load_ohm=component("16", "Resistance"),
                    model_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                    function_sha256=hashlib.sha256(script.encode()).hexdigest())


class OriginalQPR:
    def __init__(self, coefficients, single=False):
        self.co = list(map(F32, coefficients)) if single else coefficients
        self.single = single
        self.e1 = self.e2 = self.y1 = self.y2 = 0.0

    def step(self, e):
        b0, b1, b2, a1, a2 = self.co
        if self.single:
            y = F32(F32(F32(F32(F32(b0 * e) + F32(b1 * self.e1)) +
                                F32(b2 * self.e2)) - F32(a1 * self.y1)) - F32(a2 * self.y2))
        else:
            y = b0 * e + b1 * self.e1 + b2 * self.e2 - a1 * self.y1 - a2 * self.y2
        self.e2, self.e1, self.y2, self.y1 = self.e1, e, self.y1, y
        return y


class Host:
    def __init__(self, library, fs):
        self.lib = library
        self.ptr = library.qpr_test_create(fs)
        assert self.ptr

    def close(self):
        if self.ptr:
            self.lib.qpr_test_destroy(self.ptr)
            self.ptr = None

    def __enter__(self): return self
    def __exit__(self, *_): self.close()
    def reset(self): self.lib.qpr_test_reset(self.ptr)
    def step(self, reference=0., voltage=0., current=0., bus=48.):
        return self.lib.qpr_test_step(self.ptr, reference, voltage, current, bus)
    def track(self, reference=0., voltage=0., current=0., correction=0.):
        return self.lib.qpr_test_track(self.ptr, reference, voltage, current, correction)
    def error(self, error): return self.lib.qpr_test_error(self.ptr, error)
    def voltage_error(self, error): return self.lib.qpr_test_voltage_error(self.ptr, error)
    def snapshot(self):
        f, u = (ct.c_float * 20)(), (ct.c_uint * 4)()
        self.lib.qpr_test_snapshot(self.ptr, f, u)
        return list(f), list(u)


def load_library(path):
    library = ct.CDLL(str(path))
    specs = {
        "create": (ct.c_void_p, [ct.c_float]), "destroy": (None, [ct.c_void_p]),
        "reset": (None, [ct.c_void_p]),
        "step": (ct.c_float, [ct.c_void_p] + [ct.c_float] * 4),
        "track": (ct.c_uint, [ct.c_void_p] + [ct.c_float] * 4),
        "snapshot": (None, [ct.c_void_p, ct.POINTER(ct.c_float), ct.POINTER(ct.c_uint)]),
        "error": (ct.c_float, [ct.c_void_p, ct.c_float]),
        "voltage_error": (ct.c_float, [ct.c_void_p, ct.c_float]),
        "corrupt_state": (None, [ct.c_void_p, ct.c_float]),
        "null_api": (ct.c_int, []),
    }
    for name, (result, arguments) in specs.items():
        function = getattr(library, "qpr_test_" + name)
        function.restype, function.argtypes = result, arguments
    inv_specs = {
        "create": (ct.c_void_p, [ct.c_float]), "destroy": (None, [ct.c_void_p]),
        "reset": (None, [ct.c_void_p]),
        "step": (ct.c_float, [ct.c_void_p] + [ct.c_float] * 5 + [ct.c_uint] * 2),
        "snapshot": (None, [ct.c_void_p, ct.POINTER(ct.c_float), ct.POINTER(ct.c_uint)]),
        "commands": (None, [ct.POINTER(ct.c_float), ct.POINTER(ct.c_uint)]),
        "set_rms": (ct.c_uint, [ct.c_float]), "adjust_rms": (ct.c_uint, [ct.c_float]),
        "set_mode": (None, [ct.c_uint]), "set_enabled": (None, [ct.c_uint]),
        "null_api": (ct.c_int, []),
    }
    for name, (result, arguments) in inv_specs.items():
        function = getattr(library, "inv_test_" + name)
        function.restype, function.argtypes = result, arguments
    return library


class InverterHost:
    FLOAT_NAMES = ("rms", "reference", "bridge", "modulation", "last_correction",
                   "handover_correction", "handover_step", "qpr_raw", "current_reference",
                   "qpr_reference", "qpr_modulation", "ts")
    UINT_NAMES = ("handover_left", "saturation", "fault", "closed", "mode_initialized",
                  "limited", "qpr_saturation", "qpr_modulation_saturation", "qpr_fault", "initialized")

    def __init__(self, library, fs=20000.):
        self.lib, self.ptr = library, library.inv_test_create(fs)
        assert self.ptr
    def __enter__(self): return self
    def __exit__(self, *_):
        self.lib.inv_test_destroy(self.ptr)
        self.ptr = None
    def reset(self): self.lib.inv_test_reset(self.ptr)
    def step(self, sine=0., rms=32., voltage=0., current=0., bus=48., closed=1, enabled=1):
        return self.lib.inv_test_step(self.ptr, sine, rms, voltage, current, bus, closed, enabled)
    def snapshot(self):
        f, u = (ct.c_float * 12)(), (ct.c_uint * 10)()
        self.lib.inv_test_snapshot(self.ptr, f, u)
        return dict(zip(self.FLOAT_NAMES, f)) | dict(zip(self.UINT_NAMES, u))


def qpr_accuracy(library, model):
    ts, w0 = model["parameters"]["Ts"], model["w0"]
    count, tail = round(10 / ts), round(1 / ts)
    signals = {
        "50_hz": lambda t: math.sin(w0 * t),
        "49_hz": lambda t: math.sin(2 * math.pi * 49 * t),
        "51_hz": lambda t: math.sin(2 * math.pi * 51 * t),
        "dc": lambda t: 1.,
        "step_at_9_5_s": lambda t: float(t >= 9.5),
        "multitone": lambda t: math.sin(w0*t) + .3*math.sin(3*w0*t)
                               + .2*math.sin(2*math.pi*3*t) + .1,
    }
    results = {}
    for name, signal in signals.items():
        reference = OriginalQPR(model["coefficients"])
        direct_float = OriginalQPR(model["coefficients"], single=True)
        sq = sq_direct = sq_ref = max_error = 0.
        with Host(library, 1/ts) as host:
            for n in range(count):
                e = F32(signal(n * ts))
                expected, actual, direct = reference.step(e), host.error(e), direct_float.step(e)
                if n >= count-tail:
                    sq += (actual-expected)**2
                    sq_direct += (direct-expected)**2
                    sq_ref += expected**2
                    max_error = max(max_error, abs(actual-expected))
            assert host.snapshot()[1][2] == 0
        result = dict(rms_error_v=math.sqrt(sq/tail), max_error_v=max_error,
                      direct_float_rms_error_v=math.sqrt(sq_direct/tail),
                      reference_rms_v=math.sqrt(sq_ref/tail))
        assert result["rms_error_v"] < 1e-4, (name, result)
        results[name] = result
    return results


def virtual_accuracy(library, model):
    ts = model["parameters"]["Ts"]
    lv, rv = model["virtual_l_h"], model["virtual_r_ohm"]
    a, b = (2*lv-rv*ts)/(2*lv+rv*ts), ts/(2*lv+rv*ts)
    results = {}
    for name, signal, limit in [("50_hz", lambda t: math.sin(model["w0"]*t), 1e-5),
                                ("dc", lambda t: 1., .02)]:
        previous = state = sq = peak = 0.
        count, tail = round(2/ts), round(.2/ts)
        with Host(library, 1/ts) as host:
            for n in range(count):
                error = F32(signal(n*ts))
                state = a*state+b*(error+previous)
                previous = error
                actual = host.voltage_error(error)
                if n >= count-tail:
                    sq += (actual-state)**2
                    peak = max(peak, abs(actual-state))
            assert host.snapshot()[1][2] == 0
        results[name] = dict(rms_error_a=math.sqrt(sq/tail), max_error_a=peak,
                             final_reference_a=state, final_actual_a=actual)
        assert peak < limit, results[name]
    return dict(a=a, b=b, cases=results)


def behavior(library, model):
    fs = 1/model["parameters"]["Ts"]
    assert library.qpr_test_null_api() == 1
    for invalid in [0., -1., 100., float("nan"), float("inf"), -float("inf")]:
        with Host(library, invalid) as host:
            assert host.snapshot()[1][2:] == [1, 0]
            assert host.step(reference=30.) == 0.
            host.reset()
            assert host.snapshot()[1][2] == 1
    fault_cases = 0
    for field in ("reference", "voltage", "current", "bus"):
        for value in [float("nan"), float("inf"), -float("inf")]:
            with Host(library, fs) as host:
                assert host.step(**{field: value}) == 0.
                assert host.snapshot()[1][2] == 2
                assert host.step(current=-1.) == 0.  # Fault is latched.
                host.reset()
                assert host.step(current=-1.) > 0.
                fault_cases += 1
    for field, value in [("bus", 0.), ("bus", -48.), ("bus", .99)]:
        with Host(library, fs) as host:
            assert host.step(**{field: value}) == 0.
            assert host.snapshot()[1][2] == 2
            fault_cases += 1
    for value in [float("nan"), float("inf")]:
        with Host(library, fs) as host:
            library.qpr_test_corrupt_state(host.ptr, value)
            assert host.step() == 0. and host.snapshot()[1][2] == 4
    with Host(library, fs) as host:
        assert host.step(current=-F32(3.4028234663852886e38)) == 0.
        assert host.snapshot()[1][2] == 4
    with Host(library, fs) as host:
        original_coefficients = host.snapshot()[0][:7]
        host.step(current=-100.)
        host.reset()
        f, u = host.snapshot()
        assert f[:7] == original_coefficients and all(v == 0. for v in f[7:])
        assert u == [0, 0, 0, 1]
    reference = OriginalQPR(model["coefficients"])
    max_raw_difference = 0.
    with Host(library, fs) as host:
        for n in range(round(2*fs)):
            e = 100. if n < round(.1*fs) else 0.
            expected, actual = reference.step(e), host.error(e)
            max_raw_difference = max(max_raw_difference, abs(actual-expected))
            f, u = host.snapshot()
            assert abs(f[13]) <= 48. and abs(f[15]) <= F32(.96)
            if n == 0:
                assert f[12] > 48. and f[13] == 48. and f[15] == F32(.96)
        assert u[0] > 0 and u[1] > 0 and u[2] == 0
        assert abs(actual) < .01 and max_raw_difference < .005
    with Host(library, fs) as host:
        host.error(-100.)
        f, _ = host.snapshot()
        assert f[13] == -48. and f[15] == -F32(.96)
    track_max_error = 0.
    for reference in (-40., 0., 40.):
        for voltage in (-25., 0., 25.):
            for current in (-1.8, 0., 1.8):
                for correction in (-20., 0., 20.):
                    with Host(library, fs) as host:
                        host.error(3.)  # Dirty prior history must be replaced by Track.
                        assert host.track(reference, voltage, current, correction) == 1
                        host.step(reference, voltage, current)
                        raw = host.snapshot()[0][12]
                        track_max_error = max(track_max_error, abs(raw-correction))
                        assert abs(raw-correction) < 1e-4, (reference, voltage, current, correction, raw)
    return dict(invalid_input_cases=fault_cases, null_api_passed=True,
                invalid_initialization_passed=True, numeric_faults_passed=True,
                latched_fault_and_reset_passed=True, reset_preserves_coefficients=True,
                track_preload_cases=81, track_max_correction_error_v=track_max_error,
                saturation_and_recovery_passed=True,
                saturated_sequence_max_raw_error_v=max_raw_difference)


def inverter_behavior(library, model):
    fs, ts = 1/model["parameters"]["Ts"], model["parameters"]["Ts"]
    assert library.inv_test_null_api() == 1
    for invalid in (0., -1., 100., float("nan"), float("inf")):
        with InverterHost(library, invalid) as host:
            assert host.step(sine=1.) == 0.
            assert host.snapshot()["fault"] != 0
    fault_cases = 0
    for closed in (0, 1):
        for field in ("sine", "rms", "voltage", "current", "bus"):
            for value in (float("nan"), float("inf"), -float("inf")):
                with InverterHost(library, fs) as host:
                    assert host.step(closed=closed, **{field: value}) == 0.
                    assert host.snapshot()["fault"] != 0
                    assert host.step(sine=1., closed=closed) == 0.
                    host.reset()
                    assert host.step(sine=1., closed=closed) > 0.
                    fault_cases += 1
        for field, value in (("bus", 0.), ("bus", -.1), ("sine", 1.01), ("sine", -1.01)):
            with InverterHost(library, fs) as host:
                assert host.step(closed=closed, **{field: value}) == 0.
                assert host.snapshot()["fault"] != 0
                fault_cases += 1
    ramps, feedback_independence_cases = {}, 0
    for closed in (0, 1):
        with InverterHost(library, fs) as host:
            previous = 0.
            ramp_error, maximum_delta = 0., 0.
            for n in range(round(.501*fs)):
                host.step(sine=0., closed=closed)
                state = host.snapshot()
                maximum_delta = max(maximum_delta, abs(state["rms"]-previous))
                ramp_error = max(ramp_error, abs(state["rms"]-min(32., (n+1)*64.*ts)))
                assert 0. <= state["rms"] <= 32. and state["fault"] == 0
                previous = state["rms"]
            assert state["rms"] == 32. and maximum_delta < 64.*ts+2e-6
            assert ramp_error < .005
            for n in range(round(.251*fs)):
                host.step(rms=16., closed=closed)
            assert host.snapshot()["rms"] == 16.
            for n in range(round(.251*fs)):
                host.step(rms=0., closed=closed)
            assert host.snapshot()["rms"] == 0.
            host.step(sine=.25, closed=closed)
            assert host.step(sine=.5, closed=closed, enabled=0) == 0.
            state = host.snapshot()
            assert state["rms"] == state["reference"] == state["modulation"] == 0.
            assert state["mode_initialized"] == 0
            host.step(sine=.75, closed=closed)
            state = host.snapshot()
            assert abs(state["rms"]-64.*ts) < 1e-8
            assert abs(state["reference"]-math.sqrt(2)*state["rms"]*.75) < 1e-7
            ramps[str(closed)] = dict(maximum_rms_step_v=maximum_delta,
                                      maximum_ramp_quantization_error_v=ramp_error,
                                      startup_s=.5, adjustment_32_to_16_s=.25,
                                      zero_enable_and_restart_passed=True)
    # A fresh open loop must ignore finite feedback, even when it is far from target.
    with InverterHost(library, fs) as clean, InverterHost(library, fs) as disturbed:
        for n in range(round(.6*fs)):
            sine = F32(math.sin(model["w0"]*n*ts))
            first = clean.step(sine=sine, closed=0)
            second = disturbed.step(sine=sine, closed=0, voltage=100., current=-100.)
            assert first == second
            state = clean.snapshot()
            assert abs(first-state["reference"]/48.) < 1e-7
            assert abs(state["reference"]-math.sqrt(2.)*state["rms"]*sine) < 5e-6
            feedback_independence_cases += 1
    # Request limits, low bus dynamic headroom, and negative direct commands.
    limit_cases = []
    for requested, bus in ((99., 48.), (-1., 48.), (32., 24.)):
        with InverterHost(library, fs) as host:
            for _ in range(round(.51*fs)):
                modulation = host.step(sine=1., rms=requested, bus=bus, closed=0)
            state = host.snapshot()
            expected = min(max(requested, 0.), 32., .96*bus/math.sqrt(2.))
            assert abs(state["rms"]-expected) < 3e-6 and state["limited"] == 1
            assert abs(modulation) <= F32(.96) and state["fault"] == 0
            limit_cases.append(dict(requested_rms_v=requested, bus_v=bus, applied_rms_v=state["rms"]))
    with InverterHost(library, fs) as host:
        for _ in range(round(.51*fs)):
            host.step(sine=1., closed=0)
        host.step(sine=1., closed=0, bus=24.)
        state = host.snapshot()
        assert state["rms"] <= .96*24/math.sqrt(2.)+3e-6
    return dict(null_api_passed=True, invalid_initialization_passed=True,
                invalid_input_cases=fault_cases, latched_fault_and_reset_passed=True,
                ramps=ramps, open_loop_feedback_independence_samples=feedback_independence_cases,
                reference_limit_cases=limit_cases, bus_drop_clamp_passed=True)


def command_interfaces(library):
    def read():
        f, u = (ct.c_float * 2)(), (ct.c_uint * 2)()
        library.inv_test_commands(f, u)
        return list(f), list(u)
    assert read() == ([32., 48.], [1, 1])
    for value, expected in ((16., 16.), (100., 32.), (-10., 0.)):
        assert library.inv_test_set_rms(value) == 1
        assert read()[0][0] == expected
    assert library.inv_test_set_rms(12.) == 1
    assert library.inv_test_adjust_rms(.5) == 1 and read()[0][0] == 12.5
    assert library.inv_test_adjust_rms(-.5) == 1 and read()[0][0] == 12.
    assert library.inv_test_adjust_rms(100.) == 1 and read()[0][0] == 32.
    assert library.inv_test_adjust_rms(-100.) == 1 and read()[0][0] == 0.
    for value in (float("nan"), float("inf"), -float("inf")):
        before = read()
        assert library.inv_test_set_rms(value) == 0 and read() == before
        assert library.inv_test_adjust_rms(value) == 0 and read() == before
    library.inv_test_set_mode(0)
    assert read()[1] == [0, 1]
    library.inv_test_set_enabled(0)
    assert read()[1] == [0, 0]
    library.inv_test_set_mode(5)
    assert read()[1] == [1, 0]
    library.inv_test_set_enabled(5)
    assert read()[1] == [1, 1]
    library.inv_test_set_rms(32.)
    return dict(default_rms_v=32., default_bus_v=48., default_closed_loop=1,
                default_output_enabled=1, setter_and_adjuster_passed=True,
                finite_bounds_v=[0., 32.], nonfinite_rejected_without_change=True,
                mode_and_output_commands_independent=True)


def mode_handover(library, model):
    fs, ts = 1/model["parameters"]["Ts"], model["parameters"]["Ts"]
    results = []
    for phase in (0., math.pi/4, math.pi/2, 3*math.pi/4, math.pi, 3*math.pi/2):
        sine = F32(math.sin(phase))
        with InverterHost(library, fs) as host:
            # Ramp to a modest setpoint in open loop. Apply realistic finite feedback.
            for _ in range(round(.2*fs)):
                host.step(sine=sine, rms=8., closed=0)
            old = host.snapshot()
            modulation = host.step(sine=sine, rms=8., voltage=old["reference"], current=.25, closed=1)
            first_closed = host.snapshot()
            assert abs(modulation-old["modulation"]) < 2e-7
            assert first_closed["rms"] == old["rms"] and first_closed["reference"] == old["reference"]
            # One feedback correction produces a nonzero term to retire on exit.
            host.step(sine=sine, rms=8., voltage=old["reference"]-.5, current=.25, closed=1)
            previous = host.snapshot()
            host.step(sine=sine, rms=8., voltage=old["reference"], current=.25, closed=0)
            first_open = host.snapshot()
            assert abs(first_open["modulation"]-previous["modulation"]) < 2e-7
            assert first_open["rms"] == previous["rms"] and first_open["reference"] == previous["reference"]
            correction = first_open["last_correction"]
            # Reverse the request while the 20 ms handover still has a nonzero tail.
            host.step(sine=sine, rms=8., voltage=old["reference"], current=.25, closed=1)
            reversed_handover = host.snapshot()
            assert abs(reversed_handover["modulation"]-first_open["modulation"]) < 2e-7
            assert reversed_handover["handover_left"] == 0
            host.step(sine=sine, rms=8., voltage=old["reference"], current=.25, closed=0)
            for _ in range(round(.021*fs)):
                host.step(sine=sine, rms=8., voltage=100., current=-100., closed=0)
            settled = host.snapshot()
            assert settled["handover_left"] == 0 and settled["handover_correction"] == 0.
            assert abs(settled["bridge"]-settled["reference"]) < 1e-7
            # Toggle every sample, including during the retirement interval.
            maximum_switch_jump = 0.
            for n in range(200):
                before = host.snapshot()
                closed = n % 2
                host.step(sine=sine, rms=8., voltage=old["reference"]-.5, current=.25, closed=closed)
                after = host.snapshot()
                assert after["fault"] == 0 and abs(after["modulation"]) <= F32(.96)
                assert after["rms"] == before["rms"] and after["reference"] == before["reference"]
                maximum_switch_jump = max(maximum_switch_jump, abs(after["modulation"]-before["modulation"]))
                if closed != before["closed"]:
                    assert abs(after["modulation"]-before["modulation"]) < 2e-6
            results.append(dict(phase_deg=math.degrees(phase), initial_exit_correction_v=correction,
                                repeated_toggle_max_modulation_change=maximum_switch_jump,
                                retirement_time_s=.02, reversed_during_handover_passed=True,
                                repeated_toggle_samples=200))
    # Vary target and external phase while switching: mode changes must not restart either.
    with InverterHost(library, fs) as switched, InverterHost(library, fs) as open_only:
        for n in range(round(.6*fs)):
            sine = F32(math.sin(model["w0"]*n*ts + .37))
            target = 32. if n < round(.3*fs) else 8.
            open_only.step(sine=sine, rms=target, closed=0)
            expected = open_only.snapshot()
            switched.step(sine=sine, rms=target, voltage=expected["reference"],
                          current=expected["reference"]/25., closed=(n//137)%2)
            state = switched.snapshot()
            assert state["rms"] == expected["rms"] and state["reference"] == expected["reference"]
            assert state["fault"] == 0 and abs(state["modulation"]) <= F32(.96)
    return dict(fixed_phase_cases=results, simultaneous_ramp_and_phase_samples=round(.6*fs),
                simultaneous_ramp_and_phase_passed=True)


def average_lc(library, model, resistance, inductance=None, mode="closed"):
    """Ideal differential LC plant, exact ZOH and one whole sample of delay."""
    ts, omega = model["parameters"]["Ts"], model["w0"]
    inductance = model["physical_l_h"] if inductance is None else inductance
    capacitance = model["physical_c_f"]
    h = -1/(2*resistance*capacitance)
    wd = math.sqrt(1/(inductance*capacitance)-h*h)
    multiplier = math.exp(h*ts)
    cosine, sine = math.cos(wd*ts), math.sin(wd*ts)/wd
    a00, a01 = multiplier*(cosine-sine*h), -multiplier*sine/inductance
    a10, a11 = multiplier*sine/capacitance, multiplier*(cosine+sine*h)
    b0, b1 = (1-a00)/resistance-a01, 1-a11-a10/resistance
    current = voltage = delayed_voltage = 0.
    max_i = max_v = max_qpr = square = error_square = vsin = vcos = 0.
    maximum_modulation = maximum_switch_change = previous_modulation = 0.
    previous_mode, transitions = None, []
    count, tail, over_60_v_at = round(2/ts), round(.2/ts), None
    with InverterHost(library, 1/ts) as host:
        for n in range(count):
            sine_reference = math.sin(omega*n*ts)
            if mode == "open":
                closed = 0
            elif mode == "switch":
                closed = int(not (20137 <= n < 21111 or 23029 <= n < 23037))
            elif mode == "rapid":
                closed = (n % 2) if 20137 <= n < 20337 else 1
            else:
                closed = 1
            modulation = host.step(sine=sine_reference, rms=32., voltage=voltage,
                                   current=current, bus=48., closed=closed)
            state = host.snapshot()
            assert state["fault"] == 0
            if previous_mode is not None and closed != previous_mode:
                change = abs(modulation-previous_modulation)
                maximum_switch_change = max(maximum_switch_change, change)
                if len(transitions) < 8:
                    transitions.append(dict(time_s=n*ts, closed=closed, modulation_change=change))
            previous_mode, previous_modulation = closed, modulation
            maximum_modulation = max(maximum_modulation, abs(modulation))
            max_i, max_v = max(max_i, abs(current)), max(max_v, abs(voltage))
            max_qpr = max(max_qpr, abs(state["qpr_raw"]))
            if over_60_v_at is None and abs(voltage) > 60.:
                over_60_v_at = n*ts
            if n >= count-tail:
                square += voltage*voltage
                error_square += (state["reference"]-voltage)**2
                vsin += voltage*sine_reference
                vcos += voltage*math.cos(omega*n*ts)
            current, voltage = (a00*current+a01*voltage+b0*delayed_voltage,
                                a10*current+a11*voltage+b1*delayed_voltage)
            delayed_voltage = 48.*modulation
    amplitude = 2*math.hypot(vsin, vcos)/tail
    result = dict(load_ohm=resistance, differential_inductance_h=inductance,
                  capacitance_f=capacitance, duration_s=2., pwm_delay_samples=1,
                  mode=mode, dc_bus_v=48., target_rms_v=32., target_peak_v=32.*math.sqrt(2.), max_current_a=max_i,
                  max_voltage_v=max_v, max_qpr_raw_v=max_qpr,
                  last_0_2_s_voltage_rms_v=math.sqrt(square/tail),
                  last_0_2_s_error_rms_v=math.sqrt(error_square/tail),
                  fundamental_peak_v=amplitude,
                  fundamental_phase_deg=math.degrees(math.atan2(vcos, vsin)),
                  nonfundamental_rms_v=math.sqrt(max(0., square/tail-amplitude*amplitude/2)),
                  maximum_modulation=maximum_modulation,
                  maximum_switch_sample_modulation_change=maximum_switch_change,
                  first_transitions=transitions,
                  modulation_saturation_count=state["saturation"], over_60_v_at_s=over_60_v_at)
    if resistance == 25. or resistance == model["model_load_ohm"]:
        assert max_v < 47. and max_i < 2. and state["saturation"] == 0, result
        assert abs(result["last_0_2_s_voltage_rms_v"]-32.) < .15, result
        assert result["nonfundamental_rms_v"] < .1, result
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", type=Path, default=ROOT.parent.parent/"MATLAB-study"/"singlephase.slx")
    parser.add_argument("--compiler", default=shutil.which("gcc") or "gcc")
    parser.add_argument("--output", type=Path, default=ROOT/"Tests"/"results"/"qpr_verification.json")
    arguments = parser.parse_args()
    model = read_model(arguments.model.resolve())
    assert model["parameters"]["Upper"] == 48. and model["parameters"]["Lower"] == -48.
    build_directory = Path(tempfile.mkdtemp(prefix="dcac_qpr_host_"))
    library_path = build_directory/("qpr_host.dll" if os.name == "nt" else "qpr_host.so")
    command = [arguments.compiler, "-std=c99", "-O2", "-Wall", "-Wextra", "-Werror",
               "-fno-fast-math", "-ffp-contract=off", "-shared"]
    if os.name != "nt": command.append("-fPIC")
    command += ["-I", str(ROOT/"Core"/"Inc"), str(ROOT/"Core"/"Src"/"qpr_control.c"),
                str(ROOT/"Core"/"Src"/"inverter_control.c"),
                str(ROOT/"Tests"/"qpr_host_bridge.c"), "-o", str(library_path)]
    subprocess.run(command, check=True, capture_output=True, text=True)
    library = load_library(library_path)
    result = dict(timestamp_utc=dt.datetime.now(dt.timezone.utc).isoformat(),
                  scope="Host GCC executes actual qpr_control.c and inverter_control.c. SLX numeric extraction and ideal averaged LC checks; not Simulink execution or hardware validation.",
                  compiler=subprocess.check_output([arguments.compiler, "--version"], text=True).splitlines()[0],
                  compiler_options=command[1:9],
                  source_sha256=hashlib.sha256((ROOT/"Core"/"Src"/"qpr_control.c").read_bytes()).hexdigest(),
                  header_sha256=hashlib.sha256((ROOT/"Core"/"Inc"/"qpr_control.h").read_bytes()).hexdigest(),
                  inverter_source_sha256=hashlib.sha256((ROOT/"Core"/"Src"/"inverter_control.c").read_bytes()).hexdigest(),
                  inverter_header_sha256=hashlib.sha256((ROOT/"Core"/"Inc"/"inverter_control.h").read_bytes()).hexdigest(),
                  test_source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                  bridge_source_sha256=hashlib.sha256((ROOT/"Tests"/"qpr_host_bridge.c").read_bytes()).hexdigest(),
                  model=model)
    try:
        result["qpr_accuracy"] = qpr_accuracy(library, model)
        print("QPR comparison passed.", flush=True)
        result["virtual_impedance"] = virtual_accuracy(library, model)
        result["behavior"] = behavior(library, model)
        result["inverter_behavior"] = inverter_behavior(library, model)
        result["command_interfaces"] = command_interfaces(library)
        result["mode_handover"] = mode_handover(library, model)
        print("Virtual impedance, faults, limits, RMS ramp, command APIs and mode handover passed.", flush=True)
        result["average_lc"] = [average_lc(library, model, 25., inductance=l, mode=mode)
                                for l in [220e-6, 440e-6]
                                for mode in ["closed", "open", "switch", "rapid"]]
        result["model_load_comparisons"] = [average_lc(library, model, r)
                                             for r in [model["model_load_ohm"], 500.]]
        result["known_limitation"] = (
            "Original gains oscillate with a 500-ohm light load in this ideal LC model "
            "with one-sample delay. Passing numeric tests does not establish hardware stability.")
        result["checks_passed"] = True
        arguments.output.parent.mkdir(parents=True, exist_ok=True)
        arguments.output.write_text(json.dumps(result, indent=2, ensure_ascii=False)+"\n", encoding="utf-8")
        print(json.dumps({"result": str(arguments.output.resolve()), "average_lc": result["average_lc"]}, indent=2))
    finally:
        if os.name == "nt":
            import _ctypes
            _ctypes.FreeLibrary(library._handle)
        else:
            import _ctypes
            _ctypes.dlclose(library._handle)
        shutil.rmtree(build_directory)


if __name__ == "__main__":
    main()
