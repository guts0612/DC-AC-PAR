"""Compile the new key module beside Hardware/key.c and compare real C behavior.

The baseline control-action and Slave_Stop bodies are extracted from main.c.
GPIO, time, IRQ masking, and the HRTIM output-disable register are host mocks.
The reserved PE2 key is tested separately from the two-key baseline.
This verifies software behavior, not physical switch/electrical behavior.
"""
from pathlib import Path
import hashlib
import json
import re
import shutil
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
RESULT = ROOT / "Tests/results/key_control_module_verification.json"

STUB = r"""
#ifndef KEY_MODULE_HOST_MAIN_H
#define KEY_MODULE_HOST_MAIN_H
#include <stdint.h>
#define GPIOE ((void *)0x1)
#define GPIOB ((void *)0x2)
#define GPIO_PIN_0 1U
#define GPIO_PIN_2 4U
#define GPIO_PIN_9 512U
#define GPIO_PIN_RESET 0U
#define GPIO_PIN_SET 1U
uint32_t HAL_GetTick(void);
uint32_t HAL_GPIO_ReadPin(void *port, uint16_t pin);
uint32_t __get_PRIMASK(void);
void __disable_irq(void);
void __set_PRIMASK(uint32_t mask);
#endif
"""

HARNESS = r"""
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include "key.h"
#include "key_control.h"

static uint32_t checks, scans, event_reads, process_calls, tick, irq_mask, reads;
static uint8_t physical_state;
#define CHECK(condition) do { ++checks; if (!(condition)) { \
    fprintf(stderr, "FAIL line %d: %s\n", __LINE__, #condition); exit(1); \
} } while (0)
uint32_t HAL_GetTick(void) { return tick; }
uint32_t HAL_GPIO_ReadPin(void *port, uint16_t pin)
{
    uint8_t bit;
    ++reads;
    if (port == GPIOE) {
        CHECK(pin == GPIO_PIN_0 || pin == GPIO_PIN_2);
        bit = pin == GPIO_PIN_0 ? 1U : 4U;
    }
    else { CHECK(port == GPIOB && pin == GPIO_PIN_9); bit = 2U; }
    return (physical_state & bit) ? GPIO_PIN_RESET : GPIO_PIN_SET;
}
uint32_t __get_PRIMASK(void) { return irq_mask; }
void __disable_irq(void) { irq_mask = 1U; }
void __set_PRIMASK(uint32_t mask) { irq_mask = mask; }

typedef struct { struct { uint32_t ODISR; } sCommonRegs; } MockHrtim;
static MockHrtim reference_hrtim, module_hrtim;
#define HRTIM1 (&reference_hrtim)
#define MODULE_HRTIM1 (&module_hrtim)
#define SLAVE_OUTPUTS 0x3C000U
static volatile uint8_t voltage_closed_loop_enable, slave_enable_request, slave_enabled;
static volatile uint32_t slave_fault_flags;
static volatile float slave_modulation;
static volatile uint8_t module_master_closed_loop, module_slave_enable_request, module_slave_enabled;
static volatile uint32_t module_slave_fault_flags;
static volatile float module_slave_modulation;
static uint32_t reference_stops, module_stops, reference_fault_arg, module_fault_arg;

__REFERENCE_STOP__
__MODULE_STOP__
static void Slave_Stop(uint32_t fault)
{
    ++reference_stops;
    reference_fault_arg = fault;
    reference_stop_inner(fault);
}
static void module_stop(uint32_t fault)
{
    ++module_stops;
    module_fault_arg = fault;
    module_stop_inner(fault);
}
static const KeyControl_Bindings bindings = {
    &module_master_closed_loop, &module_slave_enable_request,
    &module_slave_fault_flags, module_stop
};

static uint8_t reference_process(void)
{
    uint8_t key_number;
__MAIN_ACTIONS__
    return key_number;
}

static void initialize(uint8_t state)
{
    uint32_t mask = irq_mask;
    physical_state = state;
    Key_Init();
    CHECK(irq_mask == mask);
    KeyControl_Init();
    CHECK(irq_mask == mask);
    CHECK(Key_GetState() == (KeyControl_GetState() & 3U));
    CHECK(KeyControl_GetState() == state);
}
static void scan(uint8_t state, uint32_t duration)
{
    physical_state = state;
    CHECK(Key_GetState() == (state & 3U) && KeyControl_GetState() == state);
    while (duration-- != 0U)
    {
        ++tick;
        ++scans;
        Key_Loop();
        KeyControl_Tick1ms();
    }
}
static uint8_t get_event(void)
{
    uint32_t mask = irq_mask;
    uint8_t reference = Key();
    CHECK(irq_mask == mask);
    CHECK(KeyControl_GetEvent() == reference);
    CHECK(irq_mask == mask);
    ++event_reads;
    return reference;
}
static void compare_control(void)
{
    CHECK(module_master_closed_loop == voltage_closed_loop_enable);
    CHECK(module_slave_enable_request == slave_enable_request);
    CHECK(module_slave_fault_flags == slave_fault_flags);
    CHECK(module_slave_enabled == slave_enabled);
    CHECK(module_slave_modulation == slave_modulation);
    CHECK(module_hrtim.sCommonRegs.ODISR == reference_hrtim.sCommonRegs.ODISR);
    CHECK(module_stops == reference_stops);
    CHECK(module_fault_arg == reference_fault_arg);
}
static uint8_t process(void)
{
    uint32_t mask = irq_mask;
    uint8_t reference = reference_process();
    CHECK(KeyControl_Process(&bindings) == reference);
    CHECK(irq_mask == mask);
    compare_control();
    ++process_calls;
    return reference;
}
static void set_controls(uint8_t master, uint8_t request, uint32_t faults)
{
    voltage_closed_loop_enable = module_master_closed_loop = master;
    slave_enable_request = module_slave_enable_request = request;
    slave_fault_flags = module_slave_fault_flags = faults;
    slave_enabled = module_slave_enabled = 1U;
    slave_modulation = module_slave_modulation = 0.375f;
    reference_hrtim.sCommonRegs.ODISR = module_hrtim.sCommonRegs.ODISR = 0U;
    reference_stops = module_stops = reference_fault_arg = module_fault_arg = 0U;
}

static uint8_t get_module_event(void)
{
    uint32_t mask = irq_mask;
    uint8_t event = KeyControl_GetEvent();
    CHECK(irq_mask == mask);
    return event;
}

static void verify_reserved_pe2(void)
{
    uint32_t i, mask, state;

    for (mask = 0U; mask <= 1U; ++mask)
    {
        irq_mask = mask;
        initialize(0U);
        /* Short pulses and bouncing must not become key-three events. */
        scan(4U, 10U); scan(0U, 30U);
        CHECK(get_module_event() == KEYCONTROL_EVENT_NONE);
        for (i = 0U; i < 8U; ++i) scan((i & 1U) ? 4U : 0U, 3U);
        scan(0U, 30U);
        CHECK(get_module_event() == KEYCONTROL_EVENT_NONE);
        /* A stable press only arms a release event; a hold does not repeat. */
        scan(4U, 20U);
        CHECK(get_module_event() == KEYCONTROL_EVENT_NONE);
        scan(4U, 1001U);
        CHECK(get_module_event() == KEYCONTROL_EVENT_NONE);
        for (i = 0U; i < 6U; ++i) scan((i & 1U) ? 4U : 0U, 3U);
        scan(0U, 20U);
        CHECK(get_module_event() == KEYCONTROL_EVENT_NONE);
        scan(0U, 1U);
        CHECK(get_module_event() == KEYCONTROL_EVENT_PE2);
        CHECK(get_module_event() == KEYCONTROL_EVENT_NONE);
        CHECK(irq_mask == mask && Key() == 0U);

        /* Simultaneous release keeps all three events in existing priority order. */
        initialize(0U);
        scan(7U, 30U); scan(0U, 30U);
        CHECK(get_module_event() == KEYCONTROL_EVENT_PE0);
        CHECK(get_module_event() == KEYCONTROL_EVENT_PB9);
        CHECK(get_module_event() == KEYCONTROL_EVENT_PE2);
        CHECK(get_module_event() == KEYCONTROL_EVENT_NONE);
        for (i = 0U; i < 3U; ++i) { scan(4U, 30U); scan(0U, 30U); }
        CHECK(get_module_event() == KEYCONTROL_EVENT_PE2);
        CHECK(get_module_event() == KEYCONTROL_EVENT_NONE);
        scan(4U, 30U); scan(0U, 30U); initialize(0U);
        CHECK(get_module_event() == KEYCONTROL_EVENT_NONE);
    }
    irq_mask = 0U;

    /* Boot-held PE2, alone and with both original keys, must first be released. */
    for (state = 4U; state <= 7U; state += 3U)
    {
        initialize((uint8_t)state);
        scan((uint8_t)state, 50U); scan(0U, 50U);
        CHECK(get_module_event() == KEYCONTROL_EVENT_NONE);
        scan((uint8_t)state, 30U); scan(0U, 30U);
        if (state & 1U) CHECK(get_module_event() == KEYCONTROL_EVENT_PE0);
        if (state & 2U) CHECK(get_module_event() == KEYCONTROL_EVENT_PB9);
        CHECK(get_module_event() == KEYCONTROL_EVENT_PE2);
        CHECK(get_module_event() == KEYCONTROL_EVENT_NONE);
    }

    initialize(0U);
    tick = UINT32_MAX - 10U;
    scan(4U, 30U);
    tick = UINT32_MAX - 10U;
    scan(0U, 30U);
    CHECK(get_module_event() == KEYCONTROL_EVENT_PE2);
    CHECK(get_module_event() == KEYCONTROL_EVENT_NONE);

    /* Process reports PE2 without toggling the master, requesting or stopping a slave. */
    for (state = 0U; state < 4U; ++state)
    {
        initialize(0U);
        set_controls((uint8_t)(state & 1U), (uint8_t)((state >> 1) & 1U), 0x80000000U);
        scan(4U, 30U); scan(0U, 30U);
        CHECK(KeyControl_Process(0) == KEYCONTROL_EVENT_NONE);
        CHECK(KeyControl_Process(&bindings) == KEYCONTROL_EVENT_PE2);
        compare_control();
        CHECK(KeyControl_Process(&bindings) == KEYCONTROL_EVENT_NONE);
    }
}

int main(void)
{
    uint32_t i, master, request, faults, event, random = 0x167A4B23U;
    const uint32_t fault_values[] = {0U, 1U, 0x80000000U};

    /* SysTick may run before GPIO initialization: no HAL GPIO read permitted. */
    Key_Loop();
    KeyControl_Tick1ms();
    CHECK(reads == 0U);
    CHECK(get_event() == 0U);
    initialize(0U);
    scan(0U, 100U);
    CHECK(get_event() == 0U);

    /* Both keys: pulse/bounce, exact 20 ms threshold, hold and release. */
    for (event = 1U; event <= 2U; ++event)
    {
        initialize(0U);
        scan((uint8_t)event, 10U);
        scan(0U, 30U);
        CHECK(get_event() == 0U);
        for (i = 0U; i < 8U; ++i) scan((i & 1U) ? (uint8_t)event : 0U, 3U);
        scan(0U, 30U);
        CHECK(get_event() == 0U);
        scan((uint8_t)event, 1000U);
        CHECK(get_event() == 0U);
        for (i = 0U; i < 6U; ++i) scan((i & 1U) ? (uint8_t)event : 0U, 3U);
        scan(0U, 20U);
        CHECK(get_event() == 0U);
        scan(0U, 1U);
        CHECK(get_event() == event);
        CHECK(get_event() == 0U);
        scan(0U, 100U);
        CHECK(get_event() == 0U);
    }

    initialize(0U);
    scan(3U, 30U); scan(0U, 30U);
    CHECK(get_event() == 1U && get_event() == 2U && get_event() == 0U);
    /* Pending bits deliberately coalesce repeated presses of the same key. */
    for (i = 0U; i < 3U; ++i) { scan(3U, 30U); scan(0U, 30U); }
    CHECK(get_event() == 1U && get_event() == 2U && get_event() == 0U);
    /* Reinitialization discards pending events. */
    scan(3U, 30U); scan(0U, 30U); initialize(0U);
    CHECK(get_event() == 0U);

    for (event = 1U; event <= 3U; ++event)
    {
        initialize((uint8_t)event);
        scan((uint8_t)event, 50U); scan(0U, 50U);
        CHECK(get_event() == 0U);
        scan((uint8_t)event, 30U); scan(0U, 30U);
        if (event & 1U) CHECK(get_event() == 1U);
        if (event & 2U) CHECK(get_event() == 2U);
        CHECK(get_event() == 0U);
    }
    for (event = 1U; event <= 3U; ++event)
    {
        initialize(0U);
        tick = UINT32_MAX - 10U;
        scan((uint8_t)event, 30U); scan(0U, 30U);
        if (event & 1U) CHECK(get_event() == 1U);
        if (event & 2U) CHECK(get_event() == 2U);
        CHECK(get_event() == 0U);
        scan((uint8_t)event, 30U);
        tick = UINT32_MAX - 10U;
        scan(0U, 30U);
        if (event & 1U) CHECK(get_event() == 1U);
        if (event & 2U) CHECK(get_event() == 2U);
        CHECK(get_event() == 0U);
    }
    for (i = 0U; i <= 1U; ++i)
    {
        irq_mask = i;
        initialize(0U);
        scan(3U, 30U); scan(0U, 30U);
        CHECK(get_event() == 1U && get_event() == 2U && get_event() == 0U);
        CHECK(irq_mask == i);
    }
    irq_mask = 0U;

    /* Differential controls, including noncanonical nonzero uint8_t flags. */
    for (master = 0U; master <= 2U; ++master)
    for (request = 0U; request <= 2U; ++request)
    for (faults = 0U; faults < 3U; ++faults)
    for (event = 1U; event <= 3U; ++event)
    {
        initialize(0U);
        set_controls((uint8_t)master, (uint8_t)request, fault_values[faults]);
        scan((uint8_t)event, 30U); scan(0U, 30U);
        if (event & 1U) CHECK(process() == 1U);
        if (event & 2U) CHECK(process() == 2U);
        CHECK(process() == 0U);
    }
    /* Open-loop PB9 is consumed immediately, never queued until closed-loop. */
    initialize(0U); set_controls(0U, 0U, 0U);
    scan(2U, 30U); scan(0U, 30U);
    CHECK(process() == 2U && module_slave_enable_request == 0U);
    voltage_closed_loop_enable = module_master_closed_loop = 1U;
    CHECK(process() == 0U && module_slave_enable_request == 0U);

    /* Invalid bindings preserve queued events instead of partially acting. */
    scan(1U, 30U); scan(0U, 30U);
    CHECK(KeyControl_Process(0) == 0U);
    for (i = 0U; i < 4U; ++i)
    {
        KeyControl_Bindings invalid = bindings;
        if (i == 0U) invalid.master_closed_loop = 0;
        if (i == 1U) invalid.slave_enable_request = 0;
        if (i == 2U) invalid.slave_fault_flags = 0;
        if (i == 3U) invalid.stop_slave = 0;
        CHECK(KeyControl_Process(&invalid) == 0U);
    }
    CHECK(process() == 1U && process() == 0U);

    /* Seeded random bounces/holds and consumption interleavings. */
    initialize(0U); set_controls(1U, 0U, 0U);
    for (i = 0U; i < 12000U; ++i)
    {
        random = random * 1664525U + 1013904223U;
        scan((uint8_t)((random >> 18) & 3U), 1U + ((random >> 2) % 45U));
        if ((random & 3U) == 0U) (void)process();
        if ((random & 15U) == 1U) (void)get_event();
        if ((random & 1023U) == 3U)
        {
            irq_mask ^= 1U;
            initialize(physical_state);
        }
    }
    scan(0U, 30U);
    (void)process(); (void)process(); CHECK(process() == 0U);
    verify_reserved_pe2();
    printf("{\"status\":\"passed\",\"checks\":%u,\"scan_ticks\":%u,"
           "\"event_comparisons\":%u,\"control_action_comparisons\":%u,"
           "\"control_input_combinations\":81,\"random_segments\":12000,"
           "\"reserved_PE2_verified\":true}\n",
           (unsigned int)checks, (unsigned int)scans,
           (unsigned int)event_reads, (unsigned int)process_calls);
    return 0;
}
"""


def extract_function(source, marker):
    start = source.index(marker)
    brace = source.index("{", start)
    depth, end = 1, brace + 1
    while depth:
        depth += (source[end] == "{") - (source[end] == "}")
        end += 1
    return source[start:end]


def main():
    compiler = shutil.which("gcc")
    if compiler is None:
        raise SystemExit("GCC was not found on PATH")
    source = (ROOT / "Core/Src/main.c").read_text(encoding="utf-8-sig")
    start = source.index("    key_number = Key();")
    end = source.index("#if VOFA_OUTPUT_ENABLE", start)
    actions = source[start:end]
    original_stop = extract_function(source, "static void Slave_Stop(uint32_t fault)")
    reference_stop = original_stop.replace("Slave_Stop", "reference_stop_inner")
    replacements = {
        "Slave_Stop": "module_stop_inner",
        "HRTIM1": "MODULE_HRTIM1",
        "slave_enabled": "module_slave_enabled",
        "slave_enable_request": "module_slave_enable_request",
        "slave_fault_flags": "module_slave_fault_flags",
        "slave_modulation": "module_slave_modulation",
    }
    module_stop = re.sub(r"\b(?:" + "|".join(replacements) + r")\b",
                         lambda match: replacements[match.group()], original_stop)
    harness = HARNESS.replace("__REFERENCE_STOP__", reference_stop)
    harness = harness.replace("__MODULE_STOP__", module_stop).replace("__MAIN_ACTIONS__", actions)
    with tempfile.TemporaryDirectory(prefix="dcac_key_module_") as directory:
        work = Path(directory)
        (work / "main.h").write_text(STUB, encoding="utf-8")
        test_file = work / "verify_key_control.c"
        test_file.write_text(harness, encoding="utf-8")
        executable = work / "verify_key_control.exe"
        command = [compiler, "-std=c99", "-Wall", "-Wextra", "-Werror", "-O2",
                   "-I", str(work), "-I", str(ROOT), "-I", str(ROOT / "Hardware"),
                   str(ROOT / "Hardware/key.c"), str(ROOT / "key_control.c"),
                   str(test_file), "-o", str(executable)]
        subprocess.run(command, check=True, capture_output=True, text=True)
        output = subprocess.run([str(executable)], check=True, capture_output=True, text=True)
    result = json.loads(output.stdout)
    result["coverage"] = [
        "pre_init_scan", "20ms_press_release_debounce", "bounce_and_short_pulse",
        "long_hold_no_repeat", "PE0_priority_simultaneous_release",
        "pending_bit_coalescing", "boot_held_no_release_event", "reinitialize",
        "uint32_tick_wrap_press_and_release", "PRIMASK_restore_0_and_1",
        "master_open_closed_toggle", "slave_request_toggle_and_fault_gate",
        "open_loop_PB9_no_queue", "original_Slave_Stop_side_effects",
        "null_binding_preserves_events", "seeded_random_differential",
        "PE2_debounce_hold_and_release", "PE2_boot_held_and_tick_wrap",
        "three_key_priority_and_coalescing", "PE2_process_no_control_side_effects",
    ]
    result["compiler"] = compiler
    result["compiler_flags"] = "-std=c99 -Wall -Wextra -Werror -O2"
    result["software_only"] = True
    result["baseline_main_actions_sha256"] = hashlib.sha256(actions.encode()).hexdigest()
    result["baseline_slave_stop_sha256"] = hashlib.sha256(original_stop.encode()).hexdigest()
    result["source_sha256"] = {
        str(path.relative_to(ROOT)): hashlib.sha256(path.read_bytes()).hexdigest()
        for path in [ROOT / "key_control.c", ROOT / "key_control.h", ROOT / "Hardware/key.c"]
    }
    RESULT.parent.mkdir(parents=True, exist_ok=True)
    RESULT.write_text(json.dumps(result, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(json.dumps(result, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
