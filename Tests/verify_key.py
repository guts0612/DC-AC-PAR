"""Host checks for the real Hardware/key.c (requires GCC; no target hardware)."""
from pathlib import Path
import shutil
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
STUB = r"""
#ifndef KEY_HOST_STUB_H
#define KEY_HOST_STUB_H
#define __MAIN_H
#include <stdint.h>
#define GPIOE ((void *)0x1)
#define GPIOB ((void *)0x2)
#define GPIO_PIN_9 512U
#define GPIO_PIN_0 1U
#define GPIO_PIN_SET 1U
#define GPIO_PIN_RESET 0U
uint32_t HAL_GetTick(void);
uint32_t HAL_GPIO_ReadPin(void *port, uint16_t pin);
uint32_t __get_PRIMASK(void);
void __disable_irq(void);
void __set_PRIMASK(uint32_t mask);
#endif
"""
HARNESS = r"""
#include <assert.h>
#include <stdio.h>
#include "key.h"

static uint32_t tick, irq_mask, reads;
static uint32_t level = GPIO_PIN_SET, level_b = GPIO_PIN_SET;
uint32_t HAL_GetTick(void) { return tick; }
uint32_t HAL_GPIO_ReadPin(void *port, uint16_t pin)
{
    if (port == GPIOB) { assert(pin == GPIO_PIN_9); ++reads; return level_b; }
    assert(port == GPIOE && pin == GPIO_PIN_0);
    ++reads;
    return level;
}
uint32_t __get_PRIMASK(void) { return irq_mask; }
void __disable_irq(void) { irq_mask = 1U; }
void __set_PRIMASK(uint32_t mask) { irq_mask = mask; }

static void scan(uint32_t pressed, uint32_t milliseconds)
{
    level = pressed ? GPIO_PIN_RESET : GPIO_PIN_SET;
    while (milliseconds-- != 0U) { ++tick; Key_Loop(); }
}

int main(void)
{
    unsigned int i;
    uint8_t closed_loop = 1U;

    /* SysTick can fire before GPIO initialization. */
    Key_Loop();
    assert(reads == 0U && Key() == 0U);
    Key_Init();
    scan(0U, 100U);
    assert(Key() == 0U);

    /* A short pulse and contact bounce must not create a click. */
    scan(1U, 10U);
    scan(0U, 30U);
    assert(Key() == 0U);
    for (i = 0U; i < 8U; ++i) { scan(i & 1U, 3U); }
    scan(0U, 30U);
    assert(Key() == 0U);

    /* Long hold, then bouncing release: one event only after 20 ms. */
    scan(1U, 1000U);
    assert(Key() == 0U);
    for (i = 0U; i < 6U; ++i) { scan(i & 1U, 3U); }
    scan(0U, 20U);
    assert(Key() == 0U);
    scan(0U, 1U);
    assert(Key() == 1U);
    assert(Key() == 0U);
    scan(0U, 100U);
    assert(Key() == 0U);

    /* Repeat real clicks using the same mode command as main.c. */
    for (i = 0U; i < 20U; ++i)
    {
        scan(1U, 30U);
        scan(0U, 30U);
        assert(Key() == 1U);
        closed_loop = (closed_loop == 0U) ? 1U : 0U;
        assert(closed_loop == ((i & 1U) ? 1U : 0U));
        assert(Key() == 0U);
    }

    /* A button held at startup must not cause a release-only click. */
    level = GPIO_PIN_RESET;
    Key_Init();
    scan(1U, 50U);
    scan(0U, 50U);
    assert(Key() == 0U);
    scan(1U, 30U);
    scan(0U, 30U);
    assert(Key() == 1U);

    /* The millisecond clock can wrap during press or release debounce. */
    tick = UINT32_MAX - 10U;
    Key_Init();
    scan(1U, 30U);
    scan(0U, 30U);
    assert(Key() == 1U);
    scan(1U, 30U);
    tick = UINT32_MAX - 10U;
    scan(0U, 30U);
    assert(Key() == 1U);

    /* Event consumption and initialization preserve interrupt masking. */
    irq_mask = 1U;
    (void)Key();
    assert(irq_mask == 1U);
    Key_Init();
    assert(irq_mask == 1U);
    irq_mask = 0U;
    (void)Key();
    assert(irq_mask == 0U);
    /* PB9 short pulse, hold and bouncing release; no PE0 event. */
    level_b = GPIO_PIN_RESET; scan(0U, 10U);
    level_b = GPIO_PIN_SET; scan(0U, 30U); assert(Key() == 0U);
    level_b = GPIO_PIN_RESET; scan(0U, 1000U); assert(Key() == 0U);
    level_b = GPIO_PIN_SET; scan(0U, 5U);
    level_b = GPIO_PIN_RESET; scan(0U, 5U);
    level_b = GPIO_PIN_SET; scan(0U, 30U);
    assert(Key() == 2U && Key() == 0U);
    /* Both keys released together retain both events. */
    level_b = GPIO_PIN_RESET; scan(1U, 30U);
    level_b = GPIO_PIN_SET; scan(0U, 30U);
    assert(Key() == 1U && Key() == 2U && Key() == 0U);
    /* PB9 held during init must not generate a release-only event. */
    level_b = GPIO_PIN_RESET; Key_Init(); scan(0U, 50U);
    level_b = GPIO_PIN_SET; scan(0U, 50U); assert(Key() == 0U);
    tick = UINT32_MAX - 10U;
    level_b = GPIO_PIN_RESET; scan(0U, 30U);
    level_b = GPIO_PIN_SET; scan(0U, 30U); assert(Key() == 2U);
    puts("PASS: PB9, simultaneous keys, PE0, startup, bounce, hold, one-shot release, repeated clicks, tick wrap, IRQ mask");
    return 0;
}
"""


def main():
    compiler = shutil.which("gcc")
    if compiler is None:
        raise SystemExit("GCC was not found on PATH")
    with tempfile.TemporaryDirectory(prefix="dcac_key_") as directory:
        work = Path(directory)
        stub = work / "hal_stub.h"
        harness = work / "verify_key.c"
        executable = work / "verify_key.exe"
        stub.write_text(STUB, encoding="utf-8")
        harness.write_text(HARNESS, encoding="utf-8")
        subprocess.run([
            compiler, "-std=c99", "-Wall", "-Wextra", "-Werror", "-O2",
            "-include", str(stub), "-I", str(ROOT / "Hardware"),
            "-I", str(ROOT / "Core/Inc"), str(ROOT / "Hardware/key.c"),
            str(harness), "-o", str(executable),
        ], check=True)
        subprocess.run([str(executable)], check=True)


if __name__ == "__main__":
    main()
