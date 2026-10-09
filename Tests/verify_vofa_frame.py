"""Exercise the actual VOFA sender from main.c with a host UART stub.

Checks seven-channel payload, JustFloat delimiter, frame length, memcpy bounds,
and success/error counters. Does not test the physical UART or VOFA UI.
"""
from pathlib import Path
import re
import shutil
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
source = (ROOT / 'Core/Src/main.c').read_text(encoding='utf-8-sig')
declaration = re.search(r'^static uint8_t vofa_frame\[[^\]]+\];', source, re.M)
timeout = re.search(r'^#define VOFA_UART_TIMEOUT_MS\s+\S+', source, re.M)
assert declaration and timeout
start = source.index('static void VOFA_SendJustFloat(')
brace = source.index('{', start)
depth = 1
end = brace + 1
while depth:
    depth += (source[end] == '{') - (source[end] == '}')
    end += 1
sender = source[start:end]

harness = r'''
#include <stdint.h>
#include <stdio.h>
#include <string.h>
typedef enum { HAL_OK=0, HAL_ERROR=1, HAL_BUSY=2, HAL_TIMEOUT=3 } HAL_StatusTypeDef;
static int huart4;
static uint32_t vofa_sent_count, vofa_tx_error_count;
static uint8_t captured[64];
static uint16_t captured_size;
static unsigned int bounds_errors, failures;
static HAL_StatusTypeDef next_status = HAL_OK;
__DECL__
__TIMEOUT__
static HAL_StatusTypeDef HAL_UART_Transmit(int *port, uint8_t *data,
                                         uint16_t size, uint32_t timeout)
{
    if (port != &huart4 || timeout != VOFA_UART_TIMEOUT_MS || size > sizeof(captured)) {
        failures++;
        return HAL_ERROR;
    }
    captured_size = size;
    memcpy(captured, data, size);
    return next_status;
}
/* Reject unsafe copies without corrupting the host process. The exact source
 * sender's requested offset and length still exercise the original bug. */
static void *checked_memcpy(void *dest, const void *src, size_t size)
{
    uintptr_t begin = (uintptr_t)vofa_frame, addr = (uintptr_t)dest;
    if (addr < begin || addr-begin > sizeof(vofa_frame) ||
        size > sizeof(vofa_frame)-(addr-begin)) {
        bounds_errors++;
        return dest;
    }
    return memcpy(dest, src, size);
}
#define memcpy checked_memcpy
__SENDER__
#undef memcpy
static void check(int condition, const char *label)
{
    printf("%s: %s\n", condition ? "PASS" : "FAIL", label);
    if (!condition) failures++;
}
int main(void)
{
    const float expected[7] = {1.25f, -2.5f, 3.75f, -4.0f, 5.5f, -6.25f, 7.125f};
    const uint8_t tail[4] = {0x00,0x00,0x80,0x7f};
    if (sizeof(float) != 4) return 2;
    VOFA_SendJustFloat(expected[0],expected[1],expected[2],expected[3],expected[4],expected[5],expected[6]);
    check(bounds_errors == 0, "all seven channel writes stay inside frame");
    check(captured_size == 32, "UART receives 32 bytes (seven floats plus delimiter)");
    check(captured_size >= 28 && memcmp(captured,expected,28) == 0,
          "seven channels retain their values and ordering");
    check(captured_size == 32 && memcmp(captured+28,tail,4) == 0,
          "JustFloat delimiter is 00 00 80 7F after channel seven");
    check(vofa_sent_count == 1 && vofa_tx_error_count == 0, "successful send counted");
    next_status = HAL_TIMEOUT;
    VOFA_SendJustFloat(0,0,0,0,0,0,0);
    check(vofa_sent_count == 1 && vofa_tx_error_count == 1,
          "timeout counted without claiming successful delivery");
    printf("frame_bytes=%u out_of_bounds_copies=%u failures=%u\n",
           captured_size,bounds_errors,failures);
    return failures ? 1 : 0;
}
'''.replace('__DECL__', declaration.group()).replace('__TIMEOUT__', timeout.group()).replace('__SENDER__', sender)

gcc = shutil.which('gcc')
if not gcc:
    raise SystemExit('Host GCC is required')
with tempfile.TemporaryDirectory(prefix='dcac_vofa_') as tmp:
    tmp = Path(tmp)
    path = tmp / 'vofa_test.c'
    exe = tmp / 'vofa_test.exe'
    path.write_text(harness, encoding='utf-8')
    subprocess.run([gcc,'-std=c11','-Wall','-Wextra','-Werror','-O0',str(path),'-o',str(exe)],check=True)
    raise SystemExit(subprocess.run([str(exe)]).returncode)
