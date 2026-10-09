/* Host-only adapter: the algorithm under test is Core/Src/qpr_control.c. */
#include "qpr_control.h"
#include "inverter_control.h"
#include <stdlib.h>

#ifdef _WIN32
#define TEST_EXPORT __declspec(dllexport)
#else
#define TEST_EXPORT
#endif

TEST_EXPORT QPR_Control *qpr_test_create(float fs)
{
    QPR_Control *c = (QPR_Control *)calloc(1U, sizeof(*c));
    if (c != NULL) (void)QPR_ControlInit(c, fs);
    return c;
}

TEST_EXPORT void qpr_test_destroy(QPR_Control *c) { free(c); }
TEST_EXPORT void qpr_test_reset(QPR_Control *c) { QPR_ControlReset(c); }

TEST_EXPORT float qpr_test_step(QPR_Control *c, float reference,
                                float voltage, float current, float bus)
{
    return QPR_ControlStep(c, reference, voltage, current, bus);
}

TEST_EXPORT unsigned int qpr_test_track(QPR_Control *c, float reference,
                                       float voltage, float current, float correction)
{
    return QPR_ControlTrack(c, reference, voltage, current, correction);
}

TEST_EXPORT void qpr_test_snapshot(const QPR_Control *c, float *f, unsigned int *u)
{
    f[0] = c->sample_period_s;
    f[1] = c->virtual_decay;
    f[2] = c->virtual_b;
    f[3] = c->resonant_decay;
    f[4] = c->resonant_b;
    f[5] = c->resonant_g;
    f[6] = c->resonant_h;
    f[7] = 0.0f; /* Reserved: the shared RMS ramp now belongs to Inverter_Control. */
    f[8] = c->reference_v;
    f[9] = c->voltage_error_v;
    f[10] = c->current_reference_a;
    f[11] = c->current_error_a;
    f[12] = c->qpr_unlimited_v;
    f[13] = c->qpr_correction_v;
    f[14] = c->bridge_reference_v;
    f[15] = c->modulation;
    f[16] = c->voltage_error_previous_v;
    f[17] = c->current_error_previous_a;
    f[18] = c->resonant_r;
    f[19] = c->resonant_q;
    u[0] = c->qpr_saturation_count;
    u[1] = c->modulation_saturation_count;
    u[2] = c->fault_flags;
    u[3] = c->initialized;
}

/* Isolate QPR with ev=0, iref=0, iL=-error without duplicating its code. */
TEST_EXPORT float qpr_test_error(QPR_Control *c, float error)
{
    (void)QPR_ControlStep(c, 0.0f, 0.0f, -error, 48.0f);
    return c->qpr_unlimited_v;
}

TEST_EXPORT float qpr_test_voltage_error(QPR_Control *c, float error)
{
    (void)QPR_ControlStep(c, 0.0f, -error, 0.0f, 48.0f);
    return c->current_reference_a;
}

TEST_EXPORT void qpr_test_corrupt_state(QPR_Control *c, float value)
{
    c->resonant_r = value;
}

TEST_EXPORT int qpr_test_null_api(void)
{
    QPR_ControlReset(NULL);
    return QPR_ControlInit(NULL, 20000.0f) == 0U &&
           QPR_ControlStep(NULL, 0.0f, 0.0f, 0.0f, 48.0f) == 0.0f &&
           QPR_ControlTrack(NULL, 0.0f, 0.0f, 0.0f, 0.0f) == 0U;
}

TEST_EXPORT Inverter_Control *inv_test_create(float fs)
{
    Inverter_Control *c = (Inverter_Control *)calloc(1U, sizeof(*c));
    if (c != NULL) (void)Inverter_ControlInit(c, fs);
    return c;
}

TEST_EXPORT void inv_test_destroy(Inverter_Control *c) { free(c); }
TEST_EXPORT void inv_test_reset(Inverter_Control *c) { Inverter_ControlReset(c); }

TEST_EXPORT float inv_test_step(Inverter_Control *c, float sine, float rms,
                                float voltage, float current, float bus,
                                unsigned int closed, unsigned int enabled)
{
    return Inverter_ControlStep(c, sine, rms, voltage, current, bus,
                                (uint8_t)closed, (uint8_t)enabled);
}

TEST_EXPORT void inv_test_snapshot(const Inverter_Control *c, float *f, unsigned int *u)
{
    f[0] = c->reference_rms_v;
    f[1] = c->reference_v;
    f[2] = c->bridge_reference_v;
    f[3] = c->modulation;
    f[4] = c->last_correction_v;
    f[5] = c->handover_correction_v;
    f[6] = c->handover_step_v;
    f[7] = c->qpr.qpr_unlimited_v;
    f[8] = c->qpr.current_reference_a;
    f[9] = c->qpr.reference_v;
    f[10] = c->qpr.modulation;
    f[11] = c->qpr.sample_period_s;
    u[0] = c->handover_samples_left;
    u[1] = c->modulation_saturation_count;
    u[2] = c->fault_flags;
    u[3] = c->closed_loop_active;
    u[4] = c->mode_initialized;
    u[5] = c->reference_limited;
    u[6] = c->qpr.qpr_saturation_count;
    u[7] = c->qpr.modulation_saturation_count;
    u[8] = c->qpr.fault_flags;
    u[9] = c->qpr.initialized;
}

TEST_EXPORT int inv_test_null_api(void)
{
    Inverter_ControlReset(NULL);
    return Inverter_ControlInit(NULL, 20000.0f) == 0U &&
           Inverter_ControlStep(NULL, 0.0f, 32.0f, 0.0f, 0.0f,
                                48.0f, 1U, 1U) == 0.0f;
}

TEST_EXPORT unsigned int inv_test_set_rms(float rms)
{
    return Inverter_SetVoltageRms(rms);
}

TEST_EXPORT unsigned int inv_test_adjust_rms(float delta)
{
    return Inverter_AdjustVoltageRms(delta);
}

TEST_EXPORT void inv_test_set_mode(unsigned int closed)
{
    Inverter_SetClosedLoop((uint8_t)closed);
}

TEST_EXPORT void inv_test_set_enabled(unsigned int enabled)
{
    Inverter_SetOutputEnable((uint8_t)enabled);
}

TEST_EXPORT void inv_test_commands(float *f, unsigned int *u)
{
    f[0] = voltage_reference_rms_v;
    f[1] = dc_bus_voltage_v;
    u[0] = voltage_closed_loop_enable;
    u[1] = inverter_output_enable;
}
