/*
 * Battery measurement with single 180k resistor (series), nRF Supermini
 * Uses AIN0 (AD0) to read battery voltage relative to GND.
 * 
 * - Reads raw ADC from battery through 180k resistor
 * - Converts to millivolts
 * - Smooths using EMA
 * - Maps mV to percentage using battery_level_pptt()
 */

#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/logging/log.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "battery.h"

LOG_MODULE_REGISTER(BATTERY, CONFIG_ADC_LOG_LEVEL);

/* ---- Battery curve mapping (same style as original) ---- */
struct battery_level_point {
    unsigned int lvl_pptt; // parts per ten thousand (10000 = 100%)
    unsigned int lvl_mV;   // voltage in mV
};

static const struct battery_level_point levels[] = {
#if CONFIG_BATTERY_USE_REG_BUCK_MAPPING
    {10000, 4150},
    {9500, 4075},
    {3000, 3775},
    {500, 3450},
    {0, 3200},
#elif CONFIG_BATTERY_USE_REG_LDO_MAPPING
    {10000, 4150},
    {9500, 4025},
    {3000, 3650},
    {500, 3400},
    {0, 3200},
#else
#warning "Battery voltage map not defined, using linear fallback"
    {10000, 4200},
    {0, 3300},
#endif
};

/* Min/max clamp values */
#define BATT_MAX_MV 4200
#define BATT_MIN_MV 3200

/* EMA smoothing */
#define EMA_ALPHA_NUM 1
#define EMA_ALPHA_DEN 4

/* ADC context */
static const struct device *adc_dev = DEVICE_DT_GET(DT_NODELABEL(adc));
static struct adc_channel_cfg adc_cfg = {
    .gain = ADC_GAIN_1_6,
    .reference = ADC_REF_INTERNAL,
    .acquisition_time = ADC_ACQ_TIME(ADC_ACQ_TIME_MICROSECONDS, 10),
    .input_positive = SAADC_CH_PSELP_PSELP_AnalogInput0, // AIN0
};
static struct adc_sequence adc_seq = {
    .channels = BIT(0),
    .buffer = NULL,
    .buffer_size = 0,
    .resolution = 14,
    .oversampling = 4,
    .calibrate = true,
};
static int16_t adc_raw;

/* smoothed battery voltage in mV */
static int smoothed_mV = BATT_MAX_MV;

/* ---- Helpers ---- */

static int battery_setup(void)
{
    if (!device_is_ready(adc_dev)) {
        LOG_ERR("ADC device not ready");
        return -ENODEV;
    }

    int rc = adc_channel_setup(adc_dev, &adc_cfg);
    if (rc) {
        LOG_ERR("ADC channel setup failed (%d)", rc);
    } else {
        LOG_INF("Battery ADC channel set up on AIN0");
    }
    return rc;
}

SYS_INIT(battery_setup, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);

/* Single ADC sample → millivolts */
static int sample_batt_mV(void)
{
    adc_seq.buffer = &adc_raw;
    adc_seq.buffer_size = sizeof(adc_raw);

    int rc = adc_read(adc_dev, &adc_seq);
    adc_seq.calibrate = false;
    if (rc) {
        LOG_ERR("adc_read failed: %d", rc);
        return rc;
    }

    int32_t val = adc_raw;
    rc = adc_raw_to_millivolts(adc_ref_internal(adc_dev),
                               adc_cfg.gain,
                               adc_seq.resolution,
                               &val);
    if (rc < 0) {
        LOG_ERR("adc_raw_to_millivolts failed: %d", rc);
        return rc;
    }

    return val;
}

/* Curve interpolation */
unsigned int battery_level_pptt(unsigned int batt_mV,
                                const struct battery_level_point *curve)
{
    const struct battery_level_point *pb = curve;

    if (batt_mV >= pb->lvl_mV) {
        return pb->lvl_pptt;
    }
    while ((pb->lvl_pptt > 0) && (batt_mV < pb->lvl_mV)) {
        ++pb;
    }
    if (batt_mV < pb->lvl_mV) {
        return pb->lvl_pptt;
    }

    const struct battery_level_point *pa = pb - 1;

    return pb->lvl_pptt
         + ((pa->lvl_pptt - pb->lvl_pptt) * (batt_mV - pb->lvl_mV)
            / (pa->lvl_mV - pb->lvl_mV));
}

/* ---- Public API ---- */

/* Update measurement (call periodically) */
int battery_update(void)
{
    int batt_mV = sample_batt_mV();
    if (batt_mV < 0) {
        return batt_mV; // error
    }

    /* EMA smoothing on mV */
    int32_t new_smoothed = ((int32_t)EMA_ALPHA_NUM * batt_mV
        + (int32_t)(EMA_ALPHA_DEN - EMA_ALPHA_NUM) * smoothed_mV)
        / (int32_t)EMA_ALPHA_DEN;

    /* Clamp */
    if (new_smoothed > BATT_MAX_MV) new_smoothed = BATT_MAX_MV;
    if (new_smoothed < BATT_MIN_MV) new_smoothed = BATT_MIN_MV;

    smoothed_mV = (int)new_smoothed;

    LOG_DBG("adc_raw=%d batt_mV=%d smoothed_mV=%d",
            adc_raw, batt_mV, smoothed_mV);

    return battery_level_pptt(smoothed_mV, levels);
}

/* Return battery %pptt */
unsigned int read_batt(void)
{
    return battery_update();
}

/* Return mV + %pptt */
unsigned int read_batt_mV(int *out)
{
    int pct = battery_update();
    *out = smoothed_mV;
    return pct;
}
