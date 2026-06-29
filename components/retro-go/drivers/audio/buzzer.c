#include "rg_system.h"

#if RG_AUDIO_USE_BUZZER_PIN
#include "rg_audio.h"

#ifndef ESP_PLATFORM
#error "Audio buzzer support can only be built inside esp-idf!"
#endif

#include <driver/ledc.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include "esp_timer.h"

#define BOOSTVOLUME        3
#define MS_OF_CACHED_SAMPLES 50

#define LEDC_PWM_SPEED_MODE  LEDC_LOW_SPEED_MODE
#define LEDC_PWM_CHANNEL     LEDC_CHANNEL_0
#define LEDC_PWM_TIMER       LEDC_TIMER_0

static QueueHandle_t sampleQueue;
static esp_timer_handle_t sampleTimer;
static int pwm_bits = 10;
static int sampleRate;

// esp_timer callback — runs in TASK context, NOT ISR. Safe to call LEDC API.
static void IRAM_ATTR buzzer_timer_cb(void *arg)
{
    int16_t sample;
    if (xQueueReceiveFromISR(sampleQueue, &sample, NULL) == pdTRUE)
    {
        int32_t duty = (int32_t)sample + 32768;
        duty >>= (16 - pwm_bits);
        ledc_set_duty(LEDC_PWM_SPEED_MODE, LEDC_PWM_CHANNEL, duty);
        ledc_update_duty(LEDC_PWM_SPEED_MODE, LEDC_PWM_CHANNEL);
    }
}

static bool buzzer_init(int device, int rate)
{
    sampleRate = rate;
    RG_LOGI("Configuring speaker for %d Hz samplerate", sampleRate);

    int cacheSamples = (sampleRate / 1000) * MS_OF_CACHED_SAMPLES;
    sampleQueue = xQueueCreate(cacheSamples, sizeof(int16_t));
    if (!sampleQueue) {
        RG_LOGE("could not create sampleQueue");
        return false;
    }

    // Choose PWM resolution based on frequency
    int freq = sampleRate;
    while (freq < 16000) freq *= 2;

    if (freq <= 19531)      pwm_bits = 12;
    else if (freq <= 39062) pwm_bits = 11;
    else if (freq <= 78125) pwm_bits = 10;
    else                    pwm_bits = 9;

    RG_LOGI("PWM frequency=%d Hz, resolution=%d bits", freq, pwm_bits);

    ledc_timer_config_t ledc_timer = {
        .duty_resolution = pwm_bits,
        .freq_hz = freq,
        .speed_mode = LEDC_PWM_SPEED_MODE,
        .timer_num = LEDC_PWM_TIMER,
        .clk_cfg = LEDC_USE_APB_CLK,
    };
    ledc_timer_config(&ledc_timer);

    ledc_channel_config_t ledc_channel = {
        .channel    = LEDC_PWM_CHANNEL,
        .duty       = 0,
        .gpio_num   = RG_AUDIO_USE_BUZZER_PIN,
        .speed_mode = LEDC_PWM_SPEED_MODE,
        .hpoint     = 0,
        .timer_sel  = LEDC_PWM_TIMER,
    };
    ledc_channel_config(&ledc_channel);

    ledc_set_duty(LEDC_PWM_SPEED_MODE, LEDC_PWM_CHANNEL, 0);
    ledc_update_duty(LEDC_PWM_SPEED_MODE, LEDC_PWM_CHANNEL);

    // esp_timer: callback runs in task context, NOT ISR, so LEDC calls are safe.
    // No watchdog issues unlike hardware timer ISR.
    esp_timer_create_args_t timer_args = {
        .callback = buzzer_timer_cb,
        .arg = NULL,
        .name = "buzzer_pwm",
        .skip_unhandled_events = true,
    };
    esp_timer_create(&timer_args, &sampleTimer);
    esp_timer_start_periodic(sampleTimer, 1000000 / sampleRate);

    return true;
}

static bool buzzer_deinit(void)
{
    if (sampleTimer) {
        esp_timer_stop(sampleTimer);
        esp_timer_delete(sampleTimer);
        sampleTimer = NULL;
    }
    if (sampleQueue) {
        vQueueDelete(sampleQueue);
        sampleQueue = NULL;
    }
    ledc_set_duty(LEDC_PWM_SPEED_MODE, LEDC_PWM_CHANNEL, 0);
    ledc_update_duty(LEDC_PWM_SPEED_MODE, LEDC_PWM_CHANNEL);
    return true;
}

static bool buzzer_submit(const rg_audio_frame_t *frames, size_t count)
{
    float volumeFactor = rg_audio_get_mute() ? 0.f : (rg_audio_get_volume() * 0.01f) * BOOSTVOLUME;

    for (size_t i = 0; i < count; ++i) {
        int16_t left = frames[i].left * volumeFactor;
        if (xQueueSend(sampleQueue, &left, 0) != pdPASS)
            break;
    }

    // Pace submission to match real-time playback speed
    int usSleep = count * (1000000 / sampleRate);
    rg_usleep(usSleep);

    return true;
}

static bool buzzer_set_mute(bool mute)
{
    ledc_set_duty(LEDC_PWM_SPEED_MODE, LEDC_PWM_CHANNEL, 0);
    ledc_update_duty(LEDC_PWM_SPEED_MODE, LEDC_PWM_CHANNEL);
    return true;
}

const rg_audio_driver_t rg_audio_driver_buzzer = {
    .name = "buzzer",
    .init = buzzer_init,
    .deinit = buzzer_deinit,
    .submit = buzzer_submit,
    .set_mute = buzzer_set_mute,
    .set_volume = NULL,
    .set_sample_rate = NULL,
    .get_error = NULL,
};

#endif // RG_AUDIO_USE_BUZZER_PIN
