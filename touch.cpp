// Test the ESP32 touch pads
//
// By Leland Lucius
//
// Public Domain

#include "Arduino.h"
#include "driver/touch_pad.h"
#include "freertos/task.h"
#include "soc/rtc.h"
#include "soc/rtc_cntl_reg.h"
#include "soc/sens_reg.h"

class TouchPad
{
public:
    class Config
    {
    public:
        uint32_t delay;     // # of ticks pad must be touched before notify, 0 = immediate
        uint32_t repeat;    // # of ticks between notifies if pad is held, 0 = no repeat
        uint16_t touched;   // touch detection threshold
        uint16_t released;  // release detection threshold
    };

    TouchPad(TaskHandle_t task_handle)
    : task(task_handle)
    {
        // Initialize the touch pads
        touch_pad_init();

        // Initialize the pad configurations
        for (uint8_t i = 0; i < TOUCH_PAD_MAX; ++i)
        {
            set_pad(i, 0, 0, 0, 0);
            state[i].triggered = 0;
        }

        // Set the interrupt handler
        touch_pad_isr_handler_register(isr_callback, this, 0, NULL);
    }

    ~TouchPad()
    {
        // All done with the touch pads
        touch_pad_deinit();
    }

    // Returns true if the given pad index (0-9) was triggered
    bool is_triggered(uint8_t pad)
    {
        return state[pad].triggered > 0;
    }

    // Set the pad configuration
    void set_pad(uint8_t pad, Config cfg)
    {
        config[pad] = cfg;
        reset_pad(pad);
    }

    // Set the pad configuration
    void set_pad(uint8_t pad,
                 uint32_t delay,
                 uint32_t repeat,
                 uint32_t touched,
                 uint32_t released)
    {
        config[pad].delay = delay;
        config[pad].repeat = repeat;
        config[pad].touched = touched;
        config[pad].released = released;
        reset_pad(pad);
    }

private:
    void reset_pad(uint8_t pad)
    {
        touch_pad_config((touch_pad_t) pad, config[pad].touched);
        state[pad].firstTouched = 0;
        state[pad].lastTouched = 0;
    }

    void isr(uint32_t pad_intr)
    {
        uint32_t ticks = xTaskGetTickCountFromISR();
        bool notify = false;

        for (int i = 0; i < TOUCH_PAD_MAX; ++i)
        {
            // Force a reset if the touch was stopped
            // while sleeping or measuring.  In this case,
            // we won't get an interrupt, so we have to
            // clean up first.
            if (state[i].lastTouched && (ticks - state[i].lastTouched) > 50)
            {
                reset_pad(i);
                state[i].triggered = 0;
            }

            // Continue to the next pad if this one didn't generate the interrupt
            if (((pad_intr >> i) & 0x01) == 0)
            {
                continue;
            }

            // Get the measurement value
            int8_t channel = digitalPinToTouchChannel(pad_to_pin[i]);
            uint16_t val = READ_PERI_REG(SENS_SAR_TOUCH_OUT1_REG + (channel / 2) * 4) >> ((channel & 1) ? SENS_TOUCH_MEAS_OUT1_S : SENS_TOUCH_MEAS_OUT0_S);

            // Remember the ticks
            state[i].lastTouched = ticks;

            // If the measured value is greater than our trigger
            // threshold, then the touch was stopped.  This will
            // only happen after the initial touch was detected.
            if (val > config[i].touched)
            {
                reset_pad(i);
                state[i].triggered = 0;
            }
            // Otherwise, a new or continued touch was detected
            else if (config[i].repeat || !state[i].triggered)
            {
                // No delay, so notify immediately
                if (config[i].delay == 0)
                {
                    state[i].triggered = ticks;
                    notify = true;
                }  
                // This is a new touch, so record when it happened
                else if (state[i].firstTouched == 0)
                {
                    state[i].firstTouched = ticks;
                }
                // Pad has been touched for the desired ticks
                else if (ticks - state[i].firstTouched >= config[i].delay)
                {
                    // Trigger the notification...this handles both the single
                    // fire and repeat fire cases.
                    if (ticks - state[i].triggered >= config[i].repeat)
                    {
                        state[i].triggered = ticks;
                        notify = true;
                    }
                }

                // Start looking for the release threshold.
                touch_pad_config((touch_pad_t) i, config[i].released);
            }
        }

        // Send the nofication if needed
        if (notify)
        {
            BaseType_t need_yield = false;

            vTaskNotifyGiveFromISR(task, &need_yield);
            if (need_yield)
            {
                portYIELD_FROM_ISR();
            }
        }
    }

    static void isr_callback(void *arg)
    {
        // Get the fired interrupts and clear
        uint32_t pad_intr = READ_PERI_REG(SENS_SAR_TOUCH_CTRL2_REG) & 0x3ff;
        uint32_t rtc_intr = READ_PERI_REG(RTC_CNTL_INT_ST_REG);
        WRITE_PERI_REG(RTC_CNTL_INT_CLR_REG, rtc_intr);
        SET_PERI_REG_MASK(SENS_SAR_TOUCH_CTRL2_REG, SENS_TOUCH_MEAS_EN_CLR);

        // Was it a touch interrupt? (how can we have anything else???)
        if (rtc_intr & RTC_CNTL_TOUCH_INT_ST)
        {
            static_cast<TouchPad *>(arg)->isr(pad_intr);
        }
    }

private:
    TaskHandle_t task;
    Config config[TOUCH_PAD_MAX];

    struct
    {
        uint32_t firstTouched;
        uint32_t lastTouched;
        uint32_t triggered;
    } state[TOUCH_PAD_MAX];

    const uint8_t pad_to_pin[TOUCH_PAD_MAX] =
    {
        4,      // TOUCH_PAD_NUM0
        0,      // TOUCH_PAD_NUM1
        2,      // TOUCH_PAD_NUM2
        15,     // TOUCH_PAD_NUM3
        13,     // TOUCH_PAD_NUM4
        12,     // TOUCH_PAD_NUM5
        14,     // TOUCH_PAD_NUM6
        27,     // TOUCH_PAD_NUM7
        33,     // TOUCH_PAD_NUM8
        32      // TOUCH_PAD_NUM9
    };
};

static const TouchPad::Config pad_config[TOUCH_PAD_MAX] =
{
    { 300,   0, 300, 2000 },
    {   0,   0,   0,    0 },
    { 300, 100, 300, 2000 },
    { 300, 100, 300, 2000 },
    {   0,   0,   0,    0 },
    {   0, 100, 300, 2000 },
    { 300, 300, 300, 2000 },
    { 300, 100, 300, 2000 },
    { 300, 100, 300, 2000 },
    { 300, 100, 300, 2000 },
};

extern "C" void app_main()
{
    ets_printf("Test ESP32 touch pads\n");

    // Configure the touch pads
    TouchPad tp(xTaskGetCurrentTaskHandle());
    for (int i = 0; i < TOUCH_PAD_MAX; ++i)
    {
        if (pad_config[i].touched > 0)
        {
            tp.set_pad(i, pad_config[i]);
        }
    }

    // Wait for notifications
    for (;;)
    {
        if (ulTaskNotifyTake(pdFALSE, 1000 / portTICK_PERIOD_MS) > 0)
        {
            for (int i = 0; i < TOUCH_PAD_MAX; ++i)
            {
                if (tp.is_triggered(i))
                {
                    ets_printf("touch pad %d triggered\n", i);
                }
            }
        }
    }

    // Never reached
}
