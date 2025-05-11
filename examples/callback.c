#include "pico/cyw43_arch.h"

#include "servo.h"

#define SERVO_GPIO                  22u
#define START_ANGLE                 0.0f
#define SEC                         1000000u // One second in us, since we do everything in us

// MG90S constants
#define MG90S_FRAME_PERIOD_USEC     20000u  // 10 ms (running at 2 x 50 Hz)
#define MG90S_PULSE_WIDTH_MIN_USEC  500u    // 0.5 ms
#define MG90S_PULSE_WIDTH_MAX_USEC  2500u   // 2.5 ms
#define MG90S_SEC_PER_60            0.5f    // Setting this slower as I don't have a legit MG90S
#define MG90S_MAX_ANGLE             180.0f  // 180 degrees

// Perform initialisation
int pico_led_init(void) {
    #if defined(PICO_DEFAULT_LED_PIN)
        // A device like Pico that uses a GPIO for the LED will define PICO_DEFAULT_LED_PIN
        // so we can use normal GPIO functionality to turn the led on and off
        gpio_init(PICO_DEFAULT_LED_PIN);
        gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
        return PICO_OK;
    #elif defined(CYW43_WL_GPIO_LED_PIN)
        // For Pico W devices we need to initialise the driver etc
        return cyw43_arch_init();
    #endif
}

// Turn the led on or off
void pico_set_led(bool led_on) {
    #if defined(PICO_DEFAULT_LED_PIN)
        // Just set the GPIO on or off
        gpio_put(PICO_DEFAULT_LED_PIN, led_on);
    #elif defined(CYW43_WL_GPIO_LED_PIN)
        // Ask the wifi "driver" to set the GPIO on or off
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, led_on);
    #endif
}

volatile bool forwards = true;

void callback_fn(Servo* s)
{
    forwards = !forwards;
    pico_set_led(forwards ? false : true);
    servo_time_to_deg(s, forwards ? 180.0f : 0.0f, SEC, ease_lin, NULL);
}

Servo servo = {
    .gpio               = SERVO_GPIO,
    .period_usec        = MG90S_FRAME_PERIOD_USEC,
    .duty_min_usec      = MG90S_PULSE_WIDTH_MIN_USEC,
    .duty_max_usec      = MG90S_PULSE_WIDTH_MAX_USEC,
    .start_deg          = START_ANGLE,
    .sec_per_60         = MG90S_SEC_PER_60,
    .max_degrees        = MG90S_MAX_ANGLE,
    .callback           = callback_fn,
};

int main() {

    pico_led_init();
    servo_init(&servo);
    sleep_ms(500);

    // FIXME: The first call to callback_fn *from the ISR* takes longer than expected.
    // This may be due to a cache miss or an issue with the LED control.
    pico_set_led(true);
    pico_set_led(false);

    // Start the servo movement, the rest of the logic will be handled through
    // interrupts via the callback
    servo_time_to_deg(&servo, 180.0f, SEC, ease_lin, NULL);

    // Add a loop here to prevent processor from exiting (all motor control is
    // done through interrupts)
    while(true)
    {
        tight_loop_contents();
    }
}