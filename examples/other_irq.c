#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "pico/cyw43_arch.h"

#include "servo.h"

// Config
#define SERVO_GPIO                  22u         // GPIO of our servo
#define PWM_TEST_GPIO               15u         // Separate GPIO just for IRQ testing (separate slice)
#define START_ANGLE                 0.0f
#define SEC                         1000000u    // Number of microseconds in a second
#define MAX_UINT_16                 65535u      // Maximum value of uint16_t
#define MAX_CLOCK_DIV               256.0f      // Maximum amount we can divide a PWM slice clock

#define MG90S_FRAME_PERIOD_USEC     20000u
#define MG90S_PULSE_WIDTH_MIN_USEC  500u
#define MG90S_PULSE_WIDTH_MAX_USEC  2500u
#define MG90S_SEC_PER_60            0.5f
#define MG90S_MAX_ANGLE             180.0f

// LED functions
int pico_led_init(void) {
#if defined(PICO_DEFAULT_LED_PIN)
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    return PICO_OK;
#elif defined(CYW43_WL_GPIO_LED_PIN)
    return cyw43_arch_init();
#endif
}

void pico_set_led(bool led_on) {
#if defined(PICO_DEFAULT_LED_PIN)
    gpio_put(PICO_DEFAULT_LED_PIN, led_on);
#elif defined(CYW43_WL_GPIO_LED_PIN)
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, led_on);
#endif
}

// Servo config
Servo servo = {
    .gpio               = SERVO_GPIO,
    .period_usec        = MG90S_FRAME_PERIOD_USEC,
    .duty_min_usec      = MG90S_PULSE_WIDTH_MIN_USEC,
    .duty_max_usec      = MG90S_PULSE_WIDTH_MAX_USEC,
    .start_deg          = START_ANGLE,
    .sec_per_60         = MG90S_SEC_PER_60,
    .max_degrees        = MG90S_MAX_ANGLE,
};

/* Custom PWM IRQ handler that flashes an LED */
void custom_pwm_irq_handler() {

    // IMPORTANT: Only handle interrupts for the test PWM slice
    uint32_t irq_status = pwm_get_irq_status_mask();
    uint slice = pwm_gpio_to_slice_num(PWM_TEST_GPIO);
    if (!(irq_status & (1u << slice)))
    {
        // If the interrupt isn't for the desired slice, exit the handler
        return;
    }

    // Clear the IRQ
    pwm_clear_irq(slice);

    // Toggle the LED
    static bool toggle = false;
    pico_set_led(toggle);
    toggle = !toggle;
}

void setup_pwm_test_irq(uint gpio) {

    gpio_set_function(gpio, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(gpio);

    // Set lowest possible frequency PWM (~112ms at 150 Mhz)
    pwm_config cfg = pwm_get_default_config();
    pwm_config_set_clkdiv(&cfg, MAX_CLOCK_DIV);
    pwm_config_set_wrap(&cfg, MAX_UINT_16);
    pwm_init(slice, &cfg, true);

    // Enable PWM slice interrupt and add a shared handler for the PWM IRQ line
    pwm_set_irq_enabled(slice, true);
    irq_add_shared_handler(PWM_DEFAULT_IRQ_NUM(), custom_pwm_irq_handler, PICO_SHARED_IRQ_HANDLER_DEFAULT_ORDER_PRIORITY);
    irq_set_enabled(PWM_DEFAULT_IRQ_NUM(), true);
}

int main() {

    // Initialize the LED
    pico_led_init();

    // The servo and pwm must be on a different slice
    uint servo_slice = pwm_gpio_to_slice_num(SERVO_GPIO);
    uint pwm_slice = pwm_gpio_to_slice_num(PWM_TEST_GPIO);
    assert(servo_slice != pwm_slice);

    // Set up an IRQ handler that flashes the LED (runs independently of the servo)
    setup_pwm_test_irq(PWM_TEST_GPIO);

    // Set up the servo
    // This will add another (shared) handler on the PWM IRQ line
    servo_init(&servo);
    sleep_ms(200);

    // Move the servo
    while (true) {
        servo_time_to_deg(&servo, 180.0f, SEC, ease_lin, NULL);
        servo_time_to_deg(&servo, 0.0f, SEC, ease_lin, NULL);
        tight_loop_contents();
    }

    /* The LED should blink steadily at around 5 Hz (since it is toggled at 10 Hz).
     * Although the servo is updated at 50 Hz, this should not affect the LED
     * timing if everything is working correctly.
     */
}
