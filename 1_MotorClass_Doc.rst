Motor Class Documentation
=========================

Overview
--------
The `Motor` class encapsulates:

1. **Motor Control:** Controls the speed and direction of a motor using PWM (Pulse Width Modulation) and GPIO pins.
2. **Encoder Handling:** Tracks the motor’s position and rotation via two encoder pins (A and B) using interrupts.
3. **PID Controller Variables:** Placeholder variables are included for future PID control.
4. **Low-pass Filter:** Smooths the RPM readings to reduce noise.

Class Members
-------------

Private Variables
~~~~~~~~~~~~~~~~~
- **Motor Control Pins:**
  - `LED_PIN`: Indicates motor status with an onboard LED.
  - `ENA_PIN`: PWM pin for controlling motor speed.
  - `IN1_PIN`, `IN2_PIN`: GPIO pins to set motor rotation direction.

- **Encoder Pins:**
  - `ENCODER_A_PIN`, `ENCODER_B_PIN`: Input pins from the motor’s rotary encoder.

- **Motor Specifications:**
  - `TICKS_PER_REV`: Number of encoder ticks per revolution.
  - `GR`: Gear ratio, which adjusts the number of revolutions reported by the encoder.

- **PWM Configuration:**
  - `pwmSlice`: PWM slice associated with the motor’s control pin.

- **PID Control Variables:**
  - `Kp`, `Ki`, `Kd`: Constants for proportional, integral, and derivative terms.
  - `previous_error`, `integral`, `dt`: Variables for PID calculations.

- **Encoder State Tracking:**
  - `encoder_ticks`: Cumulative ticks received from the encoder.
  - `last_ticks`: Previous tick count to calculate RPM.
  - `filtered_rpm`: Filtered RPM using a low-pass filter.

- **Filter Coefficient:**
  - `alpha`: Coefficient for the low-pass filter (smoothing RPM).

- **Motor Map:**
  - A `std::map` to associate encoder pins with motor instances to manage interrupts.

Constructor
-----------
The constructor initializes the motor with hardware pins and settings:

.. code-block:: cpp

    Motor(uint led_pin, uint ena_pin, uint in1_pin, uint in2_pin,
          uint enc_a_pin, uint enc_b_pin, int ticks_per_rev = 64,
          float gear_ratio = 50.0f, float kp = 0.1158f, float ki = 0.4634f, float kd = 0.0f)

Tasks Performed:

- **Registers the Motor in the Map:** Associates encoder pins with the motor instance.
- **Initializes GPIO Pins:** Configures motor control pins and the LED pin.
- **Sets Up PWM:** Configures PWM to control motor speed.
- **Initializes Encoder Pins:** Sets up encoder inputs with pull-up resistors and interrupts.

Interrupt Handling
------------------

Static Interrupt Handler
~~~~~~~~~~~~~~~~~~~~~~~~
This method handles interrupts on encoder pins:

.. code-block:: cpp

    static void encoder_irq_handler(uint gpio, uint32_t events) {
        if (motor_map.find(gpio) != motor_map.end()) {
            motor_map[gpio]->handle_encoder_interrupt(gpio, events);
        }
    }

Instance Method for Handling Interrupts
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Processes the encoder signals and updates the tick counter:

.. code-block:: cpp

    void handle_encoder_interrupt(uint gpio, uint32_t events) {
        bool encoder_a = gpio_get(ENCODER_A_PIN);
        bool encoder_b = gpio_get(ENCODER_B_PIN);

        if (gpio == ENCODER_A_PIN) {
            if ((encoder_a && !encoder_b) || (!encoder_a && encoder_b)) {
                encoder_ticks++;
            } else {
                encoder_ticks--;
            }
        } else if (gpio == ENCODER_B_PIN) {
            if ((encoder_a && !encoder_b) || (!encoder_a && encoder_b)) {
                encoder_ticks--;
            } else {
                encoder_ticks++;
            }
        }
    }

Motor Control
-------------

Set Motor Speed and Direction
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Adjusts the motor’s speed and direction:

.. code-block:: cpp

    void set_motor(float speed) {
        uint16_t pwm_value = (uint16_t)fabs(speed);

        if (speed > 0) {
            gpio_put(IN1_PIN, 1);
            gpio_put(IN2_PIN, 0);
        } else if (speed < 0) {
            gpio_put(IN1_PIN, 0);
            gpio_put(IN2_PIN, 1);
        } else {
            gpio_put(IN1_PIN, 0);
            gpio_put(IN2_PIN, 0);
        }

        pwm_set_gpio_level(ENA_PIN, (uint16_t)(pwm_value * 65535 / 100));
    }

RPM Calculation
---------------

Calculate RPM
~~~~~~~~~~~~~
Calculates motor revolutions and RPM:

.. code-block:: cpp

    void calculate_rpm(float* revs, float* rpm) {
        int32_t ticks_since_last = encoder_ticks - last_ticks;
        last_ticks = encoder_ticks;

        *revs = encoder_ticks / TICKS_PER_REV;

        float raw_rpm = ((float)ticks_since_last / TICKS_PER_REV) * (60.0f / dt) * (1.0f / GR);
        *rpm = apply_low_pass_filter(raw_rpm);
    }

Low-pass Filter
~~~~~~~~~~~~~~~
Smooths the RPM values:

.. code-block:: cpp

    float apply_low_pass_filter(float raw_rpm) {
        filtered_rpm = alpha * raw_rpm + (1.0f - alpha) * filtered_rpm;
        return filtered_rpm;
    }

LED Control
-----------

Toggle LED
~~~~~~~~~~
Toggles the onboard LED state:

.. code-block:: cpp

    void toggleLED() {
        gpio_put(LED_PIN, !gpio_get(LED_PIN));
    }

Summary
-------

The `Motor` class provides comprehensive motor control with:

1. **Speed and Direction Control** using PWM and GPIO.
2. **Encoder Feedback** for tracking revolutions and direction.
3. **Interrupt Handling** for precise tick counting.
4. **RPM Calculation** with a low-pass filter for smooth readings.
5. **PID Controller Variables** for future control refinement.

This design is efficient for embedded applications, allowing smooth motor operation with encoder-based feedback.
