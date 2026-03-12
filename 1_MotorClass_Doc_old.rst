Motor Class Documentation
=========================

Overview
--------

The `Motor` class encapsulates the functionality to control a DC motor using:
- **motor driver** for speed and direction control.
- **Encoder feedback** to track motor revolutions.
- **PID control** for precise speed maintenance.
- **Low-pass filtering** to smooth RPM measurements.

This documentation explains the code with key sections highlighted.

Class Overview
--------------

The `Motor` class contains:

- **Motor driver pins** for controlling speed and direction.
- **Encoder pins** to track movement.
- **PID variables** for feedback control.
- **Low-pass filtering** to smooth RPM calculations.
- **Interrupt handling** to respond to encoder signals.

Member Variables
----------------

These variables are **private** to ensure encapsulation.

**Motor Driver Pins:**

.. code-block:: cpp

   const uint LED_PIN;
   const uint ENA_PIN;  // PWM pin for speed control (Enable A)
   const uint IN1_PIN;  // Direction control pin 1 (IN1)
   const uint IN2_PIN;  // Direction control pin 2 (IN2)

**Encoder Pins:**

.. code-block:: cpp

   const uint ENCODER_A_PIN;
   const uint ENCODER_B_PIN;

**Encoder Specifications:**

.. code-block:: cpp

   const int TICKS_PER_REV;  // Ticks per revolution
   const float GR;           // Gear ratio

**Encoder State Variables:**

.. code-block:: cpp

   static volatile int32_t encoder_ticks;  // Total ticks
   static int32_t last_ticks;              // Ticks in last interval
   static constexpr float time_interval_sec = 0.1f;  // 100ms interval

**PID Control Variables:**

.. code-block:: cpp

   float Kp, Ki, Kd;       // PID gains
   float previous_error;   // Error for derivative term
   float integral;         // Accumulated error (integral)
   float dt = time_interval_sec;  // Time interval

**Low-Pass Filter:**

.. code-block:: cpp

   static constexpr float alpha = 0.15f;  // Smoothing factor
   static float filtered_rpm;             // Smoothed RPM value

Constructor
-----------

The constructor initializes motor pins, sets up PWM control, and configures interrupts.

.. code-block:: cpp

   Motor(uint led_pin = 25, uint ena_pin = 2, uint in1_pin = 3, uint in2_pin = 4,
         uint enc_a_pin = 5, uint enc_b_pin = 6,
         int ticks_per_rev = 64, float gear_ratio = 50.0f,
         float kp = 0.1158f, float ki = 0.4634f, float kd = 0.0f)
       : LED_PIN(led_pin), ENA_PIN(ena_pin), IN1_PIN(in1_pin), IN2_PIN(in2_pin),
         ENCODER_A_PIN(enc_a_pin), ENCODER_B_PIN(enc_b_pin),
         TICKS_PER_REV(ticks_per_rev), GR(gear_ratio),
         Kp(kp), Ki(ki), Kd(kd) {
   
       instance = this;

       gpio_init(LED_PIN);
       gpio_set_dir(LED_PIN, GPIO_OUT);
       gpio_put(LED_PIN, 0);  // LED off initially

       gpio_init(IN1_PIN);
       gpio_init(IN2_PIN);
       gpio_set_dir(IN1_PIN, GPIO_OUT);
       gpio_set_dir(IN2_PIN, GPIO_OUT);

       gpio_set_function(ENA_PIN, GPIO_FUNC_PWM);
       pwmSlice = pwm_gpio_to_slice_num(ENA_PIN);
       pwm_set_wrap(pwmSlice, 65535);  // 16-bit resolution
       pwm_set_enabled(pwmSlice, true);  // Enable PWM

       gpio_init(ENCODER_A_PIN);
       gpio_init(ENCODER_B_PIN);
       gpio_set_dir(ENCODER_A_PIN, GPIO_IN);
       gpio_set_dir(ENCODER_B_PIN, GPIO_IN);
       gpio_pull_up(ENCODER_A_PIN);
       gpio_pull_up(ENCODER_B_PIN);

       gpio_set_irq_enabled_with_callback(ENCODER_A_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &encoder_a_irq_handler);
       gpio_set_irq_enabled(ENCODER_B_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
   }

Interrupt Handling
------------------

Encoder interrupts are forwarded to the appropriate instance method.

.. code-block:: cpp

   static void encoder_a_irq_handler(uint gpio, uint32_t events) {
       instance->handle_encoder_interrupt(gpio, events);
   }

The instance method updates the tick count based on direction:

.. code-block:: cpp

   void handle_encoder_interrupt(uint gpio, uint32_t events) {
       bool encoder_a = gpio_get(ENCODER_A_PIN);
       bool encoder_b = gpio_get(ENCODER_B_PIN);

       if (gpio == ENCODER_A_PIN) {
           if ((encoder_a && !encoder_b) || (!encoder_a && encoder_b)) {
               encoder_ticks++;  // Forward direction
           } else {
               encoder_ticks--;  // Reverse direction
           }
       } else if (gpio == ENCODER_B_PIN) {
           if ((encoder_a && !encoder_b) || (!encoder_a && encoder_b)) {
               encoder_ticks--;  // Forward direction
           } else {
               encoder_ticks++;  // Reverse direction
           }
       }
   }

Motor Control
-------------

The `set_motor` method adjusts speed and direction using PWM.

.. code-block:: cpp

   void set_motor(float speed) {
       uint16_t pwm_value = (uint16_t)fabs(speed);

       if (speed > 0) {
           gpio_put(IN1_PIN, 1);
           gpio_put(IN2_PIN, 0);  // Forward
       } else if (speed < 0) {
           gpio_put(IN1_PIN, 0);
           gpio_put(IN2_PIN, 1);  // Reverse
       } else {
           gpio_put(IN1_PIN, 0);
           gpio_put(IN2_PIN, 0);  // Stop
       }

       pwm_set_gpio_level(ENA_PIN, (uint16_t)(pwm_value * 65535 / 100));  // Scale to 16-bit PWM
   }

RPM Calculation
---------------

The `calculate_rpm` function computes RPM based on encoder ticks.

.. code-block:: cpp

   void calculate_rpm(float *revs, float *rpm) {
       int32_t ticks_since_last = encoder_ticks - last_ticks;
       last_ticks = encoder_ticks;

       *revs = encoder_ticks / TICKS_PER_REV;
       float raw_rpm = ((float)ticks_since_last / TICKS_PER_REV) * (60.0f / time_interval_sec) * (1.0f / GR);

       *rpm = applyLowPassFilter(raw_rpm);
   }

A **low-pass filter** smooths out RPM measurements:

.. code-block:: cpp

   float applyLowPassFilter(float raw_rpm) {
       filtered_rpm = alpha * raw_rpm + (1.0f - alpha) * filtered_rpm;
       return filtered_rpm;
   }

Utility Functions
-----------------

**Reset Encoder Ticks:**

.. code-block:: cpp

   void resetEncoderTicks() {
       encoder_ticks = 0;
       last_ticks = 0;
       filtered_rpm = 0.0f;
       integral = 0.0f;
       previous_error = 0.0f;
   }

**Toggle LED:**

.. code-block:: cpp

   void toggleLED() {
       gpio_put(LED_PIN, !gpio_get(LED_PIN));
   }

Summary
-------

The `Motor` class provides complete functionality for:
- Motor speed and direction control using **PWM**.
- Tracking rotational movement with **encoders**.
- Maintaining precise speed using a **PID controller**.
- Smoothing RPM readings with a **low-pass filter**.

This class is well-suited for robotics and automation projects requiring reliable motor control.
