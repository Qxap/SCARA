Motor Control System with Micro-ROS on Raspberry Pi Pico
========================================================

Overview
--------

This program controls a DC motor using **GPIO** and **PWM** on the **Raspberry Pi Pico**. It integrates with **Micro-ROS** to receive motor speed commands and publish motor status. The key functionalities include:

- **Motor Control:** Uses GPIO and PWM to control motor speed and direction.
- **Encoder Feedback:** Reads motor position and speed through encoder interrupts.
- **ROS2 Communication:**
  - Subscribes to speed commands via the `cmd` topic.
  - Publishes motor status (position, RPM, command) to the `debug_pub` topic.
- **Timer Callbacks:** Periodically computes RPM and sends status updates.

---

Include Statements
------------------

The code includes several hardware and ROS2 headers:

.. code-block:: cpp

    #include <hardware/gpio.h>
    #include <hardware/pwm.h>
    #include <hardware/irq.h>
    #include <pico/stdlib.h>
    #include <math.h>

    // ROS2 headers
    #include "pico_uart_transports.h"
    #include <rcl/rcl.h>
    #include <rclc/rclc.h>
    #include <rclc/executor.h>
    #include <geometry_msgs/msg/twist.h>
    #include <std_msgs/msg/int32.h>
    #include <std_msgs/msg/float32.h>

- **Hardware headers:** Provide access to GPIO, PWM, and interrupt control.
- **ROS2 headers:** Used to set up Micro-ROS communication and define message types (`Twist` and `Float32`).

---

Motor Object Initialization
---------------------------

The motor object is created with specific GPIO pins and PID parameters:

.. code-block:: cpp

    // Initialize the static motor map
    std::map<uint, Motor*> Motor::motor_map;

	// Create a motor object with specific motor and encoder pins, and PID parameters
    	// Motor(led_pin,
	//	 ena_pin : pin for PWM signal,
	//	 in1_pin : pin for motor CW/CCW,
	//	 in2_pin : pin for motor CW/CCW,
	//	 enc_a_pin : pin for encoder signal A,
	//	 enc_b_pin : pin for encoder signal B,
	//	 ticks_per_rev : encoder ticks per revolution,
	//	 gear_ratio : gear ratio,
	//	 kp : PID parameters,
	//	 ki : PID parameters,
	//	 kd : PID parameters,)

    Motor motor1(25, 11, 13, 12, 26, 27, 64, 50.0f, 0.1f, 0.1f, 0.01f);  // Motor object

This initializes the motor control with GPIO pins for the motor driver and encoder, along with PID parameters.

---

Global Variables and ROS2 Setup
-------------------------------

.. code-block:: cpp

    float cmd = 0.0f;  // Command for motor speed

    rcl_subscription_t cmd_subs;  // Subscriber for speed setpoints
    geometry_msgs__msg__Twist cmd_msg;  // Incoming command message

    rcl_publisher_t debug_pub;  // Publisher for motor status
    geometry_msgs__msg__Twist debug_msg;  // Status message

- **Global Variables:** `cmd` holds the desired motor speed setpoint.
- **ROS2 Components:** 
  - **Subscriber:** Receives speed commands.
  - **Publisher:** Sends motor status updates (position, speed, command).

---

Mapping Function
----------------

.. code-block:: cpp

    float map(float x, float in_min, float in_max, float out_min, float out_max) {
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }

This function maps the input range to a target range. Example: Map speed setpoints from `[-3, 3]` to `[-100, 100]` RPM.

---

ROS2 Subscriber Callback
------------------------

.. code-block:: cpp

    void cmd_callback(const void *msgin) {
        const geometry_msgs__msg__Twist *twist_msg_const = (const geometry_msgs__msg__Twist *)msgin;
        printf("Received speed setpoint: %f\n", twist_msg_const->angular.z);

        cmd = map(twist_msg_const->angular.z, -3.0, 3.0, -100.0, 100.0);  // Map setpoint
    }

This callback processes speed commands from the `cmd` topic. It extracts the **angular velocity** and maps it to the motor speed range.

---

Timer Callback for RPM Calculation and Publishing
-------------------------------------------------

.. code-block:: cpp

    void debug_timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
        float position1 = 0.0f;
        float speed1 = 0.0f;

        motor1.calculate_rpm(&position1, &speed1);  // Calculate RPM and position
        motor1.set_motor(cmd);  // Apply speed command

        motor1.toggleLED();  // Toggle onboard LED

        // Publish motor status
        debug_msg.linear.x = position1;
        debug_msg.linear.y = speed1;
        debug_msg.linear.z = cmd;

        rcl_publish(&debug_pub, &debug_msg, NULL);
    }

- **Calculate RPM and Position:** Uses encoder feedback.
- **Apply Motor Command:** Adjusts motor speed.
- **Toggle LED:** Indicates system activity.
- **Publish Status:** Sends position, RPM, and command to the `debug_pub` topic.

---

Main Function
-------------

.. code-block:: cpp

    int main() {
        rmw_uros_set_custom_transport(
            true, NULL,
            pico_serial_transport_open,
            pico_serial_transport_close,
            pico_serial_transport_write,
            pico_serial_transport_read
        );

        rcl_timer_t debug_timer;
        rcl_node_t node;
        rcl_allocator_t allocator = rcl_get_default_allocator();
        rclc_support_t support;
        rclc_executor_t executor;

        rcl_ret_t ret = rmw_uros_ping_agent(1000, 120);
        if (ret != RCL_RET_OK) {
            printf("Failed to connect to Micro-ROS agent.\n");
            return ret;
        }

        rclc_support_init(&support, 0, NULL, &allocator);
        rclc_node_init_default(&node, "pico_node", "", &support);

        rclc_publisher_init_default(
            &debug_pub,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
            "debug_pub"
        );

        rclc_timer_init_default(
            &debug_timer,
            &support,
            RCL_MS_TO_NS(100),
            debug_timer_callback
        );

        rclc_subscription_init_default(
            &cmd_subs,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
            "cmd"
        );

        rclc_executor_init(&executor, &support.context, 3, &allocator);
        rclc_executor_add_subscription(&executor, &cmd_subs, &cmd_msg, &cmd_callback, ON_NEW_DATA);
        rclc_executor_add_timer(&executor, &debug_timer);

        while (true) {
            rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
        }

        return 0;
    }

- **Micro-ROS Setup:** Initializes the serial transport and pings the agent to confirm connection.
- **ROS2 Components:** 
  - **Node:** Created for the Raspberry Pi Pico.
  - **Publisher and Subscriber:** Initialized for motor control.
  - **Timer:** Set to 100ms intervals.
- **Main Loop:** Processes ROS2 events through the **executor**.

---

Summary
-------

This program provides a complete motor control system integrated with **ROS2**:

1. **Motor Control:** Uses GPIO and PWM to control the motor's speed and direction.
2. **Encoder Feedback:** Calculates motor position and RPM from encoder signals.
3. **ROS2 Communication:** Subscribes to the `cmd` topic for speed setpoints and publishes motor status to the `debug_pub` topic.
4. **Micro-ROS Integration:** Runs on a **Raspberry Pi Pico** to interface with the ROS2 ecosystem.

This project is suitable for robotics and embedded systems requiring real-time motor control with ROS2.
