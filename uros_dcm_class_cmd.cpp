
#include <hardware/gpio.h>
#include <hardware/pwm.h>
#include <hardware/irq.h>
#include <pico/stdlib.h>
#include <math.h>
#include <map>
#include "pico_uart_transports.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include <geometry_msgs/msg/twist.h>

//////////////////////////////////////////////////////
// --- Clase Motor
//////////////////////////////////////////////////////
class Motor {
private:
    const uint LED_PIN, ENA_PIN, IN1_PIN, IN2_PIN;
    const uint ENCODER_A_PIN, ENCODER_B_PIN;
    const uint FC1_PIN, FC2_PIN; 
    const int TICKS_PER_REV;
    const float GR;
    uint pwmSlice;
    volatile int32_t encoder_ticks = 0;
    int32_t last_ticks = 0;
  
   
    static std::map<uint, Motor*> motor_map;
   
    static void encoder_irq_handler(uint gpio, uint32_t events) {
        if (motor_map.find(gpio) != motor_map.end()) {
            motor_map[gpio]->handle_encoder_interrupt(gpio, events);
        }
    }
   
    void handle_encoder_interrupt(uint gpio, uint32_t events) {
        bool encoder_a = gpio_get(ENCODER_A_PIN);
        bool encoder_b = gpio_get(ENCODER_B_PIN);
       
        if (gpio == ENCODER_A_PIN) {
            if (((encoder_a && !encoder_b) || (!encoder_a && encoder_b))) {
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
   
public:
    Motor(uint led_pin, uint ena_pin, uint in1_pin, uint in2_pin,
          uint enc_a_pin, uint enc_b_pin, uint fc1, uint fc2,
          int ticks_per_rev = 64, float gear_ratio = 50.0f)
        : LED_PIN(led_pin), ENA_PIN(ena_pin), IN1_PIN(in1_pin), IN2_PIN(in2_pin),
          ENCODER_A_PIN(enc_a_pin), ENCODER_B_PIN(enc_b_pin),
          FC1_PIN(fc1), FC2_PIN(fc2),
          TICKS_PER_REV(ticks_per_rev), GR(gear_ratio) {
       
        motor_map[ENCODER_A_PIN] = this;
        motor_map[ENCODER_B_PIN] = this;
       
        gpio_init(LED_PIN);
        gpio_set_dir(LED_PIN, GPIO_OUT);
        gpio_put(LED_PIN, 0);
       
        gpio_init(IN1_PIN);
        gpio_init(IN2_PIN);
        gpio_set_dir(IN1_PIN, GPIO_OUT);
        gpio_set_dir(IN2_PIN, GPIO_OUT);
       
        gpio_set_function(ENA_PIN, GPIO_FUNC_PWM);
        pwmSlice = pwm_gpio_to_slice_num(ENA_PIN);
        pwm_set_wrap(pwmSlice, 65535);
        pwm_set_chan_level(pwmSlice, PWM_CHAN_A, 0);
        pwm_set_enabled(pwmSlice, true);
       
        gpio_init(ENCODER_A_PIN);
        gpio_init(ENCODER_B_PIN);
        gpio_set_dir(ENCODER_A_PIN, GPIO_IN);
        gpio_set_dir(ENCODER_B_PIN, GPIO_IN);
        gpio_pull_up(ENCODER_A_PIN);
        gpio_pull_up(ENCODER_B_PIN);
       
        if (FC1_PIN != 0) {
            gpio_init(FC1_PIN);
            gpio_set_dir(FC1_PIN, GPIO_IN);
            gpio_pull_up(FC1_PIN);
        }
        if (FC2_PIN != 0) {
            gpio_init(FC2_PIN);
            gpio_set_dir(FC2_PIN, GPIO_IN);
            gpio_pull_up(FC2_PIN);
        }
       
        gpio_set_irq_enabled_with_callback(ENCODER_A_PIN,
            GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &encoder_irq_handler);
        gpio_set_irq_enabled_with_callback(ENCODER_B_PIN,
            GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &encoder_irq_handler);
    }
   
    void set_motor(float speed) {
        // Check physical limits
        if ((speed > 0 && FC1_PIN != 0 && gpio_get(FC1_PIN) == 0) ||
            (speed < 0 && FC2_PIN != 0 && gpio_get(FC2_PIN) == 0)) {
            speed = 0;
        }
       
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
       
        uint16_t pwm_value = (uint16_t)fabs(speed);
        pwm_set_gpio_level(ENA_PIN, (uint16_t)(pwm_value * 65535 / 100));
    }
   
    float get_position_rad() {
        return (encoder_ticks / (float)TICKS_PER_REV) * (2.0f * M_PI / GR);
    }
   
    int32_t get_ticks() const { return encoder_ticks; }
    void reset_ticks() { encoder_ticks = 0; }
    uint get_fc1() const { return FC1_PIN; }
    uint get_fc2() const { return FC2_PIN; }
};

std::map<uint, Motor*> Motor::motor_map;

//////////////////////////////////////////////////////
// --- Clase PID para Velocidad
//////////////////////////////////////////////////////

//////////////////////////////////////////////////////
// --- Clase PID para Posición
//////////////////////////////////////////////////////
class PositionPID {
private:
    float Kp, Ki, Kd;
    float integral;
    float prev_error;
    float u_min, u_max; // La salida será una velocidad (rad/s)
    float integral_max;
    float dead_zone;
   
public:
    PositionPID(float Kp_, float Ki_, float Kd_, float min_val, float max_val, float int_max ,float dead_zone_)
        : Kp(Kp_), Ki(Ki_), Kd(Kd_),
          integral(0.0f), prev_error(0.0f),
          u_min(min_val), u_max(max_val), integral_max(int_max),dead_zone(dead_zone_) {}
   
    // La función 'calculate' toma un setpoint de posición y una posición medida,
    // y devuelve un setpoint de VELOCIDAD.
    float calculate(float setpoint, float measured, float dt) {
        float error = setpoint - measured;
        if (fabs(error) < dead_zone) {
            // Opcional: resetear el integral para que no se acumule mientras está parado.
            integral = 0.0f; 
            return 0.0f; // Devuelve una orden de velocidad CERO.
        }
       
        // Proportional
        float P = Kp * error;
       
        // Integral with anti-windup
        integral += error * dt;
        if (integral > integral_max) integral = integral_max;
        if (integral < -integral_max) integral = -integral_max;
        float I = Ki * integral;
       
        // Derivative
        float D = Kd * (error - prev_error) / dt;
        prev_error = error;
       
        // Total output (velocidad deseada)
        float u = P + I + D;
       
        // Saturation (limita la velocidad que puede comandar el PID de posición)
        if (u > u_max) u = u_max;
        if (u < u_min) u = u_min;
       
        return u;
    }
   
    void reset() {
        integral = 0.0f;
        prev_error = 0.0f;
    }
};


// class VelocityPID {
// private:
//     float Kp, Ki, Kd;
//     float integral;
//     float prev_error;
//     float u_min, u_max;
//     float integral_max;
   
// public:
//     VelocityPID(float Kp_, float Ki_, float Kd_, float min_val, float max_val, float int_max = 20.0f)
//         : Kp(Kp_), Ki(Ki_), Kd(Kd_),
//           integral(0.0f), prev_error(0.0f),
//           u_min(min_val), u_max(max_val), integral_max(int_max) {}
   
//     float calculate(float setpoint, float measured, float dt) {
//         float error = setpoint - measured;
       
//         // Proportional
//         float P = Kp * error;
       
//         // Integral with anti-windup
//         integral += error * dt;
//         if (integral > integral_max) integral = integral_max;
//         if (integral < -integral_max) integral = -integral_max;
//         float I = Ki * integral;
       
//         // Derivative
//         float D = Kd * (error - prev_error) / dt;
//         prev_error = error;
       
//         // Total output
//         float u = P + I + D;
       
//         // Saturation
//         if (u > u_max) u = u_max;
//         if (u < u_min) u = u_min;
       
//         return u;
//     }
   
//     void reset() {
//         integral = 0.0f;
//         prev_error = 0.0f;
//     }

class VelocityPID {
private:
    float Kp, Ti, Td;
    float Ts;  // Período de muestreo
    float q0, q1, q2;  // Coeficientes PID discretos
    
    float e0, e1, e2;  // e[k], e[k-1], e[k-2]
    float u_prev;      // u[k-1]
    
    float u_min, u_max;
    
public:
    VelocityPID(float Kp_, float Ti_, float Td_, float Ts_, float min_val, float max_val)
        : Kp(Kp_), Ti(Ti_), Td(Td_), Ts(Ts_),
          e0(0.0f), e1(0.0f), e2(0.0f), u_prev(0.0f),
          u_min(min_val), u_max(max_val) {
        
        calculate_pid_coefficients();
    }
    
    // Calcular coeficientes PID discretos
    void calculate_pid_coefficients() {
        if (Ti > 0.0f) {  // Evitar división por cero
            q0 = Kp * (1.0f + (Ts / (2.0f * Ti)) + (Td / Ts));
            q1 = Kp * ((Ts / (2.0f * Ti)) - (2.0f * Td / Ts) - 1.0f);
            q2 = Kp * (Td / Ts);
        } else {
            // Solo P y D (sin integral)
            q0 = Kp * (1.0f + (Td / Ts));
            q1 = Kp * (-(2.0f * Td / Ts) - 1.0f);
            q2 = Kp * (Td / Ts);
        }
    }
    
    float calculate(float setpoint, float measured, float dt) {
        // Actualizar Ts si es diferente (opcional)
        if (fabs(dt - Ts) > 0.001f) {
            Ts = dt;
            calculate_pid_coefficients();
        }
        
        // Calcular error actual
        float error = setpoint - measured;
        
        // Desplazar historial de errores
        e2 = e1;
        e1 = e0;
        e0 = error;
        
        // Calcular salida incremental
        float du = q0 * e0 + q1 * e1 + q2 * e2;
        
        // Salida total
        float u = u_prev + du;
        
        // Saturación
        if (u > u_max) u = u_max;
        if (u < u_min) u = u_min;
        
        // Guardar para próxima iteración
        u_prev = u;
        
        return u;
    }
    
    void reset() {
        e0 = 0.0f;
        e1 = 0.0f;
        e2 = 0.0f;
        u_prev = 0.0f;
    }
    
    // Métodos para ajustar parámetros en tiempo real (opcional)
    void set_Kp(float new_Kp) { Kp = new_Kp; calculate_pid_coefficients(); }
    void set_Ti(float new_Ti) { Ti = new_Ti; calculate_pid_coefficients(); }
    void set_Td(float new_Td) { Td = new_Td; calculate_pid_coefficients(); }
};


//////////////////////////////////////////////////////
// --- Clase Servo
//////////////////////////////////////////////////////
class Servo {
private:
    const uint PWM_PIN;
    uint pwmSlice;
    float current_angle;
   
public:
    Servo(uint pwm_pin) : PWM_PIN(pwm_pin), current_angle(0.0f) {
        gpio_set_function(PWM_PIN, GPIO_FUNC_PWM);
        pwmSlice = pwm_gpio_to_slice_num(PWM_PIN);
        pwm_set_wrap(pwmSlice, 24999);
        pwm_set_clkdiv(pwmSlice, 100.0f);
        pwm_set_chan_level(pwmSlice, PWM_CHAN_A, 0);
        pwm_set_enabled(pwmSlice, true);
    }
   
    void set_position(float angle_rad) {
        current_angle = angle_rad;
        float angle_deg = angle_rad * (180.0f / M_PI);
        float pulse_ms = 1.5f + (angle_deg / 90.0f) * 0.5f;
        if (pulse_ms < 1.0f) pulse_ms = 1.0f;
        if (pulse_ms > 2.0f) pulse_ms = 2.0f;
        uint16_t level = (uint16_t)(pulse_ms / 20.0f * 25000);
        pwm_set_chan_level(pwmSlice, PWM_CHAN_A, level);
    }
   
    float get_position() { return current_angle; }


    // Dentro de la clase Servo, después de get_position():
//     float map_linear_to_angle(float z_m) {
//     const float max_z = 0.08f;  // Límite del prismatic
//     const float max_angle_rad = M_PI;  // 180° en rad (ajusta si tu servo es <180°)
//     float angle_rad = (z_m / max_z) * max_angle_rad;
//     if (angle_rad < 0.0f) angle_rad = 0.0f;
//     if (angle_rad > max_angle_rad) angle_rad = max_angle_rad;
//     return angle_rad;
// }

// float map_linear_to_angle(float z_m) {
//     const float min_z = -0.08f;   
//     const float max_z = 0.0f;     
//     const float max_angle_rad = M_PI;  

//     float angle_rad = (1.0f-((z_m - min_z) / (max_z - min_z))) * max_angle_rad;

//     // Limitar por seguridad


//     return angle_rad;
// }


float map_linear_to_angle(float z_m) {
    // Rango esperado desde IK
    const float min_z = 0.0f;      // Prismático retraído
    const float max_z = 0.08f;     // Prismático extendido
    
    // Rango del servo
    const float min_angle_rad = 0.0f;      // 0° (retraído)
    const float max_angle_rad = M_PI;      // 180° (extendido)
    
    // Normalizar z_m a [0, 1]
    float t = (z_m - min_z) / (max_z - min_z);
    
    // Clamp
    if (t < 0.0f) t = 0.0f;
    if (t > 1.0f) t = 1.0f;
    
    // Interpolación lineal
    float angle_rad = min_angle_rad + t * (max_angle_rad - min_angle_rad);
    
    return angle_rad;
}

};

//////////////////////////////////////////////////////
// --- Homing Function
//////////////////////////////////////////////////////
void motorHome(Motor &motor, int32_t offset_ticks =0) {
    
    while (true) {
        if (gpio_get(motor.get_fc1()) == 0) {
            motor.set_motor(0);
            sleep_ms(200);
            
            motor.reset_ticks();
            break;
        }
        if (gpio_get(motor.get_fc2()) == 0) {
            motor.set_motor(0);
            break;
        }
        motor.set_motor(20);
    }
   
    
    while (true) {
        if (gpio_get(motor.get_fc2()) == 0) {
            motor.set_motor(0);
            sleep_ms(200);
            
            break;
        }
        motor.set_motor(-20);
    }

    int32_t total_ticks = motor.get_ticks();
    int32_t center_ticks = total_ticks / 2 + offset_ticks;

   
    while (abs(motor.get_ticks() - center_ticks) > 5) {
        if (motor.get_ticks() < center_ticks)
            motor.set_motor(20);
        else
            motor.set_motor(-20);
    }
   
    motor.set_motor(0);
    sleep_ms(200);
    motor.reset_ticks();
}


//amarillo motor 1 pin 20 blanco 21
//amarillo motor 2 pin 4 blanco 5   



// Motor(uint led_pin, uint ena_pin, uint in1_pin, uint in2_pin,
     //     uint enc_a_pin, uint enc_b_pin, uint fc1, uint fc2,
      //    int ticks_per_rev = 64, float gear_ratio = 50.0f)
Motor motor1(25, 2, 6, 13, 20, 21, 17, 16, 64, 50.0f);
Motor motor2(25, 9, 11, 10, 4, 5, 19, 18, 64, 50.0f);
Servo servo1(15);


// Velocity PIDs - Start with conservative gains
// VelocityPID pid_vel_1(0.5f, 3.0f, 0.0f, -100.0f, 100.0f, 20.0f);
// VelocityPID pid_vel_2(0.3f, 2.5f, 0.0f, -100.0f, 100.0f, 20.0f);

//diego velcodiad

//Ti =0.0264
//Td = 0.0066

//kp posicion 1


// VelocityPID pid_vel_1(3.5f, 3.0f, 0.0f, -100.0f, 100.0f, 20.0f);
// VelocityPID pid_vel_2(0.3f, 2.5f, 0.0f, -100.0f, 100.0f, 20.0f);


const float Ts = 0.012f;
VelocityPID pid_vel_1(
    1.0f,      // Kp
    0.0264f,   // Ti (tiempo integral)
    0.0066f,   // Td (tiempo derivativo)
    Ts,        // Ts = 0.012 s
    -100.0f,   // u_min
    100.0f     // u_max
);

// ========================================
// MOTOR 2 - VELOCITY PID
// ========================================
VelocityPID pid_vel_2(
    1.0f,      // Kp
    0.0264f,   // Ti (tiempo integral)
    0.0066f,   // Td (tiempo derivativo)
    Ts,        // Ts = 0.012 s
    -100.0f,   // u_min
    100.0f     // u_max
);



// Unos 0.57 grados de tolerancia
const float position_dead_zone = 0.00f; 

// Unos 0.86 grados de tolerancia (un poco más para el motor 2 si quieres)
const float position_dead_zone_2 = 0.00f; 

// PositionPID pid_pos_1(15.0f, 0.2f, 1.0f, -15.0f, 15.0f, 0.5f, position_dead_zone);
// PositionPID pid_pos_2(18.0f, 0.25f, 1.2f, -16.0f, 16.0f, 0.5f, position_dead_zone_2);

// PositionPID pid_pos_1(15.0f, 0.2f, 1.0f, -15.0f, 15.0f, 0.5f, position_dead_zone);
// PositionPID pid_pos_2(18.0f, 0.25f, 1.2f, -16.0f, 16.0f, 0.5f, position_dead_zone_2);
PositionPID pid_pos_1(1.0f, 0.0f, 0.0f, -16.0f, 16.0f, 0.5f, position_dead_zone);
PositionPID pid_pos_2(1.0f, 0.0f, 0.0f, -15.0f, 15.0f, 0.8f, position_dead_zone_2);


// Position controller gains (converts position error to velocity command)
        // rad/s per radian error
     // Maximum velocity command (rad/s)

// Setpoints from trajectory
float desired_pos_1 = 0.0f;
float desired_vel_1 = 0.0f;
float desired_pos_2 = 0.0f;
float desired_vel_2 = 0.0f;
float desired_servo = 0.0f;


// State variables
float prev_position_1 = 0.0f;
float prev_position_2 = 0.0f;

bool homing_done = false;
bool initial_cmd_received = false;

//////////////////////////////////////////////////////
// --- ROS2 Messages
//////////////////////////////////////////////////////
rcl_subscription_t cmd_subs;
geometry_msgs__msg__Twist cmd_msg;
rcl_publisher_t debug_pub;
geometry_msgs__msg__Twist debug_msg;

void cmd_callback(const void * msgin) {
    const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;

    const float GEAR_RATIO = 3.0f;
   
    // Positions
    desired_pos_1 = msg->angular.z * GEAR_RATIO;
    desired_pos_2 = msg->angular.x * GEAR_RATIO;
    desired_servo = msg->linear.y;
   
    // Velocities (feedforward from trajectory)
    desired_vel_1 = msg->linear.x * GEAR_RATIO;
    desired_vel_2 = msg->angular.y * GEAR_RATIO;
   
    initial_cmd_received = true;
}
// No staircase


// void control_timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
//     // One-time homing
//     if (!homing_done) {
//         motorHome(motor2,-40);
//         motorHome(motor1,-110);
//         homing_done = true;
//         return;
//     }
   
//     // Wait for initial command
//     if (!initial_cmd_received) {
//         motor1.set_motor(0);
//         motor2.set_motor(0);
//         return;
//     }
   
//     const float dt = 0.012f;  // 12 ms control period
   
//     // ========================================
//     // PARÁMETROS DE CONTROL (STAIRCASE DESACTIVADO)
//     // ========================================
//     // const float ERROR_THRESHOLD_PID = 0.5f;
//     // const float ERROR_THRESHOLD_STOP = 0.03f;
//     // const float MIN_PWM_APPROACH = 15.0f;
   
//     // ========================================
//     // MOTOR 1: CONTROL PID CONTINUO
//     // ========================================
   
//     // 1. Measure current position and velocity
//     float pos_actual_1 = motor1.get_position_rad();
//     float vel_actual_1 = (pos_actual_1 - prev_position_1) / dt;
    
//     // 2. Low-pass filter velocity
//     static float filtered_vel_1 = 0.0f;
//     const float alpha = 0.15f;
//     filtered_vel_1 = alpha * vel_actual_1 + (1.0f - alpha) * filtered_vel_1;
   
//     // 3. Calculate position error and velocity correction
//     float vel_correction_1 = pid_pos_1.calculate(desired_pos_1, pos_actual_1, dt);
   
//     // 4. Combine feedforward with correction
//     float vel_setpoint_1 = desired_vel_1 + vel_correction_1;
   
//     // 5. Limit total velocity
//     if (vel_setpoint_1 > 16.0f) vel_setpoint_1 = 16.0f;
//     if (vel_setpoint_1 < -16.0f) vel_setpoint_1 = -16.0f;
   
//     // 6. PID loop for velocity
//     float pwm_1 = pid_vel_1.calculate(vel_setpoint_1, filtered_vel_1, dt);
   
//     // 7. Apply to motor
//     motor1.set_motor(pwm_1);
   
//     // ========================================
//     // MOTOR 2: CONTROL PID CONTINUO
//     // ========================================
   
//     float pos_actual_2 = motor2.get_position_rad();
//     float vel_actual_2 = (pos_actual_2 - prev_position_2) / dt;
    
//     static float filtered_vel_2 = 0.0f;
//     filtered_vel_2 = alpha * vel_actual_2 + (1.0f - alpha) * filtered_vel_2;
   
//     // Calculate velocity correction from position PID
//     float vel_correction_2 = pid_pos_2.calculate(desired_pos_2, pos_actual_2, dt);
   
//     float vel_setpoint_2 = desired_vel_2 + vel_correction_2;
    
//     if (vel_setpoint_2 > 16.0f) vel_setpoint_2 = 16.0f;
//     if (vel_setpoint_2 < -16.0f) vel_setpoint_2 = -16.0f;
   
//     float pwm_2 = pid_vel_2.calculate(vel_setpoint_2, filtered_vel_2, dt);
   
//     motor2.set_motor(pwm_2);
   
//     // ========== SERVO CONTROL ==========
//     float angle_for_servo = servo1.map_linear_to_angle(desired_servo);
//     servo1.set_position(angle_for_servo);
   
//     // ========== UPDATE STATE ==========
//     prev_position_1 = pos_actual_1;
//     prev_position_2 = pos_actual_2;
   
//     // ========== PUBLISH DEBUG INFO ==========
//     float pos_error_1 = desired_pos_1 - pos_actual_1;
//     float pos_error_2 = desired_pos_2 - pos_actual_2;
    
//     debug_msg.linear.x = pos_actual_1;           // Current position motor 1
//     debug_msg.linear.y = angle_for_servo;        // Servo position
//     debug_msg.linear.z = pos_error_1;            // Position error motor 1
//     debug_msg.angular.x = pos_actual_2;          // Current position motor 2
//     debug_msg.angular.y = pwm_1;                 // PWM motor 1
//     debug_msg.angular.z = pos_error_2;           // Position error motor 2
   
//     rcl_publish(&debug_pub, &debug_msg, NULL);
// }

void control_timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
    // One-time homing
    if (!homing_done) {
        motorHome(motor2,-40);
        motorHome(motor1,-110);
        homing_done = true;
        return;
    }
   
    // Wait for initial command
    if (!initial_cmd_received) {
        motor1.set_motor(0);
        motor2.set_motor(0);
        return;
    }
   
    const float dt = 0.012f;  // 100 ms control period
   
    // ========================================
    // PARÁMETROS DE CONTROL STAIRCASE
    // ========================================
    const float ERROR_THRESHOLD_PID = 0.09f;      // 0.5 rad (~28.6°) - Umbral para PID completo
    const float ERROR_THRESHOLD_STOP = 0.03f;   // 0.015 rad (~0.86°) - Zona muerta final
    const float MIN_PWM_APPROACH = 10.0f;         // PWM mínimo para acercamiento (ajustar según tu motor)
   
    // ========================================
    // MOTOR 1: CONTROL CON STAIRCASE
    // ========================================
   
    // 1. Measure current position and velocity
    float pos_actual_1 = motor1.get_position_rad();
    float vel_actual_1 = (pos_actual_1 - prev_position_1) / dt;
    
    // 2. Low-pass filter velocity
    static float filtered_vel_1 = 0.0f;
    const float alpha = 0.15f;
    filtered_vel_1 = alpha * vel_actual_1 + (1.0f - alpha) * filtered_vel_1;
   
    // 3. Calculate position error
    float pos_error_1 = desired_pos_1 - pos_actual_1;
    float abs_error_1 = fabs(pos_error_1);
    
    float pwm_1 = 0.0f;
    
    // ========================================
    // MÁQUINA DE ESTADOS PARA MOTOR 1
    // ========================================
    if (abs_error_1 < ERROR_THRESHOLD_STOP) {
        // ZONA MUERTA: Error muy pequeño → DETENER motor
        pwm_1 = 0.0f;
        pid_vel_1.reset();  // Reset PID para evitar acumulación de integral
        
    } else if (abs_error_1 < ERROR_THRESHOLD_PID) {
        // STAIRCASE: Error pequeño → PWM mínimo constante
        // Aplicar PWM mínimo en la dirección del error
        if (pos_error_1 > 0) {
            pwm_1 = MIN_PWM_APPROACH;  // Positivo: acercarse
        } else {
            pwm_1 = -MIN_PWM_APPROACH; // Negativo: retroceder
        }
        pid_vel_1.reset();  // No usar PID en esta zona
        
    } else {
        // PID COMPLETO: Error grande → Control normal
        float vel_correction_1 = pid_pos_1.calculate(desired_pos_1, pos_actual_1, dt);
        float vel_setpoint_1 = desired_vel_1 + vel_correction_1;
        
        // Limit total velocity
        if (vel_setpoint_1 > 16.0f) vel_setpoint_1 = 16.0f;
        if (vel_setpoint_1 < -16.0f) vel_setpoint_1 = -16.0f;
        
        // PID de velocidad
        pwm_1 = pid_vel_1.calculate(vel_setpoint_1, filtered_vel_1, dt);
    }
   
    // Apply to motor
    motor1.set_motor(pwm_1);
   
    // ========================================
    // MOTOR 2: MISMA ESTRUCTURA
    // ========================================
   
    float pos_actual_2 = motor2.get_position_rad();
    float vel_actual_2 = (pos_actual_2 - prev_position_2) / dt;
    
    static float filtered_vel_2 = 0.0f;
    filtered_vel_2 = alpha * vel_actual_2 + (1.0f - alpha) * filtered_vel_2;
   
    float pos_error_2 = desired_pos_2 - pos_actual_2;
    float abs_error_2 = fabs(pos_error_2);
    
    float pwm_2 = 0.0f;
    
    if (abs_error_2 < ERROR_THRESHOLD_STOP) {
        // ZONA MUERTA
        pwm_2 = 0.0f;
        pid_vel_2.reset();
        
    } else if (abs_error_2 < ERROR_THRESHOLD_PID) {
        // STAIRCASE
        if (pos_error_2 > 0) {
            pwm_2 = MIN_PWM_APPROACH;
        } else {
            pwm_2 = -MIN_PWM_APPROACH;
        }
        pid_vel_2.reset();
        
    } else {
        // PID COMPLETO
        float vel_correction_2 = pid_pos_2.calculate(desired_pos_2, pos_actual_2, dt);
        float vel_setpoint_2 = desired_vel_2 + vel_correction_2;
        
        if (vel_setpoint_2 > 16.0f) vel_setpoint_2 = 16.0f;
        if (vel_setpoint_2 < -16.0f) vel_setpoint_2 = -16.0f;
        
        pwm_2 = pid_vel_2.calculate(vel_setpoint_2, filtered_vel_2, dt);
    }
   
    motor2.set_motor(pwm_2);
   
    // ========== SERVO CONTROL ==========
    float angle_for_servo = servo1.map_linear_to_angle(desired_servo);
    servo1.set_position(angle_for_servo);
   
    // ========== UPDATE STATE ==========
    prev_position_1 = pos_actual_1;
    prev_position_2 = pos_actual_2;
   
    // ========== PUBLISH DEBUG INFO ==========
    debug_msg.linear.x = pos_actual_1;
    debug_msg.linear.y = angle_for_servo;
    debug_msg.linear.z = pos_error_1;           // ← Publicar ERROR (útil para debug)
    debug_msg.angular.x = pos_actual_2;
    debug_msg.angular.y = pwm_1;                // ← Publicar PWM aplicado
    debug_msg.angular.z = pos_error_2;          // ← Error motor 2
   
    rcl_publish(&debug_pub, &debug_msg, NULL);
}
int main() {
    rmw_uros_set_custom_transport(
        true, NULL,
        pico_serial_transport_open,
        pico_serial_transport_close,
        pico_serial_transport_write,
        pico_serial_transport_read
    );
   
    rcl_timer_t control_timer;
    rcl_node_t node;
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;
    rclc_executor_t executor;
   
    rcl_ret_t ret = rmw_uros_ping_agent(1000, 120);
    if (ret != RCL_RET_OK) {
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
        &control_timer,
        &support,
        RCL_MS_TO_NS(12), //estaba en 100
        control_timer_callback
    );
   
    rclc_subscription_init_default(
        &cmd_subs,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "cmd"
    );
   
    rclc_executor_init(&executor, &support.context, 3, &allocator);
    rclc_executor_add_subscription(&executor, &cmd_subs, &cmd_msg, &cmd_callback, ON_NEW_DATA);
    rclc_executor_add_timer(&executor, &control_timer);
   
    while (true) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(12));
    }
   
    return 0;
}
