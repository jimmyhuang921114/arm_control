#ifndef __LINEAR_STEPPER_HPP__
#define __LINEAR_STEPPER_HPP__

#include <A4988.h>

class LinearStepper{
public:
    LinearStepper(
        uint16_t motor_steps, 
        uint8_t dir_pin, 
        uint8_t step_pin, 
        uint8_t en_pin,
        uint8_t endstop_pin, 
        bool endstop_level
    );

    ~LinearStepper();

    void init(uint8_t micro_steps, uint32_t linear_accel, float mm_per_rev, float homing_velocity);

    void home();

    void moveTo(float postion, float velocity, bool blocking = false);

    void move(float distance, float velocity, bool blocking = false);

    void handle();

    float get_position();

    float get_velocity();

    bool is_moving();

    bool is_homed();

private:
    A4988 stepper;
    bool moving;
    bool homed;
    float start_position;       // mm
    float position;             // mm
    float velocity;             // mm/s
    uint16_t motor_steps;
    uint8_t endstop_pin;
    bool endstop_level;
    float motor_steps_accel;    // step/s^2
    float deg_per_mm;           // distance * this = degree
    float rpm_per_mmps;         // velocity * this = rpm
    float mm_per_step;          // step * this = mm
    float homing_velocity;

    float dist2deg(float dist);

    float vel2rpm(float vel);
};



#endif