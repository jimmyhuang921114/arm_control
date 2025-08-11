#include "linear_stepper.hpp"

LinearStepper::LinearStepper(
    uint16_t motor_steps, 
    uint8_t dir_pin, 
    uint8_t step_pin, 
    uint8_t en_pin,
    uint8_t endstop_pin, 
    bool endstop_level
):
    stepper(motor_steps, dir_pin, step_pin, en_pin),
    moving(false),
    homed(false),
    start_position(0),
    position(0),
    velocity(0),
    motor_steps(motor_steps),
    endstop_pin(endstop_pin),
    endstop_level(endstop_level),
    motor_steps_accel(0),
    deg_per_mm(0),
    rpm_per_mmps(0),
    mm_per_step(0),
    homing_velocity(0)
{
    // Serial.println("start constructor");
    // stepper.begin(30, micro_steps);
    // Serial.println("end constructor");
}


LinearStepper::~LinearStepper() {}


void LinearStepper::init(uint8_t micro_steps, uint32_t linear_accel, float mm_per_rev, float homing_velocity) {
    motor_steps_accel = linear_accel / mm_per_rev * motor_steps;
    deg_per_mm = 360.0f / mm_per_rev;
    rpm_per_mmps = 60.0f / mm_per_rev;
    this->homing_velocity = homing_velocity;
    mm_per_step = mm_per_rev / motor_steps / micro_steps;
    stepper.begin(30, micro_steps);
    stepper.setEnableActiveState(LOW);
	stepper.enable();
	stepper.setSpeedProfile(stepper.LINEAR_SPEED, motor_steps_accel, motor_steps_accel);
}


void LinearStepper::home() {
    /* coarse homing */
    stepper.setRPM(vel2rpm(homing_velocity));
    // Serial.println(vel2rpm(homing_velocity));
    // stepper.rotate(1e9);
    stepper.startRotate(-999999);
    do {
        stepper.nextAction();
    }while(digitalRead(endstop_pin) != endstop_level);
    stepper.stop();
    // Serial.println("coarse finish");
    /* move out */
    move(10, 10, true);
    /* fine homing */
    stepper.setRPM(vel2rpm(10));
    // stepper.rotate(1e9);
    stepper.startRotate(-999999);
    do {
        stepper.nextAction();
    }while(digitalRead(endstop_pin) != endstop_level);
    stepper.stop();
    position = 0;
    start_position = 0;
    moving = false;
    homed = true;
}


void LinearStepper::moveTo(float postion, float velocity, bool blocking) {
    // homed check
    if(!this->homed) {
        return;
    }
    float distance = postion - this->position;
    // Serial.println(distance);
    this->move(distance, velocity, blocking);
}


void LinearStepper::move(float distance, float velocity, bool blocking) {
    stepper.setRPM(vel2rpm(velocity));
    stepper.startRotate(dist2deg(distance));
    // Serial.println(dist2deg(distance));
    moving = true;
    start_position = position;
    // blocking mode
    if(blocking) {
        unsigned wait_time = 0;
        do {
            wait_time = stepper.nextAction();
        }while(wait_time > 0);
        this->position += distance;
        moving = false;
    }
}


void LinearStepper::handle() {
    if(moving) {
        unsigned wait_time = stepper.nextAction();
        int dir = stepper.getDirection();
        velocity = dir * stepper.getCurrentRPM() / rpm_per_mmps;
        position = start_position + dir * stepper.getStepsCompleted() * mm_per_step;
        // move finish
        if(!wait_time) {
            moving = false;
        }
    }
}


float LinearStepper::get_position() {
    return position;
}


float LinearStepper::get_velocity() {
    return velocity;
}


bool LinearStepper::is_moving() {
    return moving;
}


bool LinearStepper::is_homed() {
    return homed;   
}


/* ========== [ Private ] ========== */

float LinearStepper::dist2deg(float dist) {
    return dist * deg_per_mm;
}


float LinearStepper::vel2rpm(float vel) {
    return vel * rpm_per_mmps;
}