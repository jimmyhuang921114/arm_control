#include <Arduino.h>
#include "linear_stepper.hpp"
#include "serial_packet.hpp"
#include <Servo.h>

#define ARM_NAME			("main_rail")
// #define ARM_NAME			("arm_rail1")

#define EN_PIN				(8)
#define DIR_PIN				(5)
#define STEP_PIN			(2)
#define ENDSTOP_PIN			(9)
#define BAG_SENS1_PIN		(A0)	// active low
#define BAG_SENS2_PIN		(A1)	// active low
#define SERVO_PIN			(10)	// CCW

#define MOTOR_STEPS 		(200)
#define MICROSTEPS 			(8)
#define DISTANCE_PER_REV	(60.0f*2)	// mm

#define LINEAR_ACCEL 		(3000)	// mm/s^2
#define LINEAR_MAX_VEL		(600)	// mm/s
#define LINEAR_SLOW_VEL 	(30)	// mm/s

Servo bag_servo;
LinearStepper rail(MOTOR_STEPS, DIR_PIN, STEP_PIN, EN_PIN, ENDSTOP_PIN, HIGH);

void setup() {
	Serial.begin(115200);
	pinMode(ENDSTOP_PIN, INPUT);
	pinMode(BAG_SENS1_PIN, INPUT);
	pinMode(BAG_SENS2_PIN, INPUT);
	bag_servo.attach(SERVO_PIN);
	bag_servo.write(0);

	rail.init(MICROSTEPS, LINEAR_ACCEL, DISTANCE_PER_REV, 100);
	// Serial.println("<NAME>arm_rail_1");
	// Serial.println("<NAME>main_rail");
	// delay(1000);
	// rail.home();
	// delay(100);
	// rail.moveTo(500, 100, false);
	// rail.moveTo(0, 600, false);
	// rail.move(-500, 600, true);
	// for(int i = 0; i < 10; i++) {
	// 	rail.move(300, 600, true);
	// 	rail.move(-300, 600, true);
	// }
	// delay(100);
	// Serial.println("end setup");
}

int step = 0;

void loop() {
	rail.handle();
	// Serial.println("hello");
	// delay(1000);
	handle_serial();
	static uint32_t last_status_time = 0;
	if(millis() - last_status_time >= 100) {
		last_status_time = millis();
		packet_t status = CREATE_PACKET(TYPE_STATUS);
		status.data[0] = rail.get_position() * 10.0f;
		status.data[1] = rail.get_velocity() * 10.0f;
		status.data[2] = rail.is_homed();
		status.data[3] = rail.is_moving();
		status.data[4] = !digitalRead(BAG_SENS1_PIN) && !digitalRead(BAG_SENS2_PIN);
		send_packet(&status);
	}
	static uint32_t last_name_time = 0;
	if(millis() - last_name_time >= 1000) {
		last_name_time = millis();
		packet_t name = CREATE_PACKET(TYPE_NAME);
		strcpy((char*)name.data, ARM_NAME);
		send_packet(&name);
	}
}


void handle_packet(packet_t packet) {
    switch(packet.type) {
        case TYPE_HOME:
            rail.home();
            break;
        case TYPE_GOTO:
            rail.moveTo(packet.data[0] * 0.1f, packet.data[1] * 0.1f);
            break;
        case TYPE_MOVE:
            rail.move(packet.data[0] * 0.1f, packet.data[1] * 0.1f);
            break;
        case TYPE_BAG:
			bag_servo.write(packet.data[0]);
            break;
    }
}