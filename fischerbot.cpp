/**
 * This file is part of the grappendorf.net FischerBot project.
 *
 * The contents of this file are subject to the Apache License Version
 * 2.0 (the "License"); you may not use this file except in compliance with
 * the License. You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Software distributed under the License is distributed on an "AS IS" basis,
 * WITHOUT WARRANTY OF ANY KIND, either express or implied. See the License
 * for the specific language governing rights and limitations under the
 * License.
 *
 * The Original Code is grappendorf.net FischerBot
 *
 * The Initial Developer of the Original Code is
 * Dirk Grappendorf (www.grappendorf.net)
 * Portions created by the Initial Developer are Copyright (C) 2011
 * the Initial Developer. All Rights Reserved.
 */

/**
 * Processor ATmega1284P 16 MHz (Calunium)
 *
 * Low Fuse:		0xFF
 * High Fuse:		0xD8
 * Extended Fuse:	0xFD
 */

/**
 * I/O Pins (Sanguino compatible):
 *
 * Function					Port/Pin		Arduino
 *
 * Power switch				PD7				23
 * Power enable				PC2				19
 * UART select				PB4				10
 * Light left				PB0				4
 * Light right				PB1				5
 * Bumper left				PB3				7
 * Bumper right				PB2				6
 * Motor left forward		PD4				22
 * Motor left backward		PC3				18
 * Motor right forward		PD5				9
 * Motor right backward		PD6				8
 * Buzzer					PA3				28
 * CMPS03 calibration		PA4				27
 */

/**
 * If you want to use this code inside the Arduino IDE, remove the WProgram.h
 * include and delete the path prefixes from the other includes. Also remove
 * the main() function.
 */

#include <Arduino.h>
#include "Wire/Wire.h"
#include <Bounce/Bounce.h>
#include <CMPS03/CMPS03.h>
#include <SRF02/SRF02.h>
#include <WiFlySerial/WiFlySerial.h>
#include <WheelEncoder/WheelEncoder.h>

const long BAUD_RATE = 57600;
const long BAUD_RATE_WIFLY = 9600;

const uint8_t PIN_SWITCH = 23;
const uint8_t PIN_POWER = 19;
const uint8_t PIN_UART_SELECT = 10;
const uint8_t PIN_LIGHT_L = 4;
const uint8_t PIN_LIGHT_R = 5;
const uint8_t PIN_BUMP_L = 7;
const uint8_t PIN_BUMP_R = 6;
const uint8_t PIN_MOT_LF = 22;
const uint8_t PIN_MOT_LB = 18;
const uint8_t PIN_MOT_RF = 9;
const uint8_t PIN_MOT_RB = 8;
const uint8_t PIN_BUZZ = 28;
const uint8_t PIN_CMPS03_CAL = 27;

const uint8_t UART_USB = LOW;
const uint8_t UART_XBEE = HIGH;

Bounce powerSwitch(PIN_SWITCH, 10);
Bounce bumperLeft(PIN_BUMP_L, 10);
Bounce bumperRight(PIN_BUMP_R, 10);

CMPS03 cmps03;

SRF02 srf02Left(0x72);
SRF02 srf02Center(0x70);
SRF02 srf02Right(0x71);

unsigned long lastMillis = 0;

enum State
{
	POWER_ON, WAIT, EXPLORE_START, EXPLORE_FORWARD, EXPLORE_TURN, EXPLORE_STRUCK
};

void mainStates();

void (*stateEngine)() = mainStates;

State state = POWER_ON;

unsigned int MIN_CENTER_DISTANCE_TO_OBSTACLE = 25;
unsigned int MIN_SIDE_DISTANCE_TO_OBSTACLE = 25;

void buzz(int pin, long frequency, long length)
{
	long delayValue = 1000000 / frequency / 2;
	long numCycles = frequency * length / 1000;
	for (long i = 0; i < numCycles; i++)
	{
		digitalWrite(pin, HIGH);
		delayMicroseconds(delayValue);
		digitalWrite(pin, LOW);
		delayMicroseconds(delayValue);
	}
}

enum MotorMode
{
	HALT, FORWARD, BACKWARD, TURN_LEFT, TURN_RIGHT
};

void motor(MotorMode mode)
{
	switch (mode)
	{
		case HALT:
			digitalWrite(PIN_MOT_LF, LOW);
			digitalWrite(PIN_MOT_RF, LOW);
			digitalWrite(PIN_MOT_LB, LOW);
			digitalWrite(PIN_MOT_RB, LOW);
			break;
		case FORWARD:
			digitalWrite(PIN_MOT_LF, HIGH);
			digitalWrite(PIN_MOT_RF, HIGH);
			digitalWrite(PIN_MOT_LB, LOW);
			digitalWrite(PIN_MOT_RB, LOW);
			break;
		case BACKWARD:
			digitalWrite(PIN_MOT_LF, LOW);
			digitalWrite(PIN_MOT_RF, LOW);
			digitalWrite(PIN_MOT_LB, HIGH);
			digitalWrite(PIN_MOT_RB, HIGH);
			break;
		case TURN_LEFT:
			digitalWrite(PIN_MOT_LF, LOW);
			digitalWrite(PIN_MOT_RF, HIGH);
			digitalWrite(PIN_MOT_LB, HIGH);
			digitalWrite(PIN_MOT_RB, LOW);
			break;
		case TURN_RIGHT:
			digitalWrite(PIN_MOT_LF, HIGH);
			digitalWrite(PIN_MOT_RF, LOW);
			digitalWrite(PIN_MOT_LB, LOW);
			digitalWrite(PIN_MOT_RB, HIGH);
			break;
	}
}

enum LightMode
{
	NONE, LEFT, RIGHT, BOTH
};

void light(LightMode mode)
{
	digitalWrite(PIN_LIGHT_L, mode == LEFT || mode == BOTH ? HIGH : LOW);
	digitalWrite(PIN_LIGHT_R, mode == RIGHT || mode == BOTH ? HIGH : LOW);
}

void exploreStates()
{
	switch (state)
	{
		case EXPLORE_START:
			state = EXPLORE_FORWARD;
			break;

		case EXPLORE_FORWARD:
			SRF02::setInterval(500);
			light(NONE);
			motor(FORWARD);
			if (bumperRight.fallingEdge() || bumperLeft.fallingEdge())
			{
				state = EXPLORE_STRUCK;
				break;
			}
			if (srf02Center.read() < MIN_CENTER_DISTANCE_TO_OBSTACLE || srf02Left.read() < MIN_SIDE_DISTANCE_TO_OBSTACLE
					|| srf02Right.read() < MIN_SIDE_DISTANCE_TO_OBSTACLE)
			{
				state = EXPLORE_TURN;
			}
			break;

		case EXPLORE_TURN:
			SRF02::setInterval(100);
			light(BOTH);
			if (srf02Center.read() >= MIN_CENTER_DISTANCE_TO_OBSTACLE
					&& srf02Left.read() >= MIN_SIDE_DISTANCE_TO_OBSTACLE
					&& srf02Right.read() >= MIN_SIDE_DISTANCE_TO_OBSTACLE)
			{
				state = EXPLORE_FORWARD;
			}
			else
			{
				if (srf02Left.read() >= MIN_SIDE_DISTANCE_TO_OBSTACLE)
				{
					motor(TURN_LEFT);
				}
				else if (srf02Right.read() >= MIN_SIDE_DISTANCE_TO_OBSTACLE)
				{
					motor(TURN_RIGHT);
				}
				else
				{
					state = EXPLORE_STRUCK;
				}
			}
			break;

		case EXPLORE_STRUCK:
			light(NONE);
			motor(HALT);
			stateEngine = mainStates;
			state = WAIT;
			break;

		default:
			break;
	}
}

void mainStates()
{
	switch (state)
	{
		case POWER_ON:
			if (millis() > lastMillis + 500)
			{
				light(BOTH);
				buzz(PIN_BUZZ, 523, 150);
				buzz(PIN_BUZZ, 659, 150);
				buzz(PIN_BUZZ, 783, 150);
				buzz(PIN_BUZZ, 880, 150);
				delay(100);
				buzz(PIN_BUZZ, 783, 100);
				buzz(PIN_BUZZ, 880, 200);
				light(NONE);
				state = WAIT;
			}
			break;

		case WAIT:
			SRF02::setInterval(0);
			if (bumperLeft.fallingEdge() || bumperRight.fallingEdge())
			{
				stateEngine = exploreStates;
				state = EXPLORE_START;
				break;
			}
			if (digitalRead(PIN_UART_SELECT) == UART_USB)
			{
				if (Serial.available())
				{
					Serial.read();
					light(BOTH);
					delay(100);
					light(NONE);
					Serial.print("\r\nwl=");
					Serial.print(WheelEncoder::getLeftDistance());
					Serial.print(",wr=");
					Serial.print(WheelEncoder::getRightDistance());
					Serial.print(",dl=");
					Serial.print(srf02Left.read());
					Serial.print(",dc=");
					Serial.print(srf02Center.read());
					Serial.print(",dr=");
					Serial.print(srf02Right.read());
					Serial.print(",c=");
					Serial.print(cmps03.read());
					Serial.print(",bl=");
					Serial.print(bumperLeft.read());
					Serial.print(",br=");
					Serial.print(bumperRight.read());
					Serial.print("\r\n");
				}
			}
			break;

		default:
			break;
	}
}

/**
 * System setup.
 */
void setup()
{
	pinMode(PIN_UART_SELECT, OUTPUT);
	digitalWrite(PIN_UART_SELECT, UART_USB);
	pinMode(PIN_LIGHT_L, OUTPUT);
	pinMode(PIN_LIGHT_R, OUTPUT);
	digitalWrite(PIN_LIGHT_L, HIGH);
	digitalWrite(PIN_LIGHT_R, HIGH);
	pinMode(PIN_BUMP_L, INPUT);
	pinMode(PIN_BUMP_R, INPUT);
	digitalWrite(PIN_BUMP_L, HIGH);
	digitalWrite(PIN_BUMP_R, HIGH);
	pinMode(PIN_SWITCH, INPUT);
	digitalWrite(PIN_SWITCH, HIGH);
	pinMode(PIN_POWER, OUTPUT);
	digitalWrite(PIN_POWER, HIGH);
	pinMode(PIN_MOT_LF, OUTPUT);
	pinMode(PIN_MOT_LB, OUTPUT);
	pinMode(PIN_MOT_RF, OUTPUT);
	pinMode(PIN_MOT_RB, OUTPUT);
	digitalWrite(PIN_MOT_LF, LOW);
	digitalWrite(PIN_MOT_LB, LOW);
	digitalWrite(PIN_MOT_RF, LOW);
	digitalWrite(PIN_MOT_RB, LOW);
	pinMode(2, INPUT);
	pinMode(3, INPUT);
	pinMode(PIN_BUZZ, OUTPUT);
	digitalWrite(PIN_BUZZ, LOW);
	WheelEncoder::init();
	Wire.begin();
	Serial.begin(BAUD_RATE);
}

/**
 * Main execution loop.
 */
void loop()
{
	if (state != POWER_ON)
	{
		bumperLeft.update();
		bumperRight.update();
		powerSwitch.update();
		//	WheelEncoder::update ();
		SRF02::update();

		if (powerSwitch.fallingEdge())
		{
			light(BOTH);
			buzz(PIN_BUZZ, 880, 150);
			buzz(PIN_BUZZ, 783, 150);
			buzz(PIN_BUZZ, 659, 150);
			buzz(PIN_BUZZ, 523, 150);
			digitalWrite(PIN_POWER, LOW);
		}
	}

	stateEngine();
}

/**
 * Main program entry point.
 * If you want to use this code inside the Arduino IDE, remove this function.
 */
int main()
{
	init();
	setup();
	sei();
	for (;;)
	{
		loop();
	}
	return 0;
}
