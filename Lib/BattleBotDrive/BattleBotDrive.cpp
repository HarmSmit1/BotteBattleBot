#include "Arduino.h"
#include "BattleBotDrive.h"

#define drivingLowerLimit 50
#define drivingUpperLimit 250

/**
 * This is the BattleBot constructor that initiates the I/O pins for driving the cart.
 *
 * @param leftMotorForwardPin
 * @param rightMotorForwardPin
 * @param leftMotorBackwardPin
 * @param rightMotorBackwardPin
 */
BattleBotDrive::BattleBotDrive(int leftMotorForwardPin, int rightMotorForwardPin, int leftMotorBackwardPin, int rightMotorBackwardPin)
{
    pinMode(leftMotorForwardPin, OUTPUT); // Sets the I/O pin for the left motor forward to output mode.
    pinMode(leftMotorBackwardPin, OUTPUT); // Sets the I/O pin for the left motor backward to output mode.
    pinMode(rightMotorForwardPin, OUTPUT); // Sets the I/O pin for the right motor forward to output mode.
    pinMode(rightMotorBackwardPin, OUTPUT); // Sets the I/O pin for the right motor backward to output mode.

    _leftMotorForwardPin = leftMotorForwardPin;
    _leftMotorBackwardPin = leftMotorBackwardPin;
    _rightMotorForwardPin = rightMotorForwardPin;
    _rightMotorBackwardPin = rightMotorBackwardPin;
}

/**
 * This function will stop all power on the left motor.
 */
void BattleBotDrive::breakLeft( )
{
    digitalWrite( _leftMotorBackwardPin, LOW );
    digitalWrite( _leftMotorForwardPin, LOW );
}

/**
 * This function will stop all power on the right motor.
 */
void BattleBotDrive::breakRight()
{
    digitalWrite( _rightMotorBackwardPin, LOW );
    digitalWrite( _rightMotorForwardPin, LOW );
}

/**
 * This function breaks the battle bot.
 */
void BattleBotDrive::breakCart()
{
    breakLeft();
    breakRight();
}

/**
 * This function drives controlles the left motor. You can pass it an value between -200 for moving backward
 * and +200 for moving forward.
 *
 * @param speed
 */
void BattleBotDrive::driveRightMotor( int speed )
{
    breakRight(); // Reset the current motor state.
    if( speed < 0 )
    {
        speed = abs( speed );
        speed += drivingLowerLimit;
        speed = ( speed > drivingUpperLimit ) ? drivingUpperLimit : speed;
        _currentRightDrivingSpeed = speed;
        digitalWrite( _rightMotorBackwardPin, HIGH ); // set the backward I/O pin to high
        analogWrite( _rightMotorForwardPin, 255 - speed ); // limit the backward speed.
    }
    else if( speed == 0 )
    {
        breakRight();
    }
    else
    {
        speed = abs( speed );
        speed += drivingLowerLimit;
        speed = ( speed > drivingUpperLimit ) ? drivingUpperLimit : speed;
        _currentRightDrivingSpeed = speed;
        analogWrite( _rightMotorForwardPin, speed );
    }
}

/**
 * This function drives controlles the left motor. You can pass it an value between -200 for moving backward
 * and +200 for moving forward.
 *
 * @param speed
 */
void BattleBotDrive::driveLeftMotor( int speed )
{
    breakLeft(); // Reset the current motor state.
    if( speed < 0 )
    {
        speed = abs( speed );
        speed = ( speed > 200 ) ? 200 : speed;
        speed += drivingLowerLimit;
        _currentLeftDrivingSpeed = speed;
        digitalWrite( _leftMotorBackwardPin, HIGH ); // set the backward I/O pin to high
        analogWrite( _leftMotorForwardPin, 255 - speed ); // limit the backward speed.
    }
    else if( speed == 0 )
    {
        breakLeft();
    }
    else
    {
        speed = abs( speed );
        speed = ( speed > 200 ) ? 200 : speed;
        speed += drivingLowerLimit;
        _currentLeftDrivingSpeed = speed;
        analogWrite( _leftMotorForwardPin, speed );
    }
}

/**
 * This function drives the BattleBot.
 *
 * @param left range -200 for 255 backward and +200 for 255 forward
 * @param right
 */
void BattleBotDrive::drive( int leftMotorSpeed, int rightMotorSpeed )
{
    driveRightMotor( rightMotorSpeed );
    driveLeftMotor( leftMotorSpeed );
}

int BattleBotDrive::getLeftDrivingSpeed()
{
    return _currentLeftDrivingSpeed;
}

int BattleBotDrive::getRightDrivingSpeed()
{
    return _currentRightDrivingSpeed;
}
