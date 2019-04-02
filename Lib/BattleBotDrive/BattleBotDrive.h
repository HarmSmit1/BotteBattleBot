#ifndef BattleBotDrive_H
#define BattleBotDrive_H

class BattleBotDrive
{
public:
    /**
     * The Drive constructor for initiating the I/O pins for the Drive class.
     * @param leftMotorForwardPin
     * @param rightMotorForwardPin
     * @param leftMotorBackwardPin
     * @param rightMotorBackwardPin
     */
    BattleBotDrive(int leftMotorForwardPin, int rightMotorForwardPin, int leftMotorBackwardPin, int rightMotorBackwardPin);

    /**
     * This function drives controlles the left motor. You can pass it an value between -200 for moving backward
     * and +200 for moving forward.
     * @param speed
     */
    void driveLeftMotor( int speed );

    /**
     * This function drives controlles the right motor. You can pass it an value between -200 for moving backward
     * and +200 for moving forward.
     * @param speed
     */
    void driveRightMotor( int speed );

    /**
     * Function declaration for driving the card. You can pass it an value for each motor between -200 for moving backward
     * and +200 for moving forward.
     * @param left
     * @param right
     */
    void drive( int leftMotorSpeed, int rightMotorSpeed );

    /**
     * This function will stop all power on the left motor.
     */
    void breakLeft( );

    /**
     * This function will stop all power on the right motor.
     */
    void breakRight();

    /**
     * This function breaks the battle bot.
     */
    void breakCart();

    int getLeftDrivingSpeed();

    int getRightDrivingSpeed();
    
private:
    /**
     * Properties that hold the I/O pin numbers for controlling the motors.
     */
    int _leftMotorForwardPin;
    int _rightMotorForwardPin;
    int _leftMotorBackwardPin;
    int _rightMotorBackwardPin;

    /**
     * Properties that hold the current driving speed of the left and right motor.
     */
    int _currentLeftDrivingSpeed;
    int _currentRightDrivingSpeed;
};

#endif
