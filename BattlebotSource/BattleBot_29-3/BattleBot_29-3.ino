#include "Arduino.h"
#include <SoftwareSerial.h>
#include <NewPing.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <I2Cdev.h>
#include <MPU6050.h>
#include <BattleBotDrive.h>

/**
 * Define the I/O pins that are connected to the robot.
 */
#define leftMotorForwardPin 3   // Output pin that is connected to the left motor for forward movement.
#define leftMotorBackwardPin 2  // Output pin that is connected to the left motor for backward movement.
#define rightMotorForwardPin 9  // Output pin that is connected to the right motor for forward movement.
#define rightMotorBackwardPin 4 // Output pin that is connected to the right motor for backward movement.

#define leftInfraredSensor 10  // Input pin that is connected to the left infrared sensor.
#define rightInfraredSensor 11 // Input pin that is connected to the right infrared sensor.

#define ultraEchoPin 12        // Input pin that is connected to the ultra echo sensor.
#define ultraEchoTriggerPin 13 // Input pin that is connected to the trigger from the ultra echo sensor.

#define bluetouthReceivePin A0  // The input pin for receiving bluetouth messages.
#define bluetouthTransmitPin A1 // The output pin for transmitting bluetouth messages.

#define lcdDisplayAddress 0x27 // The address of the LCD display
#define lcdDisplayColumns 16   // The amount of characters each row of the display has.
#define lcdDisplayRows 2       // The amount of rows the display has

#define drivingLowerLimit 45  // The under limit for the motor speed.
#define drivingUpperLimit 255 // The upper limit for the motor speed.

#define defaultDrivingSpeed 10 // The default speed of the cart.

enum
{
    TRUE = 1,
    FALSE = 0
}; // Simple enumeration for boolean states.

/**
 * An enumeration with the difrend infrared sensor states.
 */
enum TapeDetected
{
    NON_SENSOR,   // Sensor state when both infrared sensors don't detect any tape.
    LEFT_SENSOR,  // Sensor state when the left infrared sensor detects tape.
    RIGHT_SENSOR, // sensor state when the right infrared sensor detects tape.
    BOTH_SENSOR   // Sensor state when both of the infrared sensors detect tape.
};

/**
 * Declaration of variables that store the commands given to the battle bot.
 */
int incommingBluetouthCommand = 0; // Varialbe that stores the last bluetouth message.
int commandInt = 0;
String commandString = "S";  // Variable that stores the program command extracted from the bluetouth message.
String commandArgument = ""; // Variable that stores the program command argument extrated from the bluetouth message.
long currentDrivingSpeed = 0;
int labyrintSplitCounter = 0;
int domainTapeCounter = 0;

/**
 * Declaration of variables that store the messages that will get displayed on the LCD screen.
 */
String lcdDisplayText = "";       // Variable used to store the last text that was printed on the first line lcd screen.
String secondLcdDisplayText = ""; // Variable used to store the last text that was printed on the second line of the lcd screen.
String debugMessage = "";         // Variable that stores the last available debug message.

/**
 * Declaration of variables that are needed to initate objects.
 */
int maxPingDistance = 200;                 // The maximum distance the ultra echo sensor measures.
unsigned long previousMillisSendVelocity;  // This variable keeps track of the previous velocity data transmission over bluetouth.
unsigned long sendVelocityInterval = 3000; // This variable sets the interval of the velocity data that gets send over bluetouth.

/**
 * The initiation of of objects that are used to communicate with the battle bot's modules.
 */
// Create an new serial communication object for bluetouth communication.
SoftwareSerial BTSerial(bluetouthReceivePin, bluetouthTransmitPin);

// Create an new lcd object for displaying debugging messages on the battle bot.
LiquidCrystal_I2C lcd(lcdDisplayAddress, lcdDisplayColumns, lcdDisplayRows);

// Create an new Ping object for measuring the distance to obstacles.
NewPing sonar(ultraEchoTriggerPin, ultraEchoPin, maxPingDistance);

// Create an new MPU6050 object for using the gyroscope and accelerometer.
MPU6050 accelgyro;

// Create an new BattleBot object for controlling the battle bot.
BattleBotDrive battleBotDrive(leftMotorForwardPin, rightMotorForwardPin, leftMotorBackwardPin, rightMotorBackwardPin);
/**
 * Function to initialize the battleBot.
 */
void setup()
{
    // Initialize the I/O pins
    pinMode(leftMotorBackwardPin, OUTPUT);
    pinMode(leftMotorForwardPin, OUTPUT);
    pinMode(rightMotorBackwardPin, OUTPUT);
    pinMode(rightMotorForwardPin, OUTPUT);

    pinMode(leftInfraredSensor, INPUT);
    pinMode(rightInfraredSensor, INPUT);

    pinMode(ultraEchoTriggerPin, INPUT);
    pinMode(ultraEchoPin, INPUT);

    // Start the LCD screen.
    lcd.begin();
    lcd.backlight();
    lcd.print("Awaiting...");

    Serial.begin(9600);
    while (!Serial)
    {
    }
    BTSerial.begin(38400); // HC-05 default speed in AT command more
    while (!BTSerial)
    {
    }
}

/**
 * This method detects if the infrared sensors moved over some tape.
 */
TapeDetected detectTape()
{
    if (digitalRead(rightInfraredSensor) == HIGH && digitalRead(leftInfraredSensor) == HIGH)
    {
        // Both infrared sensors detect tape.
        return BOTH_SENSOR;
    }
    else if (digitalRead(rightInfraredSensor) == HIGH)
    {
        // The left infrared sensor detected tape.
        return LEFT_SENSOR;
    }
    else if (digitalRead(leftInfraredSensor) == HIGH)
    {
        // The right infrared sensor detected tape.
        return RIGHT_SENSOR;
    }
    else
    {
        // Both infrared sensors detected nothing.
        return NON_SENSOR;
    }
}

/**
 * This function will clear an line on the LCD screen by its line index number.
 *
 * @param the index of the line so on an 2X16 display either 0 for line 1 or 1 for line 2.
 */
void clearLcdLine(int lineIndex)
{
    lcd.setCursor(0, lineIndex);
    for (int i = 0; i < 16; ++i)
    {
        lcd.write(' ');
    }
    lcd.setCursor(0, lineIndex);
}

/**
 * This method will print an message to the first line of the lcd screen if it is the same message it will not update it
 * so that the screen wont flicker when the same message is printed every milli second.
 *
 * @param currentCommand The last printed message to the lcd screen.
 */
void updateLCDCommand(String currentCommand)
{
    if (lcdDisplayText != currentCommand)
    {
        clearLcdLine(0);
        lcd.setCursor(0, 0);
        lcd.print(currentCommand);
        lcdDisplayText = currentCommand;
        Serial.println(currentCommand);
    }
}

/**
 * This method will print an message to the second line of the lcd screen if it is the same message it will not update it
 * so that the screen wont flikker when the same message is printed every milli second.
 *
 * @param currentCommand The last printed message to the lcd screen.
 */
void updateSecondLCDCommand(String secondCurrentCommand)
{
    if (secondLcdDisplayText != secondCurrentCommand)
    {
        clearLcdLine(1);
        lcd.setCursor(0, 1);
        lcd.print(secondCurrentCommand);
        secondLcdDisplayText = secondCurrentCommand;
        Serial.println(secondCurrentCommand);
    }
}

/**
 * This function lets the bot follow the tape on the ground
 */
void followLineProgram()
{
    TapeDetected onSensor = detectTape();
    switch (onSensor)
    {
    case RIGHT_SENSOR:
        updateSecondLCDCommand("Tape right");
        battleBotDrive.drive(-45, 30); //use 10 everywhere for perfect parkour
        break;

    case LEFT_SENSOR:
        updateSecondLCDCommand("Tape left");
        battleBotDrive.drive(30, -45); //use 10 everywhere for perfect parkour
        break;

    case BOTH_SENSOR:
        updateSecondLCDCommand("Tape both");
        battleBotDrive.drive(30, -45);
        break;

    case NON_SENSOR:
        updateSecondLCDCommand("No tape");
        battleBotDrive.drive(-30, -30);
        break;

    default:
        break;
    }
}
void followLineProgram2()
{
    
    TapeDetected onSensor = detectTape();

    switch (onSensor)
    {
    case RIGHT_SENSOR:
        updateSecondLCDCommand("Tape right");
        battleBotDrive.drive(-20, 50); //use 10 everywhere for perfect parkour
        break;
    case LEFT_SENSOR:
        updateSecondLCDCommand("Tape left");
        battleBotDrive.drive(50, -20); //use 10 everywhere for perfect parkour
        break;

    case BOTH_SENSOR:
        updateSecondLCDCommand("Tape both");
        battleBotDrive.drive(-40, -40);
        break;

    case NON_SENSOR:
        updateSecondLCDCommand("No tape");
        battleBotDrive.drive(-40, -40);
        break;

    default:
        break;
    }
}
void followLineProgram3()
{
    
    TapeDetected onSensor = detectTape();

    switch (onSensor)
    {
    case RIGHT_SENSOR:
        updateSecondLCDCommand("Tape right");
        battleBotDrive.drive(-10, 20); //use 10 everywhere for perfect parkour
        break;
    case LEFT_SENSOR:
        updateSecondLCDCommand("Tape left");
        battleBotDrive.drive(20, -10); //use 10 everywhere for perfect parkour
        break;

    case BOTH_SENSOR:
        updateSecondLCDCommand("Tape both");
        battleBotDrive.drive(-50, -50);
        break;

    case NON_SENSOR:
        updateSecondLCDCommand("No tape");
        battleBotDrive.drive(-50, -50);
        break;

    default:
        break;
    }
}
/**
 * This function lets the bot avoid the tape on the ground
 */
void avoidLineProgram()
{
    TapeDetected onSensor = detectTape();
    switch (onSensor)
    {
    case LEFT_SENSOR:
        updateSecondLCDCommand("Tape left");
        battleBotDrive.drive(-70, 40);
        break;
    case RIGHT_SENSOR:
        updateSecondLCDCommand("Tape right");
        battleBotDrive.drive(30, -70);
        break;

    case BOTH_SENSOR:
        updateSecondLCDCommand("Tape both");
        battleBotDrive.drive(40, 40); //backwards and turn around
        delay(200);
        battleBotDrive.drive(-40, 40);
        break;

    case NON_SENSOR:
        updateSecondLCDCommand("No tape");
        battleBotDrive.drive(-40, -40);
        break;

    default:
        break;
    }
}

void bluetoothCommandReceiver()
{
    //runs the while if there is information to be received from the server.
    while (BTSerial.available())
    {
        char incoming = (BTSerial.read());

        String data = String(incoming);
        Serial.println(data);
        switch (incoming)
        {
        case 'F':
            commandString = "F";
            break;
        case 'B':
            commandString = "B";
            break;
        case 'L':
            commandString = "L";
            break;
        case 'R':
            commandString = "R";
            break;
        case 'S':
            commandString = "S";
            break;
        case 'w':
            commandString = "F";
            break;
        case 's':
            commandString = "B";
            break;
        case 'a':
            commandString = "L";
            break;
        case 'd':
            commandString = "R";
            break;
        case '1':
            commandString = "1";
            break;
        case '2':
            commandString = "2";
            break;
        case '3':
            commandString = "3";
            break;
        case '5':
            commandString = "5";
            break;
        default:
            break;
        }
    }
}
/*
TODO: make this the main function for letting the bot drive, then implement the anti collision stuff.
*/
void driveAndAvoid(int leftSpeed, int rightSpeed, bool enable)
{
    if (enable)
    {
        //stop if obstacles are detected.
        if (sonar.ping_cm() < 15)
        {
        }
    }
    else
    {
        //do not detect collision
    }
}

void executeStoredCommand()
{
    int driveSpeed = 100;

    if (commandString == "F")
    {
        updateLCDCommand("Driving forward");
        battleBotDrive.drive(-driveSpeed, -driveSpeed);
    }
    else if (commandString == "B")
    {
        updateLCDCommand("Driving backward");
        battleBotDrive.drive(driveSpeed, driveSpeed);
    }
    else if (commandString == "L")
    {
        updateLCDCommand("Driving left");
        battleBotDrive.drive(10, -100);
    }
    else if (commandString == "R")
    {
        updateLCDCommand("Driving right");
        battleBotDrive.drive(-100, 10);
    }
    else if (commandString == "S")
    {
        updateLCDCommand("STOP");
        battleBotDrive.drive(0, 0);
    }
    else if (commandString == "1")
    {
        updateLCDCommand("game 1");
        //followLineProgram();
        followLineProgram2();
        //followLineProgram3();
    }
    else if (commandString == "2")
    {
        updateLCDCommand("game 2");
        //schat zoeken
    }
    else if (commandString == "3")
    {
        updateLCDCommand("game 3");
        avoidLineProgram();
    }
    else if (commandString == "5")
    {
        updateLCDCommand("STOP");
        battleBotDrive.drive(0, 0);
    }
    else
    {
        updateLCDCommand("Unknown command");
    }
}

/**
 * This is the main function of the battle bot it will listen for incoming commands and execute it.
 */
void loop()
{
    bluetoothCommandReceiver();
    executeStoredCommand();
}
