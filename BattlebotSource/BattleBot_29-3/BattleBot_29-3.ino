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

#define bluetoothReceivePin A0  // The input pin for receiving bluetooth messages.
#define bluetoothTransmitPin A1 // The output pin for transmitting bluetooth messages.

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
int incommingbluetoothCommand = 0; // Varialbe that stores the last bluetooth message.
int commandInt = 0;
String commandString = "";  // Variable that stores the program command extracted from the bluetooth message.
String commandArgument = ""; // Variable that stores the program command argument extrated from the bluetooth message.
int paardenRaceRondeCounter = 0;
long currentDrivingSpeed = 0;
int labyrintSplitCounter = 0;
int domainTapeCounter = 0;

/**
 * Declaration of variables that store the messages that will get displayed on the LCD screen.
 */
String lcdDisplayText = "";       
String secondLcdDisplayText = ""; 
String debugMessage = "";         

/**
 * Declaration of variables that are needed to initate objects.
 */
int maxPingDistance = 200;                 
unsigned long previousMillisSendVelocity;  
unsigned long sendVelocityInterval = 3000; 

// variabelen voor autonoom rijden
long timeForOneMeter;
long startTimer = 0;
boolean endTimer = false;
boolean start = false;
boolean firstTime = true;
boolean coinFound = false;
long paardenraceTimer = 0;

/**
 * The initiation of of objects that are used to communicate with the battle bot's modules.
 */

// Create an new serial communication object for bluetooth communication.
SoftwareSerial BTSerial(bluetoothReceivePin, bluetoothTransmitPin);

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
    updateLCDText("N.S.A.R.R");
    updateSecondLCDText("Init bluetooth..");
    delay(1000);
    Serial.begin(9600);
    while (!Serial)
    {
    }

    BTSerial.begin(38400);
    while (!BTSerial)
    {
    }
    updateSecondLCDText("Awaiting input..");
}

/**
 * Function for detecting tape on the ground.
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
 * Remove all text from specified line (0 or 1)
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
 * Update first lcd line
 */
void updateLCDText(String ScreenText)
{
    if (lcdDisplayText != ScreenText)
    {
        clearLcdLine(0);
        lcd.setCursor(0, 0);
        lcd.print(ScreenText);
        lcdDisplayText = ScreenText;
        Serial.println(ScreenText);
    }
}

/**
 * Update second lcd line
 */
void updateSecondLCDText(String secondScreenText)
{
    if (secondLcdDisplayText != secondScreenText)
    {
        clearLcdLine(1);
        lcd.setCursor(0, 1);
        lcd.print(secondScreenText);
        secondLcdDisplayText = secondScreenText;
        Serial.println(secondScreenText);
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
        updateSecondLCDText("Tape right");
        battleBotDrive.drive(-10, 30);
        battleBotDrive.drive(-40, 80); 
        break;

    case LEFT_SENSOR:
        updateSecondLCDText("Tape left");
        battleBotDrive.drive(30, -10);
        battleBotDrive.drive(80, -40); 
        break;

    case BOTH_SENSOR:
        updateSecondLCDText("Tape both");
        battleBotDrive.drive(0, 0);
        break;

    case NON_SENSOR:
        updateSecondLCDText("No tape");
        battleBotDrive.drive(-40, -40);
        break;

    default:
        break;
    }
}

/**
 * let the bot drive in between the 2 lines on the ground
 */
void avoidLineProgram()
{
    TapeDetected onSensor = detectTape();

    switch (onSensor)
    {
    case LEFT_SENSOR:
        battleBotDrive.drive(20, 20);
        delay(200);
        battleBotDrive.drive(-20, 40);
        delay(200);
        break;

    case RIGHT_SENSOR:
        battleBotDrive.drive(10, -5);
        break;

    case BOTH_SENSOR:
        battleBotDrive.drive(20, 20);
        delay(300);
        battleBotDrive.drive(-10, 30);
        break;

    case NON_SENSOR:
        battleBotDrive.drive(-50, -35);
        break;

    default:
        break;
    }

    //count laps.
    int distance = sonar.ping_cm();
    if (distance < 25 && distance != 0)
    {
        if (millis() > (paardenraceTimer + 2000))
        {
            paardenraceTimer = millis();
            paardenRaceRondeCounter++;

            String outputDummyText = "Lap " + String(paardenRaceRondeCounter);
            updateSecondLCDText(outputDummyText);
        }
    }

    if (paardenRaceRondeCounter >= 3)
    {
        commandString = "S"; //to stop the bot from continueing the current game.
        paardenRaceRondeCounter = 0;
    }
}

/**
 * started from the loop() function, checks for available bluetooth commands.
 */
void receiveAndStoreCommand()
{
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
        case 'y':
            commandString = "5";
            break;
        case 'u':
            commandString = "5";
            break;
        case 'i':
            commandString = "5";
            break;
        case 'o':
            commandString = "5";
            break;
        default:
            break;
        }
    }
}

/**
 * started from the loop() function, executes the currently stored command.
 */
void executeStoredCommand()
{
    int driveSpeed = 200;

    if (commandString == "F")
    {
        updateLCDText("Driving forward");
        forward();
    }
    else if (commandString == "B")
    {
        updateLCDText("Driving backward");
        battleBotDrive.drive(driveSpeed, driveSpeed);
    }
    else if (commandString == "L")
    {
        updateLCDText("Driving left");
        battleBotDrive.drive(0, -100);
    }
    else if (commandString == "R")
    {
        updateLCDText("Driving right");
        battleBotDrive.drive(-100, 0);
    }
    else if (commandString == "S")
    {
        still();
    }
    else if (commandString == "1")
    {
        updateLCDText("game 1");
        followLineProgram();
    }
    else if (commandString == "2")
    {
        updateLCDText("game 2");
        schatZoeken();
    }
    else if (commandString == "3")
    {
        updateLCDText("game 3");
        avoidLineProgram();
    }
    else if (commandString == "5")
    {
        still();
    }
    else if (commandString == "y")
    { // van start naar lijnenrace
        vanSpelNaarSpel(4);
    }
    else if (commandString == "u")
    { // van lijnenrace naar plakkaatzoeken
        vanSpelNaarSpel(3);
    }
    else if (commandString == "i")
    { // van plakkaatzoeken naar paarderace
        vanSpelNaarSpel(4);
    }
    else if (commandString == "o")
    { // van paardenrace naar parcours
        vanSpelNaarSpel(11);
    }
    else
    {        
        updateLCDText("N.S.A.R.R");
        updateSecondLCDText("Awaiting input..");
    }
}

void schatZoeken()
{
    //need to program
    TapeDetected onSensor = detectTape();

    if (!coinFound)
    {
        switch (onSensor)
        {
        case LEFT_SENSOR:
            //coin found
            coinFound = true;
            break;

        case RIGHT_SENSOR:
            //coin found
            coinFound = true;
            break;

        case BOTH_SENSOR:
            //coin found
            coinFound = true;
            break;

        case NON_SENSOR:
            updateSecondLCDText("Coin found");
            // stop driving
            battleBotDrive.drive(0, 0);
            coinFound = false;
            break;

        default:
            break;
        }

        if (!coinFound)
        {
            battleBotDrive.drive(-100, -100);
            int distance = sonar.ping_cm();
            updateSecondLCDText("Searching coin");
            //detect wall
            if (distance < 15 && distance != 0)
            {
                //turn around
                battleBotDrive.drive(100, 100);
                delay(600);
                battleBotDrive.drive(-100, 100);
                delay(400);
            }
            else
            {
                //drive forward

                //delay(100);
            }
        }
    }
    else
    {
        //stop driving
        updateSecondLCDText("Coin found");
        commandString = "5";
        battleBotDrive.drive(-0, -0);
    }
}

void vanSpelNaarSpel(int distance)
{
    TapeDetected onSensor = detectTape();

    if (firstTime)
    {
        // beginnen met rijden
        // BELANGRIJK: JE MOET ER ZELF VOOR ZORGEN DAT JE ROBOT RECHT VOORUIT RIJDT!!
        forward();
        // huidige tijd vastleggen
        startTimer = millis();
        firstTime = false;
    }

    // tijd meten die de robot over één meter doet
    if (!endTimer && onSensor == BOTH_SENSOR)
    {
        timeForOneMeter = (millis() - startTimer); // timeForOneMeter is de tijd die de robot doet over één meter, in milliseconden
        endTimer = true;                           // stop huidige loop
        start = true;                              // start volgende loop
    }

    if (start)
    {
        int distanceToDrive = distance - 1;                    // de robot heeft al één meter gereden
        long timeToDrive = (distanceToDrive * timeForOneMeter); // timeToDrive is de tijd in milliseconden die de robot moet rijden voor distanceToDrive
        delay(timeToDrive);                                    // delay om robot voor een bepaalde tijd op te houden, dan rijd de bot voor een bepaalde tijd lang
        still();
        start = false;
    }
}

void still()
{
    //variabelen voor het 'autonoom' rijden
    startTimer = 0;
    endTimer = false;
    start = false;
    firstTime = true;

    //vars for the other games to reset if called.
    updateLCDText("STOP");
    paardenRaceRondeCounter = 0;
    paardenraceTimer = millis();
    coinFound = false;
    battleBotDrive.drive(0, 0);
    clearLcdLine(1);
}

//calibrated forward driving.
void forward()
{
    battleBotDrive.drive(-200, -170);
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
            battleBotDrive.drive(0, 0);
        }
        else
        {
            battleBotDrive.drive(leftSpeed, rightSpeed);
        }
    }
    else
    {
        //do not detect collision
        battleBotDrive.drive(leftSpeed, rightSpeed);
    }
}

/**
 * This is the main function of the battle bot it will listen for incoming commands and execute it.
 */
void loop()
{
    receiveAndStoreCommand();
    executeStoredCommand();
}
