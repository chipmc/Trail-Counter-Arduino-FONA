///
/// @mainpage	Trail-Counter-Arduino-FONA
///
/// @details	This is the original Project - Single processor Arduino
/// @n
/// @n
/// @n @a		Developed with [embedXcode+](http://embedXcode.weebly.com)
///
/// @author		Charles McClelland
/// @author		Charles McClelland
/// @date		5/15/16 12:25 PM
/// @version	<#version#>
///
/// @copyright	(c) Charles McClelland, 2016
/// @copyright	GNU General Public Licence
///
/// @see		ReadMe.txt for references
///


///
/// @file		Trail_Counter_Arduino_FONA.ino
/// @brief		Main sketch
///
/// @details	<#details#>
/// @n @a		Developed with [embedXcode+](http://embedXcode.weebly.com)
///
/// @author		Charles McClelland
/// @author		Charles McClelland
/// @date		5/15/16 12:25 PM
/// @version	<#version#>
///
/// @copyright	(c) Charles McClelland, 2016
/// @copyright	GNU General Public Licence
///
/// @see		ReadMe.txt for references
/// @n
///


// Core library for code-sense - IDE-based
#if defined(WIRING) // Wiring specific
    #include "Wiring.h"
#elif defined(MAPLE_IDE) // Maple specific
    #include "WProgram.h"
#elif defined(ROBOTIS) // Robotis specific
    #include "libpandora_types.h"
    #include "pandora.h"
#elif defined(MPIDE) // chipKIT specific
    #include "WProgram.h"
#elif defined(DIGISPARK) // Digispark specific
    #include "Arduino.h"
#elif defined(ENERGIA) // LaunchPad specific
    #include "Energia.h"
#elif defined(LITTLEROBOTFRIENDS) // LittleRobotFriends specific
    #include "LRF.h"
#elif defined(MICRODUINO) // Microduino specific
    #include "Arduino.h"
#elif defined(TEENSYDUINO) // Teensy specific
    #include "Arduino.h"
#elif defined(REDBEARLAB) // RedBearLab specific
    #include "Arduino.h"
#elif defined(RFDUINO) // RFduino specific
    #include "Arduino.h"
#elif defined(SPARK) || defined(PARTICLE) // Particle / Spark specific
    #include "application.h"
#elif defined(ESP8266) // ESP8266 specific
    #include "Arduino.h"
#elif defined(ARDUINO) // Arduino 1.0 and 1.5 specific
    #include "Arduino.h"
#else // error
    #   error Platform not defined
#endif // end IDE

// Set parameters
#define FONA_RX 4
#define FONA_TX 5
#define FONA_RST 6
#define FONA_KEY 7
#define FONA_PS 8
#define CARRIER 1  //  0 for T-Mobile and 1 for Vodafone
#define MMA8452_ADDRESS 0x1D  // Our Accelerometer I2C Address set with open jumper SA0



// Include application, user and local libraries
// Includes and Defines for the Libraries
#include <SoftwareSerial.h>
#include "Adafruit_FONA.h"
#include <DSRTCLib.h>
#include <stdlib.h>      // to convert floats to strings
#include "i2c.h"               // not the wire library, can't use pull-ups
#include <Wire.h>
#include "MAX17043.h"          // for the MAX17043 LiPo Fuel Gauge
#include <avr/sleep.h>        // For Sleep Code
#include <avr/power.h>


// Define structures and classes


// Define variables and constants
// Pin Constants
const int int2Pin = 2;         // This is the interrupt pin that registers taps
const int int1Pin = 3;         // This is ths RTC interrupt pin
const int ledPin = 9;          // led connected to digital pin 9
const int led2Pin = 10;        // The Yellow Signal Strength LED
const int SensitivityPot = A0; // Potentiometer used to adjust sensitivity
const int DelayPot=A1;         //  Used to adjust the time between events

//Program Execution Variables
int tries = 0;                 // keep track of connection attepmts
volatile boolean TransmitFlag = 0;      // Makes sure we send data
int ledState = LOW;            // variable used to store the last LED status, to toggle the light
char Location[32] = "{\"lat\":";  // Will build this up later
uint16_t vbat;                 // Battery voltage in milliVolts
int KeyTime = 2000;            // Time needed to turn on the Fona
unsigned long TimeOut = 20000; // How long we will give an AT command to comeplete
unsigned long TransmitRetry = 60000;  // How long will we wait to retransmit
unsigned long LastSend = 0;   // Last Time we sent data in milliseconds
unsigned long LastBump=0;      // Need to make sure we don't count again until debounce
int PersonCount = 0;           // Count the number of PIR hits in reporting interval
uint16_t returncode;           // Fona function return codes
int RetryMode = 1;             // Ensures we send an initial set of data immediately
int retries=0;                 // Number of times we need to retransmit data
int lastRetries = 0;           // So we can avoid transmitting if nothing changes
int ConnectRetryLimit = 50;    // Number of times Arduino will try to connect in GetConnected

// Some common responses from the FONA
const char error[6] = "ERROR";
const char OK[3] = "OK";


// Accelerometer
const byte SCALE = 2;          // Sets full-scale range to +/-2, 4, or 8g. Used to calc real g values.
const byte dataRate = 3;       // output data rate (kHZ) - 0=800kHz, 1=400, 2=200, 3=100, 4=50, 5=12.5, 6=6.25, 7=1.56
int InputValue = 0;            // Raw sensitivity input
byte Sensitivity = 0x00;       // Hex variable for sensitivity
int DelayTime = 0;             // Modification of delay time (in tenths of a second added to the 1 second standard delay
int threshold = 100;           // threshold value to decide when the detected sound is a knock or not
int debounce=500;             // need to debounce the sensor - figure bikes are spaced at


// Prototypes
// Instantiate all our functions from the libraries
SoftwareSerial fonaSS = SoftwareSerial(FONA_TX, FONA_RX);
SoftwareSerial *fonaSerial = &fonaSS;
Adafruit_FONA fona = Adafruit_FONA(FONA_RST);
uint8_t readline(char *buff, uint8_t maxbuff, uint16_t timeout = 0);
DS1339 RTC = DS1339(int1Pin,0);          // Instantiate the RTC on Pin 2 and interrupt 0
MAX17043 batteryMonitor;  // Instantiate the Fuel Gauge library

// From my user Functions
void wakeUpNow();             // here the accelerometer interrupt is handled after wakeup
void nap();                   // Here is where we wake up from sleep based on the RTC alarm

// FONA and Cellular  Functions
void TurnOnFona();   // Turn on the FONA
boolean InitializeFona();    // Initialize the FONA
void ReadRSSI();     // read the RSSI strength after initalization
boolean GetConnected();  // Connect and set up GPRS Data Services
void GetDisconnected();  // Disconnect correctly from the cellular network
void TurnOffFona();  // Turn off the FONA to save juice

//UBIDOTS Functions
boolean SendATCommand(char Command[], const char *response); // This has been my problem child function but I think I have it now - no more Strings
boolean sendToUbidots(); // Send multiple Updates to Ubidots using Collections

// Sensor and General functions
void initMMA8452(byte fsr, byte dataRate);   // Initialize the MMA8452 registers
void MMA8452Standby();   // Sets the MMA8452 to standby mode.
void MMA8452Active();    // Sets the MMA8452 to active mode.
byte readRegister(uint8_t address);  // Read a single byte from address and return it as a byte
void writeRegister(unsigned char address, unsigned char data);   // Writes a single byte (data) into address
void CheckForBump();     // See if there was a bump while the Arduino was awake
void sleepNow();         // here we put the arduino to sleep
void NonBlockingDelay(int millisDelay);  // Used for a non-blocking delay
int freeRam ();  // Debugging code, to check usage of RAM
void BlinkForever(); // For when things go badly wron

// Time and Location functions
void setLocationAndTime();   // Where we get info from the network to set time and location
void set_time(int year,int month, int day,int hour, int minute,int second);  // Here we set the clock with passed values
void read_time();    // Read the current time
void set_hourly_alarm(); // Sets an RTC alarm to go off every hour
void set_minute_alarm(); // Sets an RTC alarm to go off every minute
void printTime(byte type);       // Print a formatted string of the current date and time.




// Add setup code
void setup()
{
    Serial.begin(19200);
    while (!Serial) ;   // while the serial stream is not open, do nothing:
    Serial.println(F("Running sketch Solar_Micro_GPRS_2_v2"));
    RTC.start(); // ensure RTC oscillator is running, if not already
    pinMode(int1Pin, INPUT);      // RTC Interrupt pins
    digitalWrite(int1Pin, HIGH);
    pinMode(ledPin, OUTPUT);       // Bump LED
    digitalWrite(ledPin,HIGH);     // Turn it on for now
    pinMode(led2Pin, OUTPUT);     // Signal LED
    digitalWrite(led2Pin,HIGH);    // Turn it on for now
    pinMode(FONA_KEY,OUTPUT);      // This is the pin used to turn on and off the Fona
    pinMode(FONA_PS,INPUT);        // Need to declare pins for the Teensy
    digitalWrite(FONA_PS,HIGH);    //  Let's turn on the pull-up
    pinMode(int2Pin, INPUT);       // This is the pin the accelerometer will use to wake the Arduino
    digitalWrite(int2Pin, HIGH);   // Internal pull-up
    
    // Initialize the FONA
    if(digitalRead(FONA_PS)) {
        TurnOffFona();             //  Start with the Fona off
    }
    TurnOnFona();              // Then turn it on - helps to make sure stame state every time
    if(!InitializeFona()) {    // Initialize the Fona
        Serial.println(F("Initialization Failed"));
        BlinkForever;
    }
    else Serial.println(F("Initialization Succeeded"));
    ReadRSSI();
    setLocationAndTime();
    
    if (!sendToUbidots()) {                   // Sends Right Away to test connection
        Serial.println(F("Initial Send Failed - Reboot"));
        BlinkForever();
    }
    GetDisconnected();                           // Disconnect from GPRS
    TurnOffFona();                               // Turn off the FONA
    
    Serial.print(F("Free Ram Setup: "));          // Print out the free memory for diagnostics
    Serial.println(freeRam());
    
    InputValue = analogRead(SensitivityPot);     // Read the Pots for Sensitivity and Delay - Note this is done once at startup - need to press reset to take effect if changed
    InputValue = map(InputValue,0,1023,16,0);  // This value is used to set the Sensitivity (can go to 127 but not this application)
    Sensitivity = byte(InputValue);
    Serial.print(F("Input set to ="));
    Serial.println(Sensitivity);
    InputValue = analogRead(DelayPot);
    InputValue = map(InputValue,0,1023,2000,0);  // This value is used to set the delay in msec that will be added to the 1 second starting value
    debounce = debounce + InputValue;
    Serial.print(F("Debounce delay (in mSec) set to ="));
    Serial.println(debounce);
    ADCSRA = 0;  // disable ADC since the only analog reading is the input sensitivity - saves power
    
    // Read the WHO_AM_I register, this is a good test of communication
    byte c = readRegister(0x0D);  // Read WHO_AM_I register
    if (c == 0x2A) // WHO_AM_I should always be 0x2A
    {
        initMMA8452(SCALE, dataRate);  // init the accelerometer if communication is OK
        Serial.println(F("MMA8452Q is online..."));
    }
    else
    {
        Serial.print(F("Could not connect to MMA8452Q: 0x"));
        Serial.println(c, HEX);
        BlinkForever();
    }
    // Initialize the fuel gauge
    batteryMonitor.reset();
    batteryMonitor.quickStart();
    NonBlockingDelay(1000);
    
    digitalWrite(ledPin, LOW);  // Let's us know we made it through Setup
}

// Add loop code
void loop()
{
    if (!digitalRead(int1Pin))   { // This checks to see if it is time to send data or if we need to retry
        Serial.println(F("Alarm"));
        read_time();
        RTC.clear_interrupt();
        if (PersonCount > 0) {
            TransmitFlag = 1;
        }
        else if (int(RTC.getHours()) == 10) {
            setLocationAndTime();
            TransmitFlag = 1;
        }
    }
    if (TransmitFlag) {  // If we need to transmit
        if (LastSend + TransmitRetry <= millis()) {  // And we are not in a Retry Mode
            TurnOnFona();                    // Turn on the module
            CheckForBump();                  // This is where we look to see if we should count a bump
            if(GetConnected()) {             // Connect to network and start GPRS
                CheckForBump();                // This is where we look to see if we should count a bump
                if (sendToUbidots()) {         // Send data to Ubidots
                    TransmitFlag = 0;             // Sent count - we are happy
                    CheckForBump();                // This is where we look to see if we should count a bump
                }
                else {
                    LastSend = millis();        // reset the resend clock
                }
            }
            CheckForBump();                // This is where we look to see if we should count a bump
            GetDisconnected();             // Disconnect from GPRS
            CheckForBump();                // This is where we look to see if we should count a bump
            TurnOffFona();                 // Turn off the module
        }
    }
    else  {
        if (LastBump + debounce <= millis())  { // Need to stay awake for debounce period - no sleep on retransmit
            Serial.print(F("Free Ram Bump: ")); // Print out the free memory for diagnostics
            Serial.println(freeRam());
            Serial.println(F("Entering Sleep mode"));
            NonBlockingDelay(100);     // this delay is needed, the sleep function will provoke a Serial error otherwise!!
            sleepNow();     // sleep function called here
        }
    }
    CheckForBump();   // This is where we look to see if we should count a bump
}

void TurnOnFona()   // Turn on the FONA
{
    Serial.print(F("Turning on Fona: "));
    while(!digitalRead(FONA_PS))
    {
        digitalWrite(FONA_KEY,LOW);
        unsigned long KeyPress = millis();
        while(KeyPress + KeyTime >= millis()) {}
        digitalWrite(FONA_KEY,HIGH);
        NonBlockingDelay(100);
    }
    Serial.println(F("success!"));
}

boolean InitializeFona()    // Initialize the FONA
{
    Serial.println(F("FONA basic test"));
    Serial.println(F("Initializing....(May take 3 seconds)"));   // See if the FONA is responding
    fonaSerial->begin(4800);
    if (! fona.begin(*fonaSerial)) {
        Serial.println(F("Couldn't find FONA"));
        return 0;
    }
    Serial.println(F("FONA is OK"));
    GetConnected();                                              // Connect to network and start GPRS
    while (!fona.enableGPRS(true))  {
        tries ++;
        Serial.print(F("Failed to turn on GPRS Attempt #"));
        Serial.println(tries);
        NonBlockingDelay(500);
        if (tries >= 20) {
            Serial.println(F("GPRS failed to initalize"));
            return 0;                                                  // Initialization Failed
        }
    }
    return 1;                                                      // Initalization Succeeded
}

void ReadRSSI()     // read the RSSI strength after initalization
{

    // reference - http://www.speedguide.net/faq/how-does-rssi-dbm-relate-to-signal-quality-percent-439
    
    digitalWrite(led2Pin, LOW);  // Turn off the LED so we can do our thing
    NonBlockingDelay(1000);
    uint8_t n = fona.getRSSI();
    int8_t r;
    int quality=0;
    
    Serial.print(F("RSSI = ")); Serial.print(n); Serial.print(F(": "));
    if (n == 0) r = -115;
    if (n == 1) r = -111;
    if (n == 31) r = -52;
    if ((n >= 2) && (n <= 30)) {
        r = map(n, 2, 30, -110, -54);
    }
    Serial.print(r); Serial.println(F(" dBm"));
    quality = 2 * (r + 100);
    if (r <= -98) BlinkForever();
    for (int i=0; i < int(quality/10); i++) {
        digitalWrite(led2Pin,HIGH);
        Serial.println(F("Flash LED High"));
        NonBlockingDelay(500);
        digitalWrite(led2Pin,LOW);
        NonBlockingDelay(500);
    }
}

boolean GetConnected()  // Connect and set up GPRS Data Services
{
    tries = 0;
    int n = 0;
    do
    {
        tries ++;
        n = fona.getNetworkStatus();  // Read the Network / Cellular Status
        Serial.print(F("Network status Try #"));
        Serial.print(tries);
        Serial.print(F(" - "));
        Serial.print(n);
        Serial.print(F(": "));
        if (n == 0) Serial.println(F("Not registered"));
        if (n == 1) Serial.println(F("Registered (home)"));
        if (n == 2) Serial.println(F("Not registered (searching)"));
        if (n == 3) Serial.println(F("Denied"));
        if (n == 4) Serial.println(F("Unknown"));
        if (n == 5) Serial.println(F("Registered roaming"));
        
        NonBlockingDelay(500);
        if (tries >= ConnectRetryLimit)
        {
            Serial.println(F("Failed to connect to the network"));
            TurnOffFona();
            return 0;
        }
    } while (n != 1 && n != 5);
    tries = 0;
    Serial.print(F("Start task and set APN: "));
    if (!SendATCommand("AT+CSTT=\"internetd.gdsp\"",OK)) {  // here is where we modify
        Serial.println(F("Failed to set APN"));
        return 0;
    }
    Serial.print(F("Bring up wireless connection: "));
    if (! SendATCommand("AT+CIICR",OK)) {
        if(! SendATCommand("",OK)) {
            if(! SendATCommand("",OK)) {
                SendATCommand("AT+CIPCLOSE",OK);
                SendATCommand("AT+CIPSHUT",OK);
                Serial.println(F("Failed to bring up the wireless connection"));
                retries++;
                return 0;
            }
        }
    }
    Serial.print(F("Get local IP address: "));
    SendATCommand("AT+CIFSR",".");   // No good way to check this one as IP addresses can all be different
    return 1;
}

void GetDisconnected()  // Disconnect correctly from the cellular network
{
    fona.enableGPRS(false);
    Serial.println(F("GPRS Serivces Stopped"));
}

void TurnOffFona()  // Turn off the FONA to save juice
{
    Serial.print(F("Turning off Fona: "));
    while(digitalRead(FONA_PS))
    {
        digitalWrite(FONA_KEY,LOW);
        unsigned long KeyPress = millis();
        while(KeyPress + KeyTime >= millis()) {}
        digitalWrite(FONA_KEY,HIGH);
        NonBlockingDelay(100);
    }
    Serial.println(F("success!"));
}



boolean SendATCommand(char Command[], const char *response) // This has been my problem child function but I think I have it now - no more Strings
{
    char replyBuffer[64];          // this is a large buffer for replies
    int index = 0;
    char fonaInput;
    unsigned long commandClock = millis();                      // Start the timeout clock
    
    Serial.print(F("Sending "));
    Serial.print(Command);
    fona.println(Command);
    Serial.print(F(" and expecting: "));
    Serial.write(response);
    Serial.println("");
    
    while(millis() <= commandClock + TimeOut)         // Need to give the modem time to complete command
    {
        index = 0;
        while(!fona.available() &&  millis() <= commandClock + TimeOut);  //Need to look at this should it be >=
        while(fona.available()) {                                 // reading data into char array
            fonaInput = Serial.read();
            if (fonaInput != '\n') {
                replyBuffer[index]=fonaInput;    // writing data into array stripping out returns
                index++;
                replyBuffer[index] = '\0';
            }
            if(index == 63) break;
            NonBlockingDelay(50);
        }
        Serial.print(F("index="));
        Serial.print(index);
        Serial.print(F(" - Reply: "));
        Serial.println(replyBuffer);                           // Uncomment if needed to debug
        
        if (strstr(replyBuffer, response) != NULL) {
            Serial.println("Success!");
            return 1;
        }
        else if (strstr(replyBuffer,error) != NULL) {
            Serial.println("Error aborting!");
            return 0;
        }
        else Serial.println("Keep Looking");
    }
    return 0;
}




boolean sendToUbidots() // Send multiple Updates to Ubidots using Collections
{
    // Reference -- http://ubidots.com/docs/api/v1_6/collections/post_values.html#post-api-v1-6-collections-values
    float stateOfCharge = batteryMonitor.getSoC();  // Get the battery level
    if (stateOfCharge >= 255) stateOfCharge = 0;   // Bogus reading
    char batteryStr[10];                                  // put it in a character buffer
    char personCountStr[5];
    char retriesStr[5];
    dtostrf(stateOfCharge, 5, 2, batteryStr);             //5 is mininum width, 2 is precision
    int overheadLength = 172;  // 3 24-character keys plus the words (old 158)
    int succeeded;
    itoa(PersonCount,personCountStr,10);
    itoa(retries,retriesStr,10);
    int num;
    int payloadLength = strlen(batteryStr)+ strlen(Location) + strlen(personCountStr) + strlen(retriesStr) + overheadLength;
    
    Serial.print(F("Start the connection to Ubidots: "));
    if (SendATCommand("AT+CIPSTART=\"tcp\",\"things.ubidots.com\",\"80\"","CONNECT")) {
        Serial.println(F("Connected"));
    }
    else return 0;
    Serial.print(F("Begin to send data to the remote server: "));
    if (SendATCommand("AT+CIPSEND",">")) {
        Serial.println(F("Sending"));
    }
    fona.println(F("POST /api/v1.6/collections/values HTTP/1.1")); // Replace with your ID variable
    fona.println(F("Host: things.ubidots.com"));
    fona.println(F("X-Auth-Token: 6EALvOEabrFiA6XViWY3hxcFMG0yDr")); //in here, you should replace your Token
    fona.println(F("Content-Type: application/json"));
    fona.print(F("Content-Length: "));
    fona.println(payloadLength);
    fona.println();
    fona.print(F("[{\"variable\": "));
    fona.print(F("\"543754d976254236a25f27d6\", \"value\":"));
    fona.print(batteryStr);
    fona.print(F(", \"context\":"));
    fona.print(Location);
    fona.print(F("}, {\"variable\": \"543754a67625423611d25340\", \"value\":"));
    fona.print(personCountStr);
    fona.print(F("}, {\"variable\": \"543754ed7625423741b697bd\", \"value\":"));
    fona.print(retriesStr);
    fona.println(F("}]"));
    fona.println();
    fona.println((char)26); //This terminates the JSON SEND with a carriage return
    Serial.print(F("Send JSON Package: "));
    if (SendATCommand("","SEND OK")) { // The 200 code "SEND OK" from Ubidots means it was successfully uploaded
        Serial.println(F("Sent data to Ubidots!"));
        PersonCount = 0;
        succeeded = 1;
    }
    else {
        Serial.println(F("Send Timed out, will retry at next interval"));
        succeeded = 0;
    }
    Serial.print(F("Close connection to Ubidots: ")); // Close the connection
    if (SendATCommand("AT+CIPCLOSE","CLOSE")) {
        Serial.println(F("Closed"));
    }
    if (succeeded == 1) return 1;
    else return 0;
}




void initMMA8452(byte fsr, byte dataRate)   // Initialize the MMA8452 registers
{
    // See the many application notes for more info on setting all of these registers:
    // http://www.freescale.com/webapp/sps/site/prod_summary.jsp?code=MMA8452Q
    // Feel free to modify any values, these are settings that work well for me.
    MMA8452Standby();  // Must be in standby to change registers
    
    // Set up the full scale range to 2, 4, or 8g.
    if ((fsr==2)||(fsr==4)||(fsr==8))
        writeRegister(0x0E, fsr >> 2);
    else
        writeRegister(0x0E, 0);
    
    // Setup the 3 data rate bits, from 0 to 7
    writeRegister(0x2A, readRegister(0x2A) & ~(0x38));
    if (dataRate <= 7)
        writeRegister(0x2A, readRegister(0x2A) | (dataRate << 3));
    
    /* Set up single and double tap - 5 steps:
     1. Set up single and/or double tap detection on each axis individually.
     2. Set the threshold - minimum required acceleration to cause a tap.
     3. Set the time limit - the maximum time that a tap can be above the threshold
     4. Set the pulse latency - the minimum required time between one pulse and the next
     5. Set the second pulse window - maximum allowed time between end of latency and start of second pulse
     for more info check out this app note: http://cache.freescale.com/files/sensors/doc/app_note/AN4072.pdf */
    //writeRegister(0x21, 0x7F);  // 1. enable single/double taps on all axes
    writeRegister(0x21, 0x55);  // 1. single taps only on all axes
    // writeRegister(0x21, 0x6A);  // 1. double taps only on all axes
    writeRegister(0x23, Sensitivity);  // 2. x thresh from 0 to 127, multiply the value by 0.0625g/LSB to get the threshold
    writeRegister(0x24, Sensitivity);  // 2. y thresh from 0 to 127, multiply the value by 0.0625g/LSB to get the threshold
    writeRegister(0x25, Sensitivity);  // 2. z thresh from 0 to 127, multiply the value by 0.0625g/LSB to get the threshold
    writeRegister(0x26, 0xFF);  // 3. Max time limit at 100Hz odr, this is very dependent on data rate, see the app note
    writeRegister(0x27, 0x64);  // 4. 1000ms (at 100Hz odr) between taps min, this also depends on the data rate
    writeRegister(0x28, 0xFF);  // 5. 318ms (max value) between taps max
    
    // Set up interrupt 1 and 2
    writeRegister(0x2C, 0x02);  // Active high, push-pull interrupts
    writeRegister(0x2D, 0x19);  // DRDY, P/L and tap ints enabled
    writeRegister(0x2E, 0x01);  // DRDY on INT1, P/L and taps on INT2
    
    MMA8452Active();  // Set to active to start reading
}


void MMA8452Standby()   // Sets the MMA8452 to standby mode.
{
    // It must be in standby to change most register settings
    byte c = readRegister(0x2A);
    writeRegister(0x2A, c & ~(0x01));
}



void MMA8452Active()    // Sets the MMA8452 to active mode.
{
    // Needs to be in this mode to output data
    byte c = readRegister(0x2A);
    writeRegister(0x2A, c | 0x01);
}



byte readRegister(uint8_t address)  // Read a single byte from address and return it as a byte
{
    byte data;
    
    i2cSendStart();
    i2cWaitForComplete();
    
    i2cSendByte((MMA8452_ADDRESS<<1)); // Write 0xB4
    i2cWaitForComplete();
    
    i2cSendByte(address);	// Write register address
    i2cWaitForComplete();
    
    i2cSendStart();
    
    i2cSendByte((MMA8452_ADDRESS<<1)|0x01); // Write 0xB5
    i2cWaitForComplete();
    i2cReceiveByte(TRUE);
    i2cWaitForComplete();
    
    data = i2cGetReceivedByte();	// Get MSB result
    i2cWaitForComplete();
    i2cSendStop();
    
    cbi(TWCR, TWEN);	// Disable TWI
    sbi(TWCR, TWEN);	// Enable TWI
    
    return data;
}


void writeRegister(unsigned char address, unsigned char data)   // Writes a single byte (data) into address
{
    i2cSendStart();
    i2cWaitForComplete();
    
    i2cSendByte((MMA8452_ADDRESS<<1)); // Write 0xB4
    i2cWaitForComplete();
    
    i2cSendByte(address);	// Write register address
    i2cWaitForComplete();
    
    i2cSendByte(data);
    i2cWaitForComplete();
    
    i2cSendStop();
}

void CheckForBump()     // See if there was a bump while the Arduino was awake
{
    byte source;
    if (digitalRead(int2Pin)==0)    // If int2 goes LOW (inverted), either p/l has changed or there's been a single/double tap
    {
        source = readRegister(0x0C);  // Read the interrupt source reg.
        if ((source & 0x08)==0x08) { // We are only interested in the TAP register so read that
            if (millis() >= LastBump + debounce)
            {
                PersonCount++;                    // Increment the PersonCount
                LastBump = millis();              // Reset last bump timer
                Serial.print(F("Count: "));
                Serial.print(PersonCount);
                source = readRegister(0x22);  // Reads the PULSE_SRC register to reset it - Finish with Accel before talking to clock
                ledState = !ledState;              // toggle the status of the ledPin:
                digitalWrite(ledPin, ledState);    // update the LED pin itself
                RTC.readTime();                    // update RTC library's buffers from chip
                Serial.print(F(" at "));
                printTime(0);                      // Adds time to the Bump event
            }
            else {
                source = readRegister(0x22);  // Reads the PULSE_SRC register to reset it
            }
        }
    }
}

void sleepNow()         // here we put the arduino to sleep
{
    /* Now is the time to set the sleep mode. In the Atmega8 datasheet
     * http://www.atmel.com/dyn/resources/prod_documents/doc2486.pdf on page 35
     * there is a list of sleep modes which explains which clocks and
     * wake up sources are available in which sleep mode.
     *
     * In the avr/sleep.h file, the call names of these sleep modes are to be found:
     *
     * The 5 different modes are:
     *     SLEEP_MODE_IDLE         -the least power savings
     *     SLEEP_MODE_ADC
     *     SLEEP_MODE_PWR_SAVE
     *     SLEEP_MODE_STANDBY
     *     SLEEP_MODE_PWR_DOWN     -the most power savings
     *
     * For now, we want as much power savings as possible, so we
     * choose the according
     * sleep mode: SLEEP_MODE_PWR_DOWN
     *
     */
    Serial.print(F("In sleep function..."));
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);   // sleep mode is set here
    Serial.print(F("set sleep mode..."));
    
    sleep_enable();          // enables the sleep bit in the mcucr register
    // so sleep is possible. just a safety pin
    Serial.print(F("enabled sleep..."));
    
    /* Now it is time to enable an interrupt. We do it here so an
     * accidentally pushed interrupt button doesn't interrupt
     * our running program. if you want to be able to run
     * interrupt code besides the sleep function, place it in
     * setup() for example.
     *
     * In the function call attachInterrupt(A, B, C)
     * A   can be either 0 or 1 for interrupts on pin 2 or 3.
     *
     * B   Name of a function you want to execute at interrupt for A.
     *
     * C   Trigger mode of the interrupt pin. can be:
     *             LOW        a low level triggers
     *             CHANGE     a change in level triggers
     *             RISING     a rising edge of a level triggers
     *             FALLING    a falling edge of a level triggers
     *
     * In all but the IDLE sleep modes only LOW can be used.
     */
    
    attachInterrupt(0,wakeUpNow, HIGH);   // use interrupt 3 (pin 1) for the accelerometer - not inverted
    Serial.print(F("attached 0..."));
    attachInterrupt(1, nap, FALLING);    // Interrupt 2 (pin 0) for the Real Time Clock
    Serial.print(F("attached 1..."));
    Serial.println(F("entering sleep"));
    NonBlockingDelay(100);
    sleep_mode();            // here the device is actually put to sleep!!
    // THE PROGRAM CONTINUES FROM HERE AFTER WAKING UP
    Serial.print(F("Just woke up..."));
    
    sleep_disable();         // first thing after waking from sleep:
    // disable sleep...
    Serial.print(F("disabled sleep..."));
    detachInterrupt(0);      // disables interrupts so the
    Serial.print(F("detached 0..."));
    detachInterrupt(1);      // wakeUpNow code will not be executed
    // during normal running time.
    Serial.println(F("detached 1 - now back to work"));
}



void setLocationAndTime()   // Where we get info from the network to set time and location
{ // Now get and format Location
    // In this function we will set the clock based on the data returned form the GSMLoc function
    // The data looks like this: "-78.799919,35.898777,2014/12/13,17:49:28"
    // Since the Longitude could be shorter or longer, will use the commas for parsing the string
    char replyGSM[64];       // here we define a pointer to the character array
    char longitudeStr[12];
    char latitudeStr[11];
    char dateStr[11];
    char timeStr[9];
    char tempStr[5];
    const char commaSep[2] = ",";
    const char periodSep[2] = ".";
    char *token;  // We know the structure, token 1 = Long, token 2 = Lat, token 3 = date, token 4 = time
    Serial.println(F("Connected to GPRS - getting location"));
    tries = 0;                         // Reset the tries counter
    if (!fona.getGSMLoc(&returncode, replyGSM, 250)) {
        Serial.println(F("Failed!"));
        BlinkForever;                    // If we can't get the location - turn off and ask for a reboot
    }
    if (returncode == 0) {            // Return code 0 means that the get location call worked
        Serial.println(replyGSM);
        token = strtok(replyGSM,commaSep);  // This command breaks the replyGSM string into tokens that were separated by commas
        if (token != NULL) strcpy(longitudeStr, token);  // Copy it over as long as it is not NULL
        token = strtok(NULL,commaSep);            // Each time we call the function, we get the next token
        if (token != NULL) strcpy(latitudeStr, token);
        token = strtok(NULL,commaSep);
        if (token != NULL) strcpy(dateStr, token);
        token = strtok(NULL,commaSep);
        if (token != NULL) strcpy(timeStr, token);
        token = strtok(NULL,commaSep);
        
        // Now lets build Location
        char* latDecimal = strchr(latitudeStr,46);  // Points to the period  // searches character from the left
        strncat(Location,latitudeStr,(latDecimal+4-latitudeStr));  // copies the right number of digits
        strcat(Location,",\"lng\":");
        char* longDecimal = strchr(longitudeStr,46);  // Points to the period
        strncat(Location,longitudeStr,(longDecimal+4-longitudeStr));
        strcat(Location,"}");
        
        Serial.print(F("Location = "));
        Serial.println(Location);
        
        char* timeFirstColon = strchr(timeStr,58);  // Points to the first colon  - search from left
        char* timeSecondColon = strrchr(timeStr,58);  // Points to the second colon - search from right
        char* dateFirstSlash = strchr(dateStr,47);  // Points to the first slash
        char* dateSecondSlash = strrchr(dateStr,47);  // Points to the second slash
        
        strncpy(tempStr,timeStr,2);  // Copies two characters over
        int hour = atoi(tempStr);     // converts to integer
        strncpy(tempStr,timeFirstColon+1,2);
        int minute = atoi(tempStr);
        strncpy(tempStr,timeSecondColon+1,2);
        int second = atoi(tempStr);
        strncpy(tempStr,dateStr,4);
        int year = atoi(tempStr);
        strncpy(tempStr,dateFirstSlash+1,2);
        tempStr[2] = 0;                   // Had to do this as year is 4 digits
        int month = atoi(tempStr);
        strncpy(tempStr,dateSecondSlash+1,2);   // Would not terminate with a 0 so the assignment above
        int day = atoi(tempStr);
        set_time(year,month,day,hour,minute,second);  // Set's the time based on GSMLoc Results
    }
    else {
        Serial.print(F("Fail code #")); Serial.println(returncode);
        BlinkForever();
    }
}

void set_time(int year,int month, int day,int hour, int minute,int second)  // Here we set the clock with passed values
{
    Serial.println(F("Setting Time Based on GSMLoc Results"));
    // set initially to epoch
    RTC.setSeconds(second);
    RTC.setMinutes(minute);
    //  RTC.setMinutes(58);   // For Testing if you want to see the hourly alarm
    RTC.setHours(hour);
    RTC.setDays(day);
    RTC.setMonths(month);
    RTC.setYears(year);
    RTC.writeTime();
    read_time();
}

void read_time()    // Read the current time
{
    Serial.println (F("The current time is - Now "));
    //RTC.readTime(); // update RTC library's buffers from chip
    //printTime(0);
    //erial.println();
}

void set_hourly_alarm() // Sets an RTC alarm to go off every hour
{
    // Test basic functions (time read and write)
    Serial.print ("The current time is ");
    RTC.readTime(); // update RTC library's buffers from chip
    printTime(0);
    Serial.println();
    Serial.println(F("Writing alarm to go off at the top of every hour"));
    Serial.print(F("Read back: "));
    RTC.setSeconds(0);
    RTC.setMinutes(0);
    RTC.setHours(0);
    RTC.setDays(0);
    RTC.setMonths(0);
    RTC.setYears(0);
    RTC.setAlarmRepeat(EVERY_HOUR); // There is no DS1339 setting for 'alarm once' - user must shut off the alarm after it goes off.
    RTC.writeAlarm();
    delay(500);
    RTC.readAlarm();
    printTime(1);
}


void set_minute_alarm() // Sets an RTC alarm to go off every minute
{
    // Test basic functions (time read and write)
    Serial.print (F("The current time is "));
    RTC.readTime(); // update RTC library's buffers from chip
    printTime(0);
    Serial.println();
    Serial.println(F("Writing alarm to go off at the top of every minute"));
    RTC.setSeconds(0);
    RTC.setMinutes(0);
    RTC.setHours(0);
    RTC.setDays(0);
    RTC.setMonths(0);
    RTC.setYears(0);
    RTC.setAlarmRepeat(EVERY_MINUTE); // There is no DS1339 setting for 'alarm once' - user must shut off the alarm after it goes off.
    RTC.writeAlarm();
    NonBlockingDelay(500);
    RTC.readAlarm();
    Serial.print(F("Read back: "));
    printTime(1);
    Serial.println();
}

void printTime(byte type)       // Print a formatted string of the current date and time.
{
    int mins; int secs;
    // If 'type' is non-zero, print as an alarm value (seconds thru DOW/month only)
    // This function assumes the desired time values are already present in the RTC library buffer (e.g. readTime() has been called recently)
    
    if(!type)
    {
        Serial.print(int(RTC.getMonths()));
        Serial.print(F("/"));
        Serial.print(int(RTC.getDays()));
        Serial.print(F("/"));
        Serial.print(RTC.getYears());
    }
    else
    {
        //if(RTC.getDays() == 0) // Day-Of-Week repeating alarm will have DayOfWeek *instead* of date, so print that.
        {
            Serial.print(int(RTC.getDayOfWeek()));
            Serial.print(F("th day of week, "));
        }
        //else
        {
            Serial.print(int(RTC.getDays()));
            Serial.print(F("th day of month, "));      
        }
    }
    Serial.print(F("  "));
    Serial.print(int(RTC.getHours()));
    Serial.print(F(":"));
    mins = int(RTC.getMinutes());
    if (mins < 10) Serial.print(F("0")); 
    Serial.print(mins);
    Serial.print(F(":"));
    secs = int(RTC.getSeconds());  
    if (secs < 10) Serial.print(F("0"));
    Serial.println(secs);
}



int freeRam ()  // Debugging code, to check usage of RAM
{
    // Example Call: Serial.println(freeRam());
    extern int __heap_start, *__brkval; 
    int v; 
    return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval); 
}


void NonBlockingDelay(int millisDelay)  // Used for a non-blocking delay
{
    unsigned long commandTime = millis();
    while (millis() <= millisDelay + commandTime) { }
    return;
}

void BlinkForever() // For when things go badly wrong
{
    Serial.println(F("Error - Reboot"));
    TurnOffFona();
    while(1) {
        digitalWrite(ledPin,HIGH);
        NonBlockingDelay(200);
        digitalWrite(ledPin,LOW);
        NonBlockingDelay(200);
    }
}


void wakeUpNow()              // here the accelerometer interrupt is handled after wakeup
{
    // If int2 goes high, either p/l has changed or there's been a single/double tap
}

void nap()
{
    // Here is where we wake up from sleep based on the RTC alarm
}
