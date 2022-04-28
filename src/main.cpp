#include <Arduino.h>
#include <AutoPID.h>
#include <DallasTemperature.h>
#include <Encoder.h>
#include <OneWire.h>
#include <PID_v1.h>
#include <U8g2lib.h>
#include <Wire.h>
#include <max6675.h>

#define DALLAS_READ_DELAY 500
#define SCREEN_REFRESH_DELAY 100

#define DALLAS_PIN 5
#define OUTPUT_PIN 10

// Controls
#define A_PIN 2
#define B_PIN 3
#define BUTTON_PIN 4

// PID
#define TIME_STEP 500
#define PULSE_WIDTH 25
#define KP .016
#define KI 0.0005
#define KD 0

#define DEFAULT_SETUP 50
#define MAX_TEMPERATURE 290

// visualization
unsigned int sampleInterval = 2500;

double temperature = 0, setPoint = DEFAULT_SETUP;
bool relayState = false;

// controls
Encoder enc(A_PIN, B_PIN);

U8G2_ST7920_128X64_F_HW_SPI u8g2(U8G2_R0, /* CS=*/9, /* reset=*/8);

// Dallas
OneWire oneWire(DALLAS_PIN);
DallasTemperature ds(&oneWire);

// // thermal couple
int thermoDO = A2;
int thermoCS = A1;
int thermoCLK = A0;

MAX6675 thermocouple(thermoCLK, thermoCS, thermoDO);

//input/output variables passed by reference, so they are updated automatically
AutoPIDRelay pid(&temperature, &setPoint, &relayState, PULSE_WIDTH, KP, KI, KD);

// timers
unsigned long lastTempUpdate;
unsigned long lastTempSample;
unsigned long lastScreenRefresh;

// temperatures
float dsTemperature = 0;

short tempSamples[128] = {};
byte sampleID = 0;

bool isValid();

//call repeatedly in loop, only updates after a certain time interval
//returns true if update happened
bool updateTemperature()
{
    if ((millis() - lastTempUpdate) > DALLAS_READ_DELAY) {
        lastTempUpdate = millis();

        //dsTemperature = ds.getTempCByIndex(0);
        //ds.requestTemperatures();
        //temperature = dsTemperature;

        temperature = thermocouple.readCelsius();

        if ((millis() - lastTempSample) > sampleInterval) {
            lastTempSample = millis();

            tempSamples[sampleID] = round(temperature * 10);

            sampleID++;
            if (sampleID >= 128) {
                sampleID = 0;
            }
        }

        return true;
    }

    return false;
}

// updates screen
bool updateScreen()
{
    if ((millis() - lastScreenRefresh) > SCREEN_REFRESH_DELAY) {
        lastScreenRefresh = millis();

        u8g2.clearBuffer();

        char buf[13];

        // current value
        sprintf(buf, "%d\xb0", round(temperature));

        u8g2.setFont(u8g2_font_6x12_tf);
        u8g2.drawStr(127 - u8g2.getStrWidth(buf), 63, buf);

        if (pid.isStopped()) {
            u8g2.drawStr(76, 63, "OFF");
        } else {
            u8g2.drawStr(76, 63, "ON");
        }

        // secondary values
        u8g2.setFont(u8g2_font_u8glib_4_tf);

        switch (sampleInterval) {
        case 10000:
            u8g2.drawStr(117, 5, "10s");
            break;

        case 5000:
            u8g2.drawStr(121, 5, "5s");
            break;

        case 2500:
            u8g2.drawStr(114, 5, "2.5s");
            break;

        case 1000:
            u8g2.drawStr(121, 5, "1s");
            break;

        default:
            break;
        }

        // todo: rewrite

        // detecting max value
        short maxTemp = setPoint * 10, minTemp = -100;
        for (int i = 0; i < 128; i++) {
            if (tempSamples[i] <= maxTemp) {
                if (tempSamples[i] < minTemp && tempSamples[i] != NAN && tempSamples[i] != 0) {
                    minTemp = tempSamples[i];
                }

                continue;
            }

            maxTemp = tempSamples[i];
        }

        // calculating the projection
        maxTemp = (maxTemp / 10);
        minTemp = (minTemp / 10);

        short maxValue = 300, minValue = 0;

        if (maxTemp < 100) {
            maxValue = ceil((maxTemp + 60) / 50) * 50;
        } else {
            maxValue = ceil((maxTemp + 100) / 50) * 50;
        }

        // adjusting the projection
        float div = 10.0 * (maxValue - minValue) / 60;

        sprintf(buf, "%d\xb0", maxValue);
        u8g2.drawStr(1, 5, buf);

        // lowest value
        sprintf(buf, "%d\xb0", minValue);
        u8g2.drawStr(1, 63, buf);

        // temp values written as in cyclic buffer, we have to read from end to the beginning of it
        for (int i = sampleID; i < 128; i++) {
            u8g2.drawPixel(i - sampleID, 64 - round(float(tempSamples[i] - minValue * 10) / div));
        }

        if (sampleID > 0) {
            for (int i = sampleID; i >= 0; i--) {
                u8g2.drawPixel(128 - sampleID + i, 64 - round(float(tempSamples[i] - minValue * 10) / div));
            }
        }

        // todo: setPoint
        for (int i = 0; i < 43; i++) {
            u8g2.drawPixel(i * 3 + 1, 64 - round(float(setPoint * 10 - minValue * 10) / div));
        }

        sprintf(buf, "%d\xb0", round(setPoint));
        u8g2.drawStr(1, 64 - round(float(setPoint * 10 - minValue * 10) / div) - 1, buf);

        //sprintf(buf, "P: %u", round(pid.getPulseValue()*100));
        //u8g2.drawStr(16, 63, buf);

        u8g2.sendBuffer();

        return true;
    }

    return false;
}

// SETUP
void setup(void)
{
    for (int i = 0; i < 128; i++) {
        tempSamples[i] = 0;
    }

    enc.write(setPoint * 4);
    pid.stop();

    u8g2.begin();
    u8g2.setBusClock(1000000);

    pinMode(OUTPUT_PIN, OUTPUT);
    digitalWrite(OUTPUT_PIN, LOW);

    pinMode(A_PIN, INPUT_PULLUP);
    pinMode(B_PIN, INPUT_PULLUP);

    pinMode(BUTTON_PIN, INPUT);

    ds.begin();
    ds.requestTemperatures();
    while (!updateTemperature()) { }

    tempSamples[0] = round(temperature * 10);
    sampleID++;

    // set PID update interval to 250ms
    pid.setTimeStep(TIME_STEP);
}

bool buttonPressed = false;

void togglePID()
{
    if (pid.isStopped()) {
        pid.run();
    } else {
        pid.stop();
        digitalWrite(OUTPUT_PIN, LOW);
    }
}

void updateControls()
{
    if (digitalRead(BUTTON_PIN) == HIGH && !buttonPressed) {
        // todo: modes
        togglePID();
        buttonPressed = true;
    }

    if (digitalRead(BUTTON_PIN) == LOW) {
        buttonPressed = false;
    }

    long newSetPoint = enc.read();
    if (setPoint != newSetPoint / 4) {
        setPoint = newSetPoint / 4;
    }
}

// MAIN LOOP
void loop(void)
{
    updateTemperature();
    updateScreen();
    updateControls();

    if (!pid.isStopped()) {
        pid.run();
    }

    if (isValid()) {
        digitalWrite(OUTPUT_PIN, relayState);
    } else {
        digitalWrite(OUTPUT_PIN, LOW);
    }
}

bool isValid()
{
    if (temperature <= 0 || temperature > MAX_TEMPERATURE || temperature == NAN) {
        return false;
    }

    return true;
}