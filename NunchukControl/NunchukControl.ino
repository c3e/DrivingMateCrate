#include <Servo.h>
#include <Wire.h>
#include <ArduinoNunchuk.h>
//#include <FastLED.h>
#include <WS2812.h>

//#define DEBUG
#ifdef DEBUG
#define BAUDRATE 19200
#endif

#define ARRAY_SIZE(A) (sizeof(A) / sizeof((A)[0]))

// for wireless nunchuk
#define NunchukAnalogXLimitMin 0
#define NunchukAnalogXLimitMax 255
#define NunchukAnalogYLimitMin 0
#define NunchukAnalogYLimitMax 255

#define PWMMIN 1000
#define PWMMAX 2000
#define PWMMID 1500

#define MOTORLEFTPIN 5
#define MOTORRIGHTPIN 6

#define PIEZOPIN 8
#define AIRHORNPIN 7

#define LEDPin 9  // Digital output pin (default: 7)
#define NUM_LEDS 16   // Number of LEDs to drive (default: 9)

WS2812 LED(NUM_LEDS); // init the WS2812 LED libary with X LED's
cRGB ledValue; // holds the RGB color values
uint8_t ledsBrightness = 255;
uint8_t gHue = 0; // rotating "base color" used by many of the patterns

void ledAniDriveNormal();
void ledAniRotateBeacon();
void ledAniRotateBeaconDouble();
void ledAniAllWhite();
void ledAniAllBlack();
void ledAniTriplePulse();

uint8_t gCurrentPatternNumber = 0; // Index number of which pattern is current
// List of patterns to cycle through. Each is defined as a separate function below.
typedef void (*SimplePatternList[])();
SimplePatternList gPatterns = {
  ledAniDriveNormal,
  ledAniRotateBeacon,
  ledAniTriplePulse,
  ledAniRotateBeaconDouble,
  ledAniAllBlack
};

Servo motorLeftPPM;
Servo motorRightPPM;

ArduinoNunchuk nunchuk = ArduinoNunchuk();

int deadZone = 5; //joystick dead zone
// failsafe flag for after z Button release
bool driveUnlocked = false;

void nunchukDebug();
void driveMixerLoop();
void signalHornLoop();
void ledEffectLoop();


void setup() {
#ifdef DEBUG
  Serial.begin(BAUDRATE);
#endif
  nunchuk.init(); // initallize the i2c nunchuk device

  pinMode(AIRHORNPIN, OUTPUT);
  digitalWrite(AIRHORNPIN, LOW);

  LED.setOutput(LEDPin);
  LED.setColorOrderGRB();  // RGB color order

  motorLeftPPM.attach(MOTORLEFTPIN);
  motorRightPPM.attach(MOTORRIGHTPIN);

  motorLeftPPM.writeMicroseconds(PWMMID);
  motorRightPPM.writeMicroseconds(PWMMID);
}

void loop() {
  nunchuk.update();
#ifdef DEBUG
  nunchukDebug();
#endif
  driveMixerLoop();
  //signalHornLoop();
  ledEffectLoop();
}

#ifdef DEBUG
void nunchukDebug() {
  Serial.print(nunchuk.analogX, DEC);
  Serial.print(' ');
  Serial.print(nunchuk.analogY, DEC);
  Serial.print(' ');
  Serial.print(nunchuk.accelX, DEC);
  Serial.print(' ');
  Serial.print(nunchuk.accelY, DEC);
  Serial.print(' ');
  Serial.print(nunchuk.accelZ, DEC);
  Serial.print(' ');
  Serial.print(nunchuk.zButton, DEC);
  Serial.print(' ');
  Serial.println(nunchuk.cButton, DEC);
}
#endif

int8_t avgX = 0;
int8_t avgY = 0;
void driveMixerLoop() {
  // INPUTS
  int8_t nJoyX = 0; // Joystick X input (-128..+127)
  int8_t nJoyY = 0; // Joystick Y input (-128..+127)

  nJoyX = map(nunchuk.analogX, NunchukAnalogXLimitMin, NunchukAnalogXLimitMax, -128, 127);
  nJoyY = map(nunchuk.analogY, NunchukAnalogYLimitMin, NunchukAnalogYLimitMax, -128, 127);

  if (nJoyX == 0) {
    avgX = 0;
  }

  if (nJoyY == 0) {
    avgY = 0;
  }

  avgX = ((avgX * 7) / 8) + (nJoyX / 8);
  avgY = ((avgY * 7) / 8) + (nJoyY / 8);

  nJoyX = avgX;
  nJoyY = avgY;

#ifdef DEBUG
  Serial.print("ELE:");
  Serial.print(nJoyY, DEC);
  Serial.print(", AIL:");
  Serial.print(nJoyX, DEC);
#endif

  if (nunchuk.zButton == 1) {
    if (nJoyX == 0 && nJoyY == 0) {
      if (!driveUnlocked) {
        driveUnlocked = true;
      }
    }
    else {
      if (!driveUnlocked) {
#ifdef DEBUG
        Serial.print(" FailSafe active!");
        Serial.println();
#endif
      }
    }

    if (driveUnlocked) {
      // CONFIG - fPivYLimt:
      // The threshold at which the pivot action starts
      // This threshold is measured in units on the Y-axis
      // away from the X-axis (Y=0). A greater value will assign
      // more of the joystick's range to pivot actions.
      // Allowable range: (0..+127)
      float fPivYLimit = 32;

      // TEMP VARIABLES
      float   nMotPremixL;  // Motor (left)  premixed output      (-128..+127)
      float   nMotPremixR;  // Motor (right) premixed output      (-128..+127)
      int     nPivSpeed;    // Pivot Speed                        (-128..+127)
      float   fPivScale;    // Balance scale b/w drive and pivot  (   0..1   )


      // Calculate Drive Turn output due to Joystick X input
      if (nJoyY >= 0) {
        // Forward
        nMotPremixL = (nJoyX >= 0) ? 127 : (127 + nJoyX);
        nMotPremixR = (nJoyX >= 0) ? (127 - nJoyX) : 127;
      } else {
        // Reverse
        nMotPremixL = (nJoyX >= 0) ? (127 - nJoyX) : 127;
        nMotPremixR = (nJoyX >= 0) ? 127 : (127 + nJoyX);
      }

      // Scale Drive output due to Joystick Y input (throttle)
      nMotPremixL = nMotPremixL * nJoyY / 128;
      nMotPremixR = nMotPremixR * nJoyY / 128;

      // Now calculate pivot amount
      // - Strength of pivot (nPivSpeed) based on Joystick X input
      // - Blending of pivot vs drive (fPivScale) based on Joystick Y input
      nPivSpeed = nJoyX;
      fPivScale = (abs(nJoyY) > fPivYLimit) ? 0.0 : (1.0 - abs(nJoyY) / fPivYLimit);

      // OUTPUTS
      int nMotMixL; // Motor (left)  mixed output (-128..+127)
      int nMotMixR; // Motor (right) mixed output (-128..+127)

      // Calculate final mix of Drive and Pivot
      nMotMixL = (1.0 - fPivScale) * nMotPremixL + fPivScale * ( nPivSpeed);
      nMotMixR = (1.0 - fPivScale) * nMotPremixR + fPivScale * (-nPivSpeed);

      // Convert to Motor PWM range
      nMotMixL = map(nMotMixL, -128, 127, PWMMIN, PWMMAX);
      nMotMixR = map(nMotMixR, -128, 127, PWMMIN, PWMMAX);

#ifdef DEBUG
      Serial.print(" MotorL:");
      Serial.print(nMotMixL, DEC);
      Serial.print(", MotorR:");
      Serial.print(nMotMixR, DEC);
      Serial.println();
#endif

      motorLeftPPM.writeMicroseconds(nMotMixL);
      motorRightPPM.writeMicroseconds(nMotMixR);
    }
  }
  else {
#ifdef DEBUG
    Serial.println(" IDLE");
#endif

    avgX = 0;
    avgY = 0;

    motorLeftPPM.writeMicroseconds(PWMMID);
    motorRightPPM.writeMicroseconds(PWMMID);

    driveUnlocked = false;
  }
}

void signalHornLoop() {
  if (nunchuk.cButton == 1) {
    digitalWrite(AIRHORNPIN, HIGH);
  }
  else {
    digitalWrite(AIRHORNPIN, LOW);
  }
}


uint16_t patternChangeInterval = 1000;
uint32_t patternChangePrevMillis = 0;

void ledEffectLoop() {
  if (nunchuk.cButton == 1) {
    if (millis() - patternChangePrevMillis > patternChangeInterval) {
      patternChangePrevMillis = millis();
      nextPattern();
    }
  }

  // Call the current pattern function once, updating the 'ledsHelmet' array
  gPatterns[gCurrentPatternNumber]();

  LED.sync(); // Sends the data to the LED strip
}

void nextPattern()
{
  // add one to the current pattern number, and wrap around at the end
  gCurrentPatternNumber = (gCurrentPatternNumber + 1) % ARRAY_SIZE( gPatterns);
}

void ledAniAllRainbow()
{
  // FastLED's built-in rainbow generator
  //fill_rainbow( ledsMateCrate, NUM_LEDS, gHue, 7);
}

void ledAniAllWhite() {
  ledValue.r = 255;
  ledValue.g = 255;
  ledValue.b = 255;

  for (int i = 0; i < NUM_LEDS; ++i) {
    LED.set_crgb_at(i, ledValue);
  }
}

void ledAniAllBlack() {
  ledValue.r = 0;
  ledValue.g = 0;
  ledValue.b = 0;

  for (int i = 0; i < NUM_LEDS; ++i) {
    LED.set_crgb_at(i, ledValue);
  }
}


const uint8_t turnLightLeftLimit = 108;
const uint8_t turnLightRightLimit = 148; //0-128-255

void ledAniDriveNormal() {
  ledAniAllBlack(); // reset all leds to black(off)

  ledValue.r = 255;
  ledValue.g = 255;
  ledValue.b = 255;

  LED.set_crgb_at(0, ledValue);
  LED.set_crgb_at(1, ledValue);

  if (nunchuk.analogX < turnLightLeftLimit) {
    ledAniDriveTurnLeftBlink();
  }

  if (nunchuk.analogX > turnLightRightLimit) {
    ledAniDriveTurnRightBlink();
  }

  ledValue.r = 255;
  ledValue.g = 0;
  ledValue.b = 0;

  LED.set_crgb_at(14, ledValue);
  LED.set_crgb_at(15, ledValue);
}


uint16_t turnLeftInterval = 500;
uint32_t turnLeftPrevMillis = 0;
uint8_t turnLeftLightOn = false;

void ledAniDriveTurnLeftBlink() {
  if (millis() - turnLeftPrevMillis > turnLeftInterval) { // is it time to fade ?
    turnLeftPrevMillis = millis(); // save the last time the hue faded one step

    if (turnLeftLightOn) {
      turnLeftLightOn = false;
    }
    else {
      turnLeftLightOn = true;
    }
  }

  if (turnLeftLightOn) {
    ledValue.r = 255;
    ledValue.g = 200;
    ledValue.b = 0;

    LED.set_crgb_at(5, ledValue);
    LED.set_crgb_at(6, ledValue);
    LED.set_crgb_at(13, ledValue);
  }
}

uint16_t turnRightInterval = 500;
uint32_t turnRightPrevMillis = 0;
uint8_t turnRightLightOn = false;

void ledAniDriveTurnRightBlink() {
  if (millis() - turnRightPrevMillis > turnRightInterval) {
    turnRightPrevMillis = millis();

    if (turnRightLightOn) {
      turnRightLightOn = false;
    }
    else {
      turnRightLightOn = true;
    }
  }

  if (turnRightLightOn) {
    ledValue.r = 255;
    ledValue.g = 200;
    ledValue.b = 0;

    LED.set_crgb_at(2, ledValue);
    LED.set_crgb_at(9, ledValue);
    LED.set_crgb_at(10, ledValue);
  }
}


//uint8_t ledsBigRing[10] = {0, 1, 2, 9, 10, 15, 14, 13, 6, 5};
//uint8_t ledsSmallRing[6] = {4, 3, 8, 11, 12, 7};

uint8_t ledsLeftSide[8] = {0, 4, 5, 6, 7, 12, 13, 14};
uint8_t ledsRightSide[8] = {1, 2, 3, 8, 9, 10, 11, 15};

uint8_t ledsBigRing[10] = {0, 1, 2, 9, 10, 15, 14, 13, 6, 5};
uint8_t ledsSmallRing[10] = {4, 3, 3, 8, 11, 11, 12, 12, 7, 4};

//uint8_t ledsBigRingInvert[10] = {5, 6, 13, 14, 15, 10, 9, 13, 6, 5};
//uint8_t ledsSmallRingÍnvert[10] = {4, 7, 12, 12, 11, 11, 8, 3, 3, 4};

uint8_t ringBigInterval = 50;
uint32_t ringBigPrevMillis = 0;
uint8_t ringBigForward = true;
uint8_t ringBeaconBigCurrentLed = 0;

void ledAniRotateBeacon() {
  if (millis() - ringBigPrevMillis > ringBigInterval) {
    ringBigPrevMillis = millis();

    ledAniAllBlack();

    ledValue.r = 255;
    ledValue.g = 200;
    ledValue.b = 0;

    LED.set_crgb_at(ledsBigRing[ringBeaconBigCurrentLed], ledValue);
    LED.set_crgb_at(ledsSmallRing[ringBeaconBigCurrentLed], ledValue);

    if (ringBeaconBigCurrentLed < 9) {
      ringBeaconBigCurrentLed++;
    }
    else {
      ringBeaconBigCurrentLed = 0;
    }
  }
}

uint8_t ringBeaconDoubleBigCurrentLed = 0;
uint8_t ringBeaconDoubleBig2CurrentLed = 5;

void ledAniRotateBeaconDouble() {
  if (millis() - ringBigPrevMillis > ringBigInterval) {
    ringBigPrevMillis = millis();

    ledAniAllBlack();

    ledValue.r = 0;
    ledValue.g = 0;
    ledValue.b = 255;

    LED.set_crgb_at(ledsBigRing[ringBeaconDoubleBigCurrentLed], ledValue);
    LED.set_crgb_at(ledsSmallRing[ringBeaconDoubleBigCurrentLed], ledValue);

    if (ringBeaconDoubleBigCurrentLed < 9) {
      ringBeaconDoubleBigCurrentLed++;
    }
    else {
      ringBeaconDoubleBigCurrentLed = 0;
    }


    ledValue.r = 255;
    ledValue.g = 0;
    ledValue.b = 0;

    LED.set_crgb_at(ledsBigRing[ringBeaconDoubleBig2CurrentLed], ledValue);
    LED.set_crgb_at(ledsSmallRing[ringBeaconDoubleBig2CurrentLed], ledValue);

    if (ringBeaconDoubleBig2CurrentLed < 9) {
      ringBeaconDoubleBig2CurrentLed++;
    }
    else {
      ringBeaconDoubleBig2CurrentLed = 0;
    }

  }
}


uint16_t triplePauseShortInterval = 20;
uint16_t triplePauseLongInterval = 125;

uint32_t triplePulseShortMillis = 0;
uint32_t triplePulseLongMillis = 0;

bool isOn = false;
bool isRight = true;
uint8_t shortPulseCount = 1;

void ledAniTriplePulse() {
  if (shortPulseCount == 0) {
    if (millis() - triplePulseShortMillis > triplePauseShortInterval) {
      triplePulseShortMillis = millis();
      if (isOn) {
        isOn = false;
      }
      else {
        isOn = true;
        shortPulseCount++;
      }
    }
  }
  else if (shortPulseCount == 1) {
    if (millis() - triplePulseShortMillis > triplePauseLongInterval) {
      triplePulseShortMillis = millis();
      if (isOn) {
        isOn = false;
      }
      else {
        isOn = true;
        shortPulseCount++;

        if (isRight) {
          isRight = false;
        }
        else {
          isRight = true;
        }
      }
    }
  }
  else {
    if (millis() - triplePulseLongMillis > triplePauseShortInterval) {
      triplePulseLongMillis = millis();
      if (isOn) {
        isOn = false;
      }
      else {
        isOn = true;
        shortPulseCount = 0;
      }
    }
  }



  if (isOn) {
    ledValue.r = 0;
    ledValue.g = 0;
    ledValue.b = 255;

    if (isRight) {
      for (uint8_t currentLed = 0; currentLed < 8; currentLed++) {
        LED.set_crgb_at(ledsRightSide[currentLed], ledValue);
      }
    }
    else {
      for (uint8_t currentLed = 0; currentLed < 8; currentLed++) {
        LED.set_crgb_at(ledsLeftSide[currentLed], ledValue);
      }
    }
  }
  else {
    ledAniAllBlack();
  }
}