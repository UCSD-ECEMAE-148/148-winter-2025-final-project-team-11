#include <Arduino.h>
#include <pindefinitions.h>
#include <FastLED.h>
#include <Wire.h>

#define I2C_DEV_ADDR 0x55

CRGB rear_leds[NUM_LEDS];
CRGB front_leds[NUM_LEDS];
CRGB top_leds[NUM_LEDS];
CRGB stat_led[1];

uint32_t i = 0;

enum class CAR_STATES {
  IDLE,
  PURSUIT,
} CAR_STATE;

size_t animationState = 0;
size_t topAnimationState = 0;

void onRequest() {
  Wire.print(i++);
  Wire.print(" Packets.");
  Serial.println("onRequest");
  Serial.println();
}

void onReceive(int len) {
  Serial.printf("onReceive length: %d: ", len);
  
  while (Wire.available()) {
    char theInput = Wire.read();
    if (theInput == '0')
      CAR_STATE = CAR_STATES::IDLE;
    else if (theInput == '1')
      CAR_STATE = CAR_STATES::PURSUIT;
    Serial.write(theInput);
  }
  Serial.println();
}

void setup() {
  delay(750);
  Serial.begin(  115200);

  FastLED.addLeds<WS2812B, LED_SIG_REAR, RGB>(rear_leds, NUM_LEDS).setCorrection( TypicalLEDStrip );
  FastLED.addLeds<WS2812B, LED_SIG_FRONT, RGB>(front_leds, NUM_LEDS).setCorrection( TypicalLEDStrip );
  FastLED.addLeds<WS2812B, LED_TOP, RGB>(top_leds, NUM_LEDS).setCorrection( TypicalLEDStrip );
  FastLED.addLeds<WS2812B, ONBOARD_LED, GRB>(stat_led, NUM_LEDS).setCorrection( TypicalLEDStrip );
  FastLED.setBrightness( 100 );

  pinMode(LED_PWM_FRONT, OUTPUT);
  pinMode(LED_PWM_REAR, OUTPUT);

  analogWrite(LED_PWM_FRONT, 0);
  analogWrite(LED_PWM_REAR, 0);

  analogWrite(BUZZER1, 0);
  analogWrite(BUZZER2, 0);

  CAR_STATE = CAR_STATES::IDLE;

  Wire.onReceive(onReceive);
  Wire.onRequest(onRequest);
  Wire.begin((uint8_t)I2C_DEV_ADDR, JETSON_I2C_SDA, JETSON_I2C_SCL, 100000);

}

void setBrightLeds(const int &aVal) {
 analogWrite(LED_PWM_FRONT, aVal); 
 analogWrite(LED_PWM_REAR, aVal); 
}

void setContinuousColor(const int &aNumLeds, CRGB Color, const int &StripOffset, bool aSide) {
  int offset{0};
  if (aSide) offset = NUM_LEDS - 1;
  for (int i = StripOffset; i < aNumLeds + StripOffset; i++) {
    front_leds[abs(offset - i)] = Color;
    rear_leds[abs(offset - i)] = Color;
  }
}

void setTopContinuousColor(const int &aNumLeds, CRGB Color, const int &StripOffset, bool aSide) {
  int offset{0};
  if (aSide) offset = NUM_LEDS - 1;
  for (int i = StripOffset; i < aNumLeds + StripOffset; i++) {
    top_leds[abs(offset - i)] = Color;
  }
}

void loop() {

  switch (CAR_STATE) {
    case CAR_STATES::IDLE:
      stat_led[0] = CRGB::Purple;
      analogWrite(LED_PWM_FRONT, 5);
      analogWrite(LED_PWM_REAR, 5);

      fill_solid(top_leds, NUM_LEDS, CRGB::Black);
      fill_solid(front_leds, NUM_LEDS, CRGB::Black);
      fill_solid(rear_leds, NUM_LEDS, CRGB::Black);
      break;
    case CAR_STATES::PURSUIT:
      EVERY_N_MILLIS(175) {
        animationState++;
        if (animationState > 12) animationState = 0;
      }

      if (animationState == 0) {
        setBrightLeds(5);
        setContinuousColor(9, CRGB::Black, 1, 0);
        stat_led[0] = CRGB::Blue;
      } else if (animationState == 1) {
        setBrightLeds(0);
        setContinuousColor(4, CRGB::Blue, 1, 0);

      } else if (animationState == 2) {
        setContinuousColor(4, CRGB::Black, 1, 0);
        setContinuousColor(4, CRGB::Red, 1, 1);
      } else if (animationState == 3) {
        setBrightLeds(5);
        setContinuousColor(4, CRGB::Black, 1, 1);
        setContinuousColor(4, CRGB::Blue, 1, 0);
      } else if (animationState == 4) {
        setBrightLeds(0);
        setContinuousColor(4, CRGB::Black, 1, 0);
        setContinuousColor(4, CRGB::Red, 1, 1);
      } else if (animationState == 5) {
        setContinuousColor(4, CRGB::Black, 1, 1);
        setContinuousColor(4, CRGB::Blue, 1, 0);
      } else if (animationState == 6) {
        stat_led[0] = CRGB::Red;
        setContinuousColor(4, CRGB::Black, 1, 0);
      } else if (animationState == 7) {
        setContinuousColor(1, CRGB::White, 0, 1);
        setContinuousColor(1, CRGB::White, 0, 0);
      } else if (animationState == 8) {
        setBrightLeds(5);
        setContinuousColor(1, CRGB::Black, 0, 1);
        setContinuousColor(1, CRGB::Black, 0, 0);
        setContinuousColor(9, CRGB::Blue, 1, 0);
      } else if (animationState == 9) {
        setBrightLeds(0);
        setContinuousColor(1, CRGB::White, 0, 1);
        setContinuousColor(1, CRGB::White, 0, 0);
        setContinuousColor(9, CRGB::Black, 1, 0);
      } else if (animationState == 10) {
        setContinuousColor(1, CRGB::Black, 0, 1);
        setContinuousColor(1, CRGB::Black, 0, 0);
        setContinuousColor(9, CRGB::Red, 1, 0);
      } else if (animationState == 11) {
        setContinuousColor(9, CRGB::Black, 1, 0);
      } else if (animationState == 12) {
        setContinuousColor(9, CRGB::White, 1, 0);
      } 

      EVERY_N_MILLIS(250) {
        topAnimationState++;
        if (topAnimationState > 6) topAnimationState = 1;
      }

      if (topAnimationState == 0) {
      } else if (topAnimationState == 1) {
        setTopContinuousColor(4, CRGB::Black, 0, 0);
        setTopContinuousColor(4, CRGB::Blue, 0, 1);
      } else if (topAnimationState == 2) {
        setTopContinuousColor(4, CRGB::Black, 0, 1);
        setTopContinuousColor(4, CRGB::Red, 0, 0); // FIRST HALF OF LEDS RGB ORDER IS OFF
      } else if (topAnimationState == 3) {
        setTopContinuousColor(4, CRGB::Black, 0, 0);
        setTopContinuousColor(4, CRGB::Blue, 0, 1);
      } else if (topAnimationState == 4) {
        setTopContinuousColor(4, CRGB::Black, 0, 1);
        setTopContinuousColor(4, CRGB::Red, 0, 0);
      } else if (topAnimationState == 5) {
        setTopContinuousColor(4, CRGB::Black, 0, 0);
        setTopContinuousColor(4, CRGB::Blue, 0, 1);
      } else if (topAnimationState == 6) {
        setTopContinuousColor(4, CRGB::Black, 0, 1);
        setTopContinuousColor(4, CRGB::Red, 0, 0);
      }
    break;
  }

  FastLED.show();
  delay(10);

  // put your main code here, to run repeatedly:
}
