#define buzzerPin 7
#include <FastLED.h>

CRGB leds[1];

void setup() {
    pinMode(buzzerPin, OUTPUT);
    FastLED.addLeds<WS2812,21,RGB>(leds,1);
    FastLED.setBrightness(100);
    fancyBeep();
    //f1ThrottleBlip();
}

void loop() {
    delay(10000); // Pause before repeating the pattern
    fancyBeep();
}

void f1ThrottleBlip() {
    int freqs[] = {300, 600, 900, 1200}; // Harmonics

    for (int i = 0; i < 4; i++) {
        tone(buzzerPin, freqs[i]);
        delay(75);
        noTone(buzzerPin);
        delay(25);
    }

    delay(100);

    // Simulate high rev "blip"
    tone(buzzerPin, 1500); // 5th harmonic, high-pitch finale
    delay(500);
    noTone(buzzerPin);
}

void fancyBeep() {
    //tone(buzzerPin, 200); // Low pitch
    tone(buzzerPin, 262); // Low pitch
    leds[0] = CRGB(255,0,0);
    FastLED.show();
    delay(75);
    noTone(buzzerPin);
    delay(25);
    //tone(buzzerPin, 400); // Mid pitch
    tone(buzzerPin, 330); // Mid pitch
    leds[0] = CRGB(0,255,0);
    FastLED.show();
    delay(75);
    noTone(buzzerPin);
    delay(25);
    //tone(buzzerPin, 800); // High pitch
    tone(buzzerPin, 392); // High pitch
    leds[0] = CRGB(0, 0, 255);
    FastLED.show();
    delay(75);
    noTone(buzzerPin);
    delay(25);
    //tone(buzzerPin, 800); // High pitch
    tone(buzzerPin, 523); // High pitch
    leds[0] = CRGB(0, 0, 255);
    FastLED.show();
    delay(75);
    noTone(buzzerPin);
    delay(500);
    //tone(buzzerPin, 1200); // Mid pitch for 500ms
    tone(buzzerPin, 784); // Mid pitch for 500ms
    leds[0] = CRGB(255,255,255);
    FastLED.show();
    delay(750);
    noTone(buzzerPin);
}