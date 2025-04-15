const int buzzerPin = 7;
const int pwmFreq = 20000; // 20kHz PWM frequency for audio
const int pwmResolution = 8; // 8-bit resolution

#define WAVEFORM_SIZE 256
uint8_t waveform[WAVEFORM_SIZE];

void setupWaveform() {
    for (int i = 0; i < WAVEFORM_SIZE; i++) {
        float theta = (2.0f * PI * i) / WAVEFORM_SIZE;
        // Custom harmonic blend
        float sample = 1.1 * sin(theta) + 0.2 * sin(2 * theta) + 0.3 * sin(3 * theta)
                     + 0.4 * sin(4 * theta) + 0.5 * sin(5 * theta);
        sample = (sample + 1.0f) * 127.5f; // Normalize to 0-255
        waveform[i] = (uint8_t)sample;
    }
}

void setup() {
    setupWaveform();
    ledcAttach(buzzerPin, pwmFreq, pwmResolution); // Auto-assigns channel
}

// New function: play sound at a specified frequency (Hz)
void playFrequencyHz(float frequencyHz, int durationMs) {
    float delayPerSampleUs = (1.0f / (frequencyHz * WAVEFORM_SIZE)) * 1e6;

    unsigned long totalSamples = (durationMs * 1000UL) / delayPerSampleUs;

    for (unsigned long i = 0; i < totalSamples; i++) {
        ledcWrite(buzzerPin, waveform[i % WAVEFORM_SIZE]);
        delayMicroseconds((int)delayPerSampleUs);
    }
}

void loop() {
    // Example: Play at 500Hz for 1 second
    playFrequencyHz(500.0f, 1000);

    // Pause
    delay(500);

    // Example: Play at 1000Hz for 0.5 seconds
    playFrequencyHz(1000.0f, 500);

    delay(1000);
}