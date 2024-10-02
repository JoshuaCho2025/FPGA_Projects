#include <Arduino.h>
#include <Adafruit_CircuitPlayground.h>
#include "arduinoFFT.h"

#define SAMPLES 128
#define SAMPLING_FREQUENCY 200
float vReal[SAMPLES];
float vImag[SAMPLES];
float totalIntensity = 0.0; // Add totalIntensity declaration
int intensityCount = 0;     // Add intensityCount declaration

bool displayTremorTypeOnly = false;

// Flag for FFT completion
volatile bool isFFTComplete = false;
volatile bool is3MinElapsed = false;

// Timer interval for 3 minutes (in milliseconds)
const unsigned long TIMER_INTERVAL = 180000;
volatile int overflowCount = 0;
// Time tracking for LED off delay
unsigned long tremorTypeLEDStartTime = 0;
const unsigned long LED_ON_DELAY = 2000; // 2 seconds

// Create an FFT object with template float.
ArduinoFFT<float> FFT(vReal, vImag, SAMPLES, SAMPLING_FREQUENCY);

volatile int i = 0;

void calculateIntensity();
void displayIntensityOnLEDs(float intensity);
void determineTremorType(float intensity);
void timerSetUp();

ISR(TIMER0_COMPA_vect) {
  // Collect data
  vImag[i] = 0;
  if (i < 128) {
    vReal[i++] = sqrt(pow(CircuitPlayground.motionX(), 2) + pow(CircuitPlayground.motionY(), 2) + pow(CircuitPlayground.motionZ(), 2));
  } else {
    i = 0;
    // Set flag indicating FFT analysis is complete
    isFFTComplete = true;
  }
}

ISR(TIMER1_COMPA_vect) {
  overflowCount++;
  if (overflowCount >= 45) {
    // Set flag indicating 3 minutes elapsed
    is3MinElapsed = true;
    overflowCount = 0; // Reset overflow count
  }
}

void calculateIntensity() {
  float sumSquared = 0.0;
  for (int j = 3; j < 6; j++) {  
    sumSquared += pow(vReal[j], 2);
  }

  // Calculate intensity
  float N = 4; // Number of frequencies in range
  float intensity = (sqrt(sumSquared / (N * N)) - 14.23); // adjust

  // Accumulate total intensity and count
  totalIntensity += intensity;
  intensityCount++;

  Serial.print("Intensity: ");
  Serial.println(intensity);
  Serial.println("======");

  // Display current intensity on LEDs
  displayIntensityOnLEDs(intensity);
}

void displayIntensityOnLEDs(float intensity) {
  const uint8_t maxIntensityLED = 9; // Maximum number of LEDs to use for intensity
  if (!displayTremorTypeOnly) {
    uint8_t ledsToLight = (uint8_t)(maxIntensityLED * intensity / 20); // Assuming intensity scale max is 20
    for (uint8_t i = 0; i < maxIntensityLED; i++) {
      if (i < ledsToLight) {
        CircuitPlayground.setPixelColor(i, 0, 255, 0); // Green color for visible intensity
      } else {
        CircuitPlayground.setPixelColor(i, 0, 0, 0); // Turn off the LED
      }
    }
  } else {
    // When displaying tremor type, ensure all intensity LEDs are off
    for (uint8_t i = 0; i < maxIntensityLED; i++) {
      CircuitPlayground.setPixelColor(i, 0, 0, 0); // Turn off the LED
    }
  }
}

void determineTremorType(float intensity) {
  const uint8_t tremorTypePixel = 9; // The index of the last NeoPixel reserved for tremor type
  CircuitPlayground.setPixelColor(tremorTypePixel, 0, 0, 0); // Ensure LED is off before setting new color
  if (intensity >= 15 && intensity <= 20) {
    CircuitPlayground.setPixelColor(tremorTypePixel, 255, 0, 0); // Big tremor - red light
    tremorTypeLEDStartTime = millis(); // Record start time for LED
  } else if (intensity >= 3.1 && intensity < 14.9) {
    CircuitPlayground.setPixelColor(tremorTypePixel, 128, 0, 128); // Medium Tremor - purple light
    tremorTypeLEDStartTime = millis(); // Record start time for LED
  } else if (intensity >= 1 && intensity <= 3) {
    CircuitPlayground.setPixelColor(tremorTypePixel, 0, 0, 255); // Small tremor - blue light
    tremorTypeLEDStartTime = millis(); // Record start time for LED
  }
}

void timerSetUp() {
  // Set up Timer0 for data collection at 64Hz
  TCCR0A = 0b00000010; // CTC mode
  TCCR0B = 0b00000101; // Prescaler 1024
  OCR0A = 124; // 128us x 125 = 16ms => 64Hz

  TIMSK0 |= (1 << OCIE0A); // Enable Timer0 compare match A interrupt

  // Set up Timer1 for 3 minutes interval
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0; // Initialize counter value to 0
  OCR1A = 31250; // Set compare match value for 3 minutes interval
  TCCR1B |= (1 << WGM12); // Configure timer for CTC mode
  TCCR1B |= (1 << CS12) | (1 << CS10); // Prescaler = 1024
  TIMSK1 |= (1 << OCIE1A); // Enable Timer1 compare match A interrupt
}

void setup() {
  Serial.begin(115200);
  CircuitPlayground.begin();
  CircuitPlayground.setAccelRange(LIS3DH_RANGE_8_G);
  timerSetUp();
}

void loop() {
  if (isFFTComplete) {
    // Code to execute after FFT analysis is done
    // Reset isFFTComplete flag
    isFFTComplete = false;
   FFT.windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);  // Apply a window function to minimize leakage effect
   FFT.compute(FFT_FORWARD);                         // Compute the FFT
   FFT.complexToMagnitude();                         // Compute magnitudes
   // Pass the max frequency and magnitude to calculate intensity function
   calculateIntensity();
  }

  if (is3MinElapsed) {
    // Calculate average intensity
    float averageIntensity = totalIntensity / intensityCount;
    Serial.print("Average Intensity over 3 minutes: ");
    Serial.println(averageIntensity);
    determineTremorType(averageIntensity);

    // Reset intensity count and total intensity
    intensityCount = 0;
    totalIntensity = 0;

    // Reset 3 minutes elapsed flag
    is3MinElapsed = false;
  }

  // Check if it's time to turn off the tremor type LED
  if (millis() - tremorTypeLEDStartTime >= LED_ON_DELAY) {
    CircuitPlayground.setPixelColor(9, 0, 0, 0); // Turn off the LED
  }
}
