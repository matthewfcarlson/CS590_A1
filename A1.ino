#include "fast_hsv.h"
#include <CapacitiveSensor.h>
#include <Adafruit_LIS3DH.h>
#include "arduinoFFT.h"
#include <math.h>
#include "pitches.h"
//#define DEBUG

enum Mode {
  LIGHT_SENSITIVE,
  SOUND_SENSITIVE,
  PITCH_SENSITIVE,
  CAP_SENSITIVE, // Measure the capitance
  MOTION_SENSITIVE
};
const uint8_t CAP_SEND_PIN = 11;
const uint8_t CAP_SENSE_PIN = 12;
const uint8_t MODE_SWITCH_PIN = 1; // must be 0, 1, 2, 3, or 7
const uint8_t GYRO_CLK = 13;
const uint8_t GYRO_MISO = 12;
const uint8_t GRYO_MOSI = 11;
const uint8_t GYRO_CS = 10; // used for hardware SPI
const uint8_t LED_RED_PIN = 9;
const uint8_t LED_GREEN_PIN = 6;
const uint8_t LED_BLUE_PIN = 5;
const uint8_t MIC_PIN = A0;
const uint8_t LIGHT_SENSOR_PIN = A3;
const uint16_t DESIRED_TICK_LENGTH_US = 2000; //2 milliseconds
const uint8_t CLICKTHRESHHOLD = 16;
const uint16_t CAP_THRESHHOLD = 300;
const uint16_t CAP_MAX = 4000;
const uint8_t CAP_STEP_RATE = 10;

arduinoFFT FFT = arduinoFFT();
double vReal[SAMPLES];
double vImag[SAMPLES];

unsigned long tick_ms;
unsigned int sampling_period_us;
unsigned long microseconds;
uint8_t ticks_until_mode_switch_allowed;
uint8_t rgb_change_speed = 2;
boolean instant_rgb_mode = false;
double current_rotation = 0;
struct _RGB {
  uint8_t red;
  uint8_t blue;
  uint8_t green;
};
struct _RGBE {
  uint16_t red;
  uint16_t blue;
  uint16_t green;
};
uint16_t cap_hue = 0;
typedef struct _RGB RGB;
typedef struct _RGBE RGBE;
RGBE rgb_target;
RGBE rgb_current;

Mode curr_mode;
Adafruit_LIS3DH gyro;
boolean gyro_started = false;
boolean gyro_powered_down = true;
CapacitiveSensor cap_sensor = CapacitiveSensor(CAP_SEND_PIN, CAP_SENSE_PIN);
boolean tapped = false;

void setup() {
  // Start serial
  Serial.begin(115200);
  int wait_count = 0;
  // wait a half second for serial
  while (!Serial && wait_count < 50) {
    delay(10);
    wait_count++;
  }
  // Setup the mode in the state machine
  curr_mode = LIGHT_SENSITIVE;
  ticks_until_mode_switch_allowed = 0;
  pinMode(MODE_SWITCH_PIN, INPUT_PULLUP);

  // Setup the FFT
  sampling_period_us = round(1000000 * (1.0 / SAMPLING_FREQUENCY));
  pinMode(MIC_PIN, INPUT);

  // Setup the gyro
  gyro = Adafruit_LIS3DH();
  if (gyro.begin()) {   // change this to 0x19 for alternative i2c address
    Serial.println("Started Gyro");
    gyro_started = true;
    gyro.setRange(LIS3DH_RANGE_2_G);   // 2, 4, 8 or 16 G!
    gyro.setClick(1, CLICKTHRESHHOLD);
    gyro.setDataRate(LIS3DH_DATARATE_POWERDOWN); // by default, we power down
  }
  else {
    Serial.println("Failed to start Gyro");
  }

  // Setup the RGB leds
  pinMode(LED_RED_PIN, OUTPUT);
  pinMode(LED_GREEN_PIN, OUTPUT);
  pinMode(LED_BLUE_PIN, OUTPUT);

  // Setup light sensor
  pinMode(LIGHT_SENSOR_PIN, INPUT);
  Serial.println("Starting");

  // Setup cap sensor
  pinMode(CAP_SEND_PIN, OUTPUT);
  pinMode(CAP_SENSE_PIN, INPUT);
  cap_sensor = CapacitiveSensor(CAP_SEND_PIN, CAP_SENSE_PIN);       // 10 megohm resistor between pins 4 & 2, pin 2 is sensor pin, add wire, foil
  cap_sensor.set_CS_AutocaL_Millis(0xFFFFFFFF);     // turn off autocalibrate on channel 1 - just as an example

}

void tick_light_mode() {
  int level = analogRead(LIGHT_SENSOR_PIN);
  level = level > 255 ? 255 : level; // cap at 255
  RGB_light(level);
}

void sample_FFT() {
  for (int i = 0; i < SAMPLES; i++)
  {
    microseconds = micros();    //Overflows after around 70 minutes!

    vReal[i] = analogRead(MIC_PIN);
    vImag[i] = 0;

    while (micros() < (microseconds + sampling_period_us)) {
      // Just wait
    }
  }

  /*FFT*/
  FFT.Windowing(vReal, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.Compute(vReal, vImag, SAMPLES, FFT_FORWARD);
  FFT.ComplexToMagnitude(vReal, vImag, SAMPLES);
}

void tick_pitch_mode() {
  sample_FFT();
  long magnitude = CalcMagnitude(vReal, SAMPLES, 80, MAX_FREQUENCY);
  if (magnitude < 700) {
    RGB_light(0);
    return;
  }
  int peak = (int)round(FFT.MajorPeak(vReal, SAMPLES, SAMPLING_FREQUENCY));

  int distance = 0;
  int closestFreq = 0;
  int closestDist = 9999;
  for (int i = 0; i < NUM_PITCHES; i ++) {
    // Go through and print each pitch
    distance = abs(pitches[i] - peak);
    if (distance < closestDist) {
      closestDist = distance;
      closestFreq = pitches[i];
    }
  }
  // Figure out how far we are away from a pitch
#ifdef DEBUG
  Serial.print("Peak freq: ");
  Serial.print(peak);
  Serial.print("\tDist: ");
  Serial.print(closestDist);
  Serial.println();
#endif
  if (distance < 5) { // if we are less than 5 hz off
    RGB_color(0,0, 255); // We're all good
    return;
  }
  // TODO figure out if we need to go higher or lower
  int distance_level = 255 * distance / 50; // if we are more than 50 hertz
  RGB_color(distance_level, 0, 0);
  
  
}

void tick_sound_mode() {
  /*SAMPLING*/
  sample_FFT();
  long magnitude = CalcMagnitude(vReal, SAMPLES, 80, MAX_FREQUENCY);
  if (magnitude < 1000) {
    RGB_light(0);
    return; // just do nothing
  }
  long lows = CalcMagnitude(vReal, SAMPLES, 60, 400);
  long mids = CalcMagnitude(vReal, SAMPLES, 401, 2200);
  long highs = CalcMagnitude(vReal, SAMPLES, 2201, MAX_FREQUENCY);
  lows = lows < 1000 ? 0 : lows;
  mids = mids < 700 ? 0 : mids;
  highs = highs < 500 ? 0 : highs;
#ifdef DEBUG
  Serial.print("LOW:");
  Serial.println(lows);
  Serial.print("MID:");
  Serial.println(mids);
  Serial.print("HIGH:");
  Serial.println(highs);
#endif
  // TODO: do an EQ of the top frequencies?
  int red =  255 * lows / 3000;
  int green =  255 * mids / 2000;
  int blue =  255 * highs / 1000;

  RGB_color(red, green, blue);
  // Set PWM of the light to that
}

long CalcMagnitude(double *vData, uint16_t samples, int freq_low, int freq_high) {
  long magnitude = 0;
  int bufferSize = samples >> 1;
  if (freq_low >= freq_high) return 0;
  for (uint16_t i = 0; i < bufferSize; i++)
  {
    int abscissa = (int)((i * 1.0 * SAMPLING_FREQUENCY) / samples);
    if (freq_low > abscissa) continue;
    if (freq_high < abscissa) break;
    magnitude += (long)vData[i];
  }
  return magnitude;
}

void tick_motion_mode() {
  if (!gyro_started) {
    Serial.println("We don't have a gyro");
    if (!gyro.begin()) {
      switchToNextMode();
    }
    else {
      Serial.println("We started it!");
      gyro_started = true;
    }
    return;
  }
  if (gyro_powered_down) {
    // Start up the gyro if hasn't been used
    RGB_light(0);
    Serial.println("Powering up Gyro");
    gyro.setDataRate(LIS3DH_DATARATE_50_HZ);
    gyro_powered_down = false;
  }
  /* Get a new sensor event */
  uint8_t click_val = gyro.getClick() & 0x7; // just the bottom three bits
#ifdef DEBUG
  Serial.print("CLICK: ");
  Serial.println(click_val);
#endif
  boolean did_tap = click_val;
  // Get current light level
  uint8_t level = rgb_target.red > 150 ? 0 : 255;
  if (did_tap && !tapped) RGB_light(level); // if we tapped it
  tapped = click_val;
}

void tick_cap_mode() {
  static int doSense = 5;
  if (doSense != 0) {
    doSense -= 1;
    return;
  }
  doSense = 5;
  // Do the capitive sensing
  // Get that bread
  long total =  cap_sensor.capacitiveSensor(80);

  if (total < CAP_THRESHHOLD) return;
  long value = CAP_STEP_RATE * (total - CAP_THRESHHOLD) / CAP_MAX + 1;
  cap_hue += value; // add to the hue
  if (cap_hue > 720) cap_hue = 0;
#ifdef DEBUG
  Serial.print("Value: 0x");
  Serial.println(value, HEX);
  Serial.print("COLOR HUE: ");
  Serial.println(cap_hue);
#endif
  RGB new_color = {0, 0, 0};
  fast_hsv2rgb_8bit (cap_hue, HSV_SAT_MAX, HSV_VAL_MAX, &new_color.red, &new_color.green, &new_color.blue);
  RGB_color(new_color);
}


Mode nextMode() {
  switch (curr_mode) {
    case LIGHT_SENSITIVE:
      return SOUND_SENSITIVE;
    case SOUND_SENSITIVE:
      return PITCH_SENSITIVE;
    case PITCH_SENSITIVE:
      return CAP_SENSITIVE;
    case CAP_SENSITIVE:
      return MOTION_SENSITIVE;
    default:
      return LIGHT_SENSITIVE;
  }
}

void switchToNextMode() {
  curr_mode = nextMode();
  Serial.print("New mode: ");
  Serial.println(curr_mode);
}

void loop() {
  tick_ms = micros();
  if (!digitalRead(MODE_SWITCH_PIN) && ticks_until_mode_switch_allowed == 0) {
    ticks_until_mode_switch_allowed = 5;
    switchToNextMode();
  }
  if (curr_mode != MOTION_SENSITIVE && !gyro_powered_down) {
    gyro.setDataRate(LIS3DH_DATARATE_POWERDOWN); // by default, we power down
    Serial.println("Powering down Gryo");
    gyro_powered_down = true;
  }
  // figure out what mode we are in
#ifdef DEBUG
  Serial.print("Mode: ");
  Serial.print(curr_mode);
  Serial.print(" - ");
  Serial.print(ticks_until_mode_switch_allowed);
  Serial.println();
#endif
  switch (curr_mode) {
    case LIGHT_SENSITIVE:
      tick_light_mode();
      break;
    case SOUND_SENSITIVE:
      tick_sound_mode();
      break;
    case PITCH_SENSITIVE:
      tick_pitch_mode();
      break;
    case MOTION_SENSITIVE:
      tick_motion_mode();
      break;
    case CAP_SENSITIVE:
      tick_cap_mode();
      break;
    default:
      curr_mode = LIGHT_SENSITIVE;
      break;
  }
  update_rgb_led();
  while (tick_ms + DESIRED_TICK_LENGTH_US > micros()) delayMicroseconds(DESIRED_TICK_LENGTH_US / 2);
  // If the button is released, decrement the counter
  if (digitalRead(MODE_SWITCH_PIN) && ticks_until_mode_switch_allowed > 0) ticks_until_mode_switch_allowed--;
}

void update_rgb_led() {
  if (curr_mode == SOUND_SENSITIVE) {
    rgb_current = rgb_target;
  }
  else {
    if (rgb_current.red > rgb_target.red) rgb_current.red -= rgb_change_speed;
    else if (rgb_current.red < rgb_target.red) rgb_current.red += rgb_change_speed;

    if (rgb_current.green > rgb_target.green) rgb_current.green -= rgb_change_speed;
    else if (rgb_current.green < rgb_target.green) rgb_current.green += rgb_change_speed;

    if (rgb_current.blue > rgb_target.blue) rgb_current.blue -= rgb_change_speed;
    else if (rgb_current.blue < rgb_target.blue) rgb_current.blue += rgb_change_speed;
  }

  analogWrite(LED_RED_PIN, rgb_current.red / 4);
  analogWrite(LED_GREEN_PIN, rgb_current.green / 4);
  analogWrite(LED_BLUE_PIN, rgb_current.blue / 4);
#ifdef DEBUG
  Serial.print("Current: ");
  print_rgb(rgb_current);
  Serial.print("Target: ");
  print_rgb(rgb_target);
#endif
}

void print_rgb(RGB rgb) {

  Serial.print("RGB: ");
  Serial.print(rgb.red);
  Serial.print(", ");
  Serial.print(rgb.green);
  Serial.print(", ");
  Serial.print(rgb.blue);
  Serial.println();
}
void print_rgb(RGBE rgb) {

  Serial.print("RGBE: ");
  Serial.print(rgb.red / 4);
  Serial.print(", ");
  Serial.print(rgb.green / 4);
  Serial.print(", ");
  Serial.print(rgb.blue / 4);
  Serial.println();
}

void RGB_color(int red_light_value, int green_light_value, int blue_light_value) {
  rgb_target.red = (red_light_value > 255 ? 255 : red_light_value) * 4;
  rgb_target.green = (green_light_value > 255 ? 255 : green_light_value) * 4;
  rgb_target.blue = (blue_light_value > 255 ? 255 : blue_light_value) * 4;
  Serial.print("Set Target: ");
  print_rgb(rgb_target);
}
void RGB_color(RGB target) {
  Serial.print("Pre Target: ");
  print_rgb(target);
  RGB_color(target.red, target.green, target.blue);
}

void RGB_light(int level) {
  RGB_color(level, level, level);
}
