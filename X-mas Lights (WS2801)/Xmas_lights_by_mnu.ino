#include "SPI.h"
#include "Adafruit_WS2801.h"

/*****************************************************************************
 *
 * X-Mas lights by m.nu.
 *
 * Based on https://learn.adafruit.com/12mm-led-pixels/overview by Adafruit.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Example written by Limor Fried/Ladyada for Adafruit Industries.
 * BSD license, all text above must be included in any redistribution
 *
 *****************************************************************************/

const int numStrands = 2; // number of LED strands to address
const int strandLEDs = 50; //Number of LEDs per strand
const int numLED = strandLEDs*numStrands;  //Total number of LEDs


// values used by the "snow glitter" mode
/* ************************************************************************************** */
const int diodes = strandLEDs/3; //Truncated, so if 50 LEDs per strand, 16 will be lit at a time

int diodePos[diodes]; //One third of the LEDs will be lit at one time
int diodeProgress[diodes]; //Progress of specific LED, 0-170
boolean diodeFadeIn[diodes]; //Is LED fading in or out?

/* ************************************************************************************** */

uint8_t dataPin  = 17;    //Analog 3 (Analog pins used to simplify wiring. By using Analog pins we only need to have wires on one side of the Arduino Nano
uint8_t clockPin = 19;    // Analog 5
uint8_t  buttonPin = 18;  // Analog 4
uint8_t analogPin = 14;     // Analog 0  - Potentiometer wiper (middle terminal) connected to analog pin 0
// outside leads to ground and +5V

//Wariables used to keep track of button presses
int buttonState = 0;         // current state of the button
int lastButtonState = 0;     // previous state of the button
int buttonPushCounter = 0;

//wait is used to store data from the potentiometer.
int wait = 0;

// Set the first variable to the numLED of pixels.
Adafruit_WS2801 strip = Adafruit_WS2801(numLED, dataPin, clockPin);

void setup() {
  pinMode(buttonPin, INPUT); //init Pins
  pinMode(analogPin, INPUT);
  pinMode(clockPin, OUTPUT);
  pinMode(dataPin, OUTPUT);
  Serial.begin(57600);
  strip.begin(); //Lets start!
  // Update LED contents, to start they are all 'off'
  strip.show();
}

void button() { //Simple funktion to detect button press
  buttonState = digitalRead(buttonPin);
  if (buttonState != lastButtonState) {   // compare the buttonState to its previous state
    if (buttonState == HIGH) {   // if the state has changed, increment the counter
      buttonPushCounter++;
    }
  }
  // save the current state as the last state for next time through the loop
  lastButtonState = buttonState;
}

void loop() {
  // We will get stuck in each (and the correct) if statement until the button is pressed.
  if (buttonPushCounter == 0){
    while (true) {
      colorRun(Color(255, 255, 255),10,0); //run 10 white pixels thru the entire LED strip.
      if (buttonPushCounter >0) //Button is pressed, exit
        break;
    }
  }
  if (buttonPushCounter == 1){
    while (true) {
      rainbowCycle(); //run a nice rainbow effect
      if (buttonPushCounter >1) //Button is pressed, exit
        break;
    }
  }
  if (buttonPushCounter == 2){
    while (true) {
      colorWipe(Color(0, 0, 0)); //Fill the strip with off pixels.
      if (buttonPushCounter >2) //Button is pressed, exit
        break;
      colorWipe(Color(255, 0, 0)); //Fill the strip with red pixels
      if (buttonPushCounter >2) //Button is pressed, exit
        break;
      colorWipe(Color(0, 0, 0)); //Fill the strip with off pixels.
      if (buttonPushCounter >2) //Button is pressed, exit
        break;
      colorWipe(Color(0, 255, 0)); //Fill the strip with green pixels
      if (buttonPushCounter >2) //Button is pressed, exit
        break;
      colorWipe(Color(0, 0, 0)); //Fill the strip with off pixels.
      if (buttonPushCounter >2) //Button is pressed, exit
        break;
      colorWipe(Color(0, 0, 255)); //Fill the strip with blue pixels.
      if (buttonPushCounter >2) //Button is pressed, exit
        break;
    }
  }
  if (buttonPushCounter == 3){
    while (true) {
      colorRun(Color(255, 0,0),5,3); //run 5 red pixels throu the strip
      if (buttonPushCounter >3) //Button is pressed, exit
        break;
      colorRun(Color(255, 255,255),5,3); //run 5 white pixels throu the strip
      if (buttonPushCounter >3) //Button is pressed, exit
        break;
    }
  }

  if (buttonPushCounter == 4){
    colorRun(Color(255, 255,255),10,4);
    //Setup of snow glitter
    boolean setTrue = true; //Used below to set fade in or fade out
    int newRandVal; // Used for generating random numbers

    //Setup phase, choose random LEDs to start with, randomize fade progress and set fade in/out
    int i;
    for (i=0; i < diodes; i++)
    {

      setTrue = !setTrue;
      newRandVal = random(0, strandLEDs);//RAndomize from entire strand, not just one third

      //If the random value is already present, choose a new one until a unique one is found
      //definition of "contains" is below
      while (contains(diodePos, newRandVal))
      {
        newRandVal = random(0, strandLEDs);//RAndomize from entire strand, not just one third
      }
      diodePos[i] = newRandVal; //save the random diode
      diodeProgress[i] = random(85); //set random progress in the fade sequence
      diodeFadeIn[i] = setTrue; //set fade in or fade out for this diode
    }

    while (true) {
      snowGlitter();
      if (buttonPushCounter >4) //Button is pressed, exit
        break;

    }
  }
  if (buttonPushCounter == 5){
    while (true) {
      int i;
      int colorValue = analogRead(analogPin)/4;
      uint32_t color = Wheel(colorValue);
      for (i=0; i < strip.numPixels(); i++) {
        strip.setPixelColor(i, color);
      }
      strip.show();
      button();

      if (buttonPushCounter >5) //Button is pressed, exit
        break;
    }
  }

  if (buttonPushCounter == 6){
    while (true) {
      int i;
      int colorValue = analogRead(analogPin)/8;
      uint32_t color = Color(colorValue, colorValue, colorValue);
      for (i=0; i < strip.numPixels(); i++) {
        strip.setPixelColor(i, color);
      }
      strip.show();
      button();

      if (buttonPushCounter >6) //Button is pressed, exit
        break;
    }
  }

  if (buttonPushCounter >6) //Button is pressed more than 4 times, reset
    buttonPushCounter = 0;
}


void rainbowCycle() { //rainbow effect (By Adafruit)
  int i, j;
  for (j=0; j < 256 * 20; j++) {     // 5 cycles of all 25 colors in the wheel
    for (i=0; i < strip.numPixels(); i++) {
      // tricky math! we use each pixel as a fraction of the full 96-color wheel
      // (thats the i / strip.numPixels() part)
      // Then add in j which makes the colors go around per pixel
      // the % 96 is to make the wheel cycle around
      strip.setPixelColor(i, Wheel( ((i * 256 / strip.numPixels()) + j) % 256) );
    }
    button(); //Check button
    if (buttonPushCounter != 1) //is button pressed?
      break;
    strip.show();   // write all the pixels out
    wait = analogRead(analogPin)/10/numStrands;
    delay(wait);
  }

}

// fill the dots one after the other with said color
// good for testing purposes
void colorRun (uint32_t c, uint32_t l, int exitButton) { //By Adafruit
  int i;
  for (i=0; i < strip.numPixels()+l+1; i++) {
    strip.setPixelColor(i, c);
    if (i>l)
      strip.setPixelColor(i-l-1, 0);
    strip.show();
    button(); //Check button
    if (buttonPushCounter !=exitButton) //Is button pressed (Variable because we use this function in two if statements above and need to know when to exit
      break;
    wait = analogRead(analogPin)/10/numStrands;
    delay(wait);
  }
}
void colorWipe(uint32_t c) { //By Adafruit
  int i;
  for (i=0; i < strip.numPixels(); i++) {
    button();
    strip.setPixelColor(i, c);
    strip.show();
    if (buttonPushCounter !=2)
      break;
    wait = analogRead(analogPin)/10/numStrands;
    delay(wait);
  }
}

void snowGlitter() { //By www.m.nu
  int i;
  for (i=0; i < diodes; i++)
  {
    button();
    int currentDiode = diodePos[i];

    //reverse fade direction if diode is at the end, if faded out randomize new diode
    if (diodeProgress[i] < 1) {
      findNewLED(i);
      diodeFadeIn[i] = true;
    }
    else if (diodeProgress[i] > 40) {
      diodeFadeIn[i] = false;
    }

    //set LED light (based on diodeProgress)
    //"show" below to speed up processing
    int j;
    for (j = 0; j < numStrands; j++) {
      strip.setPixelColor(currentDiode+(j*strandLEDs), WhiteFade(diodeProgress[i]));
    }

    if (diodeFadeIn[i]) {
      diodeProgress[i] += 1;
    }
    else diodeProgress[i] -= 1;

    if (buttonPushCounter !=4)
      break;
    wait = analogRead(analogPin)/30/numStrands;
    // 1 is optimal pause for this mode
    delay(wait);
  }
  strip.show();
}
/* Helper functions */

// Create a 24 bit color value from R,G,B
uint32_t Color(byte r, byte g, byte b)
{
  uint32_t c;
  c = r;
  c <<= 8;
  c |= g;
  c <<= 8;
  c |= b;
  return c;
}

//Input a value 0 to 255 to get a color value.
//The colours are a transition r - g -b - back to r
uint32_t Wheel(byte WheelPos)
{
  if (WheelPos < 85) {
    return Color(WheelPos * 3, 255 - WheelPos * 3, 0);
  }
  else if (WheelPos < 170) {
    WheelPos -= 85;
    return Color(255 - WheelPos * 3, 0, WheelPos * 3);
  }
  else {
    WheelPos -= 170;
    return Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
}

//return next Color value in sequence (white fade-in/-out)
uint32_t WhiteFade(byte pos)
{
  int value = pos;
  value = value * 6;
  return Color(value, value, value);
}

//check whether array contains an integer value
boolean contains(int array[], int val)
{
  int i;
  for (i = 0; i < diodes; i++)
  {
    if (array[i] == val)
    {
      return true;
    }
  }
  return false;
}

//Works with global variables at the top. When a LED has finished its sequence,
// it uses this function to find a new LED.
void findNewLED(int diodePosition)
{
  int newRandVal = random(0, strandLEDs);
  //If the random value is already present, choose a new one until a unique one is found
  //definition of "contains" is below
  while (contains(diodePos, newRandVal))
  {
    newRandVal = random(0, strandLEDs);
  }
  diodePos[diodePosition] = newRandVal;
}