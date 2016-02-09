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

#define COLOR_WHEEL_MAX 1536

const int number_of_strands = 4; // number of LED strands to address
const int leds_per_strand = 50; //Number of LEDs per strand
const int number_of_leds = leds_per_strand * number_of_strands;  //Total number of LEDs


// values used by the "snow glitter" mode
/* ************************************************************************************** */
const int diodes = leds_per_strand/3; //Truncated, so if 50 LEDs per strand, 16 will be lit at a time

int diode_pos[diodes]; //One third of the LEDs will be lit at one time
int diode_progress[diodes]; //Progress of specific LED, 0-170
boolean diode_fade_in[diodes]; //Is LED fading in or out?

int colors[number_of_leds];
int twinkle_diodes[number_of_leds/10];  // Twinkle with ten percent of all leds.
int twinkle_diode_counter[number_of_leds/10];  // Twinkle with ten percent of all leds.
int fade_step = 0;
int shift_counter = 0;
/* ************************************************************************************** */

uint8_t data_pin  = 17;   // Analog 3 (Analog pins used to simplify wiring. By using Analog pins we only need to have wires on one side of the Arduino Nano
uint8_t clock_pin = 19;   // Analog 5
uint8_t button_pin = 3;   // Digital 3
uint8_t analog_pin = 14;  // Analog 0  - Potentiometer wiper (middle terminal) connected to analog pin 0
// outside leads to ground and +5V

enum modes
{
    white_color_run,
    rainbow_cycle,
    rgb_color_wipe,
    red_white_color_run,
    snow_glitter,
    color_select,
    white_dim,
    random_color_cycle,
    random_color_cycle_twinkle,
    random_color_cycle_shift,
    big_color_cycle,
    color_shifting_tree,
    mode_max
};

//Variables used to keep track of button presses
volatile int button_state = HIGH;         // current state of the button
volatile long last_debounce_time = 0;     // previous state of the button
volatile int debounce_delay = 10;
volatile int last_state = HIGH;
volatile int current_mode = (int) modes::random_color_cycle;
int stored_mode = current_mode;

//wait is used to store data from the potentiometer.
int wait = 0;


// Set the first variable to the number_of_leds of pixels.
Adafruit_WS2801 strip = Adafruit_WS2801(number_of_leds, data_pin, clock_pin);

void button_up();
void button_down();


// Work around for debounces
void button_up()
{
    long current_time = millis();
    if ((current_time - last_debounce_time) > debounce_delay)
    {
        last_debounce_time = current_time;
        if (button_state == LOW)
        {
            button_state = HIGH;
            attachInterrupt(digitalPinToInterrupt(button_pin), button_down, FALLING);
        }
    }
}

void button_down()
{
    long current_time = millis();
    if ((current_time - last_debounce_time) > debounce_delay)
    {
        last_debounce_time = current_time;
        if (button_state == HIGH)
        {
            button_state = LOW;
            current_mode++;
            if (current_mode == mode_max)
            {
                current_mode = (int) white_color_run;
            }
            attachInterrupt(digitalPinToInterrupt(button_pin), button_up, RISING);
        }
    }
}

void setup()
{
    pinMode(button_pin, INPUT_PULLUP); //init Pins
    pinMode(analog_pin, INPUT);
    pinMode(clock_pin, OUTPUT);
    pinMode(data_pin, OUTPUT);
    Serial.begin(57600);
    attachInterrupt(digitalPinToInterrupt(button_pin), button_down, FALLING);
    strip.begin(); //Lets start!
}


void loop()
{
    // We've just started or switched mode, store current mode and reset all leds
    store_current_mode();
    reset_all_leds();

    switch (current_mode)
    {
        case white_color_run:
            Serial.println("Mode: color_run");
            white_color_run_mode();
            break;

        case rainbow_cycle:
            Serial.println("Mode: rainbow_cycle");
            rainbow_cycle_mode();
            break;

        case rgb_color_wipe:
            Serial.println("Mode: rgb_color_wipe");
            rgb_color_wipe_mode();

        case red_white_color_run:
            Serial.println("Mode: red_white_color_run");
            red_white_color_run_mode();
            break;

        case  snow_glitter:
            Serial.println("Mode: snow_glitter");
            snow_glitter_mode();
            break;

        case color_select:
            Serial.println("Mode: color_select");
            color_select_mode();
            break;

        case white_dim:
            Serial.println("Mode: white_dim");
            white_dim_mode();
            break;

        case random_color_cycle:
        case random_color_cycle_twinkle:
        case random_color_cycle_shift:
            Serial.println("Mode: random_color_cycle");
            random_color_cycle_mode(current_mode == random_color_cycle_twinkle, current_mode == random_color_cycle_shift);
            break;

        case big_color_cycle:
            Serial.println("Mode: big_color_cycle");
            big_color_cycle_mode();
            break;

        case color_shifting_tree:
            Serial.println("Mode: color_shifting_tree");
            color_shifting_tree_mode();
            break;
    }
}


//run 10 white pixels thru the entire LED strip.
void white_color_run_mode()
{
    while (true)
    {
        color_run(create_24bit_color_value(255, 255, 255), 10);
        if (mode_change()) //Button is pressed, exit
        {
            break;
        }
    }
}

//run a nice rainbow effect
void rainbow_cycle_mode()
{
    while (true)
    {
        rainbow_cycle_effect();
        if (mode_change()) //Button is pressed, exit
        {
            break;
        }
    }
}

//Fill the strip with off pixels.
void rgb_color_wipe_mode()
{
    while (true)
    {
        if (mode_change()) //Button is pressed, exit
        {
            break;
        }
        color_wipe(create_24bit_color_value(255, 0, 0)); //Fill the strip with red pixels
        if (mode_change()) //Button is pressed, exit
        {
            break;
        }
        color_wipe(create_24bit_color_value(0, 0, 0)); //Fill the strip with off pixels.
        if (mode_change()) //Button is pressed, exit
        {
            break;
        }
        color_wipe(create_24bit_color_value(0, 255, 0)); //Fill the strip with green pixels
        if (mode_change()) //Button is pressed, exit
        {
            break;
        }
        color_wipe(create_24bit_color_value(0, 0, 0)); //Fill the strip with off pixels.
        if (mode_change()) //Button is pressed, exit
        {
            break;
        }
        color_wipe(create_24bit_color_value(0, 0, 255)); //Fill the strip with blue pixels.
        if (mode_change()) //Button is pressed, exit
        {
            break;
        }
        color_wipe(create_24bit_color_value(0, 0, 0));
    }
}

void red_white_color_run_mode()
{
    while (true)
    {
        color_run(create_24bit_color_value(255, 0, 0), 5); //run 5 red pixels throu the strip
        if (mode_change()) //Button is pressed, exit
        {
            break;
        }
        color_run(create_24bit_color_value(255, 255, 255), 5); //run 5 white pixels throu the strip
        if (mode_change()) //Button is pressed, exit
        {
            break;
        }
    }
}

void snow_glitter_mode()
{
    color_run(create_24bit_color_value(255, 255, 255), 10);
    //Setup of snow glitter
    boolean fade_in = true; //Used below to set fade in or fade out
    int random_led; // Used for generating random numbers

    //Setup phase, choose random LEDs to start with, randomize fade progress and set fade in/out
    for (int i = 0; i < diodes; i++)
    {
        fade_in = !fade_in;
        find_new_led(i);
        diode_progress[i] = random(85); //set random progress in the fade sequence
        diode_fade_in[i] = fade_in; //set fade in or fade out for this diode
    }

    while (true)
    {
        snow_glitter_effect();
        if (mode_change()) //Button is pressed, exit
        {
            break;
        }
    }
}

void color_select_mode()
{
    while (!mode_change())
    {
        int color_value = analogRead(analog_pin)*1.5;
        Serial.println(color_value);
        uint32_t color = big_wheel(color_value, 255);
        for (int i = 0; i < strip.numPixels(); i++)
        {
            strip.setPixelColor(i, color);
        }
        strip.show();
    }
}

void white_dim_mode()
{
    while (!mode_change())
    {
        int color_value = analogRead(analog_pin)/8;
        uint32_t color = create_24bit_color_value(color_value, color_value, color_value);
        for (int i = 0; i < strip.numPixels(); i++)
        {
            strip.setPixelColor(i, color);
        }
        strip.show();
    }
}

void rainbow_cycle_effect() //rainbow effect (By Adafruit)
{
    for (int j = 0; j < 256 * 20; j++) // 5 cycles of all 25 colors in the wheel
    {
        for (int i = 0; i < strip.numPixels(); i++)
        {
            // tricky math! we use each pixel as a fraction of the full 96-color wheel
            // (thats the i / strip.numPixels() part)
            // Then add in j which makes the colors go around per pixel
            // the % 96 is to make the wheel cycle around
            strip.setPixelColor(i, small_wheel( ((i * 256 / strip.numPixels()) + j) % 256) );
        }
        if (mode_change()) //is button pressed?
        {
            break;
        }
        strip.show();   // write all the pixels out
        wait = analogRead(analog_pin)/10/number_of_strands;
        delay(wait);
    }
}

// Randomize the value for each diode and cycle through each one through the big colour wheel
void random_color_cycle_mode(bool enable_twinkle_effect, bool shift_pixels)
{
    byte max = analogRead(analog_pin) / 4;
    for (int i = 0; i < strip.numPixels(); i++)
    {
        colors[i] = random(0, COLOR_WHEEL_MAX-1);
        strip.setPixelColor(i, big_wheel(colors[i], max));
    }
    if (enable_twinkle_effect)
    {
        twinkle_setup(max);
    }

    strip.show();   // write all the pixels out

    while (!mode_change())
    {
        max = analogRead(analog_pin) / 4;
        if (shift_pixels)
        {
            if (shift_counter > 10)
            {
                int first_pixel = colors[0];
                for (int i = 0; i < (strip.numPixels()-1); i++)
                {
                    colors[i] = colors[i+1];
                }
                colors[strip.numPixels()-1] = first_pixel;
                shift_counter = 0;
            }
            else
            {
                shift_counter++;
            }
        }
        for (int i = 0; i < strip.numPixels(); i++)
        {

            colors[i]++;
            if (colors[i] > (COLOR_WHEEL_MAX-1))
            {
                colors[i] = 0;
            }
            if ((enable_twinkle_effect && !contains(twinkle_diodes, i)) || !enable_twinkle_effect)
            {
                strip.setPixelColor(i, big_wheel(colors[i], max));
            }
        }
        if (enable_twinkle_effect) // randomly switch off 10% of all diodes
        {
            twinkle(max);
        }

        strip.show();   // write all the pixels out
        if (shift_pixels)
        {
            shift_counter++;
        }
        else
        {
            shift_counter = 0;
        }
    }
}

void big_color_cycle_mode()
{
    int color_step = COLOR_WHEEL_MAX / strip.numPixels();

    for (int i = 0; i < strip.numPixels(); i++)
    {
        colors[i] = i * color_step;
        strip.setPixelColor(i, big_wheel(colors[i], 255));
    }
    strip.show();   // write all the pixels out

    while (!mode_change())
    {
        for (int i = 0; i < strip.numPixels(); i++)
        {
            colors[i]++;
            if (colors[i] > (COLOR_WHEEL_MAX-1))
            {
                colors[i] = 0;
            }
            strip.setPixelColor(i, big_wheel(colors[i], 255));
        }
        strip.show();   // write all the pixels out
        wait = analogRead(analog_pin)/10/number_of_strands;
        delay(wait);
    }
}

void color_shifting_tree_mode()
{
    int color = 0;
    while(!mode_change())
    {
        for (int i = 0; i < strip.numPixels(); i++)
        {
            strip.setPixelColor(i, big_wheel(color, 255));
        }
        strip.show();
        color++;
        if (color > (COLOR_WHEEL_MAX-1))
        {
            color = 0;
        }
    }
}

// fill the dots one after the other with said color
// good for testing purposes
void color_run (uint32_t c, uint32_t l) //By Adafruit
{
    for (int i = 0; i < strip.numPixels() + l + 1; i++)
    {
        strip.setPixelColor(i, c);
        if (i > l)
        {
            strip.setPixelColor(i - l - 1, 0);
        }
        strip.show();
        if (mode_change())
        {
            break;
        }
        wait = analogRead(analog_pin)/10/number_of_strands;
        delay(wait);
    }
}

void color_wipe(uint32_t c) //By Adafruit
{
    for (int i = 0; i < strip.numPixels(); i++)
    {
        strip.setPixelColor(i, c);
        strip.show();
        if (mode_change())
        {
            break;
        }
        wait = analogRead(analog_pin)/10/number_of_strands;
        delay(wait);
    }
}

void snow_glitter_effect() //By www.m.nu
{
    for (int i = 0; i < diodes; i++)
    {
        //reverse fade direction if diode is at the end, if faded out randomize new diode
        if (diode_progress[i] < 1)
        {
            find_new_led(i);
            diode_fade_in[i] = true;
        }
        else if (diode_progress[i] > 40)
        {
            diode_fade_in[i] = false;
        }

        //set LED light (based on diode_progress)
        //"show" below to speed up processing
        for (int j = 0; j < number_of_strands; j++)
        {
            strip.setPixelColor(diode_pos[i]+(j*leds_per_strand), white_fade(diode_progress[i]));
        }

        if (diode_fade_in[i])
        {
            diode_progress[i] += 1;
        }
        else
        {
            diode_progress[i] -= 1;
        }

        if (mode_change())
        {
            break;
        }
        wait = analogRead(analog_pin)/30/number_of_strands;
        // 1 is optimal pause for this mode
        delay(wait);
    }
    strip.show();
}
/* Helper functions */

// Create a 24 bit color value from R,G,B
uint32_t create_24bit_color_value(byte r, byte g, byte b)
{
    uint32_t c;
    c = r;
    c <<= 8;
    c |= g;
    c <<= 8;
    c |= b;
    return c;
}

//A three position colour wheel, will for example miss yellow, magenta and cyan
//Input a value 0 to 255 to get a color value.
//The colours are a transition r - g -b - back to r
uint32_t small_wheel(byte wheel_pos)
{
  if (wheel_pos < 85) {
    return create_24bit_color_value(wheel_pos * 3, 255 - wheel_pos * 3, 0);
  }
  else if (wheel_pos < 170) {
    wheel_pos -= 85;
    return create_24bit_color_value(255 - wheel_pos * 3, 0, wheel_pos * 3);
  }
  else {
    wheel_pos -= 170;
    return create_24bit_color_value(0, wheel_pos * 3, 255 - wheel_pos * 3);
  }
}

//A twelve position colour wheel
//Input a value 0 to COLOR_WHEEL_MAX to get a color value, and a value to set a dimming effect
//It cycles through red, orange, yellow, spring green, green, turquiose, cyan, ocean, blue, violet, magenta, raspberry, red
uint32_t big_wheel(int wheel_pos, byte max)   // by www.m.nu
{
    int color_step = COLOR_WHEEL_MAX / 6;
    wheel_pos = wheel_pos % COLOR_WHEEL_MAX;
    byte value = 0;

    if (wheel_pos < color_step)
    {
        value = wheel_pos;
        return create_24bit_color_value(max, value * max / 255, 0); // red to orange to yellow
    }
    else if (wheel_pos < (color_step * 2))
    {
        wheel_pos -= color_step;
        value = wheel_pos;
        return create_24bit_color_value(max - value * max / 255, max, 0); // yellow to spring green to green
    }
    else if (wheel_pos < (color_step * 3))
    {
        wheel_pos -= (color_step * 2);
        value = wheel_pos;
        return create_24bit_color_value(0, max, value * max / 255); // green to turquiose to cyan
    }
    else if (wheel_pos < (color_step * 4))
    {
        wheel_pos -= (color_step * 3);
        value = wheel_pos;
        return create_24bit_color_value(0, max - value * max / 255, max); // cyan to ocean to blue
    }
    else if (wheel_pos < (color_step * 5))
    {
        wheel_pos -= (color_step * 4);
        value = wheel_pos;
        return create_24bit_color_value(value * max / 255, 0, max); // blue to violet to magenta
    }
    else if (wheel_pos < COLOR_WHEEL_MAX)
    {
        wheel_pos -= (color_step * 5);
        value = wheel_pos;
        return create_24bit_color_value(max, 0, max - value * max / 255); // magenta to raspberry to red
    }
}


//return next color value in sequence (white fade-in/-out)
uint32_t white_fade(byte pos)
{
    int value = pos;
    value = value * 6;
    return create_24bit_color_value(value, value, value);
}

//check whether array contains an integer value
boolean contains(int array[], int val)
{
    for (int i = 0; i < diodes; i++)
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
void find_new_led(int diode_position)
{
    int random_led = 0;
    //If the random value is already present, choose a new one until a unique one is found
    //definition of "contains" is below
    do
    {
        random_led = random(0, leds_per_strand);
    }
    while (contains(diode_pos, random_led));

    diode_pos[diode_position] = random_led;
}

void twinkle_setup(byte max)
{
    fade_step = max / 20;

    for (int i = 0; i < strip.numPixels()/10; i++)
    {
        twinkle_diodes[i] = random(0, strip.numPixels());
        twinkle_diode_counter[i] = random(0, 41);
        if (twinkle_diode_counter[i] < 21) // fade out
        {
            strip.setPixelColor(twinkle_diodes[i], big_wheel(colors[twinkle_diodes[i]], (20 - twinkle_diode_counter[i]) * fade_step));
        }
        else // fade in
        {
            strip.setPixelColor(twinkle_diodes[i], big_wheel(colors[twinkle_diodes[i]], fade_step * (twinkle_diode_counter[i] - 20)));
        }
    }
}

// Fade in or out some pixels in the colors array
void twinkle(byte max)
{
    fade_step = max / 20;

    for (int i = 0; i < strip.numPixels()/10; i++)
    {
        if (twinkle_diode_counter[i] < 21) // fade out
        {
            strip.setPixelColor(twinkle_diodes[i], big_wheel(colors[twinkle_diodes[i]], (20 - twinkle_diode_counter[i]) * fade_step));
        }
        else // fade in
        {
            strip.setPixelColor(twinkle_diodes[i], big_wheel(colors[twinkle_diodes[i]], fade_step * (twinkle_diode_counter[i] - 20)));
        }
        twinkle_diode_counter[i]++;
        if (twinkle_diode_counter[i] > 41)
        {
            twinkle_diode_counter[i] = 0;
            int random_led = 0;
            do
            {
                random_led = random(0, strip.numPixels());
            } while (contains(twinkle_diodes, random_led));
            twinkle_diodes[i] = random_led;
        }
    }
}

bool mode_change()
{
    return (stored_mode != current_mode);
}

void store_current_mode()
{
    stored_mode = current_mode;
}

void reset_all_leds()
{
    for (int i = 0; i < strip.numPixels(); i++)
    {
        strip.setPixelColor(i, 0);
    }
}
