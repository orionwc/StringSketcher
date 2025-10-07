#pragma region README
/* Dear HackPack User,

this is code for a wall drawing robot that hangs from two anchor points by adjusting the lengths of each string. Recommended image size is up to 250 x 250 pixels on the default "small" preset, 375 x 375 pixels on "medium", and 500 x 500 on large. Image print times vary from 30 mins - 4hrs based on size and pixel density. Sizes for presets are as follows:

small - 500mm width, 260mm initial string lengths, 300mm vertical image center
medium - 750 mm width, 500mm initial string lengths, 350mm vertical image center
large - 1000 mm width, 720mm initial string lengths, 425mm vertical image center

To add your own patterns, create .thr files on the SD card. Each file should contain one coordinate per line in the format "theta radius" where theta is in radians and radius is normalized 0-1. If you're using your own SD card, be sure to set the storage type to the FAT format so that the SdFat library can read the contents (https://www.lifewire.com/format-sd-card-to-fat32-6666166).

On Boot up, the Plotter Robot will wait for a user selection. The Red button is "next" and the green button is "select". Numbers 1 - 6 correspond to the patterns that come pre-loaded.  
  
  The rectangular icon allows selecting between image sizes.
  The pen icon selects between different pen modes:
    - White: Uses built-in PROGMEM patterns (files 1-6)
    - Red: Reads .thr files 1-6 from SD card
    - Green: Reads .thr files 7-12 from SD card  
    - Blue: Reads .thr files 13-18 from SD card
    - Yellow: Serial receiving mode (future feature)
  
  E1, error 1- SD Card couldn't be read (LED 0 pulses red)
  E2, error 2- .thr file not found (LED 1 flashes red, returns to selector) 

Visit https://orionwc.github.io/Image2Sand/ to convert images into patterns for drawing here!

Lastly, the config.h file contains key variables for custom sizing, image placement, pen mode behavior, motor behavior, etc. Editing these values will allow you to customize your robot's calibration.

Updated 10/06/2025
*/
#pragma endregion README
#pragma region LICENSE
/*
  ************************************************************************************
  * MIT License
  *
  * Copyright (c) 2025 OrionWC

  * Permission is hereby granted, free of charge, to any person obtaining a copy
  * of this software and associated documentation files (the "Software"), to deal
  * in the Software without restriction, including without limitation the rights
  * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  * copies of the Software, and to permit persons to whom the Software is furnished
  * to do so, subject to the following conditions:
  *
  * The above copyright notice and this permission notice shall be included in all
  * copies or substantial portions of the Software.
  *
  * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
  * INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
  * PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
  * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF
  * CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE
  * OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
  *
  ************************************************************************************
*/
#pragma endregion LICENSE

//////////////////////////////////////////////////
//  LIBRARIES AND CONFIGURATION //
//////////////////////////////////////////////////
#pragma region LIBRARIES_AND_CONFIGURATION
#include "config.h"
#include <AS5600.h>             // encoder
#include <Adafruit_Sensor.h>    // I2C Multiplexer
#include <Servo.h>
#include <SPI.h>                // SD
#include <SdFat.h>              // FAT storage type
#include <Adafruit_NeoPixel.h>
#include <ezButton.h>

#define TCAADDR 0x70
#define SERVOPIN 9
#define LEDPIN 10
#define SD_CS_PIN 14            // Chip select pin for the SD card module
#define PAUSEPIN 15
#define CURSORPIN 16
#define L_M_DIR 2
#define L_M_PWM 3
#define R_M_DIR 4
#define R_M_PWM 5
#define FILE_NUM 6

// Image Size presets ---------------------------------------------------------
#define SMALL_WIDTH 500
#define SMALL_INIT 260
#define SMALL_Y_OFFSET 300

#define MED_WIDTH 750
#define MED_INIT 500
#define MED_Y_OFFSET 350

#define LARGE_WIDTH 1250
#define LARGE_INIT 720
#define LARGE_Y_OFFSET 425

/* PATTERNS
1 - Diamond Test Pattern
2 - Squirrel
3 - Rose
4 - Planet
5 - Empty
6 - Elephant on a Skateboard
*/

// ************* SETTINGS *************

// Pattern completion behavior ------------------------------------------------
const bool MoveDownOnCompletion = true;  // [true/false] Move down after pattern completes

// Pen mode variable
uint8_t penMode = 0;  // [0 - 3] Pen mode: 0=White(PROGMEM), 1=Red(SD 1-6), 2=Green(SD 7-12), 3=Blue(SD 13-18), 4=Yellow(Serial) reserved

// ************* END SETTINGS *************

//Pattern drawing structures-----------------------------------
struct Positions {
  uint16_t radial;                  // radial distance (0-1000)
  uint16_t angular;                 // angular position (degrees * 10)
};

// Pen mode enumeration
enum PenMode {
  WHITE_MODE = 0,    // Default - read from PROGMEM patterns
  RED_MODE = 1,      // SD offset 0 (files 1-6)
  GREEN_MODE = 2,    // SD offset 6 (files 7-12) 
  BLUE_MODE = 3,     // SD offset 12 (files 13-18)
  YELLOW_MODE = 4    // Serial receiving mode (future)
};

// Pattern definitions (polar coordinates: radial, angular*10)--------
const Positions pattern1[] PROGMEM = { // TEST PATTERN
  {800, 0},    // Point 1: Right  (0°)
  {800, 900},  // Point 2: Up     (90°) 
  {800, 1800}, // Point 3: Left   (180°)
  {800, 2700}, // Point 4: Down   (270°)
  {800, 0}    // Point 5: Return to start (0°)

};

const Positions pattern2[] PROGMEM = {
  // Squirrel (97 points)
  {708,2836},{709,2885},{700,2894},{679,2899},{642,2882},{626,2858},{608,2793},{602,2792},{594,2841},{584,2878},{555,2932},{566,3009},{653,3026},{712,3043},{752,3039},{770,3043},{779,3049},{833,3052},{847,3058},{939,3132},{944,3142},{938,3151},{927,3156},{884,3158},{856,3149},{818,3126},{786,3126},{700,3163},{659,3202},{648,3228},{660,3287},{681,3400},{738,3426},{782,3450},{854,3500},{867,3513},{873,3534},{862,3557},{821,3593},{795,12},{736,50},{617,107},{598,154},{568,198},{536,234},{512,246},{492,238},{474,200},{432,115},{299,3596},{251,3586},{177,26},{87,53},{24,2284},{101,2137},{182,2174},{254,2221},{332,2274},{405,2324},{410,2320},{358,2214},{281,1953},{265,1682},{333,1306},{411,1208},{511,1164},{601,1160},{691,1179},{748,1201},{807,1233},{860,1268},{916,1314},{957,1359},{981,1392},{998,1438},{1000,1466},{986,1511},{964,1541},{920,1581},{884,1605},{836,1624},{795,1631},{723,1632},{683,1813},{677,1899},{674,1977},{673,2037},{672,2135},{670,2199},{666,2276},{663,2357},{659,2424},{651,2496},{635,2567},{669,2600},{681,2620},{689,2648},{708,2836}

  // Empty Pattern (use this if you want to try a complex Pattern6 below)
  // {0,0}
};

const Positions pattern3[] PROGMEM = {

  // Empty Pattern (use this if you want to try a complex Pattern6 below)
  {0,0}
};

const Positions pattern4[] PROGMEM = {
  // Planet (104 points)
  {468,2691},{469,2778},{469,2845},{469,2950},{470,3083},{470,3224},{470,3369},{470,3465},{468,3500},{284,3033},{357,2544},{457,2387},{468,2390},{468,2487},{468,2560},{468,2691},{468,2691},{251,2933},{359,3416},{560,15},{727,98},{796,126},{928,180},{980,210},{1000,238},{989,264},{960,285},{916,307},{844,337},{705,393},{506,504},{498,496},{503,378},{549,355},{613,318},{658,286},{682,259},{688,234},{678,215},{648,188},{619,170},{541,125},{450,68},{344,3576},{249,3426},{175,3059},{201,2619},{324,2330},{424,2233},{516,2172},{620,2110},{664,2077},{682,2046},{683,2032},{674,2013},{649,1991},{610,1965},{497,1899},{498,1845},{497,1775},{504,1776},{707,1889},{796,1926},{886,1963},{962,2000},{988,2021},{996,2044},{991,2056},{974,2073},{914,2105},{833,2139},{752,2171},{653,2215},{480,2322},{340,2499},{251,2933},{251,2933},{156,2687},{223,3462},{330,18},{440,102},{468,122},{470,135},{472,235},{472,364},{472,480},{472,598},{471,688},{472,827},{471,969},{471,1039},{470,1131},{470,1255},{470,1384},{469,1470},{468,1563},{467,1682},{467,1735},{467,1861},{467,1914},{467,2033},{467,2142},{462,2163},{364,2230},{271,2330},{156,2687},{156,2687},{1200,2700}

  // Empty Pattern (use this if you want to try a complex Pattern6 below)
  // {0,0}
};

// Spiral
const Positions pattern5[] PROGMEM = {
  // Paste your own pattern here and comment out the empty pattern below by placing // in front of the {0,0}


  // Empty Pattern (use this if you want to try a complex Pattern6 below)
  {0,0}
};

// Pentagon
const Positions pattern6[] PROGMEM = {
  // Large patterns require the other patterns 1-5 to be replaced with the empty pattern {0,0}.

  // Empty Pattern (select this if you need to reserve the memory for the other patterns 1-5 above)
  {0,0}

};

// Pattern sizes for easy access
const int patternSizes[6] = {
  sizeof(pattern1)/sizeof(pattern1[0]),
  sizeof(pattern2)/sizeof(pattern2[0]),
  sizeof(pattern3)/sizeof(pattern3[0]),
  sizeof(pattern4)/sizeof(pattern4[0]),
  sizeof(pattern5)/sizeof(pattern5[0]),
  sizeof(pattern6)/sizeof(pattern6[0])
};
#pragma endregion LIBRARIES_AND_CONFIGURATION

//////////////////////////////////////////////////
//  CLASS OBJECTS //
//////////////////////////////////////////////////
#pragma region CLASS_OBJECTS
//hardware definitions-----------------------------------

ezButton redButton(CURSORPIN, 20);
ezButton greenButton(PAUSEPIN, 20);

Adafruit_NeoPixel pixels (10, LEDPIN, NEO_GRB + NEO_KHZ800);

Servo myPen;               // pen Servo

AS5600 encL;               //  left
AS5600 encR;               //  right

SdFat SD;  
File thrFile;

// File reading system variables
bool fileMode = false;
bool sdCardInitialized = false;     
#pragma endregion CLASS_OBJECTS

//////////////////////////////////////////////////
//  STRUCTS //
//////////////////////////////////////////////////
#pragma region STRUCTS
//artboard & image params-------------------------------------
struct image{
  //Canvass height is effectively unlimited
  uint16_t canvassWidth;     // Distance suction cups are apart
  uint16_t centerHeight;     // Only limited by string length
  uint8_t setSize;

  float centerXOffset;       // Calculated offset to center pattern in canvass
  float centerYOffset;
};

//movement params-------------------------------------------
struct motor{
  float maxSpeed;           // 255 is 100% duty cycle
  float drawSpeed;

  float setSpeeds[2];       
};

//hardware specs--------------------------------------------
struct encoder{
  float rollerDiam;         // diameter of encoder 12.5mm spool + 0.5mm string

  float encLInit;
  float encRInit;           // inital Positions of encoders

  bool posLReached;         // left. Is string at its target length?
  bool posRReached;         // right

  long lastUpdateTime;      // tracks encoder polling
};

//Pen Placement Variables-----------------------------------
struct pen{
  uint8_t penUp;              // servo position presets
  uint8_t penDown;
  uint8_t penStart;

  float penPos;             // current servo position
};


//UI variables-----------------------------------------------
struct ui{
  bool selectMode;         // Conditional for UI while loop, no file chosen
  bool paused;             // Conditional for pause while loop

  bool sWasPressed;        // Button debounce variables for Scroll button... 
  bool pWasPressed;        // ... and Pause Button

  uint8_t cursorPos;       // cursor for choosing files or settings
};


//Robot global variables-----------------------------------
struct bot{
  float currentCoords[2];           // current XY pos. {X, Y}
  float currentLengths[2];          // current String Lengths (mm) {Left, Right}

  float nextCoords[2];              // next target coords {X, Y}
  float nextLengths[2];             // next target string length {Left, Right}

  float initStringLengths[2];       // {Left, Right} robot's startup position

  int pointIndex;                   // current point in pattern (for pattern drawing)
  bool PenDownOnArrival;            // pen state when arriving at next point
  Positions previousPoint;          // previous pattern point for duplicate detection
  bool patternComplete;             // true when pattern is finished, waiting for return button
};

///////////////////////////// INIT STRUCTS /////////////////////////////////////

//canvass width, image center height, init size setting, Image Centering Offset X, Image Centering Offset Y.
image Image = {0, 0, bootSize, 0, 0};
//motor max duty cycle, min, right speed, left speed
motor Motor = {motorTravelSpeed, drawSpeed, {0}};
//encoder diameter, zero1, zero0, reachedTarget1, reachedTarget2, lastUpdateTime
encoder Encoder = {12.5, 0.0, 0.0, false, false, 0};
//pen Up servo pos, pen down pos, init pos, current pos
pen Pen = {penUpPos, penDownPos, penStartPos, 0};
//menu flag (starts true), is paused (starts false), green button pressed, red button pressed, selected file number (list index)
ui UI = {true, false, false, false, initCursorPos};
//current XY, current string lengths, next XY, next string lengths, initial string lengths, pointIndex, PenDownOnArrival, previousPoint, patternComplete
bot Plotter = {{0}, {0}, {0}, {0}, {0}, 0, false, {0, 0}, false};
#pragma endregion STRUCTS

//////////////////////////////////////////////////
//  SETUP //
//////////////////////////////////////////////////
#pragma region SETUP_FUNCTIONS
/////////////////////// I2C Multiplexer Port Selector //////////////////////////
void tcaselect(uint8_t i) {
  if (i > 7) return;
 
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();  
}

////////////////////////////////// UI /////////////////////////////////////
// Enters this loop first. Prompts users to the use buttons to select a file, or edit settings before running the plotter.
void runUI(){
  myPen.detach();
  while(UI.selectMode){
    
    redButton.loop();    // MUST call the loop() each iteration to update button
    greenButton.loop();

    // check for scroll/cancel (red) button press and release
    if(redButton.isPressed()){
      UI.sWasPressed = true;
    }
    if(redButton.isReleased() && UI.sWasPressed){
      UI.sWasPressed = false;
      UI.cursorPos += 1;               // cycle thru menu
      UI.cursorPos %= FILE_NUM + 2;    // wrap back after the draw settings
      Serial.print("Cursor pos: ");
      Serial.println(UI.cursorPos);
      Serial.print("Pen mode: ");
      Serial.println(penStyle);
    }

    // check for choose/pause (green) button press and release
    if(greenButton.isPressed()){
      UI.pWasPressed = true;
    }

    if(greenButton.isReleased() && UI.pWasPressed){
      // if hit choose on size icon
      if(UI.cursorPos == FILE_NUM){   // if size preset selection
        pixels.clear();        

        Image.setSize += 1;           // change size
        Image.setSize %= 3;
        switch(Image.setSize){
          case 0:
            pixels.setPixelColor(8, pixels.Color(0, 255, 0));  // green, small
            break;
          case 1:
            pixels.setPixelColor(8, pixels.Color(255, 180, 0)); // yellow, med
            break;
          case 2:
            pixels.setPixelColor(8, pixels.Color(255, 0, 0));  // red, large
            break;
          default:
            break;
        }
        pixels.show();
      }
      
      // if hit choose on pen icon
      else if(UI.cursorPos == FILE_NUM + 1){     // pen mode selection
        pixels.clear();        

        penMode += 1;
        penMode %= 4;  // 4 pen modes: White, Red, Green, Blue (Yellow reserved for future)
        Serial.println("Pen mode changed to: " + String(penMode));
        updatePenModeDisplay();
      } 
      
      //otherwise, we've hit choose on a file number or pattern number
      else {
        UI.pWasPressed = false;     // ensure not paused
        
        // Try to initialize file system for selected file
        if (!initializeFileSystem()) {
          // File initialization failed, stay in selector mode
          Serial.println("File not found, staying in selector");
          return; // Stay in UI loop
        }
        
        UI.selectMode = false;      // Exit UI loop with this flag

        for(uint8_t i = 0; i < 3; i++){         // flash selected pixel green
          pixels.clear();
          pixels.show();
          pixels.setPixelColor(UI.cursorPos + 2, pixels.Color(0, 255, 0));
          pixels.show();
          delay(300);
          pixels.setPixelColor(UI.cursorPos + 2, pixels.Color(0, 0, 0));
          pixels.show();
          delay(300);
        }
        initializePatternSize();
      }
    }

    // Blink cursor LED with pen mode color if over file/pattern
    if(UI.cursorPos < 6){
      pixels.clear();
      
      // Get pen mode color and apply brightness scaling
      uint8_t r, g, b;
      getPenModeColor(penMode, &r, &g, &b);
      setLEDWithSineWave(UI.cursorPos + 2, r, g, b, 300.0);
      pixels.show();
    } 
    // change cursor LED's color to select sizes
    else if (UI.cursorPos == 6){
      int col = 255;
      pixels.clear();
      switch(Image.setSize){
          case 0:
            pixels.setPixelColor(8, pixels.Color(0, col, 0));
            break;
          case 1:
            pixels.setPixelColor(8, pixels.Color(col, 180, 0));
            break;
          case 2:
            pixels.setPixelColor(8, pixels.Color(col, 0, 0));
            break;
          default:
            break;
        }
      pixels.show();
    } 
    // change cursor LED's color to select pen mode
    else {
      pixels.clear();
      updatePenModeDisplay();
    }
    
  }
}

// Get color for pen mode (RGB values 0-255)
void getPenModeColor(uint8_t mode, uint8_t* r, uint8_t* g, uint8_t* b) {
  switch(mode) {
    case 0: // White - PROGMEM patterns
      *r = 255; *g = 255; *b = 255;
      break;
    case 1: // Red - SD files 1-6
      *r = 255; *g = 0; *b = 0;
      break;
    case 2: // Green - SD files 7-12
      *r = 0; *g = 255; *b = 0;
      break;
    case 3: // Blue - SD files 13-18
      *r = 0; *g = 0; *b = 255;
      break;
    case 4: // Yellow - Serial mode (reserved for future)
      *r = 255; *g = 255; *b = 0;
      break;
    default:
      *r = 0; *g = 0; *b = 0; // Black for unknown mode
      break;
  }
}

// Update pen mode display LED
void updatePenModeDisplay() {
  uint8_t r, g, b;
  getPenModeColor(penMode, &r, &g, &b);
  pixels.setPixelColor(9, pixels.Color(r, g, b));
  pixels.show();
}

// Set LED with sine wave brightness
void setLEDWithSineWave(uint8_t ledIndex, uint8_t r, uint8_t g, uint8_t b, float frequency) {
  int brightness = 255 * (0.5 + 0.5 * sin(millis() / frequency));
  pixels.setPixelColor(ledIndex, pixels.Color(
    (r * brightness) / 255, 
    (g * brightness) / 255, 
    (b * brightness) / 255
  ));
}

// Lift pen and wait
void liftPen() {
  Pen.penPos = Pen.penUp;
  myPen.write(Pen.penPos);
  delay(100);  // Pause 100ms after lifting pen
}

// Show pattern complete LED indication
void showPatternCompleteLEDs() {
  for(uint8_t i = 0; i < pixels.numPixels(); i++){
    uint32_t color = pixels.Color(255 * (0.5 + 0.5 * sin(millis() / 200.0)), 0, 0);
    pixels.setPixelColor(i, color);
  }
  pixels.show();
}

// Show error LED with sine wave
void showErrorLED(uint8_t ledIndex, uint8_t r, uint8_t g, uint8_t b, float frequency) {
  int sinwave = 255 * (0.5 + 0.5 * sin(millis() / frequency));
  pixels.setPixelColor(ledIndex, pixels.Color(sinwave * r / 255, sinwave * g / 255, sinwave * b / 255));
  pixels.show();
}

// Initialize pattern size settings
void initializePatternSize() {
  if(Plotter.initStringLengths[0] == 0 && Plotter.initStringLengths[1] == 0){
    switch(Image.setSize){
      case 0:
        Image.canvassWidth = SMALL_WIDTH - 80;
        Image.centerHeight = SMALL_Y_OFFSET;
        Plotter.initStringLengths[0] = SMALL_INIT;
        Plotter.initStringLengths[1] = SMALL_INIT;
        break;
      case 1:
        Image.canvassWidth = MED_WIDTH - 80;
        Image.centerHeight = MED_Y_OFFSET;
        Plotter.initStringLengths[0] = MED_INIT;
        Plotter.initStringLengths[1] = MED_INIT;
        break;
      case 2:
        Image.canvassWidth = LARGE_WIDTH - 80;
        Image.centerHeight = LARGE_Y_OFFSET;
        Plotter.initStringLengths[0] = LARGE_INIT;
        Plotter.initStringLengths[1] = LARGE_INIT;
        break;
      default:
        break;
    }
  }
}

// Convert theta-rho coordinates to radial-angular format
void convertThetaRhoToRadialAngular(float theta, float radius, uint16_t* radial, uint16_t* angular) {
  // Convert theta from radians to tenths of degrees
  float angleDeg = theta * 180.0 / PI; // Convert to degrees
  
  // Adjust for theta being measured from Y-axis instead of X-axis
  // Rotate and mirror: 270 - angleDeg
  angleDeg = 270.0 - angleDeg;
  
  // Normalize to 0-360 range
  while (angleDeg < 0) angleDeg += 360;
  while (angleDeg >= 360) angleDeg -= 360;
  
  *angular = (uint16_t)(angleDeg * 10); // Convert to tenths of degrees
  
  // Convert normalized radius (0-1) to distance (0-1000)
  *radial = (uint16_t)(radius * 1000.0);
}

// Show file error indication
void showFileError() {
  // Flash red on LED 1 for 3 seconds to indicate file not found
  for(int i = 0; i < 6; i++) {
    pixels.clear();
    pixels.setPixelColor(1, pixels.Color(255, 0, 0)); // Red on LED 1
    pixels.show();
    delay(250);
    pixels.clear();
    pixels.show();
    delay(250);
  }
}

// Initialize file system based on pen mode
bool initializeFileSystem() {
  if (penMode == WHITE_MODE) {
    fileMode = false;
    return true;
  } else {
    // Initialize SD card only once when first needed for SD modes
    if (!sdCardInitialized) {
      if (!SD.begin(SD_CS_PIN)) {
        Serial.println("SD Card initialization failed!");
        // Show error light - flash red on LED 0
        showErrorLED(0, 255, 0, 0, 500.0);
        UI.selectMode = true;
        return false;
      } else {
        sdCardInitialized = true;
        Serial.println("SD Card initialized successfully");
      }
    }
    
    // Calculate file number with offset
    uint8_t actualFileNumber = UI.cursorPos + 1 + (penMode - 1) * 6;    
    // Also try with character array
    char filenameChar[10];
    sprintf(filenameChar, "%d.thr", actualFileNumber);
    Serial.print("Trying char array: ");
    Serial.println(filenameChar);
    
    thrFile = SD.open(filenameChar, FILE_READ);
    
    if (!thrFile) {
      Serial.print("Failed to open: ");
      Serial.println(filenameChar);
      // Show error light - flash red on LED 1
      showFileError();
      // Return to selector screen
      UI.selectMode = true;
      return false; // Indicate failure
    } else {
      fileMode = true;
      Serial.print("Opened file: ");
      Serial.println(filenameChar);
      return true; // Indicate success
    }
  }
}

// Read next coordinate from file or PROGMEM
bool readNextCoordinate(Positions* coord) {
  if (!fileMode) {
    // Use existing PROGMEM pattern reading
    return readNextPatternCoordinate(coord);
  } else {
    // Read from .thr file
    return readNextThrCoordinate(coord);
  }
}

// Read next coordinate from .thr file
bool readNextThrCoordinate(Positions* coord) {
  if (!thrFile || !thrFile.available()) {
    Serial.println("End of file or file not available");
    return false; // End of file
  }
  
  String line = thrFile.readStringUntil('\n');
  line.trim(); // Remove whitespace
  
  if (line.length() == 0) {
    return false; // Empty line
  }
  
  // Parse "theta radius" format
  int spaceIndex = line.indexOf(' ');
  if (spaceIndex > 0) {
    float theta = line.substring(0, spaceIndex).toFloat();
    float radius = line.substring(spaceIndex + 1).toFloat();
    
    // Convert to existing format
    convertThetaRhoToRadialAngular(theta, radius, &coord->radial, &coord->angular);
    
    // Increment point index for file reading
    Plotter.pointIndex++;
    
    return true;
  }
  
  return false;
}

// Read next coordinate from PROGMEM pattern
bool readNextPatternCoordinate(Positions* coord) {
  if (Plotter.pointIndex+1 >= patternSizes[UI.cursorPos]) {
    return false; // End of pattern
  }
  
  Plotter.pointIndex++;
  *coord = readPatternPoint(UI.cursorPos, Plotter.pointIndex);
  return true;
}

///////////////////////////// SETUP ///////////////////////////////
void setup() { 
  pinMode(L_M_DIR, OUTPUT);
  pinMode(L_M_PWM, OUTPUT);
  pinMode(R_M_DIR, OUTPUT);
  pinMode(R_M_PWM, OUTPUT);
  pinMode(SERVOPIN, OUTPUT);
  pinMode(PAUSEPIN, INPUT_PULLUP);
  pinMode(CURSORPIN, INPUT_PULLUP);

  Serial.begin(115200);
  Serial.println("PLTTR V1.0.0 ..... 7/22/2025");
  Wire.begin();
  pixels.begin();

  redButton.setDebounceTime(25);
  greenButton.setDebounceTime(25);

  myPen.attach(SERVOPIN);                   // pen Servo on pin 9
  myPen.write(Pen.penStart);

  delay(800);
  
  pixels.setBrightness(255);
  
  // StartUp sequence, file select from UI ---------------------------------
  runUI();
  myPen.attach(SERVOPIN);
  myPen.write(Pen.penUp);

  // Offset needed to center pattern in canvass
  Image.centerXOffset = Image.canvassWidth / 2;
  Image.centerYOffset = Image.centerHeight;

  // Init Left Encoder ---------------------------------------------------------
  tcaselect(0);
  encL.begin();
  delay(50);
  encL.setDirection(AS5600_CLOCK_WISE);
  Encoder.encLInit = encL.resetCumulativePosition();

  // Init Right Encoder --------------------------------------------------------
  tcaselect(1);
  encR.begin();  
  delay(50);                          
  encR.setDirection(AS5600_CLOCK_WISE);            // default, just be explicit.
  Encoder.encRInit = encR.resetCumulativePosition(); // zero encoder at start

  // Initialize pattern drawing
  initializePatternDrawing();
  Serial.println("Pattern drawing initialized");
  
  for(uint8_t i = 2 ; i < 2 + UI.cursorPos; i++){
    pixels.setPixelColor(i, pixels.Color(5, 1, 5));
  }
  pixels.setPixelColor(2 + UI.cursorPos, pixels.Color(100, 0, 100));
  pixels.show();
}
#pragma endregion SETUP_FUNCTIONS

//////////////////////////////////////////////////
//  LOOP //
//////////////////////////////////////////////////
#pragma region LOOP
void loop() { 

  yield();
  
  // Check for user inputs
  pollButtons();
  // Check if paused
  if(UI.paused){
    pause();
    return;  // end loop here
  }
  
  // Check if pattern is complete and waiting for return button
  if(Plotter.patternComplete && !UI.selectMode){
    // Ensure pen stays up during completion movement
    if (Pen.penPos != Pen.penUp) {
      liftPen();
    }
    
    // If MoveDownOnCompletion is enabled, move to completion position
    if (MoveDownOnCompletion) {
      // Continue moving until completion position is reached
      if (moveBot()) {
        // Still moving to completion position
        return;
      } else {
        // Reached completion position, stop motors
        stopMotors();
      }
    } else {
      // Stay in place, stop motors
      stopMotors();
    }
    
    // Show pattern complete LED indication
    showPatternCompleteLEDs();
    return;  // end loop here, wait for button press
  }
  
  // Run bot - only when not in UI selector mode
  if (!UI.selectMode) {
    runPatternPlotter();
  }
  
}
#pragma endregion LOOP

//////////////////////////////////////////////////
// BUTTON PRESS ACTIONS//
//////////////////////////////////////////////////
#pragma region RUN_BUTTONS
///////////////////////////// UI Functions ////////////////////////////////
// looks for user inputs for pause or restart
void pollButtons(){
  redButton.loop();                           // MUST call the loop() first
  greenButton.loop();

  if(redButton.isPressed()){
    UI.sWasPressed = true;
  }
  if(redButton.isReleased() && UI.sWasPressed){
    UI.sWasPressed = false;
    // Always use restart() for consistent behavior
    restart(true);
    return;
  }

  if(greenButton.isPressed()){
    UI.pWasPressed = true;
  }
  if(greenButton.isReleased() && UI.pWasPressed){
    UI.pWasPressed = false;                 
    UI.paused = !UI.paused;                 // toggle pause on release
    if(UI.paused){
      myPen.detach();
    }
    else{
      myPen.attach(SERVOPIN);                // if resume, light selected file
      pixels.clear();
      pixels.setPixelColor(UI.cursorPos + 2, pixels.Color(100, 0, 100));
      pixels.show();
    }
  }
}

//Freeze motors, Lift Pen, and run light animations for paused state.
void pause(){
  stopMotors();
  myPen.write(Pen.penUp);
  delay(100);  // Pause 100ms after lifting pen

  for(uint8_t i = 0; i < pixels.numPixels(); i++){     // "paused" LED animation
    uint32_t color = pixels.Color(0, 255 * (0.5 + 0.5 * sin(millis() / 1000.0)), 0);

    pixels.setPixelColor(i, color);
  }
  pixels.show();
}


// homes the plotter bot, then goes back to file selection
void restart(bool move){
  // Close .thr file if open
  if (thrFile) {
    thrFile.close();
  }

  // Lift pen before moving to home
  liftPen();

  uint32_t nextUpdate = 0;                     // for light animations
  uint8_t lightPos = pixels.numPixels();
  uint8_t fillPos = 0;

  UI.selectMode = true;
  UI.paused = false;

  // Set target to initialized string lengths (true home position)
  Plotter.nextLengths[0] = Plotter.initStringLengths[0];  // left string length
  Plotter.nextLengths[1] = Plotter.initStringLengths[1];  // right string length

  //Bot should move to "home" when canceling a drawing (move == true), but should not move when the drawing is finished (move == false), or it erases over itself.
  if(move){
    while(moveBot()){                 // runs until home is reached
      if(millis() > nextUpdate){      // animate homing light effect
        nextUpdate = millis() + 100;
        pixels.clear();
        pixels.setPixelColor(lightPos, pixels.Color(0, 255, 0));
        lightPos--;

        if(lightPos < fillPos + 1){
          lightPos = pixels.numPixels();
          fillPos++;
          fillPos %= pixels.numPixels();
        }

        for (uint8_t i = 0; i < fillPos; i++){
          pixels.setPixelColor(i, pixels.Color(0, 255, 0));
        }
        pixels.show();
      }
    }
  }
  
  //Stop both motors after homing
  stopMotors();

  Plotter.initStringLengths[0] = Plotter.currentLengths[0];
  Plotter.initStringLengths[1] = Plotter.currentLengths[1];

  Serial.flush();
  delay(100);
  setup();
}
#pragma region RUN_BUTTONS

//////////////////////////////////////////////////
//  MOVE ROBOT //
//////////////////////////////////////////////////
#pragma region RUN_BOT
/////////////////////////// Plotter Run Functions //////////////////////////////

// Initialize pattern drawing
void initializePatternDrawing() {
  Plotter.pointIndex = -1;  // Use -1 to indicate "not started"
  Plotter.previousPoint = {0, 0};
  Plotter.patternComplete = false;  // Reset pattern complete flag
}

// Run pattern plotter - draws one pattern point per call
void runPatternPlotter() {
  // Exit immediately if pattern is complete
  if (Plotter.patternComplete) {
    return;
  }
  
  // Move to next point
  if (Plotter.pointIndex < 0 || !moveBot()) {
    // Target reached or just initialized - update state for next point
  
    if (Plotter.pointIndex < 0) {
      // Just initialized - load first coordinate.
      Plotter.PenDownOnArrival = true;
    } else if (Plotter.PenDownOnArrival) {
      Pen.penPos = Pen.penDown;
      myPen.write(Pen.penPos);
      Plotter.PenDownOnArrival = false;
    }

    // Declare currentPoint
    Positions currentPoint;

    bool cont = false;
    while (!cont) {
      
      // Read next coordinate (from file or PROGMEM)
      if (!readNextCoordinate(&currentPoint)) {
        // No more coordinates - pattern complete
        Serial.println("Pattern complete - Press red button to return to start");
        
        // Lift pen when pattern finishes
        liftPen();
        
        // Move down if enabled, otherwise stop motors
        if (MoveDownOnCompletion) {
          setTargetFromPolar(1100, 2700);
          Serial.println("Moving down to completion position...");
        } else {
          // Stop motors only if not moving to completion position
          stopMotors();
        }
        
        Plotter.patternComplete = true;
        return;
      }
      
      cont = true;

      // Check for duplicate coordinates (pen up signal)
      if (Plotter.pointIndex > 0 && currentPoint.radial == Plotter.previousPoint.radial && 
        currentPoint.angular == Plotter.previousPoint.angular) {
        cont = false;

        // Lift pen
        liftPen();
        Plotter.PenDownOnArrival = true;
      }
    }

    Plotter.previousPoint = currentPoint; // Store the point it's heading toward
    
    // Set target position from pattern coordinates
    setTargetFromPolar(currentPoint.radial, currentPoint.angular);
    
    // Debug output
    Serial.print("Point ");
    Serial.print(Plotter.pointIndex);
    Serial.print(": ");
    Serial.print(currentPoint.radial);
    Serial.print(" ");
    Serial.println(currentPoint.angular);

  }
  
}


// Moves motor towards next coordinate, returns true if either motor moved. Code continually tries to move towards the exact target position every iteration, boolean "target reached" flags will update if error is less than epsilon set by user. 
bool moveBot(){
  int nextSpeeds[2];                              // define motor speeds {L, R}
  float currentXY[2];                             // current position, local var

  updateStringLength();                           // updates currentLengths[]

  lengthsToXY(Plotter.currentLengths[0], Plotter.currentLengths[1], currentXY);
  Plotter.currentCoords[0] = currentXY[0];        // updates currentCoords
  Plotter.currentCoords[1] = currentXY[1];
  

  // Get motor speeds
  getSpeeds(Plotter.nextLengths[0], Plotter.nextLengths[1], nextSpeeds);

  
  // Drive left motor until within epsilon, then set "position reached" flag to true. 
  if (abs(Plotter.currentLengths[0] - Plotter.nextLengths[0]) < epsilon){
    driveMotors(nextSpeeds[0], true);   // left
    Encoder.posLReached = true;
  } else {
    driveMotors(nextSpeeds[0], true);   // left
    Encoder.posLReached = false;
  }

  // Drive right motor.
  if (abs(Plotter.currentLengths[1] - Plotter.nextLengths[1]) < epsilon){
    driveMotors(nextSpeeds[1], false);   // right
    Encoder.posRReached = true;
  } else {
    driveMotors(nextSpeeds[1], false);   // right
    Encoder.posRReached = false;
  }

  // Returns true if either motor needs to move. Returns false when position is reached within the defined error.
  if (Encoder.posLReached && Encoder.posRReached){
    return false;
  } else {
    return true;
  }
}

 // Call function with true to move left motor, false for right motor.
void driveMotors(int speed, bool left){  
  if(left){
    if(speed < 0){
      digitalWrite(L_M_DIR, HIGH);
    } else {
      digitalWrite(L_M_DIR, LOW);
    }
    analogWrite(L_M_PWM, abs(speed));
    Motor.setSpeeds[0] = speed;
  } else {
    if(speed < 0){
      digitalWrite(R_M_DIR, HIGH);
    } else {
      digitalWrite(R_M_DIR, LOW);
    }
    analogWrite(R_M_PWM, abs(speed));
    Motor.setSpeeds[1] = speed;
  }
}

// Stop both motors
void stopMotors() {
  driveMotors(0, true);   // left
  driveMotors(0, false);  // right
}

// Set target position from polar coordinates
void setTargetFromPolar(uint16_t radial, uint16_t angular) {
  // Convert polar to cartesian coordinates
  float nextXY[2];
  polarToXY(radial, angular, nextXY);
  
  // Set target coordinates
  Plotter.nextCoords[0] = nextXY[0];
  Plotter.nextCoords[1] = nextXY[1];
  
  // Convert to string lengths
  float nextL[2];
  XYToLengths(nextXY[0], nextXY[1], nextL);
  Plotter.nextLengths[0] = nextL[0];
  Plotter.nextLengths[1] = nextL[1];
}

// Set target position from cartesian coordinates
void setTargetFromXY(float x, float y) {
  // Set target coordinates
  Plotter.nextCoords[0] = x;
  Plotter.nextCoords[1] = y;
  
  // Convert to string lengths
  float nextL[2];
  XYToLengths(x, y, nextL);
  Plotter.nextLengths[0] = nextL[0];
  Plotter.nextLengths[1] = nextL[1];
}


/////////////////////// Movement Control Functions //////////////////////////
// Gives speeds to move each motor towards it's target position. Once within  Linearly scales speed relative to distance-to-go with a lower bound at the minimum duty cycle required to lift the robot.
void getSpeeds(float toLengthLeft, float toLengthRight, int* nextSpeeds){
  float leftMScale = 1;
  float rightMScale = 1;

  // First, store distance remaining in these variables for future calculations
  float speedL = (toLengthLeft - Plotter.currentLengths[0]);   // left
  float speedR = (toLengthRight - Plotter.currentLengths[1]);  // right
  
  // Calculate distances for coordination
  float distL = abs(speedL);
  float distR = abs(speedR);
  float maxDist = max(distL, distR);
  
  //If target position is hit, slow the motor linearlly near exact position
  if(abs(speedL) < epsilon){
    leftMScale = max(abs(speedL) / epsilon, epsilonScaling);
  } 
  // For medium and long distances, coordinate speeds so both arrive simultaneously
  else {
    float speedRatioL = (distL > 0) ? distL / maxDist : 0;
    if (speedL >= 0){
      speedL = Motor.maxSpeed * speedRatioL;
    } else {
      speedL = -Motor.maxSpeed * speedRatioL;
    }
  }

  if(abs(speedR) < epsilon){
    //rightMScale = max(epsilonScaling * abs(speedR) / epsilon, epsilonScaling);
    rightMScale = max(abs(speedR) / epsilon, epsilonScaling);
  } 
  // For medium and long distances, coordinate speeds so both arrive simultaneously
  else {
    float speedRatioR = (distR > 0) ? distR / maxDist : 0;
    if (speedR >= 0){
      speedR = Motor.maxSpeed * speedRatioR;
    } else {
      speedR = -Motor.maxSpeed * speedRatioR;
    }
  }

  // Apply scaling only for close distances (epsilon zone)
  if (maxDist < epsilon) {
    //set left motor speed to speedL variable
    if (speedL >= 0){
      speedL = Motor.drawSpeed * leftMScale;
    } else{
      speedL = -Motor.drawSpeed * leftMScale;
    }
    
    //set right motor speed to speedR variable
    if (speedR >= 0){
      speedR = Motor.drawSpeed * rightMScale;
    } else{
      speedR = -Motor.drawSpeed * rightMScale;
    }
  }
  // For medium and long coordinated movement, speeds are already calculated correctly above

  // flipping motors
  if(flipLeftMotor){
    speedL = - speedL;
  }
  if(flipRightMotor){
    speedR = - speedR;
  }

  nextSpeeds[0] = speedL;
  nextSpeeds[1] = speedR;
}
#pragma endregion RUN_BOT

//////////////////////////////////////////////////
//  READ ENCODERS FOR STRING LENGTHS //
//////////////////////////////////////////////////
#pragma region ENCODERS 
//////////////////////// Encoder Functions ////////////////////////////
// Updates currentLengths global variable based on encoder positions.
void updateStringLength(){  
  //  update every 10 ms
  if (millis() - Encoder.lastUpdateTime >= sampleTime){  
    Encoder.lastUpdateTime = millis();

    //Left Encoder
    tcaselect(0);
    int32_t thetaL = encL.getCumulativePosition();
    Plotter.currentLengths[0] = lengthFromCounts(thetaL, 0);

    //Right Encoder
    tcaselect(1);
    int32_t thetaR = encR.getCumulativePosition();
    Plotter.currentLengths[1] = lengthFromCounts(thetaR, 1);
  }
}

// returns lengths (in mm) from encoder movments and inital zero.               
// encNum = 0 - left; 1 - right
float lengthFromCounts(int32_t pos, uint8_t encNum){
  // Left
  if (encNum == 0) {
    if(flipLeftEncoder){
      return - pos / 4096.0 * Encoder.rollerDiam * PI 
            + Plotter.initStringLengths[0];
    } else{
      return pos / 4096.0 * Encoder.rollerDiam * PI 
            + Plotter.initStringLengths[0];
    }
  }
  // Right
  else if (encNum == 1){
    if(flipRightEncoder){
      return - pos / 4096.0 * Encoder.rollerDiam * PI 
            + Plotter.initStringLengths[1];
    } else {
      return  pos / 4096.0 * Encoder.rollerDiam * PI 
            + Plotter.initStringLengths[1];
    }
  }
  else {
    return -1;                                                     // error
  }
}
#pragma endregion ENCODERS 

//////////////////////////////////////////////////
//  INVERSE KINEMATICS //
//////////////////////////////////////////////////
#pragma region COORDINATE_MAPPING
////////////////////// Inverse Kinetmatic Functions ////////////////////////
// The heart of the program lies in these few lines of code. The math below converts string lengths to cartesian coordinates, allowing us to position the robot where we want. 
void lengthsToXY(float lengthLeft, float lengthRight, float* XY){
  float s = (lengthLeft + lengthRight + Image.canvassWidth) / 2.0;
  float area = sqrt(s * (s - lengthLeft) * (s - lengthRight)
               * (s - Image.canvassWidth));

  XY[1] = 2.0 * area / Image.canvassWidth;                    // Y - coord
  XY[0] = sqrt(pow(lengthLeft, 2.0) - pow(XY[1], 2.0));       // X - coord
}

// converts cartesian coords to string lengths. Call with next = 0 to update current lengths, or next = 1 to set next lengths.
void XYToLengths(float x, float y, float* L){
  L[0] = sqrt(pow(x, 2.0) + pow(y, 2.0));                             //left
  L[1] = sqrt(pow((Image.canvassWidth - x), 2.0) + pow(y, 2.0));      //right
}

// converts polar coordinates to cartesian coordinates
void polarToXY(uint16_t radial, uint16_t angular, float* XY){
  // Convert angular from degrees*10 to radians
  float angleRad = (angular / 10.0) * PI / 180.0;
  
  // Convert polar to cartesian (0°=right, 90°=top, 180°=left, 270°=bottom)
  float x = radial * cos(angleRad);
  float y = radial * sin(angleRad);
  
  // Scale to canvas coordinates and center
  // X: scale and center horizontally
  XY[0] = x * (Image.canvassWidth / 2400.0) + (Image.canvassWidth / 2.0);
  // Y: scale and center vertically (smaller Y = higher up)
  XY[1] = Image.centerHeight - y * (Image.canvassWidth / 2400.0);
}

// Read pattern point from PROGMEM
Positions readPatternPoint(uint8_t patternIndex, uint16_t pointIndex) {
  Positions point;
  
  switch(patternIndex) {
    case 0:
      point.radial = pgm_read_word(&(pattern1[pointIndex].radial));
      point.angular = pgm_read_word(&(pattern1[pointIndex].angular));
      break;
    case 1:
      point.radial = pgm_read_word(&(pattern2[pointIndex].radial));
      point.angular = pgm_read_word(&(pattern2[pointIndex].angular));
      break;
    case 2:
      point.radial = pgm_read_word(&(pattern3[pointIndex].radial));
      point.angular = pgm_read_word(&(pattern3[pointIndex].angular));
      break;
    case 3:
      point.radial = pgm_read_word(&(pattern4[pointIndex].radial));
      point.angular = pgm_read_word(&(pattern4[pointIndex].angular));
      break;
    case 4:
      point.radial = pgm_read_word(&(pattern5[pointIndex].radial));
      point.angular = pgm_read_word(&(pattern5[pointIndex].angular));
      break;
    case 5:
      point.radial = pgm_read_word(&(pattern6[pointIndex].radial));
      point.angular = pgm_read_word(&(pattern6[pointIndex].angular));
      break;
    default:
      point.radial = 0;
      point.angular = 0;
      break;
  }
  
  return point;
}

#pragma endregion COORDINATE_MAPPING