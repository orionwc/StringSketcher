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

#define NUM_PATTERNS 6          // Number of patterns selectable in one Pattern Mode
#define PATTERN_SELECTOR_LED 9   // LED index for pattern mode selection (pen icon)

// Pattern completion position (radial, angular*10)
#define REVEAL_POSITION_RADIAL 1100
#define REVEAL_POSITION_ANGULAR 2700

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
1 - Test Pattern
2 - Squirrel
3 - Rose
4 - Planet
5 - Empty
6 - Elephant on a Skateboard
*/

// ************* SETTINGS *************

// Set this to true or false to enable or disable SD card functionality
// Note that several images can be included in the .ino file directly as patterns if this is disabled
// If you include multiple patterns and enable this, it's likely the sketch will exceed the Arduino memory limit
#define ENABLE_SD_CARD false    // [true/false] Enable SD card functionality (disable to  large embedded patterns)

// Set this to true or false to enable or disable serial streaming functionality
// When enabled, allows receiving drawing coordinates via serial port in STREAMING mode
// Protocol: <COORDS> -> <START> -> <READY> -> <POS:r,theta> -> <DONE> -> ... -> <END>
#define ENABLE_STREAMING false  // [true/false] Enable serial streaming mode (requires ENABLE_SD_CARD to be true)

#if ENABLE_SD_CARD
#include <SPI.h>                // SD
#include <SdFat.h>              // FAT storage type
#endif

// Pattern completion behavior ------------------------------------------------
const bool MoveDownOnCompletion = true;  // [true/false] Move below image after pattern completes


// Serial streaming variables (always declared, but only used when ENABLE_STREAMING is true)
const byte numCharsSerialBuffer = 32;
char receivedString[numCharsSerialBuffer];
bool streamingInitialized = false;
bool waitingForStart = false;
bool streamingActive = false;
unsigned long lastSerialTime = 0;
const unsigned long SERIAL_TIMEOUT = 30000; // 30 second timeout

// Pen LED color cycling for active drawing feedback
uint8_t penColorCycleIndex = 0;
const uint8_t PEN_COLOR_CYCLE_COUNT = 6;
const uint32_t PEN_COLOR_CYCLE[] = {
  0xFF0000, // Red
  0xFF8000, // Orange  
  0xFFFF00, // Yellow
  0x00FF00, // Green
  0x0000FF, // Blue
  0x8000FF  // Purple
};


// Starting Pattern Mode
uint8_t patternMode = 0;  // Current pattern mode selection (0=EMBEDDED, 1-3=SDCARD_1-3, 4=STREAMING)

// RGB colors for each pattern mode (R, G, B)
const uint8_t patternModeColors[][3] = {
  {255, 255, 255},  // EMBEDDED - White
  {255, 0, 0},      // SDCARD_1 - Red
  {0, 255, 0},      // SDCARD_2 - Green
  {0, 0, 255},      // SDCARD_3 - Blue
  {255, 255, 0}     // STREAMING - Yellow
};

// Which pattern modes are enabled based on SD card setting
const bool patternModeEnabled[] = {
  true,             // EMBEDDED - always enabled
#if ENABLE_SD_CARD    // SDCARD_1, SDCARD_2, SDCARD_3 - enabled when SD card available
  true, true, true,            
#else
  false, false, false,         
#endif
#if ENABLE_STREAMING
  true             // STREAMING - enabled when serial streaming is enabled
#else
  false            // STREAMING - disabled when serial streaming is disabled
#endif
};

// ************* END SETTINGS *************

// Pattern mode enumeration
enum PatternMode {
  EMBEDDED = 0,    // PROGMEM patterns
  SDCARD_1 = 1,    // SD files 1-6
  SDCARD_2 = 2,    // SD files 7-12
  SDCARD_3 = 3,    // SD files 13-18
  STREAMING = 4    // Serial receiving mode (future)
};

//Pattern drawing structures-----------------------------------
struct Positions {
  uint16_t radial;                  // radial distance (0-1000)
  uint16_t angular;                 // angular position (degrees * 10)
};

// Pattern definitions (polar coordinates: radial, angular*10)--------
const Positions pattern1[] PROGMEM = { // TEST PATTERN
  {800, 0},    // Point 1: Right  (0°)
  {800, 900},  // Point 2: Up     (90°) 
  {800, 1800}, // Point 3: Left   (180°)
  {800, 2700}, // Point 4: Down   (270°)
  {800, 0}     // Point 5: Return to start (0°)
};

const Positions pattern2[] PROGMEM = {
  // Squirrel (97 points)
  {708,2836},{709,2885},{700,2894},{679,2899},{642,2882},{626,2858},{608,2793},{602,2792},{594,2841},{584,2878},{555,2932},{566,3009},{653,3026},{712,3043},{752,3039},{770,3043},{779,3049},{833,3052},{847,3058},{939,3132},{944,3142},{938,3151},{927,3156},{884,3158},{856,3149},{818,3126},{786,3126},{700,3163},{659,3202},{648,3228},{660,3287},{681,3400},{738,3426},{782,3450},{854,3500},{867,3513},{873,3534},{862,3557},{821,3593},{795,12},{736,50},{617,107},{598,154},{568,198},{536,234},{512,246},{492,238},{474,200},{432,115},{299,3596},{251,3586},{177,26},{87,53},{24,2284},{101,2137},{182,2174},{254,2221},{332,2274},{405,2324},{410,2320},{358,2214},{281,1953},{265,1682},{333,1306},{411,1208},{511,1164},{601,1160},{691,1179},{748,1201},{807,1233},{860,1268},{916,1314},{957,1359},{981,1392},{998,1438},{1000,1466},{986,1511},{964,1541},{920,1581},{884,1605},{836,1624},{795,1631},{723,1632},{683,1813},{677,1899},{674,1977},{673,2037},{672,2135},{670,2199},{666,2276},{663,2357},{659,2424},{651,2496},{635,2567},{669,2600},{681,2620},{689,2648},{708,2836}

  // Empty Pattern (use this if you want to try a complex Pattern6 below)
  //{0,0}
};

const Positions pattern3[] PROGMEM = {
  // Rose (197 points)
  {979,2715},{701,2720},{574,2784},{535,2844},{661,2982},{709,3055},{743,3135},{760,3231},{794,3433},{571,3442},{454,3400},{376,3311},{343,3040},{443,2863},{466,3121},{549,3244},{571,3249},{476,2934},{534,2730},{207,2769},{159,2631},{186,2577},{599,2659},{547,2421},{624,2207},{588,2231},{523,2349},{518,2564},{411,2422},{396,2302},{425,2179},{493,2088},{585,2039},{710,2021},{823,2026},{809,2229},{794,2321},{769,2382},{723,2452},{678,2504},{613,2560},{702,2658},{909,2675},{979,2715},{979,2715},{239,2997},{288,3231},{357,3428},{312,3362},{265,3316},{215,3315},{121,2749},{202,2873},{239,2997},{239,2997},{131,2497},{241,2036},{309,2040},{403,1956},{337,2102},{253,2342},{221,2428},{131,2497},{131,2497},{92,2244},{80,1357},{167,905},{234,1019},{406,1308},{521,1353},{654,1356},{810,1324},{858,1401},{879,1466},{874,1478},{683,1529},{533,1673},{432,1777},{274,1923},{92,2244},{92,2244},{458,1245},{469,1103},{569,768},{670,699},{812,761},{839,780},{861,821},{853,889},{799,941},{797,895},{761,853},{777,825},{713,861},{680,958},{664,944},{656,847},{718,762},{653,793},{615,849},{613,931},{646,987},{724,1039},{826,1060},{667,1145},{508,1238},{572,1234},{651,1198},{743,1143},{837,1078},{976,1192},{907,1245},{820,1285},{705,1309},{583,1302},{458,1245},{458,1245},{704,978},{754,874},{771,912},{768,951},{744,971},{744,971},{735,1000},{776,1021},{854,1030},{897,1024},{927,996},{979,916},{970,889},{901,800},{897,875},{876,920},{815,973},{735,1000},{735,1000},{673,951},{714,859},{796,816},{762,850},{797,895},{795,935},{850,895},{861,821},{812,761},{776,750},{747,756},{677,807},{650,901},{673,951},{673,951},{727,1003},{831,962},{876,918},{897,875},{892,803},{857,753},{814,725},{715,690},{835,678},{841,669},{783,649},{720,646},{590,689},{466,830},{421,1203},{288,997},{204,822},{329,639},{614,445},{714,443},{828,465},{909,495},{973,529},{872,585},{797,554},{689,530},{607,527},{519,555},{704,568},{810,590},{916,639},{1000,711},{902,798},{964,880},{979,916},{953,968},{899,1023},{811,1028},{727,1003},{727,1003},{235,606},{152,703},{80,805},{64,2553},{220,3422},{309,3524},{397,8},{512,126},{684,278},{828,309},{808,355},{759,413},{671,392},{546,392},{235,606}

  // Empty Pattern (use this if you want to try a complex Pattern6 below)
  //{0,0}
};

const Positions pattern4[] PROGMEM = {
  // Planet (104 points)
  {468,2691},{469,2778},{469,2845},{469,2950},{470,3083},{470,3224},{470,3369},{470,3465},{468,3500},{284,3033},{357,2544},{457,2387},{468,2390},{468,2487},{468,2560},{468,2691},{468,2691},{251,2933},{359,3416},{560,15},{727,98},{796,126},{928,180},{980,210},{1000,238},{989,264},{960,285},{916,307},{844,337},{705,393},{506,504},{498,496},{503,378},{549,355},{613,318},{658,286},{682,259},{688,234},{678,215},{648,188},{619,170},{541,125},{450,68},{344,3576},{249,3426},{175,3059},{201,2619},{324,2330},{424,2233},{516,2172},{620,2110},{664,2077},{682,2046},{683,2032},{674,2013},{649,1991},{610,1965},{497,1899},{498,1845},{497,1775},{504,1776},{707,1889},{796,1926},{886,1963},{962,2000},{988,2021},{996,2044},{991,2056},{974,2073},{914,2105},{833,2139},{752,2171},{653,2215},{480,2322},{340,2499},{251,2933},{251,2933},{156,2687},{223,3462},{330,18},{440,102},{468,122},{470,135},{472,235},{472,364},{472,480},{472,598},{471,688},{472,827},{471,969},{471,1039},{470,1131},{470,1255},{470,1384},{469,1470},{468,1563},{467,1682},{467,1735},{467,1861},{467,1914},{467,2033},{467,2142},{462,2163},{364,2230},{271,2330},{156,2687}
  
  // Empty Pattern (use this if you want to try a complex Pattern6 below)
  //{0,0}
};

const Positions pattern5[] PROGMEM = {
  // Insert your own pattern here (comment out the empty pattern if you do)

  // Empty Pattern (use this if you want to try a complex Pattern6 below)
  {0,0}
};

const Positions pattern6[] PROGMEM = {
#if ENABLE_SD_CARD
  // Empty Pattern - a large pattern is not supported when SD card is enabled as it exceeds the Arduino memory limit
  {0,0}
#else
  // Large embedded pattern - add your ~500 point pattern here when SD card is disabled
  // Example large pattern (replace with your actual data)
  // Elephant on a Skateboard
  {240,2657},{270,2747},{274,2786},{269,2801},{149,2893},{156,2524},{172,2444},{204,2366},{207,2372},{214,2460},{240,2657},{240,2657},{303,2859},{362,2843},{396,2830},{415,2809},{451,2755},{465,2745},{488,2743},{492,2735},{780,2188},{801,2169},{815,2149},{852,2086},{865,2075},{882,2067},{899,2068},{909,2077},{910,2087},{898,2116},{871,2160},{854,2181},{838,2197},{822,2210},{776,2241},{769,2247},{761,2270},{764,2272},{800,2271},{824,2278},{848,2295},{857,2311},{858,2324},{865,2325},{980,2250},{982,2252},{765,2475},{762,2918},{980,3147},{977,3149},{865,3075},{858,3076},{856,3092},{848,3103},{827,3120},{812,3125},{791,3129},{776,3129},{763,3125},{757,3127},{765,3149},{819,3188},{838,3203},{865,3228},{877,3243},{906,3286},{920,3315},{921,3326},{915,3333},{908,3336},{900,3337},{880,3332},{869,3325},{851,3306},{821,3253},{810,3238},{785,3215},{564,2995},{542,2961},{536,2969},{538,2991},{534,3006},{511,3018},{441,3027},{397,3050},{349,3107},{335,3139},{326,3180},{327,3229},{340,3280},{403,3367},{432,3401},{450,3426},{482,3477},{523,3538},{527,3538},{529,3532},{514,3492},{491,3436},{472,3400},{437,3347},{410,3313},{373,3266},{377,3258},{438,3243},{468,3250},{574,3282},{624,3286},{649,3284},{678,3278},{696,3271},{740,3238},{753,3232},{773,3233},{780,3239},{790,3256},{799,3285},{817,3346},{820,3373},{809,3384},{773,3393},{691,3399},{622,3413},{587,3430},{574,3443},{570,3470},{635,2},{681,78},{685,77},{696,62},{705,34},{712,3561},{725,3539},{745,3522},{744,3496},{755,3476},{772,3465},{792,3456},{830,3446},{831,3449},{826,3451},{819,3461},{810,3488},{812,3497},{809,3509},{803,3517},{786,3528},{764,3534},{747,3548},{737,3567},{739,82},{740,173},{739,252},{736,288},{732,324},{720,380},{709,411},{694,441},{668,484},{614,595},{602,686},{607,739},{554,700},{509,681},{466,682},{439,694},{422,710},{403,735},{370,810},{345,874},{326,924},{282,1049},{235,1265},{235,1345},{244,1366},{274,1364},{290,1343},{293,1349},{248,1424},{214,1563},{208,1690},{226,1821},{190,1775},{174,1731},{160,1648},{155,1543},{175,1328},{188,1275},{189,1257},{183,1245},{139,1371},{121,1576},{130,1717},{146,1800},{168,1857},{261,1944},{307,1959},{357,1985},{383,2009},{411,2051},{439,2120},{455,2165},{472,2191},{537,2238},{565,2267},{563,2286},{550,2300},{520,2326},{452,2382},{419,2407},{410,2411},{393,2400},{381,2379},{369,2323},{351,2268},{335,2241},{313,2221},{262,2216},{220,2240},{185,2284},{135,2426},{116,2820},{115,2919},{112,3009},{105,3122},{90,3367},{146,412},{198,539},{203,528},{186,461},{145,256},{129,103},{120,3468},{238,3585},{312,19},{314,8},{252,3543},{150,3361},{137,3285},{136,3147},{150,3056},{176,2981},{303,2859},{303,2859},{762,3078},{767,3067},{785,3059},{800,3061},{807,3065},{815,3077},{814,3083},{806,3091},{791,3097},{773,3093},{764,3085},{764,3085},{703,2304},{688,2316},{694,3087},{722,3099},{725,3097},{719,3083},{720,3072},{726,3054},{736,3043},{761,3030},{795,3029},{816,3033},{818,3035},{822,3034},{826,3035},{765,2966},{760,2442},{828,2362},{825,2360},{811,2367},{788,2370},{773,2370},{760,2368},{739,2357},{730,2348},{721,2320},{725,2303},{733,2296},{730,2294},{730,2294},{764,2317},{770,2333},{786,2340},{804,2335},{814,2326},{815,2317},{811,2311},{798,2304},{789,2302},{773,2307},{773,2307},{626,1723},{580,1706},{531,1672},{501,1632},{473,1557},{475,1552},{531,1557},{552,1602},{589,1647},{627,1674},{670,1692},{709,1699},{751,1702},{792,1699},{826,1692},{832,1694},{831,1698},{815,1708},{790,1718},{731,1729},{671,1730},{626,1723},{626,1723},{645,1655},{599,1633},{572,1607},{542,1543},{482,1534},{473,1522},{466,1500},{461,1496},{456,1502},{455,1537},{452,1573},{439,1599},{429,1603},{418,1594},{407,1547},{394,1500},{342,1408},{316,1319},{347,1287},{398,1231},{431,1193},{463,1151},{459,1145},{375,1204},{325,1244},{285,1274},{270,1282},{267,1275},{301,1106},{352,953},{382,876},{413,802},{437,751},{451,734},{474,721},{497,719},{524,724},{551,737},{600,774},{650,821},{680,855},{693,875},{703,897},{707,913},{710,950},{708,981},{701,1047},{694,1104},{698,1109},{704,1105},{709,1071},{713,1070},{736,1176},{745,1217},{739,1243},{689,1353},{664,1426},{658,1465},{659,1498},{673,1533},{684,1546},{713,1560},{743,1564},{770,1557},{789,1546},{803,1529},{806,1519},{806,1500},{793,1442},{796,1412},{815,1382},{842,1364},{876,1354},{896,1353},{916,1355},{964,1369},{1000,1386},{999,1390},{992,1395},{965,1404},{956,1411},{951,1419},{939,1426},{930,1425},{925,1421},{918,1401},{910,1394},{892,1395},{879,1404},{874,1410},{871,1430},{890,1484},{898,1520},{896,1543},{890,1561},{863,1600},{837,1622},{813,1635},{789,1646},{741,1659},{686,1662},{645,1655},{645,1655},{569,1315},{583,1345},{583,1362},{593,1355},{600,1337},{596,1326},{586,1316},{572,1312}
#endif
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
  bool drawing;                     // true when actively drawing a pattern
  bool doneSent;                    // true when DONE message has been sent for current coordinate
  bool patternEnded;                // true when END signal was received
  bool WaitingForCoordinate;        // true when waiting for next coordinate to arrive
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
//current XY, current string lengths, next XY, next string lengths, initial string lengths, pointIndex, PenDownOnArrival, previousPoint, patternComplete, drawing, doneSent, patternEnded, WaitingForCoordinate
bot Plotter = {{0}, {0}, {0}, {0}, {0}, 0, false, {0, 0}, false, false, false, false, false};
#pragma endregion STRUCTS

//////////////////////////////////////////////////
//  SD CARD FUNCTIONALITY //
//////////////////////////////////////////////////
#if ENABLE_SD_CARD
SdFat SD;                           // SD card filesystem object
File thrFile;                       // Currently open .thr pattern file
bool sdCardInitialized = false;     // Track if SD card has been initialized
#endif

// Initialize system based on pattern mode
bool initializeSystem() {
  switch (patternMode) {
    case EMBEDDED:
      return true;

#if ENABLE_STREAMING
    case STREAMING:
      return initializeSerialStreaming();
#endif

#if ENABLE_SD_CARD
    case SDCARD_1:
    case SDCARD_2:
    case SDCARD_3:
      return initializeSDCard();
#endif

    default:
      return false;
  }
}

#if ENABLE_SD_CARD
// Initialize SD card and open pattern file
bool initializeSDCard() {
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
    }
  }
  
  // Calculate file number with offset
  uint8_t actualFileNumber = UI.cursorPos + 1 + (patternMode - 1) * 6;    
  // Also try with character array
  char filenameChar[10];
  sprintf(filenameChar, "%d.thr", actualFileNumber);
  
  // Check if SD card is available
  if (!SD.exists(filenameChar)) {
    Serial.print("File does not exist: ");
    Serial.println(filenameChar);
    return false;
  }
  
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
    Serial.print("Opened file: ");
    Serial.println(filenameChar);
    return true; // Indicate success
  }
}

// Read next coordinate from .thr file
bool readNextThrCoordinate(Positions* coord) {
  if (!thrFile || !thrFile.available()) {
    return false; // End of file
  }
  
  String line = thrFile.readStringUntil('\n');
  line.trim(); // Remove whitespace
  
  Serial.print("Read line: ");
  Serial.println(line);
  
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
#endif

//////////////////////////////////////////////////
//  SERIAL STREAMING FUNCTIONALITY //
//////////////////////////////////////////////////
#if ENABLE_STREAMING

// Parse serial input with start and end markers
char* receivedWithStartEndMarkers() {
    static boolean recvInProgress = false;
    static bool newData = false;
    static byte ndx = 0;

    char startMarker = '<';
    char endMarker = '>';
    char rc;
    newData = false;
    
    while (Serial.available() > 0 && newData == false) {
        rc = Serial.read();
        if (recvInProgress == true) {
            if (rc != endMarker) {
                receivedString[ndx] = rc;
                ndx++;
                if (ndx >= numCharsSerialBuffer) {
                    ndx = numCharsSerialBuffer - 1;
                }
            } else {
                receivedString[ndx] = '\0'; // terminate the string
                recvInProgress = false;
                ndx = 0;
                newData = true;
            }
        } else if (rc == startMarker) {
            recvInProgress = true;
        }
    }
    
    if (newData) {
        lastSerialTime = millis(); // Update last serial activity time
        return receivedString;
    } else {
        return nullptr;
    }
}

// Wait for specific serial response
void waitUntilReceived(String s) {
    while (s.compareTo(String(receivedString)) != 0) {
        char* receivedString = receivedWithStartEndMarkers();
        if (receivedString == nullptr) {
            delay(10); // Small delay to prevent busy waiting
        }
    }
}

// Initialize serial streaming handshake
bool initializeSerialStreaming() {
    if (streamingInitialized) {
        return true;
    }
    
    Serial.println("COORDS");
    waitingForStart = true;
    streamingInitialized = true;
    lastSerialTime = millis();
    
    return true; // Initialization started successfully, waiting handled in main loop
}

bool checkForStartResponse() {
    if (!waitingForStart) {
        return false;
    }
    
    char* received = receivedWithStartEndMarkers();
    if (received != nullptr && String(received).compareTo("START") == 0) {
        Serial.println("READY");
        waitingForStart = false;
        streamingActive = true;
        return true;
    }
    
    return false;
}

// Read next coordinate from serial stream
bool readNextStreamingCoordinate(Positions* coord) {
    if (!streamingActive) {
        return false;
    }
    
    char* received = receivedWithStartEndMarkers();
    if (received != nullptr) {
        // Check for explicit end signal
        if (strncmp(received, "END", 3) == 0) {
            Serial.println("END_RECEIVED");
            Plotter.patternEnded = true;
            return false; // Signal pattern completion
        }
        // Check if receivedString contains "POS:"
        else if (strncmp(received, "POS:", 4) == 0) {
            int r = 0, theta = 0;
            if (sscanf(received + 4, "%d,%d", &r, &theta) == 2) {
                // Echo back the coordinate message (confirmation of receipt)
                Serial.println(received);
                
                // Increment point index for streaming
                Plotter.pointIndex++;
                
                // Reset doneSent flag for new coordinate
                Plotter.doneSent = false;
                
                coord->radial = (uint16_t)r;
                coord->angular = (uint16_t)theta;
                return true;
            }
        }
    }
    
    return false;
}

// Check for streaming timeout (only when robot is waiting for next coordinate)
bool checkStreamingTimeout() {
    if (!streamingActive) {
        return false;
    }
    
    // Only check timeout when robot is waiting for next coordinate
    // (i.e., when we're actively trying to read the next coordinate)
    if (millis() - lastSerialTime > SERIAL_TIMEOUT) {
        Serial.println("STREAMING_TIMEOUT");
        streamingActive = false;
        return true; // Timeout occurred
    }
    
    return false; // No timeout
}

#else
// Stub functions when streaming is disabled (only those called outside #if ENABLE_STREAMING blocks)

bool checkStreamingTimeout() {
    return false;
}
#endif


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
      
      // Special handling for STREAMING mode - skip from position 0 to size selector
      if (patternMode == STREAMING && UI.cursorPos == 0) {
        UI.cursorPos = NUM_PATTERNS;  // Skip directly to size selector
      } else {
        UI.cursorPos += 1;               // cycle thru menu
      }
      UI.cursorPos %= NUM_PATTERNS + 2;    // wrap back after the draw settings
    }

    // check for choose/pause (green) button press and release
    if(greenButton.isPressed()){
      UI.pWasPressed = true;
    }

    if(greenButton.isReleased() && UI.pWasPressed){
      // if hit choose on size icon
      if(UI.cursorPos == NUM_PATTERNS){   // if size preset selection
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
      
      // if hit choose on pattern mode icon
      else if(UI.cursorPos == NUM_PATTERNS + 1){     // pattern mode selection
        pixels.clear();        

        // Cycle through enabled pattern modes only
        do {
          patternMode = (patternMode + 1) % (sizeof(patternModeEnabled) / sizeof(patternModeEnabled[0]));  // Cycle through all available modes
        } while (!patternModeEnabled[patternMode]);  // Keep cycling until we find an enabled mode
        updatePatternModeDisplay();
      } 
      
      //otherwise, we've hit choose on a file number or pattern number
      else {
        UI.pWasPressed = false;     // ensure not paused
        
        // Try to initialize system for selected file
        if (!initializeSystem()) {
          // System initialization failed, stay in selector mode
          Serial.println("Initialization failed, staying in selector");
          return; // Stay in UI loop
        }
        
        UI.selectMode = false;      // Exit UI loop with this flag

        for(uint8_t i = 0; i < 3; i++){         // flash selected pixel green
          pixels.clear();
          pixels.show();
          
          pixels.setPixelColor(UI.cursorPos + 2, pixels.Color(0, 255, 0));
          pixels.show();
          delay(300);
          
          pixels.clear();
          pixels.show();
          delay(300);
        }
        initializePatternSize();
      }
    }

    // Blink cursor LED with pattern mode color if over file/pattern
    if(UI.cursorPos < NUM_PATTERNS){
      pixels.clear();
      
      // Get pattern mode color and apply brightness scaling
      uint8_t r, g, b;
      getPatternModeColor(patternMode, &r, &g, &b);
      
      setLEDWithSineWave(UI.cursorPos + 2, r, g, b, 300.0);
      pixels.show();
    } 
    // change cursor LED's color to select sizes
    else if (UI.cursorPos == NUM_PATTERNS){
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
    // change cursor LED's color to select pattern mode
    else {
      pixels.clear();
      updatePatternModeDisplay();
    }
    
  }
}

// Get color for pattern mode (RGB values 0-255)
void getPatternModeColor(uint8_t mode, uint8_t* r, uint8_t* g, uint8_t* b) {
  if (mode < (sizeof(patternModeColors) / sizeof(patternModeColors[0]))) {
    *r = patternModeColors[mode][0];
    *g = patternModeColors[mode][1];
    *b = patternModeColors[mode][2];
  } else {
    *r = 0; *g = 0; *b = 0; // Black for unknown mode
  }
}

// Update pattern mode display LED
void updatePatternModeDisplay() {
  uint8_t r, g, b;
  getPatternModeColor(patternMode, &r, &g, &b);
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
  angleDeg = 90.0 - angleDeg;
  
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

// Cycle pen LED color for active drawing feedback
void cyclePenLEDColor() {
  if (Plotter.drawing && !UI.paused) {
    uint32_t color = PEN_COLOR_CYCLE[penColorCycleIndex];
    pixels.setPixelColor(PATTERN_SELECTOR_LED, color);
    pixels.show();
    penColorCycleIndex = (penColorCycleIndex + 1) % PEN_COLOR_CYCLE_COUNT;
  }
}

// Reset pen LED to normal state (off)
void resetPenLEDColor() {
  pixels.setPixelColor(PATTERN_SELECTOR_LED, pixels.Color(0, 0, 0)); // Off
  pixels.show();
}

// Show pulsing yellow LEDs on pattern LEDs 2-7
void showPulsingYellowLEDs() {
  int brightness = 255 * (0.1 + 0.9 * (0.5 + 0.5 * sin(millis() / 200.0)));
  for (uint8_t i = 2; i <= NUM_PATTERNS + 1; i++) {
    pixels.setPixelColor(i, pixels.Color(brightness, brightness, 0));
  }
  pixels.show();
}



// Handle pattern completion - lift pen and move to reveal position if enabled
void handlePatternCompletion() {
  // Lift pen when pattern finishes
  liftPen();
  
  // Move down if enabled, otherwise stop motors
  if (MoveDownOnCompletion) {
    setTargetFromPolar(REVEAL_POSITION_RADIAL, REVEAL_POSITION_ANGULAR);
    Serial.println("Moving down to completion position...");
  } else {
    // Stop motors only if not moving to completion position
    stopMotors();
  }
  
  Plotter.patternComplete = true;
  Plotter.drawing = false;  // Set drawing flag to false
  
#if ENABLE_STREAMING
  // Reset streaming state when pattern completes
  if (patternMode == STREAMING) {
    streamingActive = false;
    streamingInitialized = false;
    waitingForStart = false;
    lastSerialTime = 0;
  }
#endif
}


// Read next coordinate from file or PROGMEM
bool readNextCoordinate(Positions* coord) {
  switch (patternMode) {
    case EMBEDDED:
      // Use existing PROGMEM pattern reading
      return readNextPatternCoordinate(coord);

#if ENABLE_STREAMING
    case STREAMING:
      // Read from serial stream
      return readNextStreamingCoordinate(coord);
#endif

#if ENABLE_SD_CARD
    case SDCARD_1:
    case SDCARD_2:
    case SDCARD_3:
      // Read from .thr file
      return readNextThrCoordinate(coord);
#endif

    default:
      return false;
  }
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

  #if ENABLE_STREAMING
  Serial.begin(9600);
  #else
  Serial.begin(115200);
  #endif

  Serial.println("PLTTR-StringSketcher V1.0.1 ..... 10/07/2025");
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
  
  // Handle streaming initialization and waiting for START
  if (patternMode == STREAMING && waitingForStart && !UI.selectMode) {
#if ENABLE_STREAMING
    // Check for START response
    checkForStartResponse();
#endif    
    // Show pulsing yellow LEDs while waiting for START
    showPulsingYellowLEDs();
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
    
    // If in streaming mode and waiting for START, cancel listening
    if (patternMode == STREAMING && waitingForStart) {
      Serial.println("STREAMING_CANCELLED");
      streamingInitialized = false;
      waitingForStart = false;
      streamingActive = false;
      UI.selectMode = true;
    }
    
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

#if ENABLE_SD_CARD
  // Close .thr file if open
  if (thrFile) {
    thrFile.close();
  }
#endif

  // Lift pen before moving to home
  liftPen();

  uint32_t nextUpdate = 0;                     // for light animations
  uint8_t lightPos = pixels.numPixels();
  uint8_t fillPos = 0;

  UI.selectMode = true;
  UI.paused = false;
  Plotter.drawing = false;  // Reset drawing flag
  Plotter.WaitingForCoordinate = false;  // Reset waiting flag

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
  Plotter.drawing = true;  // Set drawing flag to true
  Plotter.patternEnded = false;  // Reset pattern ended flag
  Plotter.WaitingForCoordinate = false;  // Reset waiting flag
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
    } else if (Plotter.PenDownOnArrival && !Plotter.WaitingForCoordinate) {
      Pen.penPos = Pen.penDown;
      myPen.write(Pen.penPos);
      Plotter.PenDownOnArrival = false;
    }
    
    // Target reached - send DONE for streaming mode (only once per coordinate)
    if (patternMode == STREAMING && !Plotter.doneSent) {
      Serial.println("DONE");
      Plotter.doneSent = true;
    }

    // Declare currentPoint
    Positions currentPoint;

    bool cont = false;
    while (!cont) {
#if ENABLE_STREAMING
      // Check for streaming timeout before reading next coordinate
      if (patternMode == STREAMING && checkStreamingTimeout()) {
        // Timeout occurred - pattern complete
        Serial.println("Streaming timeout - Press red button to return to start");
        
        // Handle pattern completion for streaming timeout
        handlePatternCompletion();
        return;
      }
#endif
      
      // Read next coordinate (from file or PROGMEM)
      if (!readNextCoordinate(&currentPoint)) {
        // No coordinate available yet - check if we should wait or timeout
        if (patternMode == STREAMING) {
          // Check if pattern ended with END signal
          if (Plotter.patternEnded) {
            handlePatternCompletion();
            return;
          }
          // For streaming mode, check timeout before ending pattern
          if (checkStreamingTimeout()) {
            Serial.println("Streaming timeout - Press red button to return to start");
            handlePatternCompletion();
            return;
          }
          // No timeout yet - keep waiting for next coordinate
          Plotter.WaitingForCoordinate = true;
          return;
        } else {
          // For other modes, no more coordinates - pattern complete
          Serial.println("Pattern complete - Press red button to return to start");
          handlePatternCompletion();
          return;
        }
      }
      
      Plotter.WaitingForCoordinate = false;
      cont = true;

      // Check for duplicate coordinates (pen up signal)
      if (Plotter.pointIndex > 0 && currentPoint.radial == Plotter.previousPoint.radial && 
        currentPoint.angular == Plotter.previousPoint.angular) {
        cont = false;

        if (patternMode == STREAMING && !Plotter.doneSent) {
          Serial.println("DONE");
          Plotter.doneSent = true;
        }

        // Lift pen
        liftPen();
        Plotter.PenDownOnArrival = true;
      }
    }

    Plotter.previousPoint = currentPoint; // Store the point it's heading toward
    
    // Set target position from pattern coordinates
    setTargetFromPolar(currentPoint.radial, currentPoint.angular);
    
    // Debug output
    /*
    Serial.print("Point ");
    Serial.print(Plotter.pointIndex);
    Serial.print(": ");
    Serial.print(currentPoint.radial);
    Serial.print(" ");
    Serial.println(currentPoint.angular);
    */
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
  
  // Cycle pen LED color for active drawing feedback
  cyclePenLEDColor();
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

//////////////////////////////////////////////////
//  FUTURE IMPROVEMENTS //
//////////////////////////////////////////////////
/*
 * Future improvements that could be added to enhance the StringSketcher:
 * 
 * 1. Line Interpolation:
 *    - Implement drawLine() function similar to Voice2Sand.ino
 *    - Interpolate between pattern coordinates for smoother movement
 *    - Reduce jerkiness in all drawing modes (embedded, SD card, and streaming)
 *    - Improve drawing quality by adding intermediate points between coordinates
 * 
 * 2. Enhanced Serial Error Handling:
 *    - Add retry mechanisms for failed serial connections
 *    - Implement connection quality monitoring
 *    - Add automatic reconnection for dropped connections
 * 
 */