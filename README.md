# StringSketcher
A hack for Mark Rober's Crunchlabs StringPlotter Hackpack that allows you to sketch patterns using predefined coordinate sequences instead of processing bitmap images pixel by pixel.

This approach can be substantially faster and uses less memory than the original bitmap-processing version. The robot will trace these patterns by moving between coordinate points in sequence. Images can be converted into the needed format for drawing using the linked website.

## Summary
To implement this hack yourself, you'll need to follow these two steps:

1. Generate coordinates using the Image2Sand website
2. Update the code for the Arduino by pasting in coordinates either directly into the .ino file, or into a .thr file on the SD card

## Part 1 - Generate Coordinates
*Note: If you just want to try out the included demo patterns, jump ahead to Part 2.*

The website is available at: https://orionwc.github.io/Image2Sand/

### Steps:
* Upload an image or generate one using AI
* **Important**: Check the "Repeat Coordinates on PenUp" checkbox - this will ensure the pen is lifted off the whiteboard when moving between contours.
* Use the Default output type to embed the patterns directly in the .ino file or the Theta-Rho format if you're using the SD card. If this is your first time trying this hack, start by embedding the pattern directly in the .ino file using the Default output format.
* Click Generate Coordinates to process the image
* Copy the coordinates from the textbox at the bottom of the page
* For more information on the settings and what they mean, visit https://github.com/orionwc/Image2Sand

## Part 2 - Arduino Implementation - Embedding Patterns in the code
Adds a pattern to the Arduino code that traces a sequence of points specified as an array of polar coordinates in the units: r=radial, scaled from 0-1000, and theta=angle in tenths of a degree.

* You can find the code in StringSketcher.ino. You can either copy and paste in the full code into the HackPack IDE (on Level 3) over the StockCode, or you can see the specific changes to make and include them individually.
* To draw your own image, paste in the coordinates you copied from the website (Part 1) into one of the Patterns 1-6 at the top of the code. Be sure to comment out any other patterns there by default by inserting // in front of any other coordinates so only one set of coordinates is used for each pattern. If this is your first time trying a custom pattern, I recommend pasting new patterns into Pattern 6, replacing the large pattern in there by default. Start with patterns of fewer than 200 points, but you can explore the limitations.
* Compile the code and deploy to the Arduino.

### File Format for Embedded Patterns
- Format: `{radial, angular}` where radial is 0-1000 and angular is degrees × 10
- Example: `{800, 0}` represents a point at radius 800, angle 0°
- Stored directly in the Arduino code

## Running the Pattern
Once you've updated the code, the pattern will be ready to draw. **Important**: you have to select the special pattern pen mode by pressing the green button when the pen selector LED is highlighted - you need the mode indicated by the White color (EMBEDDED mode). Now use the red button to scroll through the pattern numbers and the green button to select one. Once the pattern has finished drawing, the LEDs will flash red and the robot will stop moving. Pressing the red button will return the robot to the initialization position, ready for the next pattern. It's advisable to check that the robot returned accurately to the initialization position and adjust the strings manually if needed before starting another pattern.

## Enabling more and more detailed patterns using the SD Card

SD card support is disabled by default, but you can turn it on to include larger patterns.
Note that enabling SD card mode will require additional memory for the Arduino script and this will limit the size of patterns that can be embedded directly in the script. If you run into issues with the sketch beign too big to compile, you'll need to replace the some or all of the embedded patterns with {0,0} empty patterns.

### Code changes
* In the StringSketcher.ino code, change EnableSDCard to true.
* Replace the embedded patterns 3-5 with {0,0}. 6 will be disabled automatically, and if 1-2 are unchanged they should still fit in memory.

### SD Card Pattern File Format: Theta-Rho
- This is a supported output format in the Image2Sand website. Select this before clicking the Generate Coordinates button to output to this format.
- Format: `theta rho` where theta is in radians and rho is normalized 0-1
- Example: `0 0.5` represents a point at angle 0 radians, radius 0.5
- Copy this output into a text file and name it 1.thr, 2.thr, ..., 18.thr
- Place the file in the root directly of the SD card

### Interface changes
- The pen LED can now be used to cycle through the different modes
- **EMBEDDED mode (White)**: Uses built-in PROGMEM patterns (patterns 1-6)
- **SDCARD_1 mode (Red)**: Reads .thr files 1-6 from SD card
- **SDCARD_2 mode (Green)**: Reads .thr files 7-12 from SD card  
- **SDCARD_3 mode (Blue)**: Reads .thr files 13-18 from SD card
- After selecting the appropriate pen LED, you can proceed to select the pattern number.
-- If you selected white, it will run the embedded patterns 1-6
-- If you selected red and then pattern number 1, it will read 1.thr from the SD card
-- If you selected red and then pattern number 4, it will read 4.thr from the SD card
-- If you selected green and then pattern number 2, it will read 8.thr from the SD card, i.e. filename: (6 + number).thr
-- If you selected blue, then pattern number 5, it will read 17.thr from the SD card, i.e. filename: (12 + number).thr

## Known bugs:
* The robot runs the motors to get from one point to the next at a roughly constant speed, which can be different from each other to ensure the line is smooth. However, this doesn't mean the line will be straight. The curved path taken between points can result in some abnormalities in the image, more noticeable when the image is expected to be composed of straight lines.
* Sometimes the compiler will compile successfully, but it will not draw -- this usually happens when the number of points across all the stored images is very large. If you observe this, try decreasing the number of points, or replacing the patterns you're not interested in with {0,0}.
* The website that generates the image doesn't automatically move the robot away from the image once it has finished drawing, but it may be desirable to reveal the image. A workaround is to ensure the last coordinate in the pattern is duplicated (which will cause the pen to lift), then add a coordinate at the bottom of the image area - {1200, 2700} works well.
* The pen is placed down (instead of up) when the image finishes drawing.

## Possible future improvements:
* Remove the need to select the PenStyle when drawing
* Clean up code to make it simpler just for drawing pattern mode, or simultaneously enable bitmap support with other PenStyle options (this latter option was disabled due to memory constraints).
* Fix known bugs

## More info
See this video: [Dot Matrix to DaVinci: I hacked Mark Rober's String Plotter to sketch images in seconds](https://youtu.be/Ba969sJ4UOo)