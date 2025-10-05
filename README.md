# StringSketcher
A hack for Mark Rober's Crunchlabs StringPlotter Hackpack that allows you to sketch patterns using predefined coordinate sequences instead of processing bitmap images pixel by pixel.

This approach can be substantially faster and uses less memory than the original bitmap-processing version. It also does not utilize the SD card for images, but stores these directly in the .ino file. It's not going to work for every image, but patterns that work well are geometric designs, logos, simple line drawings, or other patterns that can be represented as a series of connected points. The robot will trace these patterns by moving between coordinate points in sequence. Images can be converted into the needed format for drawing using the linked website.

## Summary
To implement this hack yourself, you'll need to follow these two steps:

1. Generate coordinates using the Image2Sand website
2. Update the code for the Arduino by pasting in coordinates in the output format supported by Part (1)

## Part 1 - Generate Coordinates
*Note: If you just want to try out the included demo patterns, jump ahead to Part 2.*

The website is available at: https://orionwc.github.io/Image2Sand/

### Steps:
* Upload an image or generate one using AI
* Click Generate Coordinates to process the image
* **Important**: Check the "Repeat Coordinates on PenUp" checkbox - this will ensure the pen is lifted off the whiteboard when moving between contours.
* Copy the coordinates from the textbox at the bottom of the page
* For more information on the settings and what they mean, visit https://github.com/orionwc/Image2Sand

## Part 2 - Arduino Implementation
 Adds a pattern to the Arduino code that traces a sequence of points specified as an array of polar coordinates in the units: r=radial, scaled from 0-1000, and theta=angle in tengths of a degree
* You can find the code in StringSketcher.ino. You can either copy and paste in the full code into the HackPack IDE (on Level 3) over the StockCode, or you can see the specific changes to make and include them individually.
* To draw your own image, paste in the coordinates you copied from the website (Part 1) into one of the Patterns 1-6 at the top of the code. Be sure to comment out any other patterns there by default by inserting // in front of any other coordinates so only one set of coordinates is used for each pattern. If this is your first time trying a custom pattern, I recommend pasting new patterns into Pattern 6, replacing the large pattern in there by default. Start with patterns of fewer than 200 points, but you can explore the limitations.
* Compile the code and deploy to the Arduino.

## Running the Pattern
 Once you've updated the code, the pattern will be ready to draw. **Important**: you have to select the special pattern pen mode by pressing the green button when the pen selector LED is highlighted - you need the mode indicated by the Cyan color. You can make this the default by setting PenStyle to 4 in config.h. Now use the red button to scroll through the pattern numbers and the green button to select one. Once the pattern has finished drawing, the LEDs will flash red and the robot will stop moving. Pressing the red button will return the robot to the initialization position, ready for the next pattern. It's advisable to check that the robot returned accurately to the initialization position and adjust the strings manually if needed before starting another pattern.

## Known bugs:
* The robot runs the motors to get from one point to the next at a roughly contant speed, which can be different from each other to ensure the line is smooth. However, this doesn't mean the line will be straight. The curved path taken between points can result in some abnormalities in the image, more noticable when the image is expected to be composed of straight lines.
* Sometimes the compiler will compile successfully, but it will not draw -- this usually happens when the number of points across all the stored images is very large. If you observe this, try decreasing the number of points, or replacing the patterns you're not interested in with {0,0}.
* The website that generates the image doesn't automatically move the robot away from the image once it has finished drawing, but it may be desirable to reveal the image. A workaround is to ensure the last coordinate in the pattern is duplicated (which will cause the pen to lift), then add a coordinate at the bottom of the image area - {1200, 2700} works well.
* The pen is placed down (instead of up) when the image finishes drawing.

## Possible future improvements:
* Remove the need to select the PenStyle when drawing
* Clean up code to make it simpler just for drawing pattern mode, or simultaneously enable bitmap support with other PenStyle options (this latter option was disabled due to memory constraints).
* Fix known bugs

## More info
See this video: [Dot Matrix to DaVinci: I hacked Mark Rober's String Plotter to sketch images in seconds](https://youtu.be/Ba969sJ4UOo)
