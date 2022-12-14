# Emma - Homerton IDP
Group M108 (Steve the porridge bot)
## Structure
 ### Main Program
 - `main.ino` is the main program that calls the other modules
 - `src` contains the modules for each subsystem
    - `src\include` contains the header files
        - `src\include\util.h` contains the key constants, pin assignments and enums
 - `Wifi debug Python client` contains the PC code that connects to the arduino for debugging

### Other Programs
 - `LineSensitivityPlot` contains the code for calibrating the line sensor positions

## System Diagram
![System Diagram](images/system_diagram.png)
## Setup Instructions

First install:
 - Arduino IDE
 - VS Code (recommended) https://code.visualstudio.com/
 - Git

Then setup the required libraries:

1. In Arduino IDE, select Tools > Board > Board Manager. Install `Arduino megaAVR Boards`
2. Select Tools > Manage libraries. Install:
    - WifiNINA
    - Adafruit Motor Shield V2 Library
3. Close Arduino IDE, the rest can be done in VSCode.

Finally, setup VSCode:

4. Open VSCode. Goto Extensions (icon on the left). Install `Arduino`.
5. Once installed goto File > Preferences > Settings > Extensions > Arduino configuration > Arduino: Path. Find where Arduino IDE is installed on your computer and type the path here (e.g `C:\Program Files\Arduino`).
6. Restart VSCode
7. File > New Folder and create a folder for the project.
8. Terminal > new Terminal. Run `git clone https://github.com/Joe-Speers/IDP-Emma-Homerton.git` You should now see all the project files appear in the left hand pane. This will not work unless git is installed!
9. Open one of the .ino files. In the bottom pane on the right select the board as `Arduino Uno WiFi Rev2`. If using the Orange borad this will be different, i think it is just an `Arduino Uno`??
10. Again at the bottom, Select Programmer with whatever comes up (there should be only one option)
11. Connect the Arduino. Then `Select Serial Port`, again there should be one option.
12. At the top right there are icons for `Verify` and `Upload`. Click upload and check this is succesful.

To upload changes to the repository you can use the inbuilt git manager (`Source Control`) or use the terminal. Please add helpful commit messages!


