# ImmersiveGloves

I want to share how I am making vr gloves using a Tundra Tracker, an interface board and some inertial sensors. These gloves do not have force feedback but haptics are a possibility.

[![Video](https://img.youtube.com/vi/IRRQcPaeXkk/maxresdefault.jpg)](https://www.youtube.com/watch?v=IRRQcPaeXkk

A DIY solution could be achievable for around 100 EURO/USD for each glove if you already have the Tundra Tracker and considering the simplest version of the glove with 6 inertial sensors.
In the future I want to get the gloves working with 5 more sensors, so 11 on each hand, but some more software development will be needed.

Everything is in a prototyping phase, but I managed to get a trigger working in Pavlov and 4 fingers in Steam VR controller testing, so that's the state of the project.

Using a Tundra Tracker and a development board avoids having to deal with batteries, since the Tundra Tracker can power everything. Also, the tracker also takes care of transmiting the data from the gloves to the computer wirelessly. An RP2040 takes care of acquiring the data from the 6 inertial sensors, processing it and delivering it to the tracker.

What you need to get a pair of gloves working:
Windows 10 PC - Windows 11 probably works, never tried VR on Linux
2 Tundra Trackers
2 Tundra development boards
12 BNO085 boards, the smallest I got were from Slime VR

## Steps to make the gloves:

### Prepare the Tundra Tracker
- Disconnect every vr device you have from your computer and using a USB cable, connect just one Tundra Tracker to your computer.
- Go to folder {Steam}\steamapps\common\SteamVR\tools\lighthouse\bin\win32 ({Steam} is probably C:\Program Files (x86)\Steam on your system, but who knows) and write "cmd" on the address bar to open a Command Line in that folder.
- Write "lighthouse_console.exe" on the Command Line and press return.
- You should see only one device connected, if there are more, make sure to disconnect them, this is **important**. Write "downloadconfig backup.json" to store a backup of the Tundra Tracker config file that is stored inside the tracker.
- Open the file "TundraTrackerJSON/ImmersiveGloves.json" from this repository and the "backup.json" file from before. Copy the "device_serial_number" line from the "backup.json" file to "ImmersiveGloves.json" and save it.
- Store the config of "ImmersiveGloves.json" to the tracker with "uploadconfig ImmersiveGloves.json" from the command line.
- Finally install the bindings and configuration files for the new repurposed Tundra Tracker for the gloves in SteamVR, by copying the folder tmi_hand_controller on the repository to {Steam}\steamapps\common\SteamVR\drivers\

### Flash the RP2040 board
- Download the latest stable version of circuitpython version from here https://circuitpython.org/board/raspberry_pi_pico/
- Connect your tundra development board to the Tundra Tracker and then to your PC using the included USB board.
- With the Tracker OFF, press the button on the USB board and while it is pressed, power up the Tundra Tracker, this will let you put the RP2040 in the development board in bootloader mode.
- A window should appear on your PC showing the new drive that is emulated by the RP2040. Copy the circuitpython file you downloaded earlier into this folder. The RP2040 will reboot automatically.
- Like before, a new window should appear on your PC showing another drive but now with a different structure. This is where you will put your program that processes the sensor data and sends it to the computer. Just copy the contents of the rp2040Firmware folder into the root of this new folder that appeared.

## Important resources that made this project possible
 
Tundra Labs have a repo with some documentation on how to use their development board https://github.com/tundra-labs/rp2040_examples

CircuitPython allowed me to easily get the inertial sensors working without me having to program everything from scratch, you can learn more about this here https://learn.adafruit.com/adafruit-9-dof-orientation-imu-fusion-breakout-bno085/python-circuitpython

Finally Functional's Open VR Driver Tutorial that was a great introduction on how OpenVR inputs work https://www.youtube.com/watch?v=LzEIOBnbC8k

OpenVR repository has lots of documentation, it takes a while to figure out the parts that are important for this project, but it's worth the effort https://github.com/ValveSoftware/openvr