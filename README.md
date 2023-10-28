# ImmersiveGloves

I want to share how I am making vr gloves using 2 Tundra Trackers, 2 interface boards and 12 inertial sensors. These gloves do not have force feedback and probably will never have, but haptics are a possibility in the future.

https://github.com/ras-marques/ImmersiveGloves/assets/6479742/ab621e96-3ca8-48bc-adb8-41644092043d

Everything is in a prototyping phase, but I managed to get a trigger working in Pavlov and 4 fingers in Steam VR controller testing, so that's the state of the project.

Why base these gloves around the Tundra Tracker? Using a Tundra Tracker and a development board avoids having to deal with batteries, since the Tundra Tracker can power everything. Also, the tracker also takes care of transmiting the data from the gloves to the computer wirelessly. An RP2040 takes care of acquiring the data from the 6 inertial sensors, processing it and delivering it to the tracker.

This project is for people that already have a laser tracked VR setup with base stations and ideally for people using Tundra Trackers for full body tracking. In any case, if you're based in Europe, I can recommend a list of supplies:

- ~100€ (can't remember how much I paid for import taxes): 2 Tundra development boards https://tundra-labs.com/products/tundra-tracker-developer-board
- 144€: 12 BNO085 boards, the smallest I got were from Slime VR https://shop.slimevr.dev/products/slimevr-imu-module-bno085
- 459€: 3 Tundra Trackers + SW3 dongle https://unboundxr.eu/tundra-x3-bundle?sqr=tundra

If you don't have the Tundra Trackers yet, you could buy 2 bundles of individual tundra tracker and SW1 dongle https://unboundxr.eu/tundra-tracker?sqr=tundra + https://unboundxr.eu/tundra-sw1-dongle, but that would be ~400€ and you will need 2 USB ports, or you could go for the 4 Tundra Tracker bundle https://unboundxr.eu/tundra-x4-bundel?sqr=tundra for ~600€. On top of this, if you don't have base stations, those go for 159€ from valve and it is better to have 2.

I have already designed smaller BNO085 boards compatible with SparkFun's Qwiic Connect System, but I haven't ordered them yet, if you do, do it at your own risk and tell me if it works :) There are boards for the finger tip and finger base (digital phalanx and proximal phalanx bones).

I am also designing a custom interface board for the Tundra Tracker, but I am still unsure if I can get the data from the Tundra Tracker into OpenVR or if I need to use an ESP and OpenGloves (https://github.com/LucidVR/opengloves-driver) for processing inputs. Anyway, I don't expect cost savings from designing my own boards, but I do this for smaller IMU boards on the finger tips, no solder assembly and good looks.

## Steps to make the gloves:

### Prepare the Tundra Tracker
- Download the files from this repository.
- Open folder {Steam}\steamapps\common\SteamVR\tools\lighthouse\bin\win32 ({Steam} is probably C:\Program Files (x86)\Steam on your system, but who knows).
- Copy "ImmersiveGlovesLeft.json" and "ImmersiveGlovesRight.json" from this repository into this folder.
- Disconnect every vr device you have from your computer, including the dongles that receive data from the trackers. Using a USB cable, connect just one Tundra Tracker to your computer.
- Write "cmd" on the address bar to open a Command Line inside {Steam}\steamapps\common\SteamVR\tools\lighthouse\bin\win32.
- Write "lighthouse_console.exe" on the Command Line and press return.
- You should see only one device connected, if there are more, make sure to disconnect them, this is **important**. Write "downloadconfig backup.json" to store a backup of the Tundra Tracker config file that is stored inside the tracker.
- Store the config of "ImmersiveGlovesRight.json" to the tracker with "uploadconfig ImmersiveGlovesRight.json" from the command line. This will be the tracker for your right glove.
- Disconnect the tracker and connect another one.
- Store the config of "ImmersiveGlovesLeft.json" to the tracker with "uploadconfig ImmersiveGlovesLeft.json" from the command line. This will be the tracker for your left glove.
- Finally install the ImmersiveGloves driver by copying the folder driverProject/immersive_gloves_controller/build/immersive_gloves_controller on the repository to {Steam}\steamapps\common\SteamVR\drivers\

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
