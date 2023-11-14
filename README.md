# ImmersiveGloves

I want to share how I am making vr gloves using 2 Tundra Trackers, 2 interface boards and 12 inertial sensors.

https://github.com/ras-marques/ImmersiveGloves/assets/6479742/3d3f5d54-22ea-44d6-a413-07e7460a17d0

Everything is in a prototyping phase, but currently I have finger curl and splay working on all 5 fingers, next development efforts are directed at implementing a gesture based joystick and button interface.

Why base these gloves around the Tundra Tracker? Using a Tundra Tracker and a development board avoids having to deal with batteries, since the Tundra Tracker can power everything. Also, the tracker takes care of transmiting the data from the gloves to the computer wirelessly. An RP2040 takes care of acquiring the data from the 6 inertial sensors, processing it and delivering it to the tracker.

This project is for people that already have a laser tracked VR setup with base stations and ideally for people using Tundra Trackers for full body tracking. In any case, if you're based in Europe, I can recommend a list of supplies:

- ~100€ (can't remember how much I paid for import taxes): 2 Tundra development boards https://tundra-labs.com/products/tundra-tracker-developer-board
- 144€: 12 BNO085 boards, the smallest I got were from Slime VR https://shop.slimevr.dev/products/slimevr-imu-module-bno085
- 459€: 3 Tundra Trackers + SW3 dongle https://unboundxr.eu/tundra-x3-bundle?sqr=tundra

If you don't have the Tundra Trackers yet, you could buy 2 bundles of individual tundra tracker and SW1 dongle https://unboundxr.eu/tundra-tracker?sqr=tundra + https://unboundxr.eu/tundra-sw1-dongle, but that would be ~400€ and you will need 2 USB ports, or you could go for the 4 Tundra Tracker bundle https://unboundxr.eu/tundra-x4-bundel?sqr=tundra for ~600€. On top of this, if you don't have base stations, those go for 159€ from valve and it is better to have 2.

## End goal for these gloves

- These will always use the Tundra tracker unless another lighthouse based tracker is available that can send inputs to openvr like the Tundra can. I expect this to always be the largest part of the cost.
- The inertial sensors are going to be next. Currently the gloves use the Slime VR BNO085 boards that go for 12€ each. Even in bulk, these don't get much cheaper, but I will support two additional versions of the gloves in the future, one with 11 BNO085 boards instead of 6 per glove and another with 6 cheaper IMU boards that I expect to be approximately half the cost but that may or may not include a magnetometer to correct for drift, needing a callibration gesture/button press from time to time.
- I have already designed smaller BNO085 boards compatible with SparkFun's Qwiic Connect System but haven't ordered or tested anything yet. There are boards for the finger tip and finger base (digital phalanx and proximal phalanx bones).
- I am also designing a custom interface board for the Tundra Tracker so it is easier to assemble the glove without soldering and for good looks. This should be much cheaper if you want to make them yourself than the ones from Tundra.
- My gloves have finger mobility and a low profile as top priorities, so I have no plans for implementing physical buttons unless absolutelly necessary nor force feedback, but vibration haptics are possible if small enough. I do encourage you to do so and share your own variation!
- Oh and maybe the most endest of the goals is to have fun in the development process!!

## Steps to make the gloves:

### 3D Print the mount points for the tracker
- There are 3D printing files available in the 3DPrintingFiles folder of this repo. This is not a one size fits all kind of mount point. I measured the curvature of the back of my hand with a 3D printed matrix of screw inserts, so it fits me perfectly. I will show my process in the future so you can do the same if you want.

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

### Drivers
- We are using the OpenGloves driver so we can easily get inputs working with all apps that support the Index Controllers, which are most of them. So download it from steam: https://store.steampowered.com/app/1574050/OpenGloves/
- Copy the contents of driverProject/immersive_gloves_controller/build/immersive_gloves_controller in this repo to C:\Program Files (x86)\Steam\steamapps\common\SteamVR\drivers or wherever you have your SteamVR app installed.
- Be sure to enable both opengloves and immersive_gloves_controller addon by going to SteamVR settings, Startup/Shutdown tab and clicking Manage SteamVR Add-Ons. These two should be toggled on. Everytime SteamVR shuts down due to an error, it is likely one or both will be toggled off, so check this if something is not working.
- You should calibrate the rotation and position offsets on the OpenGloves configuration panel. I did that manually since I don't have buttons on the glove yet to trigger the callibration interface. Pre-calibrated files may be available in the future with instructions on how to use them.

## Important resources that made this project possible
 
Tundra Labs have a repo with some documentation on how to use their development board https://github.com/tundra-labs/rp2040_examples

CircuitPython allowed me to easily get the inertial sensors working without me having to program everything from scratch, you can learn more about this here https://learn.adafruit.com/adafruit-9-dof-orientation-imu-fusion-breakout-bno085/python-circuitpython

Finally Functional's Open VR Driver Tutorial that was a great introduction on how OpenVR inputs work https://www.youtube.com/watch?v=LzEIOBnbC8k

OpenVR repository has lots of documentation, it takes a while to figure out the parts that are important for this project, but it's worth the effort https://github.com/ValveSoftware/openvr

[danwillm](https://github.com/danwillm) helped me get this thing started by pointing me in the right direction, telling me what technologies to use to speak with the tracker and openvr.
