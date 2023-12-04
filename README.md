# ImmersiveGloves

I want to share how I am making vr gloves using 2 Tundra Trackers, 2 interface boards and 12 inertial sensors.

https://github.com/ras-marques/ImmersiveGloves/assets/6479742/3d3f5d54-22ea-44d6-a413-07e7460a17d0

Everything is in a prototyping phase, but currently I have finger curl and splay working on all 5 fingers and an emulated joystick from thumb rotation. Now moving to custom boards that will make everything smaller, prettier and more robust.

Why base these gloves around the Tundra Tracker? Using a Tundra Tracker and an expansion board avoids having to deal with batteries, since the Tundra Tracker can power everything. Also, the tracker takes care of transmiting the data from the gloves to the computer wirelessly. An RP2040 takes care of acquiring the data from the 6 inertial sensors, processing it and delivering it to the tracker. I'm prototyping tiny IMU boards based around the BNO085 chip, one for each finger, and an expansion board for the Tundra Tracker. Total component cost for the electronics, not counting with the Tundra Tracker, should be around 200€ for a pair of gloves.

This project is for people that already have a laser tracked VR setup with base stations and ideally for people using Tundra Trackers for full body tracking. That said, if you don't have the Tundra Trackers yet, there are some options:
- 400€: 2 bundles of individual tundra tracker and SW1 dongle (2 USB ports needed) https://unboundxr.eu/tundra-tracker?sqr=tundra + https://unboundxr.eu/tundra-sw1-dongle
- 459€: 3 Tundra Tracker bundle (1 USB port needed) https://unboundxr.eu/tundra-x3-bundle
- 609€: 4 Tundra Tracker bundle (1 USB port needed) https://unboundxr.eu/tundra-x4-bundel

On top of this, if you don't have base stations, those go for 159€ from valve and it is better to have 2.

## End goal for these gloves

- ~~These will always use the Tundra Tracker unless another lighthouse based tracker is available that can send inputs to openvr like the Tundra can. I expect this to always be the largest part of the cost.~~
- The Tundra Tracker is amazing, it's the smaller tracker available and it even supports sending sensor data to the computer. This is the intended way of using these gloves, but I want to make these more accessible for everyone, so I will want to support more tracking methods such as my own tracking method - looking at something similar to the original PSVR controller but with several cheap cameras mounted on the walls/shelves - or even cheaper, using your available controllers.
- The inertial sensors are going to be next. We have used the Slime VR BNO085 boards that go for 12€ each, but I have designed a custom BNO085 that is smaller and am looking to designing an alternative IMU board that is cheaper based around the BMI270 and an added magnetometer.
- My gloves have finger mobility and a low profile as top priorities, so I have no plans for implementing physical buttons unless absolutelly necessary nor force feedback, but vibration haptics are possible if small enough. I do encourage you to do so and share your own variation!

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

### Flash the RP2040 board (files not yet available!)
- Download the latest stable version of circuitpython version from here https://circuitpython.org/board/raspberry_pi_pico/
- Connect your expansion board to the Tundra Tracker and then to your PC using the included USB board connected to USB1.
- With the Tracker OFF, press the button on the USB board and while it is pressed, power up the Tundra Tracker, this will let you put the RP2040 in the expansion board in bootloader mode.
- A window should appear on your PC showing the new drive that is emulated by the RP2040. Copy the circuitpython file you downloaded earlier into this folder. The RP2040 will reboot automatically.
- Like before, a new window should appear on your PC showing another drive but now with a different structure. This is where you will put your program that processes the sensor data and sends it to the computer. Just copy the contents of the rp2040Firmware/usb1 folder into the root of this new folder that appeared.
- Repeat the steps above for the second RP2040 mcu by using USB2 and the files in rp2040Firmware/usb2

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

## Contributing workflow

Here’s how I suggest you go about proposing a change to this project:

1. [Fork this project][fork] to your account.
2. [Create a branch][branch] for the change you intend to make.
3. Make your changes to your fork.
4. [Send a pull request][pr] from your fork’s branch to our `main` branch.

Using the web-based interface to make changes is fine too, and will help you by automatically forking the project and prompting to send a pull request too.

[fork]: https://help.github.com/articles/fork-a-repo/
[branch]: https://help.github.com/articles/creating-and-deleting-branches-within-your-repository
[pr]: https://help.github.com/articles/using-pull-requests/
