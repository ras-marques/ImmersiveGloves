# ImmersiveGloves

I want to share how I am making vr gloves using 2 Tundra Trackers, 2 interface boards and 12 inertial sensors.

https://github.com/ras-marques/ImmersiveGloves/assets/6479742/4af1af61-4261-44ce-aa3d-e45de0df7709

Check the full presentation on youtube https://www.youtube.com/watch?v=5OSYiYDkOE8

Hand tracking works with curl and splay on all fingers with 1 IMU on each finger. There is a joystick that is enabled by touching the thumb to the index finger and B, A and System buttons are available by touching the thumb to the middle, ring and pinky fingers respectively.

## Overall repository state

Instructions are incomplete.
The files for the main board and IMU boards are updated.
There is only code for the right glove.
3D printing files need tweaking.

## Necessary equipment and skills

 - Bare PCBs: https://github.com/ras-marques/ImmersiveGloves/tree/main/CustomPCBs/ReadyToTest
 - Components for populating the PCBs: https://github.com/ras-marques/ImmersiveGloves/blob/main/CustomPCBs/cost_analysis.ods
 - Solder paste
 - Hot air gun or hot plate
 - Soldering iron
 - Hot glue gun or super glue
 - 3d printer
 - Pre-crimped JST 1.0mm silicon wire kit: https://www.amazon.es/gp/product/B07PDQKHJ2/ref=ppx_yo_dt_b_asin_title_o00_s00?ie=UTF8&psc=1

The hardest part in this project is making the boards. I am currently not supplying boards, so you will need to order them from somewhere.

The boards are 4 layer and have fine pitch components. Making the PCBs at home is out of the question, but you can order them from JLCPCB, PCBWAY, etc.

I assembled the components on my glove by hand using a stencil to dispense solder paste and used a hot air gun for reflowing. A hot plate is better.

If you don't have a 3d printer, you could order the models from somewhere or ask a friend.

Currently everything is glued to the glove fabric, you could use hot glue or super glue.

The pre-crimped JST wires are necessary to make the assembly process easier, more below.

## Cost analysis

A realistic ballpark for cost is around 250€ per pair of gloves with component and shipping cost and assuming you already have all the tools like a 3d printer, and soldering tools.

You can refer to https://github.com/ras-marques/ImmersiveGloves/blob/main/CustomPCBs/cost_analysis.ods for the complete BOM, but in summary the current cost is described below:
- 4€/pair times 50 = 200€: Cost to make enough boards for exactly 50 pairs of these gloves
- 170€/pair: Cost to source enough components and some spares for a pair of these gloves
- 130€/pair times 50 = 6500€: Cost to source enough compoents for 50 pairs of these gloves

## Tracking method

Why base these gloves around the Tundra Tracker? Using a Tundra Tracker and an expansion board avoids having to deal with batteries, since the Tundra Tracker can power everything. Also, the tracker takes care of transmiting the data from the gloves to the computer wirelessly. Two RP2040's take care of acquiring the data from the 6 inertial sensors, processing it and delivering it to the tracker.

This project is for people that already have a laser tracked VR setup with base stations and ideally for people using Tundra Trackers for full body tracking. That said, if you don't have the Tundra Trackers yet, there are some options:
- 400€: 2 bundles of individual tundra tracker and SW1 dongle (2 USB ports needed) https://unboundxr.eu/tundra-tracker?sqr=tundra + https://unboundxr.eu/tundra-sw1-dongle
- 459€: 3 Tundra Tracker bundle (1 USB port needed) https://unboundxr.eu/tundra-x3-bundle
- 609€: 4 Tundra Tracker bundle (1 USB port needed) https://unboundxr.eu/tundra-x4-bundel

On top of this, if you don't have base stations, those go for 159€ from valve and it is better to have 2.

## End goal for these gloves

- The Tundra Tracker is amazing, it's the smaller tracker available and it even supports sending sensor data to the computer. This is the ultimate tracker for these gloves, but I want to make these more accessible for everyone, so I will want to support more tracking methods, begining with using your available controllers just like LucidVR does. This will probably imply a redesign of the main board to make it possible to power it using an external battery.
- I started this project with the Slime VR BNO085 boards and have since designed a custom BNO085 that is as small as I could make it. I didn't even include a JST connector and wires are soldered directly to it. To make it easier to assemble though, I am considering including the connector in my next IMU boards. I also want to explore using a cheaper IMU chip like the BMI270 with and without a magnetometer.
- My gloves have finger mobility and a low profile as top priorities, so I have no plans for implementing physical buttons unless absolutelly necessary nor force feedback, but vibration haptics are possible if small enough. I do encourage you to do so and share your own version of these gloves!

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
