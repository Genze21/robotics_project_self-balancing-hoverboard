# RBT Self-balancing hoverboard

The following hardware is expected
- AT32F403RCT7 
- 350w brushless hub 
- ST-LINK V2 to control to motor speeds independently,
which is important when trying to balance the robot.
- Arduino Nano 
- BNO055 
- Flysky i6 remote

Please download and install the firmware for the hoverboard based on the instructions from: 
https://github.com/someone42/hoverboard-firmware-hack

The use of an Arduino Nano is expcted
The Arduino can be connected via usb to a computer and the code could be uploaded via the use of the **Arduino IDE**

Wire the Arduino to motor controls of the AT32F403RCT7 acording to the schemtic from the aforementioned github page
Wire the BNO055 to the Arduino on pins 6 & 9
Wire the RC remote to the Arduino on pins 3 & 5

If everything is wired correctly start the hoverboard up and it should be controlable via the remote control and try to balance itself.

Demo: https://youtu.be/Ii-0nm4FswQ