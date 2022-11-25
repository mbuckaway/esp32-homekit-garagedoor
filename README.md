# ESP32 GarageDoor Controller for HomeKit

## What is it

ESP32 dedicated Garage Door controller. When Espressif, created their Homekit SDK for the ESP32, the time was ripe to move the garage door controller to a smaller platform. The ESP32 IDF and Homekit SDKs made incorporating homekit functionity into a sample platform easy.

For complete information on the project, please see the WIKI in the github repo: [WIKI](https://github.com/mbuckaway/esp32-homekit-garagedoor/wiki)

## How this Works

This device is based on the Espressif SDK and uses the open source version of the Espressif Homekit SDK. It uses my own network component. This network component uses BluFi to configure WIFI creds and control the device. You will need the Espressif EspBluFi app from the Apple Appstore.

The difference with this controller and others is it uses both a switch on the lower part of the door to sensor it's actually closed and a switch on the upper side of the door to sensor it's actually open. With only a close switch, one can only assume the garage door actually makes it all the way up to be open. The two switches allow us to add a motion sensor service to sensor when the door is in motion. I use that to turn the garage light automatically on when the door moves.

The code configures the following services that make up the garage door accessory:

- Garage Door
- Close Sensor
- Open Sensor
- Motion Sensor (sense when the door is in motion)
- Switch to force the door to move (used for automations to override security "issues")
- Switch to close the door when it is open (and only open). Homekit sometimes loses the door status and this control is required for automation to work reliably.

What is missing from this unit is the obstruction detection. At some point, I may add a eye beam to detect something is in the way, but the old garage door already has that as a safety features, so having your phone notify you of the obstruction seems moot.

The code uses a button library to monitor the door switches and automatically debounces the switches. Additionally, the routine the monitors the open and close switches updates Homekit with the changes in status, so the home app almost instantly. However, Homekit seems to have a bug in it where if you control the door without using the Home App, it loses track of the door status. The code updates the status correctly, but Homekit seems to ignore it. That said, it does correct itself when the door is fully closed.

For my device, I included two LED's connected to the switches through resistors. There is no GPIO for these and they are completely optional. I had issues with the door switches not working. During the winter, ice built up on the bottom of the door, and the close switch needed to be repositioned. Include them if you want, but they are not required.

See the WIKI for information on building the device, the circuit, etc.

## Photos

Below are some of the photos from the device I created.

![Garage Door Controller in 3D Printed Case](images/garage1.jpg)
![Lower magnetic door switch](images/garage2.jpg)
![Upper roller switch](images/garage3.jpg)
