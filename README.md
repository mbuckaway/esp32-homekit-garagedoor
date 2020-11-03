# ESP32 GarageDoor Controller for HomeKit

## The Rational

The first iteration of this controller was a plugin to Homebridge running on a Raspberry PI. While an interesting concept, it seemed complete overkill. This setup required a complete linux operating system to do one thing: open the garage door. Maintaining the linux OS and keeping homebridge up to date became a pain. Several times the PI chewed up the SD card after a power failure. This was unacceptable for a simple device that should just work. Nevermind, a Raspberry PI is a power hungry beast for something that sits idle 99% of the time. ESP32's are also much cheaper than a PI.

Enter the ESP32 dedicated Garage Door controller. When Espressif, created their Homekit SDK for the ESP32, the time was ripe to move the garage door controller to a smaller platform. The ESP32 IDF and Homekit SDKs made incorporating homekit functionity into a sample platform easy. The first iteration was up and running in an evening. After some comparing the Javacript code from the Homebridge plugin to the ESP32 Homekit SDK, it appeared to work largely the same. After some work, all the same services that were used on the Homebridge version were working on the ESP32 version. Best yet, the ESP never has a SD card to get chewed or an OS to maintain.

## How this Works

There is one oddity with the Espressif homekit implementation - it does not automatically connect to the WIFI network like every other Homekit devices. Most homekit devices use Bluetooth to transfer over the WIFI credentials. The ESP version does not. This means the WIFI creds must either be hard coded into the device or the user must first use the Espressif BLE or SoftAP provisioning app on a iPhone. However, this also means that the homekit configuration is much faster as the WIFI network is already setup.

The garage door software uses three GPIO pins on the ESP32:
- GPIO 26 for the relay
- GPIO 25 for the close sensor
- GPIO 27 for the open sensor

The difference with this controller is it uses both a switch on the lower part of the door to sensor it's actually closed and a switch on the upper side of the door to sensor it's actually open. With only a close switch, one can only assume the garage door actually makes it all the way up to be open. The two switches allow us to add a motion sensor service to sensor when the door is in motion. I use that to turn the garage light automatically on when the door moves.

The code configures the following services that make up the garage door accessory:
- Garage Door
- Close Sensor
- Open Sensor
- Motion Sensor (sense when the door is in motion)
- Switch to force the door to move (used for automations to override security "issues")
- Switch to close the door when it is open (and only open). Homekit sometimes loses the door status and this control is required for automation to work reliably.

What is missing from this unit is the obstruction detection. At some point, I may add a eye beam to detect something is in the way, but the old garage door already has that as a safety features, so having your phone notify you of the obstruction seems moot.

Because the code support interrupts on input changes which the PI version didn't have, the opening and closing statuses become more reliable. If the door switches trigger because of the garage door is moving, it knows is the garage door is opening (if the close sensor opened) or closing (the open sensor opened). We use the set the interrupts to fire on the rising edge to make this work. Additionally, the hardware of the device was change to set the inputs to use pull up resistors internally setting the switches on close to bring the inputs to ground. This saves introducing noise on the 3.3v bus.

A note on switch bounce: we ignore it. All the interrupt handlers do is set the default door status flag to closing or opening. So, it that happens 100 times, there is no conciquence.

## Setting up the Device

Make sure to run the updatemodules.sh script if you did not get the code recursively. It has a dependancy on the Espressif HomeKit SDK.

First, configure idf with menuconfig:

```bash
idf.py menuconfig
```

Make sure to set the GPIO pins to what you intend to use. Also, use the hard coded Homekit code, but make sure there are no duplicates on your network. If you use the WIFI Autoprovisioning (bluetooth version is recommended), this the Apple App Store and search for the Espressif BLE Provisioning tool. This will be required to configure WIFI.

Flash and run the monitor:

```bash
idf.py flash
idf.py monitor
```

The monitor is required for the first time to get the QR codes for WIFI provisioning and Homekit setup.

With the monitor running, you will get a screen full of data and two QR codes. The first QR code is for homekit, and the second is for the Espressif provisioning tool. Open the Espressif BLE provisioning tool, and press the Provision Device button. You will be asked to scan the second QR code (provisioning QR code). Next, enter your WIFI password. Watch the monitor. If will indicate if provisioning was succcesful as will the app. If you mess up the password, hold the boot button down for 10 seconds to wipe the configuration, and start over. Once the WIFI is working, open the Homekit app on your iPhone, and add an accessory. Scan the first QR code with Home app, and it should find the GarageDoor accessory. At this point, it's fairly quick to add the accessory. Because of all the different sensors in this device, it's a good idea to use "separate tiles" to display them. Open the setting for the garage door item in the Home app, and select the separate files item.

That is it. You can now use the Home app or Siri to control the device. The default names are ESP "item name". You can change them on your Home app.

## Resetting the ESP device

There is an button handler tied to GPIO0 which on most devices is attached to the boot button. Holding the button has two effects:
- hold for 3 seconds to reset the WIFI setup
- hold for 10 seconds to wipe the WIFI and homekit config and start over (factory default)

For a reset, one does not necessary need the monitor. You can manually add the WIFI password and manually add the device to homekit....but the monitor makes things easier.
