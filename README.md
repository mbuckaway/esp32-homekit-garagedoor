# ESP32 GarageDoor Controller for HomeKit

## What is it

ESP32 dedicated Garage Door controller. When Espressif, created their Homekit SDK for the ESP32, the time was ripe to move the garage door controller to a smaller platform. The ESP32 IDF and Homekit SDKs made incorporating homekit functionity into a sample platform easy. 

## How this Works

Because this is based on the open source version the Espressif SDK, it is not possible to automatically transfer the WIFI creds over the HAP connection. While auto-provisioning of the WIFI creds can be used it was overkill for the one garage door in my home. This module uses my WIFI component to hard code the WIFI creds into the device. It then starts up the Homekit SDK and allows the device to be auto-provisioned in the Apple Home App.

The garage door software uses three GPIO pins on the ESP32:

- GPIO 26 for the relay
- GPIO 25 for the close sensor
- GPIO 27 for the open sensor

The relay should be a buffered relay board. You can find them on Aliexpress or elsewhere.

The difference with this controller is it uses both a switch on the lower part of the door to sensor it's actually closed and a switch on the upper side of the door to sensor it's actually open. With only a close switch, one can only assume the garage door actually makes it all the way up to be open. The two switches allow us to add a motion sensor service to sensor when the door is in motion. I use that to turn the garage light automatically on when the door moves.

The code configures the following services that make up the garage door accessory:

- Garage Door
- Close Sensor
- Open Sensor
- Motion Sensor (sense when the door is in motion)
- Switch to force the door to move (used for automations to override security "issues")
- Switch to close the door when it is open (and only open). Homekit sometimes loses the door status and this control is required for automation to work reliably.

What is missing from this unit is the obstruction detection. At some point, I may add a eye beam to detect something is in the way, but the old garage door already has that as a safety features, so having your phone notify you of the obstruction seems moot.

The code uses a button library to monitor the door switches and automatically debounces the switches. Additionally, the routine the monitors the open and close switches updates Homekit with the changes in status, so the home app almost instantly. However, Homekit seems to have a bug in it where if you control the door without using the Home App, it loses track of the door status. The code updates the status correctly, but Homekit seems to ignore it. That said, it does correct itself when the door is fully closed.

## Setting up the Device

This project is based on the ESP-IDF.

Make sure to run the updatemodules.sh script if you did not get the code recursively. It has a dependancy on the Espressif HomeKit SDK, my own WIFI module, and my modified button module orginally from craftmetrics/esp32-button.

First, configure idf with menuconfig:

```bash
idf.py menuconfig
```

Make sure to set the GPIO pins to what you intend to use. Also, use the hard coded Homekit code, but make sure there are no duplicates on your network. WIFI is configured manually. Under the WIFI menu add your WIFI creds. Two are supported in case you have two SSIDs. The reconnect timeout of 8 secs is sufficient. The reboot count of 100 is also good. This will cause the ESP32 to reboot when it hits 100 WIFI connect retries. For the button configuration, there is a 2 second delay on the RESET button. Change this to 5 seconds or whatever. When the devices reset button is held for x secords, the homekit setup is whipped, and the device restarted. This is required to move the device to another homekit installation or to reset the device from scratch.

Flash and run the monitor:

```bash
idf.py flash
idf.py monitor
```

The easiest way to setup Homekit is to use the USB cable and the ESP-IDF monitor. This allows you to see the QR code generated in the log. Unlike production Homekit devices, there is no way to autoprivision the WIFI password through homekit. While it is possible, it requires a Homekit license from Apple and the MFi from Espressif. For home use, the hard coded WIFI credentials is usually good enough.

With the monitor running, you will get a screen full of data and a QR code. The first QR code is for homekit. If you mess up anything, hold the boot button down for 10 seconds to wipe the configuration, and start over. Open the Homekit app on your iPhone, and add an accessory. Scan the first QR code with Home app, and it should find the GarageDoor accessory. At this point, it's fairly quick to add the accessory. Because of all the different sensors in this device, it's a good idea to use "separate tiles" to display them. Open the setting for the garage door item in the Home app, and select the separate files item.

That is it. You can now use the Home app or Siri to control the device. The default names are ESP "item name". You can change them on your Home app.
