# ESP32 GarageDoor Controller for HomeKit

The first iteration of this controller was a plugin to Homebridge running on a Raspberry PI. While an interest concept, it seemed complete overkill. Nevermind, a Raspberry PI is a power hungry beat for something that sits idle 99% of the time.

When Espressif, created their Homekit SDK for the ESP32, the time was ripe to move the garage door controller to a smaller platform. The ESP32 IDF and Homekit SDKs made incorporating homekit functionity into a sample platform easy. The first iteration was up and running in an evening.

## More to come...

This code is work in progress. More will come as it is updated. The goal is to create Homekit items:
- Garage Door
- Close Sensor
- Motion Sensor (sense when the door is in motion)
- Switch to force the door to move (used for automations to override security "issues")
- Switch to close the door when it is open (and only open). Homekit sometimes loses the door status and this control is required for automation to work reliably.

The Open Sensor used in the Homebridge version will be removed.

The code also needs some cleanup.
