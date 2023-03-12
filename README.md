# CyBell
"Sentient Bike Bell"

This is the FIFTH major revision to my ongoing virtual / electronic bike bell project.

Initially (v1) a webapp in Javascript available online at https://Bedno.com/bell
Later I built a hardware implementation (v2), then a microcontroller version (v3) documented at https://bedno.com/ebb

Version 4 added Bluetooth (after aux connection proved flaky on bike due to vibration) and capacitive touch sensitivity replacing the physical button.
But it turns out BT bike speakers turn off if not used for too long so I'm adding it's own speakers for the next version.

Version 5 also adds basic sentience.  Given  the ESP32's great CPU power and memory and ability to easily connect sensors, I've added several features such as awareness of degrees of light and touch, hunger, work satisfaction and self expression.  I've researched the topic some for the implementation, and can safely argue this approaches insect level intelligence.

It's mostly working but the upgrade to amp and speaker using I2S is going to take some days.
So I'm seeding the repository for now with that connection diagram to help me finish.
