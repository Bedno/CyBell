"CyBell" - Sentient Bike Bell (v5.55) - 2023.05.05
Andrew Bedno - andrew@bedno.com - AndrewBedno.com

Compact electronic bike bell with multiple sounds, basic sentience and more.

Plays over a dozen selectable noises for use as a bicycle bell with built in amp and speaker.
But also has a number of senses and feelings about them, memories of its past, and satisfaction with doing its job well.

For full details see https://bedno.com/cybell

Version 5.x adds built-in amp and speaker and basic sentience.  v4 Added capacitive touch sensing and Bluetooth.  v3 First ESP32 implementation, with physical button and audio line out (See bedno.com/ebb).  v2. Hardware version with MP3 player chip.  v1. SmartPhone WebApp (See bedno.com/vbb)

Version 5.x revision notes (changes since "dumb" version 4.x).

Added sense of HUNGER, using a 2*100k resistor voltage divider from battery to an ADC pin.  App internally scales to millivolts.
Added a DISTRESS sound when hungry.  Makes a hungry baby bird chirp sound every few minutes varying with increasing urgency, constituting sense of PAIN.

Changed to capacitive TOUCH sensing for trigger, replacing physical button (though still supported as optional).  App dynamically scales from experience.
The degree of contact is captured as sense of touch.  Also captures time since last touched as sense of LONELINESS.

Added sense of LIGHT, using a photo resistor (20-100k) to ground over a 100k resistor to V+ to an ADC pin.  App dynamically scales from experience.
Also detects decreases in light as sense of PROXIMITY, effecting some behavior.

Added sense of SPACE using built-in hall effect sensor.  App dynamically scales from experience.

Added basic SENTIENCE.  Device regularly reviews its senses and pauses varying intervals to reflect on them.  After playing any sound it expresses its contentment with those senses, encoded in LED flickers.
Attempts to thoroughly address standard criteria for Sentience by combining elements including several physical senses, some self determination, ability to feel pain, power of self expression and persitent memories.

Added volume control.  Multitap during a sound to lower the volume.  Goes through several steps then back to max.
Volume setting and sound selection are now stored in non-volatile memory and recalled on boot.

Added automatic hands-free talking TWISTER spinner mode.  Speaks moves for the popular Twister game, allowing play without a referee.  Spins every seventeen seconds or when touched.  Multitap during first sound at boot goes to Twister mode.

Added persistent MEMORIES (in non-volatile storage).  Keeps a summary of every feelings it has ever expressed.  When booted it now recalls its prior life experiences and continues adding to them.
Also now remembers LIFETIME minutes running (and maximum continuous uptime).
Also remembers the lowest power survived (and duration of distress) in non-volatile storage for review and calibration.  Can be reset using hidden stuck button reboot function.

Interrupt service function completely overhauled.  Now runs at 12800Hz, being reasonable quality, both compact and an even divisor of 80MHz.  Supports several new ansynchronous features using flags.

Audio feed completely overhauled to support I2S protocol outputs for modern amps.  Source wav stored as 8 bit signed for compactness, expanded to 16bit stereo on use.  Also .h file generation from source wavs has been automated.
A few sounds have been removed, others rearranged, and all are now <2 seconds each.

This version removes the BlueTooth output of prior versions due to too many problems in practice.
