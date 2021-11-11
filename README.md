# AutoWindowBlinds
automatic window blind controller based on the Teensy 3.2 board

Automatically open the window blinds when it's bright outside, and close it when it's dark.

I use duff2013's Snooze library to put the system in lower power sleep and only periodically wake up, and cut power to unneeded components when the system is in low power sleep using a relay. The idle power draw measured at the battery power input is about 50uA. It should last at least a year on 4 rechargeable AA batteries.

Use commit d1b25d4b122fbc02c98a257f9ddc8b473f4e621c (2019-04-30) of the Snooze library. The latest version has a different API.

* schematic/ contains the KiCad schematic for the circuit
* CAD/ contains the FreeCad design for the 3D printed adapter that goes between the motor and the stick the motor needs to turn
