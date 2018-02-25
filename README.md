# AutoWindowBlinds
automatic window blind controller based on the Teensy 3.2 board

Automatically open the window blinds when it's bright outside, and close it when it's dark.

I use duff2013's Snooze library to put the system in lower power sleep and only periodically wake up, and cut power to unneeded components when the system is in low power sleep using a relay. The idle power draw measured at the battery power input is about 50uA. It should last at least a year on 4 rechargeable AA batteries.
