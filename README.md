## PowerWheelsControl

This project aims to enhance throttle control for kids 12v style ride-on cars that typically
use simple on/off style pedal switches by moving all motor control to PWM based drivers controlled
via a hall effect type pedal.

Most parts utilized can be substituted for others as most functions rely only on ground-switched inputs.

Individual BTS7960 H-bridge PWM motor drivers are used for each left/right motor, but controlled by a common PWM output.  Pedal input works on a 0.8-4.2V range.

A primary "DEBUG" flag is defined at the start of the project.  When enabled an active serial console is produced to provide live feedback of primary inputs/outputs.  This is best viewed in putty or similar software that interperts ANSI codes.  Commenting out the "DEBUG" define will diable the console output and allow things to run as fast as possible.
