MWLite_gke
==========

CAUTION: This is NOT an official release of MultiWii. 

MWLite-gke is a radically cutdown version of MultiWii 2.2. 

There are many many changes and in fact it is close to a rewrite. Code loop can run down to 1.3mS which is more than enough and is mainly I2C time reading the MPU6050. I have it set to around 2mS currently.

Any parameter that is zero when you load the defaults means it is not used by the code.

You must calibrate the accelerometers at least once otherwise the motors will not start. You only need to do this again if you do a major frame bending crash or if the calibration is radically wrong. The gyro stick programming "angle" trim now trims the accelerometer neutrals. The default trim step is 1 which allows very fine trimming. You can change this in config.h. To use the trims you must be disarmed. Select full throttle and then move the elevator or aileron stick (not both) in the direction away from the drift. You should only have to do this once and NOT every flight.

The defines at the top of config.h (GENERAL_USE) should work. Comment out or uncomment the things you wish to use.

The original MW control code is selected by default and supports all the flip behaviour as per the original. There are a number of my control options as well. WOLFERL is angle mode only and should be OK for just flying.

When you connect the battery their will be a longer delay than for the original code so you have more time to connect the battery and to get it down and still. Wait until you see a single Blue flash then three flashes.

You may connect other I2C sensors using SCK/YAW pin for SCL MISO/Pitch pin for SDA. Devices include the BMP085 and MS5611 barometers and the HMC65883L magnetometer. Code is plug and play so if you don't have various sensors MWLite works it out.

If you connect the throttle pin to ground before you power up you can bind the Spektrum receiver in the usual way. The PUMQ board will stay in bind mode until the link is removed and the power is reconnected.

This is a work in progress. I would appreciate any positive comments/suggestions on the PUMQ Thread.

http://www.rcgroups.com/forums/showthread.php?t=1885026

G.K. Egan May 2013.

