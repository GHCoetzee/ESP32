BrewSaurus is our first try at a GrainFater/Braumeister clone.  The system is a 50l urn, 30l grain basket, high temperature magnetic pump and counter current chiller.

ESP32 is used as microcontroller and Cayenne is used as interface.  MAX31855 breakout board and K-type thermocouple is used to read temperature, with temperature control done using a SSR and P&ID control.  The pump is controlled using a normal 10A relay.

ESP32 was programmed using Arduino IDE (yes I know, I was a bit lazy), but sometimes you just want to start all-grain brewing.  The attached program is quick and dirty, but it does the job.

Advantages of this system: cheap clone for Grainfather, Cayenne does not cost any money for HMI, and made some great beer
Disadvatages: sytem is already too small, designing a 200l setup.  ON/OFF temp control within bands will work better.  I could clean-up the program a bit.

The .ino file will give you a great headstart to replicate a cheap alternative.  Use as you please, and some ideas will be fantastic.
