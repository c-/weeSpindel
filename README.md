# weeSpindel

I like the idea of the [iSpindel](https://github.com/universam1/iSpindel),
but I wanted something smaller; ideally, something which I could use in
my glass carboys. This contraint means I need to get it into a tube with
an outside diameter of approximately 25mm.

I've also been trying to get away from Li-ion as a power source and
stick to simple NiMH rechargeables. Using ESP-NOW as a communications
protocol makes a single NiMH cell with a boost converter a realistic
option. However, it turns out that a 25mm tube containing a AA NiMH cell
doesn't have the necessary buoyancy to actually float, much less tilt,
so I've been forced to switch to an AAA cell. In a nod to practicality,
I've also included the option to use a 10440 (AAA-size) Li-ion cell with
a conventional LDO regulator.

Stripping out all the bits of the iSpindel that I can't use or don't need,
I'm left with an ESP-12 module, the MPU-6050 module, the power circuitry,
assorted passives, and a single battery cell. Even the DS18B20 temperature
sensor is dropped in favour of the MPU-6050.  Programming the ESP is
handled by either removing it (I mount mine on pin headers) or some sort
of TBD pogo pin jig.

weeSpindel is running a stripped down firmware which transmits
reports using ESP-NOW.  A ESP-NOW-to-MQTT (ESP32 with W5500 ethernet)
gateway makes the sensor results available in Home Assistant. With this
arrangement, the entire wake-read-send-sleep cycle is approximately 250ms.

Note that we don't bother with something like a MOSFET to cut power to the
MPU module; between it and the (on my board, at least) Torex XC6204 LDO
it'll use maybe 5uA during sleep. We do remove the LED though.
