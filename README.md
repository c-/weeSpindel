# weeSpindel

I like the idea of the [iSpindel](https://github.com/universam1/iSpindel),
but I wanted something smaller; ideally, something which I could use in
my glass carboys. This contraint means I need to get it into a tube with
an outside diameter of approximately 25mm.

I've also been trying to get away from Li-ion as a power source and
stick to simple NiMH rechargeables. Using 
[ESP-NOW](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/network/esp_now.html)
as a communications
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
reports using an ESP-NOW broadcast rather than WiFi. ESP-NOW allows
for low-overhead transmission, which reduces the wake-read-send-sleep
cycle to approximately 100-120ms (we only bother with five samples from
the MPU, which honestly might even be excessive).  A ESP-NOW-to-MQTT
(ESP32 with an W5500 ethernet module) gateway makes the sensor results
available in Home Assistant, etc. Configurability is... minimal.

Note that we don't bother with something like a MOSFET to cut power to the
MPU module; between it and the (on my board, at least) Torex XC6204 LDO
it'll use maybe 5uA during sleep. We do remove the LED in the usual
fashion, as well as the 2.2k I2C pullups (we can get away with the internal
pullups on the ESP).

# Future

The prototype is done on a PCB mill, and that naturally limits what sort
of component footprints I can comfortably get away with. Future design
using a manufactured PCB opens up a few possibilities.

Lose the MPU-6050 module and put the chip directly on the board.

The failure mode of the HT7733SA boost regulator when the voltage drops
too low (under 1.2v with a AAA NiMH cell) is to go into a approx 200mA
battery draining loop. This is bad. In extreme cases, it'll reverse-charge
the NiMH cell. The fix for this would be to change to a boost regulator
with under-voltage lockout (UVLO). Something like the TPS61097 should do
the trick.

I'd love to get a USB controller onboard for programming. Probably the
CH340E.
