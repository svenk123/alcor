The Alcor drive corrector produces a variable frequency 120 V AC output for a telescope's right ascension (RA) drive using synchronous motors.
And 5V/12V PWM DC output for the declination axis motors. I have designed it for the old Celestron/Meade Fork mounts from the 80s.
It allows tracking corrections to be made during long-exposure photography.

The Aclor drive corrector can be used with modern guiding software like PHD2, etc. Guiding pulses can be send over the ST-4 input or the USB port. It supports
the LX200 command protocol set.

![](/images/Alcor_Drive_Corrector5.JPG) ![](/images/Alcor_Drive_Corrector6.JPG)


# Hardware

![](/images/Alcor_Drive_Corrector2.JPG)

## Electronic Circuit

You can find the schematic and Gerber files in the pcb/ directory. I have exported the Gerber files so that you can let the board manufacture by JLCPCB ( [www.jlcpcb.com](https://www.jlcpcb.com)).

![](/pcb/alcor3_cpu.png)

This is the final pcb after soldering all the components. Note to set the Jumper between X3-2 and X3-3 for 5V output voltage for the DEC motor.

![](/images/Alcor_Drive_Corrector4.JPG)

## Bill of material

| Part | Qty | Description | order number | Link |
| --- | --- | --- | --- | --- |
| _1 | 1 | Teensy 2.0 Development Board | Teensy 2.0 | https://www.pjrc.com/store/teensy.html |
| Sockets for _1 | 2 | Sockets 2.54 mm, 1X16, straight | MPE 094-1-016 | https://www.reichelt.de/buchsenleisten-2-54-mm-1x16-gerade-mpe-094-1-016-p119919.html |
| Sockets for _1 | 1 | Sockets 2.54 mm, 1X5, straight | MPE 094-1-005 | https://www.reichelt.de/buchsenleisten-2-54-mm-1x05-gerade-mpe-094-1-005-p119914.html |
| C3 | 1 | Axial electrolytic capacitor, 105°C, 16x36 mm, 3300 µF/25 V, low | AX 105 3300/25 | https://www.reichelt.de/elko-axial-3-3-mf-25-v-105-c-2000h-20--ax-105-3300-25-p126789.html |
| C5 | 1 | Pulse capacitor, 2.2µF, 400V, RM27.5 | MKP10-400 2,2µ | https://www.reichelt.de/impulskondensator-2-2-f-400v-rm27-5-mkp10-400-2-2--p173243.html |
| D1 | 1 | Switching diode, 100 V, 150 mA, DO-35 | 1N4148 | https://www.reichelt.de/schalt-diode-100-v-150-ma-do-35-1n-4148-p1730.html |
| D3 | 1 | Schottky diode, DO41, 20 V, 1 A | 1N5817 | https://www.reichelt.de/schottkydiode-20-v-1-a-do-41-1n-5817-p41848.html |
| IC1 | 1 | SN754410 Half-Bridge-Driver IC | 296-9911-5-ND | https://www.digikey.de/de/products/detail/texas-instruments/SN754410NE/380180 |
| Socket for IC1 | 1 | IC socket, 16-pin, double spring contact | GS 16 | https://www.reichelt.de/ic-sockel-16-polig-doppelter-federkontakt-gs-16-p8208.html |
| IC2 | 1 | Recom Power Regulator R-78E5.0-1.0 | 945-2201-ND | https://www.digikey.de/de/products/detail/recom-power/R-78E5-0-1-0/4930585 |
| J1 | 1 | RJ11 Jack 6P6C unshielded | RJ11-6L-S | https://www.digikey.de/de/products/detail/te-connectivity-corcom-filters/RJ11-6L-S/142227 |
| J2 | 1 | DC-Socket, flat contacts, 2,1mm, 90° | DC BU21 90 | https://www.reichelt.de/de/en/dc-socket-flat-contacts-2-1mm-90--dc-bu21-90-p183502.html |
| Q1, Q2 | 2 | MOSFET N-Channel 30V 62A TO220AB | IRF3708-ND | https://www.digikey.de/de/products/detail/infineon-technologies/IRF3708/360319 | 
| Heat sinks for Q1, Q2 | 2 | U-shaped heat sink, 25x17x13mm, 28K/W, elongated hole | V 5074A | https://www.reichelt.de/u-kuehlkoerper-25x17x13mm-28k-w-langloch-v-5074a-p22232.html |
| R1, R2, R4, R5, R6, R7 | 6 | WCarbon film resistor 1/4W, 5%, 10 kilo-ohms | 1/4W 10K | https://www.reichelt.de/widerstand-kohleschicht-10-kohm-0207-250-mw-5--1-4w-10k-p1338.html |
| R3 | 1 | Varistor, RM 5 mm, 0.25 W, 130 VAC = JVR7N201K | VDR-0,25 130 | https://www.reichelt.de/varistor-rm-5mm-0-25w-130vac-vdr-0-25-130-p22308.html |
| TR1 | 1 | Transformer 6 VA, 2 x 6 V, 2 x 500 mA | UI 30/10,5 206 | https://www.reichelt.de/trafo-6va-2x-6v-2x-500ma-ui-30-10-5-206-p27546.html |
| X3 | 1 | Pin headers 2.54 mm, 1X03, straight | MPE 087-1-003 | https://www.reichelt.de/stiftleisten-2-54-mm-1x03-gerade-mpe-087-1-003-p119880.html  |
| Jumper for X3 | 1 | Jumper, black, RM 2.54 | JUMPER 2,54 SW | https://www.reichelt.de/kurzschlussbruecke-schwarz-rm-2-54-jumper-2-54-sw-p9017.html |
| X4 | 1 | Terminal — 3-pin, pitch 5.08 mm, 90° | LAKL 1,5 3 5,08 | https://www.reichelt.de/anschlussklemme-3-pol-rm-5-08-mm-90--lakl-1-5-3-5-08-p169872.html |
| X6 | 1 | Jack socket, 3.5 mm, PCB with switch contact | EBS 35 | https://www.reichelt.de/klinkeneinbaubuchse-3-5-mm-stereo-ebs-35-p7301.html |

### Optional Parts ###

* Enclosure 120 mm x 84 mm x 44 mm, Model: [Hammond 1457K1202BK](https://www.hammfg.com/electronics/small-case/extruded/1457)

* DEC Motor JMI MOTODEC [JMI MOTODEC](http://www.jimsmobile.com/buy_motodec.htm)

* Celestron 4 Button Hand Controller [Discussion on cloudy nights](https://www.cloudynights.com/topic/888791-what-is-this-4-button-hand-control-from/)

# Firmware

The firmware was written specifically for, and tested with, a Teensy 2.0 microcontroller board. It may work on other boards with an ATmega23u4 microcontroller. 

## Flashing the firmware

Follow these instructions to flash the firmware.
[https://www.pjrc.com/teensy/first_use.html](https://www.pjrc.com/teensy/first_use.html)
[https://www.pjrc.com/teensy/loader.html](https://www.pjrc.com/teensy/loader.html)

# Using the Alcor drive controller

## PHD2 settings

![](/images/Alcor3_PHD2_Screenshot1.jpg)

