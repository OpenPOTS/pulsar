# Pulsar

Control a landline phone using an Ag1171 and an ATtiny85.

## Hardware and software requirements

To run this project, you will need:

* [Digispark](http://digistump.com/products/1) (or an ATtiny85 with [Micronucleus](https://github.com/micronucleus/micronucleus) loaded onto it)
* [Silvertel Ag1171](https://silvertel.com/ag1171/) (or compatible) SLIC
* 5 V / 300 mA power supply
* Some way of connecting everything together, either with jumper leads or on a breadboard (PCB design coming soon?)
* A telephony device that you want to control, such as a landline phone or modem (referred to as FXO)
* Connector to plug your telephony device into (referred to as FXS)

* Arduino Studio (tested with v2.10)
* CDC ACM drivers on your OS (usually
  * For Linux, use the `USB_ACM` Kernel driver
  * For Windows, use [LowCDC](https://github.com/protaskin/LowCDC-Win10x64)

## Usage

### Compiling

Compiling has been tested with Arduino Studio v2.10 with DigiStump package v1.6.7.
The code compiles to nearly the whole ROM size of the ATtiny85, so if you're compiling manually your compiler will need to have space optimisations turned on.

### Hardware setup

Connect the following pins:

| Power supply  | Ag1171 (pin no.) | ATtiny85 (pin no.) | FXS |
| ------------- | ---------------- | ------------------ | --- |
| +5 V          | +Vpwr (13)       |                    |     |
| GND           | GNDpwr (12)      | GND (4)            |     |
|               | RM (4)           | PB0 (5)            |     |
|               | F/R (3)          | PB1 (6)            |     |
|               | SHK (5)          | PB2 (7)            |     |
|               | RING (1)         |                    | B   |
|               | TIP (2)          |                    | A   |

Pins for A and B on the FXS depend on the standard your phone uses.

* For US phones (RJ11 plugs), A is pin 3, B is pin 2.
* For UK phones (631A/431A plugs), A is pin 5, B is pin 2.
  Note: for older UK phones, you'll also need to connect the Bell wire (pin 3 on a UK FXS) to the B wire (pin 2 on a UK FXS) through a 1.8 µF / 100 V capacitor to protect the ringer from DC current.

ATtiny85 pins PB3 (2) and PB4 (3) are used for USB communication on the Digispark.
If you are building your own board, refer to the [Digispark schematic](https://s3.amazonaws.com/digispark/DigisparkSchematicFinal.pdf) for more info.

### Commands

Pulsar uses the DigiCDC library to expose itself as a CDC ACM device.
Commands can be send to the virtual COM device made by your ACM driver.
Commands start with a single character, followed by a comma separated list of parameters, and are terminated by a linefeed (`\n`) character.
No spaces must be used in the command.

Commands:

| Command | Parameters                  | Description |
| ------- | --------------------------- | ----------- |
| r       | ring frequency (1 - 255 Hz)<br>number of rings (1 - 255)<br>on time (0 - 32767 ms)<br>off time (0 - 32767 ms)<br>pause time (0 - 32767 ms) | Start ringing the phone with the specified ring cadence. Ringing will continue until the stop command is sent or off-hook is detected. |
| s       |                             | Stop ringing the phone |
| l       |                             | Get line state. Returned as two 0/1 characters for ringing and off-hook states, in that order. |

For example, to start ringing with a UK cadence (ringing at 25 Hz for 400 ms on, 200 ms off, 400 ms on, then pause off for 2 s), use the command:

```
r25,2,400,200,2000
```

For a US cadence (ringing at 20 Hz for 2 s on, then pause off for 4 s), use the command:

```
r20,1,2000,0,4000
```
