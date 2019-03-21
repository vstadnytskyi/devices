# Ultra-Compact-UC160-190
Ultra Compact UC160-190 by Solid State Cooling Systems

Oasis chiller driver 

See: Oasis Thermoelectric Chiller Manual, Section 7 "Oasis RS-232
communication", p. 15-16 (https://www.sscooling.com/images/stories/Manuals/uc160-190%20manual%2052-160g1-1%20rev%20m13%20.pdf)

Settings: 9600 baud, 8 bits, parity none, stop bits 1, flow control none
DB09 connector pin 2 = TxD, 3 = RxD, 5 = Ground

The controller accepts binary commands and generates binary replies.
Commands are have the length of one to three bytes.
Replies have a length of either one or two bytes, depending on the command.

Command byte: 
				bit 7: remote control active (1 = remote control,0 = local control)
				bit 6  remote on/off (1 = Oasis running, 0 = Oasis in standby mode)
				bit 5: communication direction (1 = write,0 = read)
				bits 4-0: 00001: [1] Set-point temperature (followed by 2 bytes: temperature in C * 10)
						00110: [6] Temperature low limit (followed by 2 bytes: temperature in C * 10)
 						00111: [7] Temperature high limit(followed by 2 bytes: temperature in C * 10)
						01000: [8] Faults (followed by 1 byte)
						01001: [9] Actual temperature (followed by 2 bytes: temperature in C * 10)

The 2-byte value is a 16-bit binary number enoding the temperature in units
of 0.1 degrees Celsius (range 0-400 for 0-40.0 C)

The fault byte is a bit map (0 = OK, 1 = Fault):
bit 0: Tank Level Low
bit 2: Temperature above alarm range
bit 4: RTD Fault
bit 5: Pump Fault
bit 7: Temperature below alarm range

Authors: Valentyn Stadnytskyi
created: March 19 2019
last updated: March 21 2019

avaiable properties:
    - target_temperature
    - actual_temperature
    - faults

special functions:
    - get_PID
    - set_PID
    - set_default_PID


How to use:

driver.init() - will initalize the driver and find the connected Oasis chiller
driver.close() - will propeprly close the port

Example:
>>> driver.init()
2019-03-21 18:15:53,584 INFO: open Com port (u'COM19') found (u'Prolific USB-to-Serial Comm Port (COM19)')
2019-03-21 18:15:53,786 INFO: the requested device is connected to COM Port u'COM19'
2019-03-21 18:15:53,848 INFO: initialization of the driver is complete
>>> driver.actual_temperature
22.0
>>> driver.target_temperature
22.0
>>> driver.faults
(0, 0)
>>> driver.target_temperature = 25
>>> 
