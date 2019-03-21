"""
Oasis chiller driver 

See: Oasis Thermoelectric Chiller Manual, Section 7 "Oasis RS-232
communication", p. 15-16

Settings: 9600 baud, 8 bits, parity none, stop bits 1, flow control none
DB09 connector pin 2 = TxD, 3 = RxD, 5 = Ground

The controller accepts binary commands and generates binary replies.
Commands are have the length of one to three bytes.
Replies have a length of either one or two bytes, depending on the command.

Command byte: bit 7: remote control active (1 = remote control,0 = local control)
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

Commands not specified in the manual:
C6:       Receive the lower limit. (should receive back C6 14 00)
E6 14 00: Set set point low limit to 2C
C7:       Receive the upper limit. (should receive back C7 C2 01)
E7 C2 01: Set set point high limit to 45C
E-mail by John Kissam <jkissam@sscooling.com>, May 31, 2016,
"RE: Issue with Oasis 160 (S/N 8005853)"

Cabling:
"NIH-Instrumentation" MacBook Pro -> 3-port USB hub ->
"ICUSB232 SM3" UBS-Serial cable -> Oasis chiller

Authors: Valentyn Stadnytskyi
created: March 19 2019
last updated: March 21 2019

designed with Python 3 in mind.

Author: Valentyn Stadnytskyi
Date:November - July 5 2018

avaiable properties:
    - target_temperature
    - actual_temperature
    - faults

special functions:
    - get_PID
    - set_PID
    - set_default_PID

The fault byte is a bit map (0 = OK, 1 = Fault):
bit 0: Tank Level Low
bit 2: Temperature above alarm range
bit 4: RTD Fault
bit 5: Pump Fault
bit 7: Temperature below alarm range

version 1.0.0 - basic working version

"""

from serial import Serial
from time import time, sleep, clock
import sys
import os.path
from pdb import pm
from time import gmtime, strftime
import logging
from struct import pack, unpack
from timeit import Timer

from logging import debug,info,warning,error

from numpy import nan

__version__ = '1.0.0' #


class Driver(object): #Oasis driver
    def __init__(self):
        #tested dec 17, 2017  
        self.name = 'oasis chiller driver'
        self.driver_version = __version__
        self.timeout_time = 10.0 #timeout time in seconds
        self.fault_description = {}
        self.fault_description[0] = 'Tank Level Low'
        self.fault_description[2] = 'Temperature above alarm range'
        self.fault_description[4] = 'RTD Fault'
        self.fault_description[5] = 'Pump Fault'
        self.fault_description[7] = 'Temperature below alarm range'
        
    def init(self):
        """
        initializes the driver
        """
        self.find_port()
        self.ser.flushInput()
        self.ser.flushOutput()
        info("initialization of the driver is complete")

    def close(self):
        """
        executes proper shutdown of the driver
        """
        self._close_port()

    def find_port(self):
        import serial.tools.list_ports
        lst = serial.tools.list_ports.comports()
        flag = False
        id_query_command = 'A'
        for item in lst:
            port = item.device
            try:
                self.ser = Serial(port, baudrate=9600, timeout=0.1)
                sleep(0.5)
                try:
                    info('open Com port (%r) found (%r)' % (port,item.description))
                    if self.check_id():
                        info("the requested device is connected to COM Port %r" % self.ser.port)
                        flag = True
                        break
                    else:
                        info("Oasis is not found")
                        self.ser.close()
                        info("closing com port")
                except:
                    self.ser.close()
            except:
                pass
        return flag
    
    """Basic serial communication functions"""   
    def _readall(self):
        #tested dec 17, 2017
        return self.ser.readall()

    def _readN(self,N):
        #tested dec 17, 2017
        from numpy import nan
        data = ""
        if self._waiting()[0] >= N:
            data = self.ser.read(N)
            if len(data) != N:
                print("%r where requested to read and only %N where read" % (N,len(data)))
                data = nan
        else:
            data = nan
        return data
    
    def _write(self,command):
        #tested dec 17, 2017
        self.ser.flushOutput()
        self.ser.write(command)
        
    def _flush(self):
        #tested dec 17, 2017
        self.ser.flushInput()
        self.ser.flushOutput()
        
    def _inquire(self,command, N):
        #tested dec 17, 2017
        from time import time
        from numpy import nan
        result = None
        if 'ser' in driver.__dict__.keys():
            if self.ser.isOpen():
                self.ser.write(command)
                tstart = time()
                while self.ser.inWaiting() < N:
                    sleep(0.05)
                    if time() - tstart > self.timeout_time:
                        break   
                if self.ser.inWaiting() == N:
                    result = self._readN(N)
                else:
                    result = nan
        return result
        
    def _waiting(self):
        #tested dec 17, 2017
        return [self.ser.in_waiting,self.ser.out_waiting]
        
    def _close_port(self):
        #tested dec 17, 2017
        self.ser.close()


    def _open_port(self):
        #tested dec 17, 2017
        self.ser.open()

    def check_id(self):
        id_query_command = 'A'
        if self._inquire(id_query_command,3) is not None:
            response = self._inquire(id_query_command,3)[0]
        else:
            response = ''
        return id_query_command == response
    
    def set_target_temperature(self,temperature):
        local_byte = pack('h',round(temperature*10,0))
        byte_temp = local_byte[0]+local_byte[1]
        self._inquire('\xe1'+byte_temp,1)
    def get_target_temperature(self):  
        res = self._inquire('\xc1',3)
        if res is not None:
            temperature = unpack('h',res[1:3])[0]/10.
        else:
            temperature = None
        return temperature
    target_temperature = property(get_target_temperature,set_target_temperature)
        
    def get_actual_temperature(self):
        res = self._inquire('\xc9',3)
        if res is not None:
            temperature = unpack('h',res[1:3])[0]/10.
        else:
            temperature = None
        return temperature
    actual_temperature = property(get_actual_temperature)
    
    def get_faults(self):
        from numpy import log2
        res_temp = self._inquire('\xc8',2)
        if res_temp is not None:
            res = unpack('b',res_temp[1])[0]
        else:
            res = -1
        
        if res == 0:
            result = (0,int(res))
        elif res == -1:
            result = None
        else:
            result = (1,int(log2(abs(res))))
        return result
    faults = property(get_faults)


    def get_lower_limit(self):
        res = self._inquire('\xc6',3)
        if res is not None:
            lower_limit = unpack('h',res[1:3])[0]/10.
        else:
            lower_limit = None
        return lower_limit
    def set_lower_limit(self,temperature):
        local_byte = pack('h',round(temperature*10,0))
        byte_temp = local_byte[0]+local_byte[1]
        self._inquire('\xe6'+byte_temp,1)
    lower_limit = property(get_lower_limit,set_lower_limit)

    def get_PID(self):
        from time import sleep
        dic = {}
        res_dic = {}
        try:
            dic['p1'] = ('\xd0',3)
        except:
            dic['p1'] = nan
        try:
            dic['i1'] = ('\xd1',3)
        except:
            dic['p1'] = nan
        try:
            dic['d1'] = ('\xd2',3)
        except:
            dic['p1'] = nan
        try:
            dic['p2'] = ('\xd3',3)
        except:
            dic['p1'] = nan
        try:   
            dic['i2'] = ('\xd4',3)
        except:
            dic['p1'] = nan
        try:
            dic['d2'] = ('\xd5',3)
        except:
            dic['p1'] = nan
        for key in dic.keys():
            res = self._inquire(dic[key][0],dic[key][1])
            if res is not None or res is not nan:
                res_dic[key] = unpack('H',res[1]+res[2])
            else:
                res = nan
            sleep(0.5)
        return res_dic

    def set_default_PID(self, pid_dic = ('','')):
        pid_dic = {}
        #factory settings: good settings
        pid_dic['p1'] = 90
        pid_dic['i1'] = 32
        pid_dic['d1'] = 2
        pid_dic['p2'] = 50
        pid_dic['i2'] = 35
        pid_dic['d2'] = 3
        dic = {}
        dic['p1'] = '\xf0'
        dic['i1'] = '\xf1'
        dic['d1'] = '\xf2'
        dic['p2'] = '\xf3'
        dic['i2'] = '\xf4'
        dic['d2'] = '\xf5'
        for key in pid_dic.keys():
            byte_temp =  pack('h',round(pid_dic[key],0))    
            self._inquire(dic[key]+byte_temp,1)
            sleep(0.5)

    def set_PID(self, pid_dic = None):
        """sets p1,i1,d1,p2,i2,d2 pid parameters submitted as dictionary

        example: {'p2': 50, 'p1': 90, 'i1': 32, 'i2': 35, 'd2': 3, 'd1': 2}
        """
        pid_dic = {}
        #factory settings: good settings
        pid_dic['p1'] = 90
        pid_dic['i1'] = 32
        pid_dic['d1'] = 2
        pid_dic['p2'] = 50
        pid_dic['i2'] = 35
        pid_dic['d2'] = 3
        dic = {}
        dic['p1'] = '\xf0'
        dic['i1'] = '\xf1'
        dic['d1'] = '\xf2'
        dic['p2'] = '\xf3'
        dic['i2'] = '\xf4'
        dic['d2'] = '\xf5'
        for key in pid_dic.keys():
            byte_temp =  pack('h',round(pid_dic[key],0))    
            self._inquire(dic[key]+byte_temp,1)
            sleep(0.5)
##            
        

        

driver = Driver()
        
if __name__ == "__main__": #for testing
    from tempfile import gettempdir
    import logging
    logging.basicConfig(#filename=gettempdir()+'/di245_DL.log',
                        level=logging.DEBUG, format="%(asctime)s %(levelname)s: %(message)s")
    print('driver.find_port()')
    print('driver.actual_temperature')
    print('driver.target_temperature')
    print('driver.target_temperature = 25')
    print('driver.faults')
    print('driver.get_PID')
    print('driver.set_default_PID')
    print('driver.set_PID')
	
