#
# Lifted from this thread: https://loboris.eu/forum/showthread.php?tid=329
#
# I2C Version for MicroPython (ESP32 tested)
# Christoph Schuhn
# 2019 Jan

# V 1.04 changed back to mV scale and use fixed point to reduce runtime
# V 1.03 added "write" to limit registers
# V 1.02 added Die_ID & Manu
# V 1.01 fault handling for read / write registers to handle exceptions
# V 1.00 initial version

# based on ....
# SwitchDoc Labs March 4, 2015 
# V 1.2

# and ...
# ina219 driver for micropython

#encoding: utf-8

from micropython import const

class INA3221_I2C():
##### Provides all the functionality to interact with the INA3221 sensor ######
#  --- name --------------------- reg ----- description ----------- reset val
    __ADDRESS              = const(0x40)  # 1000000 (A0+A1=GND)
    __READ                = const(0x01)  # read register
    __REG_CONFIG          = const(0x00)  # config register
    __REG_SHUNTVOLTAGE_1  = const(0x01)  # ch 1 avg shunt voltage
    __REG_BUSVOLTAGE_1    = const(0x02)  # ch 1 avg bus voltage
    __REG_SHUNTVOLTAGE_2  = const(0x03)  # ch 2 avg shunt voltage
    __REG_BUSVOLTAGE_2    = const(0x04)  # ch 2 avg bus voltage
    __REG_SHUNTVOLTAGE_3  = const(0x05)  # ch 3 avg shunt voltage
    __REG_BUSVOLTAGE_3    = const(0x06)  # ch 3 avg bus voltage
    __REG_CHANNEL_1_CAL    = const(0x07)  # ch 1 critical alert limit 0x7FF8
    __REG_CHANNEL_1_WAL    = const(0x08)  # ch 1 warning alert limit  0x7FF8
    __REG_CHANNEL_2_CAL    = const(0x09)  # ch 2 critical alert limit 0x7FF8
    __REG_CHANNEL_2_WAL    = const(0x0A)  # ch 2 warning alert limit  0x7FF8
    __REG_CHANNEL_3_CAL    = const(0x0B)  # ch 3 critical alert limit 0x7FF8
    __REG_CHANNEL_3_WAL    = const(0x0C)  # ch 3 warning alert limit  0x7FF8
   
    __REG_LIMIT_EN        = const(0x0F)  # mask alert/status/sum    0x0002
   
    __REG_POWVAL_UL        = const(0x10)  # power valid upper limit  0x2710
    __REG_POWVAL_LL        = const(0x11)  # power valid lower limit  0x2328

    __REG_MANU_ID          = const(0xfe)  # manufacturer id register
                                          #      Texas Intruments    0x5449
    __REG_DIE_ID          = const(0xff)  # chip id register          0x3220
   
    __RESET                = const(0x8000) # Reset Bit
   
    __ENABLE_CHAN1        = const(0x4000) # Enable Channel 1
    __ENABLE_CHAN2        = const(0x2000) # Enable Channel 2
    __ENABLE_CHAN3        = const(0x1000) # Enable Channel 3
   
    __AVG2                = const(0x0800) # AVG Samples Bit 2 - See table 3 spec
    __AVG1                = const(0x0400) # AVG Samples Bit 1 - See table 3 spec
    __AVG0                = const(0x0200) # AVG Samples Bit 0 - See table 3 spec
   
    __VBUS_CT2            = const(0x0100) # VBUS bit 2 Conversion time - See table 4 spec
    __VBUS_CT1            = const(0x0080) # VBUS bit 1 Conversion time - See table 4 spec
    __VBUS_CT0            = const(0x0040) # VBUS bit 0 Conversion time - See table 4 spec
   
    __VSH_CT2              = const(0x0020) # Vshunt bit 2 Conversion time - See table 5 spec
    __VSH_CT1              = const(0x0010) # Vshunt bit 1 Conversion time - See table 5 spec
    __VSH_CT0              = const(0x0008) # Vshunt bit 0 Conversion time - See table 5 spec
   
    __MODE_2              = const(0x0004) # Operating Mode bit 2 - See table 6 spec
    __MODE_1              = const(0x0002) # Operating Mode bit 1 - See table 6 spec
    __MODE_0              = const(0x0001) # Operating Mode bit 0 - See table 6 spec

    SHUNT_RESISTOR_VALUE  = 0.1      # default shunt resistor value of 0.1 Ohm
   

    ###########################
    # INA3221 Code
    ###########################
    def __init__(self, i2c, address=__ADDRESS, shunt_resistor = SHUNT_RESISTOR_VALUE  ):
        self._i2c = i2c
        self._address = address
        self._shunt_resistor = shunt_resistor
        config = __ENABLE_CHAN1 |  \
                __ENABLE_CHAN2 |  \
                __ENABLE_CHAN3 |  \
                __AVG1 |          \
                __VBUS_CT2 |      \
                __VSH_CT2 |        \
                __MODE_2 |        \
                __MODE_1 |        \
                __MODE_0
        self.__write_register(__REG_CONFIG, config)
   
    def __write_register(self, register, register_value):
        register_bytes = self.__to_bytes(register_value)
        try:
            self._i2c.writeto_mem(self._address, register, register_bytes)
        except OSError as er:
            print ("WriteReg ", er)
 
    def __to_bytes(self, register_value):
        return bytearray([(register_value >> 8) & 0xFF, register_value & 0xFF])
   
    def __read_register(self, register, negative_value_supported=False):
        try:
            register_bytes = self._i2c.readfrom_mem(self._address, register, 2)
        except OSError as er:
            # device busy
            print ("ReadReg ", er)
            return 0
        register_value = int.from_bytes(register_bytes, 'big')
        if negative_value_supported:
            # Two's compliment
            if register_value > 32767:
                register_value -= 65536
        return register_value

    def _getBusVoltage_raw(self, channel):
        #Gets the raw bus voltage (16-bit signed integer, so +-32767)
        value = self.__read_register(__REG_BUSVOLTAGE_1+(channel -1) *2, True)
        return value

    def _getBusVoltage_raw(self, channel):
        #Gets the raw bus voltage (16-bit signed integer, so +-32767)
        value = self.__read_register(__REG_BUSVOLTAGE_1+(channel -1) *2, True)
        return value

    def _getShuntVoltage_raw(self, channel):
        #Gets the raw shunt voltage (16-bit signed integer, so +-32767)
        value = self.__read_register(__REG_SHUNTVOLTAGE_1+(channel -1) *2, True)
        return value

    def _getBus_POWVAL_UL_raw(self):
        #Gets the raw __REG_POWVAL_UL (16-bit signed integer, so +-32767)
        value = self.__read_register(__REG_POWVAL_UL, True)
        return value

    def _getBus_POWVAL_LL_raw(self):
        #Gets the raw __REG_POWVAL_LL (16-bit signed integer, so +-32767)
        value = self.__read_register(__REG_POWVAL_LL, True)
        return value

    def _setBus_POWVAL_UL_raw(self, value):
        #Sets the raw __REG_POWVAL_UL (16-bit signed integer, so +-32767)
        check = self.__write_register(__REG_POWVAL_UL, value)
        return check

    def _setBus_POWVAL_LL_raw(self, value):
        #Sets the raw __REG_POWVAL_UL (16-bit signed integer, so +-32767)
        check = self.__write_register(__REG_POWVAL_LL, value)
        return check

    ###########################
    # INA3221 public functions
    ###########################
   
    def getBusVoltage(self, channel):
    # Gets the Bus voltage in mV
        value = self._getBusVoltage_raw(channel)
        return value

    def getShuntVoltage_mV(self, channel):
    # Gets the shunt voltage in mV (so +-168.3mV)
        value = self._getShuntVoltage_raw(channel)
        return value * 0.005

    def getCurrent(self, channel):
    #Gets the current value in mA, taking into account the config settings and current LSB     
        valueDec = self.getShuntVoltage_mV(channel)/ self._shunt_resistor             
        return valueDec;
   
    def check_Die_ID(self):
    #Gets the DIE ID and compared to definition value
        value = self._i2c.readfrom_mem(self._address, __REG_DIE_ID, 2)
        return value;

    def check_Manu_ID(self):
    #Gets the MANU ID and compared to definition value
        value = self._i2c.readfrom_mem(self._address, __REG_MANU_ID, 2)
        return value;

    def getPowerValid_UL(self):
    # Gets the Power Valid Upper Limit Set voltage in mV
        value = self._getBus_POWVAL_UL_raw()
        return value

    def getPowerValid_LL(self):
    # Gets the Power Valid Lower Limit Set voltage in mV
        value = self._getBus_POWVAL_LL_raw()
        return value

    def setPowerValid_UL(self, mV):
    # Sets the Power Valid Upper Limit Set voltage in mV
        check = self._setBus_POWVAL_UL_raw(mV)
        return check
     
    def setPowerValid_LL(self, mV):
    # Sets the Power Valid Lower Limit Set voltage in mV
        check = self._setBus_POWVAL_LL_raw(mV)
        return check

   
##############################################################################
# EOF
