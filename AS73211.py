from enum import IntEnum
from time import sleep as delaySeconds
import smbus as i2c
from smbus import SMBus as i2cBus

_COLOR6_REG_OPERATIONAL_STATE = 0x00
_COLOR6_REG_API_GENERATION = 0x02
_COLOR6_REG_CONFIGURATION_REGISTER_1 = 0x06
_COLOR6_REG_CONFIGURATION_REGISTER_2 = 0x07
_COLOR6_REG_CONFIGURATION_REGISTER_3 = 0x08
_COLOR6_REG_BREAK_TIME = 0x09
_COLOR6_REG_EDGE_COUNT_VALUE = 0x0A
_COLOR6_REG_OPTIONS_REGISTER = 0x0B

_COLOR6_MREG_STATUS_REGISTER = 0x00
_COLOR6_MREG_TEMPERATURE_MEASUREMENT = 0x01
_COLOR6_MREG_MEASUREMENT_X_CHANNEL = 0x02
_COLOR6_MREG_MEASUREMENT_Y_CHANNEL = 0x03
_COLOR6_MREG_MEASUREMENT_Z_CHANNEL = 0x04
_COLOR6_MREG_OUT_CONVERSION_LSB = 0x05
_COLOR6_MREG_OUT_CONVERSION_MSB = 0x06

_COLOR6_STATUS_OUTCONVOF = 0x8000
_COLOR6_STATUS_MRESOF = 0x4000
_COLOR6_STATUS_ADCOF = 0x2000
_COLOR6_STATUS_LDATA = 0x1000
_COLOR6_STATUS_NDATA = 0x0800
_COLOR6_STATUS_NOTREADY = 0x0400
_COLOR6_STATUS_STANDBY = 0x0200
_COLOR6_STATUS_POWER = 0x0100


class OperationalState(IntEnum):
    Configuration_state_Power_Down_state_on = 0x42
    Configuration_state = 0x02
    Measurement_state = 0x03
    Measurement_state_Start_measurement = 0x83
    If_Measurement_state_Start_measurement = 0x80
    Measurement_state_Power_Down_state_on = 0x43
    Measurement_state_Start_measurement_and_internal_startup_Power_Down_state_on = 0xC3
    If_Measurement_state_Start_measurement_and_internal_startup_Power_Down_state_on = 0xC0
    Software_reset = 0x0A


class IntegrationGain(IntEnum):
    Gain2048 = 0x00
    Gain1024 = 0x10
    Gain512 = 0x20
    Gain256 = 0x30
    Gain128 = 0x40
    Gain64 = 0x50
    Gain32 = 0x60
    Gain16 = 0x70
    Gain8 = 0x80
    Gain4 = 0x90
    Gain2 = 0xA0  # default
    Gain1 = 0xB0


class IntegrationTime(IntEnum):
    Time1ms = 0x00
    Time2ms = 0x01
    Time4ms = 0x02
    Time8ms = 0x03
    Time16ms = 0x04
    Time32ms = 0x05
    Time64ms = 0x06  # default
    Time128ms = 0x07
    Time256ms = 0x08
    Time512ms = 0x09
    Time1024ms = 0x0A
    Time2048ms = 0x0B
    Time4096ms = 0x0C
    Time8192ms = 0x0D
    Time16384ms = 0x0E


class MeasurementDivider(IntEnum):
    Divider1 = 0x00  # default
    Divider2 = 0x08
    Divider4 = 0x09
    Divider8 = 0x0A
    Divider16 = 0x0B
    Divider32 = 0x0C
    Divider64 = 0x0D
    Divider128 = 0x0E
    Divider256 = 0x0F


class MeasurementMode(IntEnum):
    ModeContinuous = 0x00
    ModeCommand = 0x40  # default
    ModeSynchronizedStart = 0x80
    ModeSynchronizedStartStop = 0xC0


class ReadyPinMode(IntEnum):
    ModePushPull = 0x00  # default
    ModeOpenDrain = 0x08


class InternalClockFrequency(IntEnum):
    Clock1024MHz = 0x00  # default
    Clock2048MHz = 0x01
    Clock4096MHz = 0x02
    Clock8192MHz = 0x03


class AS73211:
    def __init__(self, bus: i2cBus, address: int = 0x74):
        self._setTime = 0
        self.bus = bus
        self.address = address

    def begin(self):
        self.softwareReset()
        self.setGainAndTime(IntegrationGain.Gain1, IntegrationTime.Time256ms)
        self.setConfiguration()
        self.color6_writeByte(_COLOR6_REG_BREAK_TIME, 0x52)
        # self.color6_writeByte(_COLOR6_REG_EDGE_COUNT_VALUE, 0x01)
        self.setState(OperationalState.Measurement_state_Start_measurement)

    def setState(self, state: OperationalState) -> bool:
        delaySeconds(.5)
        self.color6_writeByte(_COLOR6_REG_OPERATIONAL_STATE, state)
        delaySeconds(.5)
        return self.getState() == state

    def getState(self) -> OperationalState:
        return OperationalState(self.color6_readByte(_COLOR6_REG_OPERATIONAL_STATE))

    def getDeviceID(self) -> int:
        return self.color6_readByte(_COLOR6_REG_API_GENERATION)

    def setGainAndTime(self, gain: IntegrationGain, time: IntegrationTime) -> bool:
        value = gain | time
        self.color6_writeByte(_COLOR6_REG_CONFIGURATION_REGISTER_1, value)
        return self.__getGainAndTime() == value

    def setGain(self, gain: IntegrationGain) -> bool:
        value = gain | self.getTime()
        self.color6_writeByte(_COLOR6_REG_CONFIGURATION_REGISTER_1, value)
        return self.__getGainAndTime() == value

    def setTime(self, time: IntegrationTime) -> bool:
        value = self.getGain() | time
        self.color6_writeByte(_COLOR6_REG_CONFIGURATION_REGISTER_1, value)
        return self.__getGainAndTime() == value

    def __getGainAndTime(self) -> int:
        return self.color6_readByte(_COLOR6_REG_CONFIGURATION_REGISTER_1)

    def getGain(self) -> IntegrationGain:
        return IntegrationGain(self.__getGainAndTime() & 0xF0)

    def getGainValue(self) -> int:
        switch = {
            IntegrationGain.Gain2048: 2048,
            IntegrationGain.Gain1024: 1024,
            IntegrationGain.Gain512: 512,
            IntegrationGain.Gain256: 256,
            IntegrationGain.Gain128: 128,
            IntegrationGain.Gain64: 64,
            IntegrationGain.Gain32: 32,
            IntegrationGain.Gain16: 16,
            IntegrationGain.Gain8: 8,
            IntegrationGain.Gain4: 4,
            IntegrationGain.Gain2: 2,
            IntegrationGain.Gain1: 1
        }
        return switch.get(self.getGain(), 0)

    def getTime(self) -> IntegrationTime:
        return IntegrationTime(self.__getGainAndTime() & 0x0F)

    def getTimeValue(self) -> int:
        switch = {
            IntegrationTime.Time1ms: 1,
            IntegrationTime.Time2ms: 2,
            IntegrationTime.Time4ms: 4,
            IntegrationTime.Time8ms: 8,
            IntegrationTime.Time16ms: 16,
            IntegrationTime.Time32ms: 32,
            IntegrationTime.Time64ms: 64,
            IntegrationTime.Time128ms: 128,
            IntegrationTime.Time256ms: 256,
            IntegrationTime.Time512ms: 512,
            IntegrationTime.Time1024ms: 1024,
            IntegrationTime.Time2048ms: 2048,
            IntegrationTime.Time4096ms: 4096,
            IntegrationTime.Time8192ms: 8192,
            IntegrationTime.Time16384ms: 16384
        }
        return switch.get(self.getTime(), 0)

    def getTimeValueBuffer(self) -> int:
        switch = {
            IntegrationTime.Time1ms: 1,
            IntegrationTime.Time2ms: 2,
            IntegrationTime.Time4ms: 4,
            IntegrationTime.Time8ms: 8,
            IntegrationTime.Time16ms: 16,
            IntegrationTime.Time32ms: 32,
            IntegrationTime.Time64ms: 64,
            IntegrationTime.Time128ms: 128,
            IntegrationTime.Time256ms: 256,
            IntegrationTime.Time512ms: 512,
            IntegrationTime.Time1024ms: 1024,
            IntegrationTime.Time2048ms: 2048,
            IntegrationTime.Time4096ms: 4096,
            IntegrationTime.Time8192ms: 8192,
            IntegrationTime.Time16384ms: 16384
        }
        return switch.get(self._setTime, 0)

    def setConfiguration(self,
                         measureTemperature: bool = True,
                         divider: MeasurementDivider = MeasurementDivider.Divider1,
                         mmode: MeasurementMode = MeasurementMode.ModeContinuous,
                         standby: bool = False,
                         readyMode: ReadyPinMode = ReadyPinMode.ModePushPull,
                         freq: InternalClockFrequency = InternalClockFrequency.Clock1024MHz) -> bool:
        conf2 = (0x40 if measureTemperature else 0x00) | divider
        conf3 = mmode | (0x10 if standby else 0x00) | readyMode | freq
        self.color6_writeByte(_COLOR6_REG_CONFIGURATION_REGISTER_2, conf2)
        self.color6_writeByte(_COLOR6_REG_CONFIGURATION_REGISTER_3, conf3)
        return (0b01001111 & self.color6_readByte(_COLOR6_REG_CONFIGURATION_REGISTER_2)) == conf2 and \
               (0b11011011 & self.color6_readByte(_COLOR6_REG_CONFIGURATION_REGISTER_3)) == conf3

    def getDivider(self) -> MeasurementDivider:
        value = self.color6_readByte(_COLOR6_REG_CONFIGURATION_REGISTER_2) & 0x0F
        return MeasurementDivider(0 if value < 0x8 else value)

    def setDivider(self, divider: MeasurementDivider) -> bool:
        value = self.color6_readByte(_COLOR6_REG_CONFIGURATION_REGISTER_2) & 0xF0 | divider
        self.color6_writeByte(_COLOR6_REG_CONFIGURATION_REGISTER_2, value)
        return self.color6_readByte(_COLOR6_REG_CONFIGURATION_REGISTER_2) == value

    def getDividerValue(self) -> int:
        switch = {
            MeasurementDivider.Divider1: 1,
            MeasurementDivider.Divider2: 2,
            MeasurementDivider.Divider4: 4,
            MeasurementDivider.Divider8: 8,
            MeasurementDivider.Divider16: 16,
            MeasurementDivider.Divider32: 32,
            MeasurementDivider.Divider64: 64,
            MeasurementDivider.Divider128: 128,
            MeasurementDivider.Divider256: 256
        }
        return switch.get(self.getDivider(), 0)

    def getClock(self) -> InternalClockFrequency:
        value = self.color6_readByte(_COLOR6_REG_CONFIGURATION_REGISTER_3) & 0x03
        return InternalClockFrequency(value)

    def getClockValue(self) -> int:
        switch = {
            InternalClockFrequency.Clock1024MHz: 1024,
            InternalClockFrequency.Clock2048MHz: 2048,
            InternalClockFrequency.Clock4096MHz: 4096,
            InternalClockFrequency.Clock8192MHz: 8192
        }
        return switch.get(self.getClock(), 0)

    def setClock(self, freq: InternalClockFrequency) -> bool:
        value = self.color6_readByte(_COLOR6_REG_CONFIGURATION_REGISTER_3) & 0xFC | freq
        self.color6_writeByte(_COLOR6_REG_CONFIGURATION_REGISTER_3, value)
        return self.color6_readByte(_COLOR6_REG_CONFIGURATION_REGISTER_3) == value

    def newDataAvailable(self) -> bool:
        return self.color6_readData(_COLOR6_MREG_STATUS_REGISTER) & _COLOR6_STATUS_NDATA != 0

    def color6_writeByte(self, reg: int, data: int):
        self.bus.write_byte_data(self.address, reg, data)

    def color6_readByte(self, reg: int) -> int:
        return self.bus.read_byte_data(self.address, reg)

    def color6_readData(self, reg: int) -> int:
        return self.bus.read_word_data(self.address, reg)

    def color6_readDataX(self) -> int:
        return self.bus.read_word_data(self.address, _COLOR6_MREG_MEASUREMENT_X_CHANNEL)

    def color6_readDataY(self) -> int:
        return self.bus.read_word_data(self.address, _COLOR6_MREG_MEASUREMENT_Y_CHANNEL)

    def color6_readDataZ(self) -> int:
        return self.bus.read_word_data(self.address, _COLOR6_MREG_MEASUREMENT_Z_CHANNEL)

    def getTemperature(self) -> float:
        channelData = self.color6_readData(_COLOR6_MREG_TEMPERATURE_MEASUREMENT)
        channelData = channelData & 0x0FFF
        floatData = (channelData * 0.05) - 66.9
        return floatData

    def softwareReset(self):
        self.setState(OperationalState.Software_reset)
        delaySeconds(.5)
