import AS73211
import socket
import smbus as i2c
from threading import *
from time import sleep as delaySeconds


def control(b: int) -> bool:
    cmd = (b >> 6) & 0b11
    if cmd == 3:
        cmd = (b >> 4) & 0b11
        data = b & 0b1111
        if cmd == 0 and data <= 0x0b:
            return color.setGain(AS73211.IntegrationGain(data))
        elif cmd == 1 and data <= 0x0e:
            return color.setTime(AS73211.IntegrationTime(data))
        elif cmd == 2 and data <= 0x08:
            if data > 0:
                data = data - 1
                data = data | 0x8
            return color.setDivider(AS73211.MeasurementDivider(data))
        elif cmd == 3:
            if data <= 3:
                return color.setClock(AS73211.InternalClockFrequency(data))
            elif data == 4:
                return color.setState(AS73211.OperationalState.Configuration_state)
            elif data == 5:
                return color.setState(AS73211.OperationalState.Measurement_state_Start_measurement)
            elif data == 6:
                color.softwareReset()
                return True
    return False


def sendStringln(s: socket, stringToSend:str):
    s.sendAll((stringToSend + "\n").encode("utf-8"))


def send(s: socket):
    available = color.newDataAvailable()
    state = color.getState()
    if state == AS73211.OperationalState.Configuration_state:
        sendStringln(s, "g:" + color.getGainValue().__str__())
        sendStringln(s, "t:" + color.getTimeValue().__str__())
        sendStringln(s, "d:" + color.getDividerValue().__str__())
        sendStringln(s, "c:" + color.getClockValue().__str__())
        delaySeconds(.25)
    elif state == AS73211.OperationalState.Measurement_state_Start_measurement:
        if available:
            sendStringln(s, "X:" + color.color6_readDataX().__str__())
            sendStringln(s, "Y:" + color.color6_readDataY().__str__())
            sendStringln(s, "Z:" + color.color6_readDataZ().__str__())
            sendStringln(s, "T:" + color.getTemperature().__str__())
        else:
            delaySeconds(color.getTimeValueBuffer() / 4000)


class Client(Thread):
    def __init__(self, client: socket, address):
        Thread.__init__(self)
        self.sock = client
        self.addr = address
        self.start()

    def run(self):
        while True:
            try:
                data = self.sock.recv(1)
                control(data[0])
            except BlockingIOError:
                send(self.sock)


color = AS73211.AS73211(i2c.SMBus(1))
color.begin()

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

print(socket.gethostname())
sock.bind((socket.gethostname(), 9001))
sock.listen(5)

while True:
    client, address = sock.accept()
    client.setblocking(False)
    Client(client, address)
