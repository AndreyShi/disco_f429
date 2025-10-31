#pip install pymodbus
from pymodbus.client import ModbusSerialClient
from pymodbus.client import ModbusTcpClient
import time

client = ModbusSerialClient('COM11', baudrate=115200)
client.connect()
#client.write_coil(1, True)
while 1:
    result = client.read_coils(address=0, count=8) #its, works
    print(result.bits)
    time.sleep(0.5)
client.close()