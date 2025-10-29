#pip install pymodbus
from pymodbus.client import ModbusSerialClient
from pymodbus.client import ModbusTcpClient

client = ModbusSerialClient('COM4', baudrate=9600)
client.connect()
#client.write_coil(1, True)
result = client.read_coils(address=0, count=8) #its, works
print(result.bits)
client.close()