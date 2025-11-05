#pip install pymodbus
from pymodbus.client import ModbusSerialClient
from pymodbus.client import ModbusTcpClient
import time
import threading


def wait_for_enter(stop_event):
    input("Нажмите Enter для остановки...\n")
    stop_event.set()  # Устанавливаем событие

client = ModbusSerialClient('COM3', baudrate=115200)
client.connect()

# Создаем событие для остановки
stop_event = threading.Event()

# Запускаем поток для ожидания Enter
stop_thread = threading.Thread(target=wait_for_enter, args=(stop_event,))
stop_thread.daemon = True
stop_thread.start()

#client.write_coil(1, True)
while not stop_event.is_set():
    result = client.read_coils(address=0, count=8) #its, works
    print(result.bits)
    stop_event.wait(1)
client.close()