from pymodbus.client import ModbusSerialClient

client = ModbusSerialClient(
    port='/dev/ttyACM0',
    baudrate=9600,
    parity='E',
    stopbits=1,
    bytesize=8,
    timeout=1
)

if client.connect():
    print("CONNECTED")
    res = client.read_holding_registers(address=2, count=1, slave=1)
    print("RESULT:", res.registers if not res.isError() else res)
    client.close()
else:
    print("FAILED TO CONNECT")
