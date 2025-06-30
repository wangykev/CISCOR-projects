from pymodbus.client import ModbusSerialClient

# Set up Modbus RTU connection
client = ModbusSerialClient(
    port='/dev/ttyACM0',     # Adjust if needed
    baudrate=9600,
    parity='E',
    stopbits=1,
    bytesize=8,
    timeout=1
)

if client.connect():
    print("CONNECTED")

    # Set speed reference (P001) to 20 RPM
    result = client.write_register(address=1, value=20, slave=1)
    res = client.read_holding_registers(address=1, count=1, slave=1)
    print("Speed reference (P001):", res.registers[0])

    if result.isError():
        print("Failed to write speed.")
    else:
        print("Speed set to 20 RPM.")

    client.close()
    print("Now press the green RUN button on the drive (while in REM mode).")

else:
    print("FAILED TO CONNECT")
