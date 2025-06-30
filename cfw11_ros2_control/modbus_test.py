from pymodbus.client import ModbusSerialClient

# Set up the Modbus RTU client
client = ModbusSerialClient(
    port='/dev/ttyACM0',
    baudrate=9600,
    stopbits=1,
    bytesize=8,
    parity='N',
    timeout=1
)

# Connect to the device
if client.connect():
    print("✅ Connected to CFW-11")

    # Try reading holding register 2 (speed feedback)
    result = client.read_holding_registers(2, 1, slave=1)

    if not result.isError():
        print(f"Register 2 value (speed feedback): {result.registers[0]}")
    else:
        print("❌ Failed to read register")

    client.close()
else:
    print("❌ Could not connect to the CFW-11")
