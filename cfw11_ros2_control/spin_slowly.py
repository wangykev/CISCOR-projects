from pymodbus.client import ModbusSerialClient
import time

# Set up Modbus RTU connection
client = ModbusSerialClient(
    port='/dev/ttyACM0',     # Change if needed
    baudrate=9600,
    parity='E',
    stopbits=1,
    bytesize=8,
    timeout=1
)

if client.connect():
    print("CONNECTED")

    # Write 20 RPM to P0683 (speed reference via serial)
    client.write_register(address=683, value=300, slave=1)
    print("Speed set to 20 RPM.")

    # Write 3 to P0682 (enable + run)
    client.write_register(address=682, value=3, slave=1)
    print("Motor running...")

    # Wait for 5 seconds
    time.sleep(5)

    # Stop motor: write 0 to P0682 (disable)
    client.write_register(address=682, value=0, slave=1)
    print("Motor stopped.")

    client.close()

else:
    print("FAILED TO CONNECT")
