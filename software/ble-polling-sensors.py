from bluepy import btle
import struct
import time

# Function to convert Little Endian byte data to Float
def bytes_to_float_le(byte_data):
    return struct.unpack('<f', byte_data)[0]

# Map characteristic UUIDs to their corresponding variable names
CHARACTERISTIC_NAMES = {
    "2A19": "Battery Level",
    "0000fff1-0000-1000-8000-00805f9b34fb": "Temperature",
    "0000fff2-0000-1000-8000-00805f9b34fb": "Humidity",
    "0000fff3-0000-1000-8000-00805f9b34fb": "Reduction Sensor",
    "0000fff4-0000-1000-8000-00805f9b34fb": "NH3 Sensor",
    "0000fff5-0000-1000-8000-00805f9b34fb": "OX Sensor",
}

def read_characteristics(peripheral, characteristics):
    print("\n--- Reading Characteristics ---")
    for uuid, name in characteristics.items():
        try:
            char = peripheral.getCharacteristics(uuid=uuid)[0]
            data = char.read()
            if uuid == "2A19":  # Battery Level is 1 byte
                value = data[0]
            else:  # Other values are float (Little Endian)
                value = bytes_to_float_le(data)
            print(f"{name}: {value:.2f}")
        except Exception as e:
            print(f"Failed to read {name}: {e}")

def main():
    try:
        # Scan and connect to the BLE device
        scanner = btle.Scanner()
        devices = scanner.scan(5.0)  # Scan for 5 seconds

        target_device = None
        DEVICE_NAME = "Winsource-sense"
        for device in devices:
            if DEVICE_NAME in [adv[2] for adv in device.getScanData()]:
                target_device = device
                break

        if not target_device:
            print(f"Device '{DEVICE_NAME}' not found!")
            return

        print(f"Connecting to {DEVICE_NAME} ({target_device.addr})...")
        global peripheral
        peripheral = btle.Peripheral(target_device.addr, btle.ADDR_TYPE_PUBLIC)

        print("Connected successfully!")
        print("Polling data every 5 seconds. Press Ctrl+C to stop.")

        while True:
            read_characteristics(peripheral, CHARACTERISTIC_NAMES)
            time.sleep(5)  # Poll every 5 seconds

    except KeyboardInterrupt:
        print("\nStopping the script...")
    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        if 'peripheral' in globals():
            peripheral.disconnect()
            print("Disconnected successfully.")

if __name__ == "__main__":
    main()
