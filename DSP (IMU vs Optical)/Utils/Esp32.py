from collections import deque
import random
import serial

ser: serial.Serial
# SensorRAW = deque(maxlen=1000)

def register_esp():
    global ser
    #Configure the serial port (replace with your port and baudrate)
    ser = serial.Serial('COM7', 250000, timeout=1)
    return ser

def get_random_data():
    """
    Simulate Bluetooth data for 14 body parts with values rounded to 2 decimal places.
    """
    return {
        # "head": tuple(round(random.random(), 2) for _ in range(13)),
        "chest": tuple(round(random.random(), 2) for _ in range(13)),
        "l_shoulder": tuple(round(random.random(), 2) for _ in range(13)),
        "l_elbow": tuple(round(random.random(), 2) for _ in range(13)),
        # "l_wrist": tuple(round(random.random(), 2) for _ in range(13)),
        "r_shoulder": tuple(round(random.random(), 2) for _ in range(13)),
        "r_elbow": tuple(round(random.random(), 2) for _ in range(13)),
        # "r_wrist": tuple(round(random.random(), 2) for _ in range(13)),
        # "l_hip": tuple(round(random.random(), 2) for _ in range(13)),
        # "l_knee": tuple(round(random.random(), 2) for _ in range(13)),
        # "l_ankle": tuple(round(random.random(), 2) for _ in range(13)),
        # "r_hip": tuple(round(random.random(), 2) for _ in range(13)),
        # "r_knee": tuple(round(random.random(), 2) for _ in range(13)),
        # "r_ankle": tuple(round(random.random(), 2) for _ in range(13)),
    }



def read_esp32():
    global ser
    global parsed_data
    global body_parts

    if ser is None:
        raise ValueError("Serial port is not initialized. Call `register_esp` first.")



    # Initialize all body parts with zeroed data
    # body_parts = [
    # "head", "chest", "l_shoulder", "l_elbow", "l_wrist",
    # "r_shoulder", "r_elbow", "r_wrist", "l_hip",
    # "l_knee", "l_ankle", "r_hip", "r_knee", "r_ankle"
    # ]


    body_parts = ["chest", "l_shoulder", "l_elbow", "r_shoulder", "r_elbow" ]
    parsed_data = {part: tuple(0.0 for _ in range(13)) for part in body_parts}
    
    data_buffer = []

    try:
        max_tries = 100  # wait for ~100 empty reads
        tries = 0
        while True:
            # print(".")
            line = ser.readline().decode('utf-8').strip()
            if line == '':
                tries += 1
                if tries >= max_tries:
                    print("Timeout: No complete data received.")
                    return parsed_data  # or return None
                continue
            tries = 0  # reset tries if line received


            if "sent" in line.lower():
                # Process the received data
                for received_line in data_buffer:
                    try:
                        # Extract the body part and values
                        part_name, values = received_line.split(":")
                        values_tuple = tuple(map(float, values.split(',')))

                        # Update the parsed data for the received part
                        if part_name in body_parts:
                            parsed_data[part_name] = values_tuple
                    except Exception as e:
                        print(f"Error parsing line '{received_line}': {e}")

                # SensorRAW.append(parsed_data)
                # Return the complete parsed data with missing parts set to zero
                return parsed_data

            else:
                data_buffer.append(line)

    except Exception as e:
        print(f"Error reading data from ESP32: {e}")
        return None



def printData(data, filters):
    if not data:
        print("No data available.")
        return
    for part in filters:
        if part in data:
            values = data[part]
            print(f"{part}\t: {', '.join(f'{v:.2f}' for v in values)}")
        else:
            print(f"{part}: No data")
