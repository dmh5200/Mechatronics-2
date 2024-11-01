import paho.mqtt.client as mqtt
import numpy as np
import struct

# Example function to convert and send array elements to Simulink
def convert_to_simulink_format(array, MQTT_topic):

    for element in array:
        # Convert the element to the required format
        sign = np.uint8(0) if element >= 0 else np.uint8(1)  # 0 for positive, 1 for negative
        abs_element = abs(element)
        int_part_high = np.uint8(abs_element // 100)
        int_part_low = np.uint8((abs_element % 100) // 1)
        decimal_part = np.uint8((abs_element % 1) * 100)

        data = str(f"{sign}, {int_part_high}, {int_part_low}, {decimal_part}")


        # Convert numpy array to bytearray
        #uint8_data_bytearray = bytearray(int_data_np)

        # Publish the uint8 array as bytearray
        client.publish(MQTT_topic, data, qos=2, retain=True)

# Initialize MQTT client
client = mqtt.Client()

# Set the username and password
client.username_pw_set("student", password="HousekeepingGlintsStreetwise")

# Connect to the server using a specific port with a timeout delay (in seconds)
client.connect("fesv-mqtt.bath.ac.uk", 31415, 60)

# Start the MQTT client loop in a separate thread
client.loop_start()

# Example usage
while True:
    array = [123.45, -267.89, 0.12]
    MQTT_topic = "Wall-E/sensor/data"
    convert_to_simulink_format(array, MQTT_topic)