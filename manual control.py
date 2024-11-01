import paho.mqtt.client as mqtt
from pynput import keyboard
import time

# MQTT setup
broker = "your_broker_address"
port = 1883
client = mqtt.Client()

def on_connect(client, userdata, flags, rc):
    print("Connected with result code " + str(rc))

client.on_connect = on_connect
client.connect(broker, port, 60)

# Key status dictionary
key_status = {
    'up': 0,
    'down': 0,
    'left': 0,
    'right': 0
}

# Define key press and release handlers
def on_press(key):
    try:
        if key == keyboard.Key.up:
            key_status['up'] = 1
            client.publish('up', 1)
        elif key == keyboard.Key.down:
            key_status['down'] = 1
            client.publish('down', 1)
        elif key == keyboard.Key.left:
            key_status['left'] = 1
            client.publish('left', 1)
        elif key == keyboard.Key.right:
            key_status['right'] = 1
            client.publish('right', 1)
    except AttributeError:
        pass

def on_release(key):
    try:
        if key == keyboard.Key.up:
            key_status['up'] = 0
            client.publish('up', 0)
        elif key == keyboard.Key.down:
            key_status['down'] = 0
            client.publish('down', 0)
        elif key == keyboard.Key.left:
            key_status['left'] = 0
            client.publish('left', 0)
        elif key == keyboard.Key.right:
            key_status['right'] = 0
            client.publish('right', 0)
    except AttributeError:
        pass

# Start the MQTT client loop
client.loop_start()

# Start listening to keyboard events
with keyboard.Listener(on_press=on_press, on_release=on_release) as listener:
    listener.join()

# Stop the Client
client.loop_stop()
# Disconnect
client.disconnect()