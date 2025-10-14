#!/usr/bin/env python

import json
import grovepi
from grovepi import *
import time
import paho.mqtt.publish as publish
import paho.mqtt.client as mqtt
import math

# Set I2C to use the hardware bus
grovepi.set_bus("RPI_1")

# Connect the Grove Ultrasonic Ranger to digital port D4
# SIG,NC,VCC,GND
ultrasonic_ranger = 4
sound_port = 1
buzzer = 3
pinMode(buzzer, "OUTPUT")

def on_message(client, userdata, msg):
    print(f"received message: {msg.payload.decode()} from {msg.topic}")

    message = msg.payload.decode('utf-8')
    if message.strip().lower() == "danger":
        digitalWrite(buzzer, 1)
    elif message.strip().lower() == "safe":
        digitalWrite(buzzer, 0)

client = mqtt.Client()
client.on_message = on_message
client.connect("10.148.187.246", 1883, 60)
client.subscribe("turtlebot/status")
client.loop_start()

while True:
    try:
        sound = grovepi.analogRead(sound_port)
        # print(f"sound = {sound}")

        # Read distance value from Ultrasonic
        distance = grovepi.ultrasonicRead(ultrasonic_ranger)
        # print(f"Distance = {distance}")

        data = {"sound": sound, "distance": distance}
        message = json.dumps(data)

        publish.single("rangeTopic", message)

    except Exception as e:
        print("Error: {}".format(e))

    time.sleep(1)  # don't overload the I2C bus
