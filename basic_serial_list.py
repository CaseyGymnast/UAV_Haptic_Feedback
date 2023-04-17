# Importing Libraries
import serial
import time
import json

arduino = serial.Serial(port='/dev/ttyACM0', baudrate=9600, timeout=.1)
drv = {}

while True:
    drv["up"] = input("Enter first number: ") # Taking input from user
    drv["right"] = input("Enter second number: ") # Taking input from user
    data = json.dumps(drv)
    
    if arduino.isOpen():
        arduino.write(data.encode('ascii'))
        arduino.flush()
        try:
            incoming = arduino.readline().decode("utf-8")
            print(incoming)
        except Exception as e:
            print(e)
            pass
    else:
        print("opening error")