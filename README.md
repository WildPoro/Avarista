# Avarista

Avarista is a voice-controlled smart coffee machine built using a Raspberry Pi and Python. It lets users order drinks through voice commands, handling everything from heating and milk cooling to pumping and real-time feedback. The goal was to create a fully hands-free, interactive coffee experience that feels both simple and intelligent.

Features:

Custom wake word and voice command detection using Picovoice

Voice-to-text command parsing and confirmation

Text-to-speech responses using Google Cloud Text-to-Speech

Real-time temperature monitoring with DS18B20 sensors

Milk cooling via Peltier module and heater control with safety logic

Ingredient control through pumps, servo, and stepper motors

Weight-based water detection and ultrasonic level sensing

OLED display to show live system status and temperatures

Flask-based web interface for manual override and status monitoring

Built-in logic to block unsafe operations (e.g. milk too cold)


Technologies Used:

Raspberry Pi 4B

Python

Flask

Google Cloud TTS

Picovoice (Porcupine + Rhino)

DS18B20 sensors, Peltier module, relays, pumps, servo and stepper motors

Load cell, ultrasonic sensor, I2C OLED display

GPIO, I2C, and 1-Wire protocols



How It Works:

The machine waits for a wake word like "Hey Ava"

It listens for a drink command (for example: "Make me a large coffee double milk")

Ava replies with a confirmation using speech

The drink process starts by activating pumps, motors, and temperature control

OLED display shows system status, temperatures, and alerts during the process

