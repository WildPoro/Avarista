#!/usr/bin/env python3

# Standard library imports
import struct
import wave
import time
import os
import random

# Hardware control imports
import RPi.GPIO as GPIO  # Raspberry Pi GPIO control
from threading import Thread, Event, Lock  # For concurrent operations
from gpiozero import OutputDevice  # For stepper motor control
from w1thermsensor import W1ThermSensor, SensorNotReadyError  # Temperature sensors

# Display imports
from luma.core.interface.serial import i2c  # OLED interface
from luma.oled.device import sh1106  # OLED driver
from luma.core.render import canvas  # Display rendering
from PIL import Image, ImageDraw, ImageFont  # Image handling

# Voice recognition imports
import pvporcupine  # Wake word detection
import pvrhino  # Speech-to-intent engine
from pvrecorder import PvRecorder  # Audio recording

# Web interface imports
from flask import Flask, jsonify, render_template, request  # Web server

# Audio imports
from gtts import gTTS  # Text-to-speech
import pygame  # Audio playback

# ================ CONFIGURATION SECTION ================
# Disable GPIO warnings to avoid console clutter
GPIO.setwarnings(False)

# Initialize Flask web application
app = Flask(__name__)

# ================ GLOBAL VARIABLES ================
# System state flags
cleaning_active = False  # True when cleaning cycle is running
cleaning_lock = Lock()  # Thread lock for cleaning operations
heater_on = False  # Water heater state (True when on)
manual_control = False  # True during manual override
manual_control_end_time = 0  # Timestamp when manual control ends
peltier_on = False  # Milk cooler state (True when on)
oled_on = True  # OLED display state (True when on)
coffee_count = 0  # Counter for maintenance reminders

# ================ GPIO SETUP ================
# Pin Definitions (BCM numbering)
BUTTON_PIN = 17    # System toggle button
SERVO_PIN = 12     # Coffee gate servo
WATER_PUMP_PIN = 6 # Water pump relay
MILK_PUMP_PIN = 13 # Milk pump relay
STEP_PIN = 24      # Stepper motor step
DIR_PIN = 23       # Stepper motor direction
HEATER_PIN = 5     # Water heater relay
PELTIER_PIN = 19   # Peltier cooler
TRIG = 21          # Ultrasonic sensor trigger
ECHO = 20          # Ultrasonic sensor echo

# Initialize GPIO
GPIO.setmode(GPIO.BCM)  # Use BCM numbering scheme

# Set up input pin with pull-up resistor
GPIO.setup(BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# Set up all output pins
GPIO.setup(SERVO_PIN, GPIO.OUT)
GPIO.setup(WATER_PUMP_PIN, GPIO.OUT)
GPIO.setup(MILK_PUMP_PIN, GPIO.OUT)
GPIO.setup(HEATER_PIN, GPIO.OUT)
GPIO.setup(PELTIER_PIN, GPIO.OUT)
GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)

# Initialize all outputs to OFF state (HIGH for relay control)
GPIO.output(WATER_PUMP_PIN, GPIO.HIGH)
GPIO.output(MILK_PUMP_PIN, GPIO.HIGH)
GPIO.output(HEATER_PIN, GPIO.HIGH)
GPIO.output(PELTIER_PIN, GPIO.HIGH)

# ================ HARDWARE INITIALIZATION ================
# Initialize servo motor with 50Hz PWM
servo_pwm = GPIO.PWM(SERVO_PIN, 50)
servo_pwm.start(0)  # Start with 0% duty cycle

# Initialize temperature sensors with their hardware IDs
watersensor = W1ThermSensor(sensor_id="0620186c0481")  # Water temp
milksensor = W1ThermSensor(sensor_id="0620189f6271")   # Milk temp

# Initialize OLED display over I2C
serial = i2c(port=1, address=0x3C)  # I2C port 1, address 0x3C
oled = sh1106(serial)  # SH1106 OLED display
font = ImageFont.load_default()  # Load default system font

# Initialize stepper motor control
step_pin_device = OutputDevice(STEP_PIN)  # Step pin control
dir_pin_device = OutputDevice(DIR_PIN)    # Direction pin control
stepper_event = Event()  # Event to control stepper operation
stepper_event.set()  # Initially allow stepper to run

# Voice control event flag
rhino_running = Event()
rhino_running.set()  # Initially enable voice control

# ================ CORE FUNCTIONS ================
def speak(text, recorder=None):
    """
    Convert text to speech and play through audio output
    Args:
        text: The text to be spoken
        recorder: Optional audio recorder to pause during speech
    """
    # Generate speech file using Google TTS
    tts = gTTS(text=text, lang='en')
    tts.save("output.mp3")
    
    # Initialize pygame mixer and load speech file
    pygame.mixer.init()
    pygame.mixer.music.load("output.mp3")
    
    # Pause recording while speaking if recorder is provided
    if recorder:
        recorder.stop()
    
    # Play the generated speech
    pygame.mixer.music.play()
    # Wait for playback to complete
    while pygame.mixer.music.get_busy():
        time.sleep(0.1)
    
    # Resume recording if it was paused
    if recorder:
        recorder.start()

def get_distance():
    """
    Measure distance using ultrasonic sensor
    Returns:
        Distance in centimeters (999.0 if timeout occurs)
    """
    # Send trigger pulse
    GPIO.output(TRIG, False)
    time.sleep(0.1)
    GPIO.output(TRIG, True)
    time.sleep(0.00001)  # 10µs pulse
    GPIO.output(TRIG, False)

    # Measure echo pulse duration
    start_time = time.time()
    while GPIO.input(ECHO) == 0:
        pulse_start = time.time()
        if pulse_start - start_time > 0.05:  # Timeout after 50ms
            return 999.0
    
    while GPIO.input(ECHO) == 1:
        pulse_end = time.time()
        if pulse_end - pulse_start > 0.05:  # Timeout if echo too long
            break
    
    # Calculate distance (speed of sound is 34300 cm/s)
    distance = (pulse_end - pulse_start) * 34300 / 2
    return round(distance, 2)

def set_servo_angle(angle):
    """
    Set servo motor to specified angle
    Args:
        angle: Desired angle (0-180 degrees)
    """
    # Convert angle to duty cycle (2-12% for 0-180 degrees)
    duty = 2 + (angle / 18.0)
    servo_pwm.ChangeDutyCycle(duty)
    time.sleep(0.5)  # Allow time for servo to move
    servo_pwm.ChangeDutyCycle(0)  # Stop sending signal

def rotate_stepper():
    """
    Continuously oscillate stepper motor 45 degrees forward and back
    Runs until stepper_event is set
    """
    steps_per_degree = 20  # Steps needed per degree of rotation
    steps_for_45_deg = int(45 * steps_per_degree)
    
    # Main stepper control loop
    while not stepper_event.is_set():
        # Rotate 45 degrees clockwise
        dir_pin_device.on()  # Set direction
        for _ in range(steps_for_45_deg):
            if stepper_event.is_set():
                break
            step_pin_device.on()  # Send step pulse
            time.sleep(0.002)     # Pulse width
            step_pin_device.off() # End pulse
            time.sleep(0.002)     # Time between steps
        
        if stepper_event.is_set():
            break
            
        time.sleep(0.1)  # Brief pause
        
        # Rotate 45 degrees counter-clockwise
        dir_pin_device.off()  # Reverse direction
        for _ in range(steps_for_45_deg):
            if stepper_event.is_set():
                break
            step_pin_device.on()
            time.sleep(0.002)
            step_pin_device.off()
            time.sleep(0.002)
        
        time.sleep(0.1)  # Brief pause
    
    print("Stepper motor stopped")

def update_temperature_display():
    """
    Continuously update OLED display with system status
    Runs in a dedicated thread
    """
    global cleaning_active
    while True:
        if not oled_on:  # Display is turned off
            oled.clear()
            oled.show()
            time.sleep(2)
            continue

        try:
            if cleaning_active:  # Show cleaning mode
                with canvas(oled) as draw:
                    draw.text((0, 25), "MILK CLEANING MODE", font=font, fill=255)
                time.sleep(2)
                continue

            # Get current sensor readings
            water_temp = watersensor.get_temperature()
            milk_temp = milksensor.get_temperature()
            distance = get_distance()
            
            # Determine milk level indicator
            if 0 <= distance < 7:
                milk_indicator = "●●●"
            elif 7 <= distance < 14:
                milk_indicator = "●●"
            else:
                milk_indicator = "●"

            # Update display with current readings
            with canvas(oled) as draw:
                draw.text((0, 0), f"Water: {water_temp:.2f}°C", font=font, fill=255)
                draw.text((0, 20), f"Milk: {milk_temp:.2f}°C", font=font, fill=255)
                draw.text((0, 40), f"Milk Level: {milk_indicator}", font=font, fill=255)
                
        except SensorNotReadyError:
            # Show error if sensors fail
            with canvas(oled) as draw:
                draw.text((0, 25), "Sensor Error!", font=font, fill=255)
        time.sleep(2)  # Update interval

def run_cleaning_cycle():
    """
    Run 2-minute milk system cleaning cycle
    Activates milk pump continuously for cleaning
    """
    global cleaning_active
    GPIO.output(MILK_PUMP_PIN, GPIO.LOW)  # Start pump
    start_time = time.time()

    # Run for 2 minutes or until cleaning_active is False
    while cleaning_active and (time.time() - start_time < 120):
        time.sleep(1)

    with cleaning_lock:
        cleaning_active = False
        GPIO.output(MILK_PUMP_PIN, GPIO.HIGH)  # Stop pump
        rhino_running.set()  # Re-enable voice control
    print("Milk cleaning cycle completed")

def auto_control_heater():
    """
    Automatically control water heater based on temperature
    Maintains water temperature above 2°C
    Runs in a dedicated thread
    """
    global heater_on, manual_control, manual_control_end_time
    while True:
        try:
            water_temp = watersensor.get_temperature()
        except SensorNotReadyError:
            print("Water sensor not ready!")
            time.sleep(2)
            continue

        # Check for manual override
        if manual_control:
            if time.time() <= manual_control_end_time:
                print("Manual override active...")
                time.sleep(2)
                continue
            else:
                manual_control = False

        # Automatic temperature control
        if water_temp < 2 and not heater_on:
            GPIO.output(HEATER_PIN, GPIO.LOW)  # Turn on heater
            heater_on = True
            print(f"Heater ON (auto): {water_temp:.2f}°C")
        elif water_temp >= 2 and heater_on:
            GPIO.output(HEATER_PIN, GPIO.HIGH)  # Turn off heater
            heater_on = False
            print(f"Heater OFF (auto): {water_temp:.2f}°C")
        time.sleep(2)  # Check interval

def auto_control_peltier():
    """
    Automatically control milk cooler based on temperature
    Keeps milk temperature below 60°C
    Runs in a dedicated thread
    """
    global peltier_on
    while True:
        try:
            milk_temp = milksensor.get_temperature()
        except SensorNotReadyError:
            print("Milk sensor not ready!")
            time.sleep(2)
            continue

        if milk_temp > 60 and not peltier_on:
            GPIO.output(PELTIER_PIN, GPIO.LOW)  # Turn on cooler
            peltier_on = True
            print(f"Peltier ON (auto): {milk_temp:.2f}°C")
            speak("Milk temperature is high. Peltier is running.")
        elif milk_temp <= 60 and peltier_on:
            GPIO.output(PELTIER_PIN, GPIO.HIGH)  # Turn off cooler
            peltier_on = False
            print(f"Peltier OFF (auto): {milk_temp:.2f}°C")
        time.sleep(2)  # Check interval

def make_coffee(size, milk_count):
    """
    Prepare coffee based on size and milk amount
    Args:
        size: Coffee size ("regular" or "large")
        milk_count: Number of milk portions (0-3)
    """
    global coffee_count

    # Ensure water is warm enough
    water_temp = watersensor.get_temperature()
    if water_temp < 20:
        GPIO.output(HEATER_PIN, GPIO.LOW)  # Turn on heater
        print("Heater ON for coffee making...")
        while True:
            water_temp = watersensor.get_temperature()
            if water_temp >= 20:
                break
            time.sleep(2)
    else:
        GPIO.output(HEATER_PIN, GPIO.HIGH)  # Ensure heater is off

    # Set brewing parameters based on size
    if size == "regular":
        servo_duration = 5  # Seconds to leave gate open
        water_pump_intervals = [(4, 3), (4, 0)]  # (ON, OFF) seconds
    elif size == "large":
        servo_duration = 8
        water_pump_intervals = [(3, 3), (3, 3), (3, 3), (3, 0)]
    else:
        print("Invalid coffee size requested.")
        return

    # Check milk temperature if adding milk
    try:
        milk_temp = milksensor.get_temperature()
        if milk_temp > 60 and milk_count > 0:
            print("Milk temperature is too high. Cannot add milk.")
            speak("Sorry, milk temperature is too high. Please order again without milk.")
            return
    except SensorNotReadyError:
        print("Milk sensor not ready!")
        speak("Milk sensor error. Please check the system.")
        return

    # Check milk level if adding milk
    if milk_count > 0:
        milk_distance = get_distance()
        print(f"Milk level distance: {milk_distance} cm")
        if milk_distance >= 15:  # Threshold for empty
            print("Out of milk, please refill milk. Order cancelled.")
            speak("Out of milk, please refill milk. Order cancelled.")
            return

    # Coffee preparation sequence
    # Open coffee gate
    set_servo_angle(90)  # Open position
    time.sleep(servo_duration)
    set_servo_angle(180)  # Close position

    # Start stepper motor for grinding
    stepper_event.clear()
    stepper_thread = Thread(target=rotate_stepper)
    stepper_thread.start()

    # Pump water in intervals
    for on_time, off_time in water_pump_intervals:
        GPIO.output(WATER_PUMP_PIN, GPIO.LOW)  # Pump ON
        time.sleep(on_time)
        GPIO.output(WATER_PUMP_PIN, GPIO.HIGH)  # Pump OFF
        if off_time > 0:
            time.sleep(off_time)

    speak("Your coffee is being prepared. Don't forget to remove the filter", None)

    # Add milk if requested
    if milk_count > 0:
        milk_pump_duration = 6 * milk_count  # 6 seconds per milk portion
        GPIO.output(MILK_PUMP_PIN, GPIO.LOW)  # Milk pump ON
        time.sleep(milk_pump_duration)
        GPIO.output(MILK_PUMP_PIN, GPIO.HIGH)  # Milk pump OFF
        print(f"Milk pump cycle complete ({milk_pump_duration}s).")
    else:
        print("No milk requested.")

    # Clean up
    stepper_event.set()  # Stop stepper
    stepper_thread.join()
    print("Coffee making complete.")

    # Maintenance reminder logic
    coffee_count += 1
    if coffee_count >= 3:  # Remind every 3 coffees
        coffee_count = 0
        speak("Please check water level")

class CoffeeMachineRhino(Thread):
    """
    Voice control interface using Picovoice Rhino
    Handles wake word detection and command processing
    Runs in a dedicated thread
    """
    def __init__(self):
        super().__init__()
        self.stop = False  # Flag to stop the thread
        self.rhino = None  # Speech-to-intent engine
        self.porcupine = None  # Wake word detector
        self.recorder = None  # Audio recorder

    def run(self):
        """Main voice control loop"""
        global rhino_running, stepper_event, cleaning_active
        try:
            # Initialize Picovoice components
            self.porcupine = pvporcupine.create(
                access_key="#########################################################",
                keyword_paths=["#########################################################"]
            )
            self.rhino = pvrhino.create(
                access_key="#########################################################",
                library_path="#########################################################",
                model_path="#########################################################",
                context_path="#########################################################"
            )
            self.recorder = PvRecorder(device_index=-1, frame_length=self.porcupine.frame_length)
            self.recorder.start()

            print("Listening for wakeword...")
            while not self.stop:
                rhino_running.wait()  # Pause if voice control disabled
                if cleaning_active:  # Skip during cleaning
                    time.sleep(1)
                    continue

                # Process audio input
                pcm = self.recorder.read()
                keyword_index = self.porcupine.process(pcm)
                
                if keyword_index >= 0:  # Wake word detected
                    print("Wakeword Detected: 'Hi Ava'")
                    speak("Hello, what drink would you like to have?", self.recorder)
                    self.process_command()
                    print("Returning to wakeword detection...")
                    
        except Exception as e:
            print(f"Error in Rhino run: {str(e)}")
        finally:
            # Clean up resources
            if self.recorder:
                self.recorder.delete()
            if self.porcupine:
                self.porcupine.delete()
            if self.rhino:
                self.rhino.delete()

    def process_command(self):
        """Process voice commands after wake word"""
        print("Listening for coffee orders...")
        try:
            while True:
                rhino_running.wait()
                if cleaning_active:
                    return

                pcm = self.recorder.read()
                if self.rhino.process(pcm):
                    inference = self.rhino.get_inference()
                    if inference.is_understood:
                        # Extract order details from voice command
                        size = inference.slots.get("size", "regular")
                        milk_text = inference.slots.get("numberOfMilks", "no milk")
                        milk_count = {
                            "no milk": 0,
                            "single milk": 1,
                            "double milk": 2,
                            "triple milk": 3
                        }.get(milk_text.lower(), 0)
                        print(f"Received order: {size} coffee with {milk_count} milk(s)")

                        # Acknowledge order
                        response = random.choice(["OK", "Coming right up"])
                        speak(response, self.recorder)

                        # Start stepper motor
                        stepper_event.clear()
                        stepper_thread = Thread(target=rotate_stepper)
                        stepper_thread.start()

                        # Prepare the coffee
                        make_coffee(size, milk_count)

                        print("Returning to wakeword detection...")
                        return
                    else:
                        print("Command not understood.")
                        speak("I didn't get that, can you please repeat?", self.recorder)
                        return
        except Exception as e:
            print(f"Error in process_command: {str(e)}")

def toggle_devices():
    """
    Handle button press to toggle OLED and microphone
    Runs in a dedicated thread
    """
    global rhino_running, oled_on
    button_was_pressed = False
    while True:
        button_state = GPIO.input(BUTTON_PIN)
        
        # Button pressed (LOW because of pull-up)
        if button_state == GPIO.LOW and not button_was_pressed:
            print("Button Pressed! Turning OFF OLED & Mic...")
            button_was_pressed = True
            oled_on = False
            oled.clear()
            oled.show()
            os.system("sudo modprobe -r snd_usb_audio")  # Disable USB mic
            print("USB Microphone Disabled")
            rhino_running.clear()  # Disable voice control
            print("Wake-word detection paused")
            speak("Bye bye")
        
        # Button released
        elif button_state == GPIO.HIGH and button_was_pressed:
            print("Button Released! Turning ON OLED & Mic...")
            button_was_pressed = False
            oled_on = True
            os.system("sudo modprobe snd_usb_audio")  # Enable USB mic
            print("USB Microphone Enabled")
            rhino_running.set()  # Enable voice control
            print("Wake-word detection resumed")
            speak("Welcome back")
        
        time.sleep(0.1)  # Debounce interval

# ================ WEB INTERFACE ROUTES ================
@app.route('/')
def index():
    """Serve main web interface page"""
    return render_template('index.html')

@app.route('/api/temperature', methods=['GET'])
def get_temperature():
    """API endpoint for temperature data"""
    try:
        water_temp = watersensor.get_temperature()
        milk_temp = milksensor.get_temperature()
        return jsonify({
            "water_temperature": f"{water_temp:.2f}°C",
            "milk_temperature": f"{milk_temp:.2f}°C"
        })
    except SensorNotReadyError:
        return jsonify({"error": "Sensor Error"}), 500

@app.route('/api/milk-check', methods=['GET'])
def milk_check():
    """API endpoint for milk level check"""
    try:
        distance = get_distance()
        if 0 <= distance < 7:
            indicator = "●●●"
            message = "Good milk level"
        elif 7 <= distance < 14:
            indicator = "●●"
            message = "Moderate milk level"
        else:
            indicator = "●"
            message = "Low milk level"
        return jsonify({
            "distance": distance,
            "emoji": indicator,
            "message": message
        })
    except Exception as e:
        return jsonify({"error": str(e)}), 500

@app.route('/api/heater/<state>', methods=['POST'])
def set_heater(state):
    """API endpoint for manual heater control"""
    global heater_on, manual_control, manual_control_end_time
    if state == 'on':
        GPIO.output(HEATER_PIN, GPIO.LOW)  # Turn on heater
        heater_on = True
        manual_control = True
        manual_control_end_time = time.time() + 120  # 2-minute override
        return jsonify({"message": "Heater turned ON manually"}), 200
    elif state == 'off':
        GPIO.output(HEATER_PIN, GPIO.HIGH)  # Turn off heater
        heater_on = False
        manual_control = True
        manual_control_end_time = time.time() + 120
        return jsonify({"message": "Heater turned OFF manually"}), 200
    else:
        return jsonify({"error": "Invalid state"}), 400

@app.route('/api/clean/start', methods=['POST'])
def start_cleaning():
    """API endpoint to start cleaning cycle"""
    global cleaning_active, rhino_running
    with cleaning_lock:
        if cleaning_active:
            return jsonify({"error": "Cleaning already in progress"}), 400

        cleaning_active = True
        rhino_running.clear()  # Disable voice during cleaning
        cleaning_thread = Thread(target=run_cleaning_cycle)
        cleaning_thread.start()
        return jsonify({"message": "Milk tank cleaning started"}), 200

@app.route('/api/clean/stop', methods=['POST'])
def stop_cleaning():
    """API endpoint to stop cleaning cycle"""
    global cleaning_active, rhino_running
    with cleaning_lock:
        if not cleaning_active:
            return jsonify({"error": "No cleaning in progress"}), 400

        cleaning_active = False
        GPIO.output(MILK_PUMP_PIN, GPIO.HIGH)  # Stop pump
        rhino_running.set()  # Re-enable voice
        return jsonify({"message": "Milk tank cleaning stopped"}), 200

# ================ MAIN EXECUTION ================
def main():
    """Initialize and start all system components"""
    # Play startup sound
    pygame.mixer.init()
    pygame.mixer.music.load("/home/A09/tpj655/ding-101492.mp3")
    pygame.mixer.music.play()
    while pygame.mixer.music.get_busy():
        time.sleep(0.1)

    # Start all system threads
    coffee_machine = CoffeeMachineRhino()  # Voice control
    coffee_machine.start()

    button_thread = Thread(target=toggle_devices, daemon=True)  # Button monitor
    button_thread.start()

    display_thread = Thread(target=update_temperature_display, daemon=True)  # OLED
    display_thread.start()

    heater_thread = Thread(target=auto_control_heater, daemon=True)  # Heater
    heater_thread.start()

    peltier_thread = Thread(target=auto_control_peltier, daemon=True)  # Cooler
    peltier_thread.start()

    flask_thread = Thread(target=lambda: app.run(host='0.0.0.0', port=8080), daemon=True)  # Web
    flask_thread.start()

    try:
        # Main thread just sleeps while others work
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("Shutting down...")
        coffee_machine.stop = True
        coffee_machine.join()
    finally:
        GPIO.cleanup()  # Clean up GPIO on exit
        print("GPIO cleanup complete.")

if __name__ == "__main__":
    main()
