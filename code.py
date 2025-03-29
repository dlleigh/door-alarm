import time
import board
import digitalio
import adafruit_vl53l4cd
import adafruit_tca9548a
import busio
import pwmio
import os
import audiocore
import time

#time.sleep(5)

# Define beep parameters
BEEP_FREQUENCY = 2000  # Hz
BEEP_DURATION = 0.1    # seconds
PAUSE_DURATION = 0.05   # seconds between beeps

# Define sensor parameters
INTER_MEASUREMENT = 3000
TIMING_BUDGET = 200

# Define addresses
sensors = {}
sensors[1] = {'name': 'freezer', 'mux_port': 0}
sensors[2] = {'name': 'left-door', 'mux_port': 1}
sensors[3] = {'name': 'right-door', 'mux_port': 2}

def initialize_led():
    led = digitalio.DigitalInOut(board.LED)
    led.direction = digitalio.Direction.OUTPUT

def initialize_audio():
    print('initializing audio')
    # Setup the PWM output for the audio signal
    # The PAM8302 typically has an A+ input pin that should be connected to a PWM pin
    audio_pwm = pwmio.PWMOut(board.A0, frequency=440, duty_cycle=0, variable_frequency=True)

    # Setup the enable pin for the amplifier if connected
    # If you don't have an enable pin connected, you can remove these lines
    # enable_pin = digitalio.DigitalInOut(board.A1)
    # enable_pin.direction = digitalio.Direction.OUTPUT
    # enable_pin.value = True  # Enable the amplifier
    return audio_pwm

def beep(audio_pwm,frequency, duration, count):
    for _ in range(count):
        """Generate a beep sound at the specified frequency for the given duration."""
        audio_pwm.frequency = frequency
        audio_pwm.duty_cycle = 32767  # 50% duty cycle (32767 is half of 65535)
        time.sleep(duration)
        audio_pwm.duty_cycle = 0      # Turn off the sound
        time.sleep(PAUSE_DURATION)    # Pause between beeps
    # Make sure to turn off the audio and disable the amp when done
    audio_pwm.duty_cycle = 0

def initialize_sensor(sensor):
    sensor.inter_measurement = INTER_MEASUREMENT
    sensor.timing_budget = TIMING_BUDGET
    sensor.start_ranging()

def configure_sensors(sensors):
    i2c = board.STEMMA_I2C()    # Create the I2C bus
    # Create the PCA9546A object and give it the I2C bus
    mux = adafruit_tca9548a.PCA9546A(i2c)
    # For each sensor, create it using the PCA9546A channel instead of the I2C object
    for sensor_num, info in sensors.items():
        print(f"Initializing sensor {sensor_num} ({info['name']}) on mux port {info['mux_port']}...")
        info['sensor'] = adafruit_vl53l4cd.VL53L4CD(mux[info['mux_port']])
        initialize_sensor(info['sensor'])

def sensor_loop():
    # Main loop to read sensors
    print("\nStarting measurement loop...")
    while len(sensors) > 0:
        for sensor_num, info in sensors.items():
            try:
                sensor = info["sensor"]
                while not sensor.data_ready:
                    pass
                distance = sensor.distance
                print(f"Sensor {sensor_num} ({info['name']}): {distance} cm")
                sensor.clear_interrupt()
            except Exception as e:
                print(f"Error reading sensor {sensor_num}: {e}")
        time.sleep(5)

audio_pwm = initialize_audio()
beep(audio_pwm, BEEP_FREQUENCY, BEEP_DURATION, 2)

# Main program
def main():
    print("Initializing sensors...")
    configure_sensors(sensors)
    print("Sensors initialized successfully.")
    beep(audio_pwm, BEEP_FREQUENCY, BEEP_DURATION, 2)

    # Start the sensor loop
    sensor_loop()

# Run the main program
main()
