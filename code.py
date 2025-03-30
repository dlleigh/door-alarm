import time
import board
import digitalio
import adafruit_vl53l4cd
import adafruit_tca9548a
import busio
import pwmio
import os
import audiocore
import analogio
import time

#time.sleep(5)

# Define beep parameters
BEEP_FREQUENCY = 1000  # Hz
BEEP_DURATION = 0.1    # seconds
PAUSE_DURATION = 0.05   # seconds between beeps

# Define sensor parameters
INTER_MEASUREMENT = 3000
TIMING_BUDGET = 200
THRESHOLD_EXCEEDED_TIME = 20  # seconds

# Add battery monitoring constants
BATTERY_LOW_THRESHOLD = 3.4  # Volts
BATTERY_CHECK_INTERVAL = 60  # seconds
BATTERY_LOW_ALARM_FREQUENCY = 2000  # Hz
BATTERY_LOW_BEEP_DURATION = 0.1  # seconds
BATTERY_ALARM_MINIMUM_INTERVAL = 600  # 10 minutes in seconds

# Define addresses
sensors = {}
sensors[1] = {
    'name': 'freezer', 
    'mux_port': 0, 
    'min_threshold': 1.2,
    'max_threshold': 20,
    'exceeded_since': None
}
sensors[2] = {
    'name': 'left-door', 
    'mux_port': 1, 
    'min_threshold': 2.7,
    'max_threshold': 20,
    'exceeded_since': None
}
sensors[3] = {
    'name': 'right-door', 
    'mux_port': 2, 
    'min_threshold': 2.2,
    'max_threshold': 20,
    'exceeded_since': None
}

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

def read_battery_voltage():
    voltage_pin = analogio.AnalogIn(board.A3)  # Changed from VOLTAGE_MONITOR to A3
    voltage = (voltage_pin.value * 3.3) / 65536 * 2
    voltage_pin.deinit()
    return voltage

def sensor_loop(audio_pwm):
    print("\nStarting measurement loop...")
    last_battery_check = 0
    last_battery_alarm = 0  # Track when we last sounded the battery alarm
    is_battery_low = False
    
    while len(sensors) > 0:
        current_time = time.monotonic()
        
        # Check battery periodically
        # if current_time - last_battery_check > BATTERY_CHECK_INTERVAL:
        #     voltage = read_battery_voltage()
        #     print(f"Battery Voltage: {voltage:.2f}V")
        #     last_battery_check = current_time
        #     is_battery_low = voltage < BATTERY_LOW_THRESHOLD
        #     if is_battery_low:
        #         print("Warning: Low battery!")

        for sensor_num, info in sensors.items():
            try:
                sensor = info["sensor"]
                while not sensor.data_ready:
                    pass
                distance = sensor.distance
                print(f"Sensor {sensor_num} ({info['name']}): {distance} cm")
                
                # Check if distance is between min and max thresholds
                if info['min_threshold'] < distance < info['max_threshold']:
                    if info['exceeded_since'] is None:
                        info['exceeded_since'] = time.monotonic()
                    elif time.monotonic() - info['exceeded_since'] > THRESHOLD_EXCEEDED_TIME:
                        print(f"Alert! {info['name']} in warning zone too long!")
                        beep(audio_pwm, BEEP_FREQUENCY, BEEP_DURATION, 3)
                elif distance > info['max_threshold'] and is_battery_low:
                    # Check if enough time has passed since last battery alarm
                    if current_time - last_battery_alarm > BATTERY_ALARM_MINIMUM_INTERVAL:
                        print(f"Alert! {info['name']} exceeded max threshold with low battery!")
                        beep(audio_pwm, BATTERY_LOW_ALARM_FREQUENCY, BATTERY_LOW_BEEP_DURATION, 2)
                        last_battery_alarm = current_time
                else:
                    info['exceeded_since'] = None
                
                sensor.clear_interrupt()
            except Exception as e:
                print(f"Error reading sensor {sensor_num}: {e}")
        time.sleep(1)

# Main program
def main():
    audio_pwm = initialize_audio()
    beep(audio_pwm, BEEP_FREQUENCY, BEEP_DURATION, 2)
    print("Initializing sensors...")
    configure_sensors(sensors)
    print("Sensors initialized successfully.")
    beep(audio_pwm, BEEP_FREQUENCY, BEEP_DURATION, 2)

    # Start the sensor loop
    sensor_loop(audio_pwm)

# Run the main program
main()
