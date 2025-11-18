import time
import board
import digitalio
import adafruit_vl53l4cd
import adafruit_tca9548a
import pwmio
import time

#time.sleep(5)

# Define beep parameters
BEEP_FREQUENCY = 1000  # Hz
BEEP_DURATION = 0.1    # seconds
PAUSE_DURATION = 0.05   # seconds between beeps

# Define sensor parameters
INTER_MEASUREMENT = 3000
TIMING_BUDGET =200
THRESHOLD_EXCEEDED_TIME = 40  # seconds
READINGS_WINDOW = 5  # Number of readings to average

# Define addresses
sensors = {}
# sensors[1] = {
#     'name': 'freezer', 
#     'mux_port': 0, 
#     'min_threshold': 2.0,
#     'max_threshold': 20,
#     'exceeded_since': None
# }
sensors[2] = {
    'name': 'left-door',
    'mux_port': 1,
    'min_threshold': 3.0,
    'max_threshold': 20,
    'exceeded_since': None,
    'readings': []
}
sensors[3] = {
    'name': 'right-door',
    'mux_port': 2,
    'min_threshold': 1.9,
    'max_threshold': 20,
    'exceeded_since': None,
    'readings': []
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
    """Generate a beep sound at the specified frequency for the given duration."""
    for _ in range(count):
        audio_pwm.frequency = frequency
        audio_pwm.duty_cycle = 32767  # 50% duty cycle (32767 is half of 65535)
        time.sleep(duration)
        audio_pwm.duty_cycle = 0      # Turn off the sound
        time.sleep(PAUSE_DURATION)    # Pause between beeps
    # Make sure to turn off the audio and disable the amp when done
    audio_pwm.duty_cycle = 0

def initialize_sensor(sensor):
    # Remove start_ranging() from here since we'll call it before each reading
    sensor.inter_measurement = INTER_MEASUREMENT
    sensor.timing_budget = TIMING_BUDGET
    sensor.distance_mode = 1  # 1 = short (up to 1.3m), 2 = long (up to 4m)

def read_sensor(sensor):
    """Helper function to read a single sensor with proper power management"""
    sensor.start_ranging()
    # Wait for data to be ready
    while not sensor.data_ready:
        pass
    distance = sensor.distance
    sensor.clear_interrupt()
    sensor.stop_ranging()
    return distance

def configure_sensors(sensors):
    i2c = board.STEMMA_I2C()    # Create the I2C bus
    # Create the PCA9546A object and give it the I2C bus
    mux = adafruit_tca9548a.PCA9546A(i2c)
    # For each sensor, create it using the PCA9546A channel instead of the I2C object
    for sensor_num, info in sensors.items():
        print(f"Initializing sensor {sensor_num} ({info['name']}) on mux port {info['mux_port']}...")
        info['sensor'] = adafruit_vl53l4cd.VL53L4CD(mux[info['mux_port']])
        initialize_sensor(info['sensor'])

def sensor_loop(audio_pwm):
    print("\nStarting measurement loop...")
    # Initialize timers to current time to ensure first check/print occurs after the interval
    
    last_alert_details = None
    # last_alert_print_time is for printing status every minute
    last_alert_print_time = time.monotonic() # For printing status every minute
    
    while len(sensors) > 0:
        current_time = time.monotonic()
        
        # Periodic status print (every minute)
        if current_time - last_alert_print_time >= 60.0:  # 60 seconds
            if last_alert_details:
                details = last_alert_details  # Alias for brevity
                time_since_alert = current_time - details['time']
                print("Status update: Last alert:")
                print(f"  Sensor: '{details['sensor_name']}'")
                print(f"  Reason: '{details['reason']}'")
                print(f"  Distance: {details['distance']:.1f} cm")
                print(f"  Alert recorded at monotonic time: {details['time']:.0f} (occurred {time_since_alert:.0f}s ago).")
            else:
                print("Status update: No alerts recorded since startup.")
            last_alert_print_time = current_time
        
        for sensor_num, info in sensors.items():
            try:
                distance = read_sensor(info["sensor"])

                # Skip invalid readings
                if info["sensor"].range_status != 0:
                    print(f"Sensor {sensor_num} ({info['name']}): invalid reading (status={info['sensor'].range_status}), skipping")
                    continue

                # If reading is clearly below min_threshold (door definitely closed) and we currently have
                # readings in the warning zone, reset the buffer to immediately respond to door closing
                if distance < info['min_threshold'] and len(info['readings']) > 0:
                    avg_distance = sum(info['readings']) / len(info['readings'])
                    if avg_distance >= info['min_threshold']:
                        info['readings'] = []
                        info['exceeded_since'] = None

                # Update running average
                info['readings'].append(distance)
                if len(info['readings']) > READINGS_WINDOW:
                    info['readings'].pop(0)

                # Calculate average
                avg_distance = sum(info['readings']) / len(info['readings'])

                # Format readings buffer for display
                readings_str = '[' + ', '.join(f'{r:.1f}' for r in info['readings']) + ']'
                print(f"Sensor {sensor_num} ({info['name']}): {distance} cm (avg: {avg_distance:.1f} cm, buffer: {readings_str})")

                # Check if average distance is between min and max thresholds
                if info['min_threshold'] < avg_distance < info['max_threshold']:
                    if info['exceeded_since'] is None:
                        info['exceeded_since'] = time.monotonic()
                    elif time.monotonic() - info['exceeded_since'] > THRESHOLD_EXCEEDED_TIME:
                        print(f"Alert! {info['name']} in warning zone too long!")
                        beep(audio_pwm, BEEP_FREQUENCY, BEEP_DURATION, 3)
                        last_alert_details = {
                            'sensor_name': info['name'],
                            'reason': 'In warning zone too long',
                            'distance': avg_distance,
                            'time': current_time
                        }
                else:
                    info['exceeded_since'] = None

            except Exception as e:
                print(f"Error reading sensor {sensor_num}: {e}")
        time.sleep(0.5)

# Main program
def main():
    audio_pwm = initialize_audio()
    beep(audio_pwm, BEEP_FREQUENCY, BEEP_DURATION, 2)
    print ("waking up!")
    print("Initializing sensors...")
    configure_sensors(sensors)
    print("Sensors initialized successfully.")
    beep(audio_pwm, BEEP_FREQUENCY, BEEP_DURATION, 2)

    # Start the sensor loop
    sensor_loop(audio_pwm)

# Run the main program
main()
