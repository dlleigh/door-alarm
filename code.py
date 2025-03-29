import time
import board
import digitalio
import adafruit_vl53l4cd
import busio
import pwmio
import os
import audiocore

#time.sleep(5)

# Define beep parameters
BEEP_FREQUENCY = 2000  # Hz
BEEP_DURATION = 0.1    # seconds
PAUSE_DURATION = 0.05   # seconds between beeps

# Define sensor parameters
INTER_MEASUREMENT = 3000
TIMING_BUDGET = 200

# Define addresses
DEFAULT_ADDRESS = 0x29  # Default VL53L4CD address (0x52 >> 1)
SENSOR1_ADDRESS = 0x30  # New address for first sensor
SENSOR2_ADDRESS = 0x31  # New address for second sensor
sensors = {}
sensors[1] = {'address': SENSOR1_ADDRESS}
sensors[2] = {'address': SENSOR2_ADDRESS}
sensors[3] = {'address': DEFAULT_ADDRESS}

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

def configure_sensors():
    # Define XSHUT pins for the first two sensors
    xshut_pin1 = digitalio.DigitalInOut(board.A1)
    xshut_pin1.direction = digitalio.Direction.OUTPUT
    xshut_pin2 = digitalio.DigitalInOut(board.A2)
    xshut_pin2.direction = digitalio.Direction.OUTPUT

    # Start with all controllable sensors off
    print("Turning off controlled sensors ...")
    xshut_pin1.value = False
    xshut_pin2.value = False
    time.sleep(0.5)

    i2c = board.STEMMA_I2C()  # For using the built-in STEMMA QT connector on a microcontroller

    # Configure first sensor
    print("\nConfiguring first sensor...")
    try:
        xshut_pin1.value = True  # Turn on first sensor
        time.sleep(0.5)
        # Should connect at default address
        sensor1 = adafruit_vl53l4cd.VL53L4CD(i2c)
        # Change to new address
        sensor1.set_address(SENSOR1_ADDRESS)
        print(f"Changed first sensor address to {hex(SENSOR1_ADDRESS)}")
        # Reconnect with new address
        sensor1 = adafruit_vl53l4cd.VL53L4CD(i2c, address=SENSOR1_ADDRESS)
        sensors[1]['sensor'] = sensor1
        initialize_sensor(sensor1)
        print("First sensor configured successfully")
        beep(BEEP_FREQUENCY, BEEP_DURATION, 2)
    except Exception as e:
        print(f"Error configuring first sensor: {e}")


    # Configure second sensor
    print("\nConfiguring second sensor...")
    try:
        xshut_pin2.value = True  # Turn on first sensor
        time.sleep(0.5)
        # Should connect at default address
        sensor2 = adafruit_vl53l4cd.VL53L4CD(i2c)
        # Change to new address
        sensor2.set_address(SENSOR2_ADDRESS)
        print(f"Changed second sensor address to {hex(SENSOR2_ADDRESS)}")
        # Reconnect with new address
        sensor2 = adafruit_vl53l4cd.VL53L4CD(i2c, address=SENSOR2_ADDRESS)
        initialize_sensor(sensor2)
        sensors[2]['sensor'] = sensor2
        print("Second sensor configured successfully")
        beep(BEEP_FREQUENCY, BEEP_DURATION, 2)
    except Exception as e:
        print(f"Error configuring second sensor: {e}")

    time.sleep(5)

    # Try to configure the third sensor (the one without XSHUT control)
    try:
        print("Attempting to configure third sensor at default address...")
        sensor3 = adafruit_vl53l4cd.VL53L4CD(i2c, address=DEFAULT_ADDRESS)
        initialize_sensor(sensor3)
        sensors[3]['sensor'] = sensor3
        print(f"Third sensor configured at address {hex(DEFAULT_ADDRESS)}")
        beep(BEEP_FREQUENCY, BEEP_DURATION, 2)
    except Exception as e:
        print(f"Error configuring third sensor: {e}")
        print("This could be normal if the third sensor needs more time or is at a different address")

    # Display summary of configured sensors
    print("\nConfigured sensors:")
    for sensor_num, info in sensors.items():
        print(f"Sensor {sensor_num}: address {hex(info['address'])}")

    print("\nConfiguration complete. Sensors are ready for use.")

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
                print(f"Sensor {sensor_num} ({hex(info['address'])}): {distance} cm")
                sensor.clear_interrupt()
            except Exception as e:
                print(f"Error reading sensor {sensor_num}: {e}")

audio_pwm = initialize_audio()
beep(audio_pwm, BEEP_FREQUENCY, BEEP_DURATION, 2)

# Function to play a WAV file using PWM
def play_wav(filename):
    try:
        # Check if file exists
        if filename not in os.listdir("/"):
            print(f"File {filename} not found in Feather's internal storage")
            return False

        # Open the WAV file
        file_path = "/" + filename
        print(f"Opening {file_path}")

        # Create the WAV file object
        # Note: CircuitPython's audiocore.WaveFile handles mono and stereo files
        wave = audiocore.WaveFile(open(file_path, "rb"))

        print(f"Playing {filename}")
        print(f"Sample rate: {wave.sample_rate} Hz")
        print(f"Number of channels: {wave.channel_count}")

        # Start playback using PWM
        # This is a simplified method that works with 8-bit or 16-bit WAV files
        try:
            # Create a buffer to hold audio samples
            buffer = bytearray(1024)  # Adjust buffer size if needed

            # Loop through the WAV file data
            while True:
                # Read a chunk of data from the WAV file
                num_bytes_read = wave.readinto(buffer)
                if num_bytes_read == 0:
                    break  # End of file

                # Process each sample and output via PWM
                # Note: The format depends on your WAV file (8/16-bit, mono/stereo)
                for i in range(0, num_bytes_read, 2):  # Assuming 16-bit samples (2 bytes)
                    # Convert 2 bytes to a 16-bit value (little endian)
                    if i + 1 < num_bytes_read:
                        sample = buffer[i] + (buffer[i+1] << 8)
                        # Scale from 16-bit (0-65535) to PWM duty cycle
                        audio_pwm.duty_cycle = sample

                    # Brief delay to maintain sample rate
                    # This is a crude way to maintain timing
                    # For better timing, use a timer interrupt
                    # time.sleep(1/wave.sample_rate)  # Uncomment if needed for slower files

        except Exception as e:
            print(f"Error during playback: {e}")

        print(f"Finished playing {filename}")
        return True

    except Exception as e:
        print(f"Error playing WAV: {e}")
        return False

# Main program
def main():
    # List available WAV files in internal storage
    print("Available WAV files in Feather's internal storage:")
    try:
        wav_files = [f for f in os.listdir("/") if f.lower().endswith(".wav")]
    except OSError as e:
        print(f"Error accessing filesystem: {e}")
        wav_files = []

    if not wav_files:
        print("No WAV files found in internal storage")
        print("Please upload a WAV file to the Feather's root directory")
        print("Format should be: 8 or 16-bit, mono, 22050Hz or lower")
        return

    for i, file in enumerate(wav_files):
        print(f"{i+1}. {file}")

    # Play the first WAV file (change the index to play a different file)
    if wav_files:
        play_wav(wav_files[0])

    # Reset PWM and disable amp
    audio_pwm.duty_cycle = 0
    enable_pin.value = False

# Run the main program
main()
