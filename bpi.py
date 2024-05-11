import blynklib
import adafruit_dht
import RPi.GPIO as GPIO
import board
import busio
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn
from time import sleep

# Define pin for LDR
LDR_PIN = 27

# Initialize GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(LDR_PIN, GPIO.IN)

# Initialize DHT11 sensor
dht11 = adafruit_dht.DHT11(board.D17)

# Initialize I2C bus and ADC
i2c = busio.I2C(board.SCL, board.SDA)
ads = ADS.ADS1115(i2c, address=0x48)

# Define analog input channel for rain sensor
RAIN_SENSOR_CHANNEL = 0
rain_sensor = AnalogIn(ads, RAIN_SENSOR_CHANNEL)

# Initialize Blynk
BLYNK_AUTH = 'YourAuthToken'  # Replace with your Blynk authentication token
blynk = blynklib.Blynk(BLYNK_AUTH)

# Function to read sensor data
def read_sensor_data():
    temperature = dht11.temperature
    humidity = dht11.humidity
    ldr_value = GPIO.input(LDR_PIN)
    rain_sensor_value = rain_sensor.value
    return temperature, humidity, ldr_value, rain_sensor_value

# Function to send data to Blynk
def send_data_to_blynk():
    temperature, humidity, ldr_value, rain_sensor_value = read_sensor_data()
    blynk.virtual_write(0, temperature)  # Virtual pin V0 for temperature
    blynk.virtual_write(1, humidity)     # Virtual pin V1 for humidity
    blynk.virtual_write(2, ldr_value)    # Virtual pin V2 for LDR value
    blynk.virtual_write(3, rain_sensor_value)  # Virtual pin V3 for rain sensor value

# Register virtual pin handler
@blynk.handle_event('write V10')
def write_virtual_pin_handler(pin, value):
    if value[0] == '1':
        send_data_to_blynk()

while True:
    blynk.run()
    sleep(1)
