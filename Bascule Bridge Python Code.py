'''

This script will control 2x step motors (28BYJ), 2x servo motors (SG90), and 1x Grove speaker.
The script uses Rpi.GPIO, pymodbus, and the time library.
pymodbus is used to connect to client PiSlave1 (192.168.1.44), which is used as a RTU.
PLC slave device (RTU) and is connected to 4xIR sensors, which will turn green LED "lamp" on and off.
When the "lamp" is on, initially, the Grove speaker will generate an alarm tone for a few seconds, then
the servo motor will turn 90 degrees, and the step motor will open the bridge.
When the lamp is off, the step motor will rotate to close the bridge and
the servo motor will turn -90 degrees to close the traffic barrier.

The loop continues until the user presses Ctrl+C to raise a KeyboardInterrupt exception,
at which point we stop the PWM signal using p1.stop(), p2.stop(), and clean up the GPIO pins using GPIO.cleanup().

'''
import RPi.GPIO as GPIO
import time
import pymodbus

from pymodbus.client.sync import ModbusTcpClient
client=ModbusTcpClient('192.168.1.44')
GPIO.setmode(GPIO.BOARD)

# Set the GPIO pin for the servo motor
servoPIN1 = 3 # Set the GPIO pin for the servo motor
servoPIN2 = 5 # Set the GPIO pin for the servo motor
GPIO.setup(servoPIN1, GPIO.OUT)
GPIO.setup(servoPIN2, GPIO.OUT)

# Set the GPIO pin for the Buzzer
BUZZER_PIN = 19
GPIO.setup(BUZZER_PIN, GPIO.OUT)

# buzzer function
def buzz(pitch, duration):
    period = 2.0 / pitch
    delay = period / 2
    cycles = int(duration * pitch)
    for i in range(cycles):
        GPIO.output(BUZZER_PIN, True)
        time.sleep(delay)
        GPIO.output(BUZZER_PIN, False)
        time.sleep(delay)

#Set up pins for the Step motor
ControlPina=[7,11,13,15]
ControlPinb=[36,29,31,37]
ControlPin=ControlPina + ControlPinb

p1 = GPIO.PWM(servoPIN1, 50) # Set PWM signal frequency to 50Hz
p2 = GPIO.PWM(servoPIN2, 50)
p1.start(2.5) # Set initial position of the servo to 0 degrees
p2.start(2.5)

#Set the ControlPin as output pin and set the initial value to 0
for pin in ControlPin:
    GPIO.setup(pin,GPIO.OUT)
    GPIO.output(pin,0)

try:
    while True:
        #This loop look for the state of the Lamp to start opening the bridge.
        rr=client.read_coils(0,1)
        Lamp=rr.getBit(0)
        
        #Close the traffic barrier and Oppen the bridge when the Lamp is on.
        if Lamp==True:
            buzz(2000,7) # plays a 2000 Hz tone for few seconds
            time.sleep(7)
            print ('Close the barrier')
            angle=90
            dutyCycle = 2.5 + angle / 18
            p1.ChangeDutyCycle(dutyCycle)
            p2.ChangeDutyCycle(dutyCycle)

            time.sleep(5)

            seq= [[1,0,0,0],[1,1,0,0],[0,1,0,0],[0,1,1,0],[0,0,1,0],[0,0,1,1],[0,0,0,1],[1,0,0,1]]
            print('Bridge Opening')
            for i in range(128):
                for halfstep in range(8):
                    for pin in range(4):
                        GPIO.output(ControlPina[pin],seq[halfstep][pin])
                        GPIO.output(ControlPinb[pin],seq[halfstep][pin])
                    time.sleep(0.002)

            while True:
                #This loop will check for the Lamp to go off once the bridge has been lifted.
                rr=client.read_coils(0,1)
                Lamp=rr.getBit(0)
                #Close the bridge when Lamp is off and open the traffic barrier.
                if Lamp==False:
                    #closing the bridge
                    print('Bridge Closing')
                    
                    seq2=[[1,0,0,1],[0,0,0,1],[0,0,1,1],[0,0,1,0],[0,1,1,0],[0,1,0,0],[1,1,0,0],[1,0,0,0]]
                    for i in range(128):
                       for halfstep in range(8):
                           for pin in range(4):
                               GPIO.output(ControlPina[pin],seq2[halfstep][pin])
                               GPIO.output(ControlPinb[pin],seq2[halfstep][pin])
                           time.sleep(0.002)
                              
                    #opening the barrier
                    time.sleep(10)
                    angle=0
                    print('Opening Barrier')
                    dutyCycle = 2.5 + angle / 18
                    p1.ChangeDutyCycle(dutyCycle)
                    p2.ChangeDutyCycle(dutyCycle)
                    time.sleep(2)
                    print ('completed')
                    
                    break
finally:
    p1.stop()
    p2.stop()
    GPIO.cleanup()
