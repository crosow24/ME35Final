from machine import Pin, PWM
from rp2 import PIO, StateMachine, asm_pio
import sys
from time import sleep
import time, struct
import uasyncio as asyncio
import network
import ubinascii
import urequests as requests
import mqtt
from keys import Tufts_Wireless as wifi

@asm_pio(set_init=(PIO.OUT_LOW,) * 4, out_init=(PIO.OUT_LOW,) * 4, out_shiftdir=PIO.SHIFT_RIGHT, in_shiftdir=PIO.SHIFT_LEFT)

def prog():
    #grab settings
    pull() #pull the first number off the FIFO into the output shift register
    mov(x, osr) # num steps (pull off of the output shift register and put it on x (one of 2 possible variables)
    pull() #pull 2nd number into the output shift register
    mov(y, osr) # store Steps - in case nothing on the OSR

    # run through x steps for y times
    jmp(not_x, "end")  #quit if no steps

    label("loop")  # loop out each bit
    jmp(not_osre, "step") # if OSR is not exhausted, pull more values from it
    mov(osr, y)  # if it was exhausted, refill it with the full 32 bit pattern

    label("step")  # note using "out" rather than "set" - out takes bits from the OSR
    out(pins, 4) [31] #grab 4 bits, set the pins to that, wait 31 clock cycles

    jmp(x_dec,"loop")
    label("end")

    irq(rel(0))  #fire interrupt

#3500
sm = StateMachine(0, prog, freq=3200, set_base=Pin(8), out_base=Pin(8))
CCW = [0b0001,0b0011,0b0010,0b0110,0b0100,0b1100,0b1000,0b1001]  #stepper patterns
CW =  CCW.copy() #stepper patterns
CW.reverse()

steps = 0

def turn(sm, goal = 200, CW_dir = True):  # callback when done
    global steps
    global data

    sleep(1)  # pause

    data = CW if CW_dir else CCW
    a = 0
    for i,byte in enumerate(data):
        a += byte << i*4
    print("Going to send {0:b}".format(a))

    sm.put(goal)
    sm.put(a)

    steps = (steps - goal) if CW_dir else (steps + goal)

spool = machine.Pin(16, machine.Pin.OUT)
spool2 = machine.Pin(17, machine.Pin.OUT)
servo1 = PWM(Pin(0))
servo1.freq(50)

def servoSet(angle):
    angle = int(angle)
    print(int(((angle/180)*8000)))
    return int(((angle/180)*8000))

def cw():
    spool2.on()
    spool.off()

def ccw():
    spool2.off()
    spool.on()

def spoolOff():
    spool.off()
    spool2.off()

def connect_wifi(wifi):
    station = network.WLAN(network.STA_IF)
    station.active(True)
    mac = ubinascii.hexlify(network.WLAN().config('mac'),':').decode()
    print("MAC " + mac)

    station.connect(wifi['ssid'],wifi['pass'])
    while not station.isconnected():
        time.sleep(1)
    print('Connection successful')
    print(station.ifconfig())
def whenCalled(topic, msg):
    global wcdata
    print((topic.decode(), msg.decode()))
    # wcdata = (topic.decode(), msg.decode())
    wcdata = msg.decode()
    time.sleep(0.5)
def whenCalled2(topic, msg):
    global wcdata2
    print((topic.decode(), msg.decode()))
    # wcdata = (topic.decode(), msg.decode())
    wcdata2 = msg.decode()
    time.sleep(0.5)
def whenCalled3(topic, msg):
    global wcdata3
    print((topic.decode(), msg.decode()))
    # wcdata = (topic.decode(), msg.decode())
    wcdata3 = msg.decode()
    time.sleep(0.5)

connect_wifi(wifi)
fred = mqtt.MQTTClient('PicoxAxisME35', 'broker.hivemq.com', keepalive=60)
fred.connect()
fred.set_callback(whenCalled)
fred2 = mqtt.MQTTClient('PicoyAxisME35', 'broker.hivemq.com', keepalive=60)
fred2.connect()
fred2.set_callback(whenCalled2)
fred.subscribe('xAxisME35')
fred2.subscribe('yAxisME35')
pickingPiece = True
xData = 0
yData = 0

#(115, 650)
#ymax is 959 (bottom 30 degrees) ymin (top 70 degrees) is 0
# bottom left corner 5.25 inches x 1.75 inches y

servoAngle = 30
servo1.duty_u16(servoSet(servoAngle))
xAlign = True
currentLocationX= 0
offset = 200
runCounter = 0
sm.active(1)
turn(sm,266, True)
time.sleep(2)
while pickingPiece:
    while xAlign:
        fred.check_msg()
        try:
            xData = int(wcdata)
            #some sort of conversion from centroid position to stepper code
            stepperMove = int(round(xData/1.43))
            sm.active(1)
            turn(sm,stepperMove, True)
            print("stepperMove")
            print(stepperMove)
            xAlign = False
        except:
            print("except1")
        '''
        fred3.check_msg()
        try:
            currentLocationX = int(wcdata3)
            print("currentLocationX")
            print(currentLocationX)

            if currentLocationX == (stepperMotor + 5):
                xAlign = False
            elif (currentLocationX == (stepperMotor - 5)):
                xAlign = False

            if stepperMove > currentLocationX:
                print("sending forward")
                print(stepperMove-currentLocationX)
               #turn(sm,(stepperMove-currentLocation), True)
                time.sleep(2)
            elif stepperMove < currentLocationX:
                print("sending backward")
                print(currentLocation-stepperMove)
               #turn(sm,-(currentLocation-stepperMove), True)
                time.sleep(2)
            elif runCounter > 100:
                xAlign = False

        except:
            print("except3")
        '''
        time.sleep(.1)
    fred2.check_msg()
    print("moving to y")
    time.sleep(8)
    try:
        yData = int(wcdata2)
        yData = 1000 - yData
        #some sort of trig for y centroid position should be 20-70 degrees
        servoAngle = (yData/23)+30
        servo1.duty_u16(servoSet(servoAngle))
        pickingPiece = False
    except:
        print("except2")
time.sleep(3)
servoAngle = 30
servo1.duty_u16(servoSet(servoAngle))
cw()
time.sleep(1.5)
spoolOff()
time.sleep(20)
