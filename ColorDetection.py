import cv2
import time
import numpy as np
from paho.mqtt import client as mqtt
import sys

ada_url='broker.hivemq.com'
mq=mqtt.Client()

xclient= 'xAxisME35'
fred1=mqtt.Client(xclient)
fred1.connect(ada_url)

yclient= 'yAxisME35'
fred2=mqtt.Client(yclient)
fred2.connect(ada_url)

arm='xLocME35'
fred3=mqtt.Client(arm)
fred3.connect(ada_url)

try:
    fred1.connect(ada_url)
    fred2.connect(ada_url)
    print('connected?')
except Exception as e:
    print('could not connect to MOTT server {}}'.format (type(e).__name__,e))
    sys.exit()

def send(xdata, ydata):
    #print('green',xdata,ydata)
    fred1.publish(xclient,xdata)
    fred2.publish(yclient,ydata)
def read():
    x1=fred1.subscribe(xclient,xdata)
    y1=fred2.subscribe(yclient,ydata)
def send2(armdata):
    #print('blue',armdata)
    fred3.publish(arm,armdata)

def findblue(largest_contour2):
    global cx2, cy2
    cx2=0
    cy2=0
    if largest_contour2 is not None:
        # Highlight the largest contour
        cv2.drawContours(frame, [largest_contour2], -1, (255, 0, 0), 2)
        # Compute the center of the largest contour
        M = cv2.moments(largest_contour2)
        #print('blue')
        if M["m00"] != 0:
            cx2 = int(M['m10'] / M['m00'])
            cy2 = int(M['m01'] / M['m00'])
            armdata= cx2
            ydata= cy2
            print('blue',armdata)
            # Draw the track window on the frame
            img2 = cv2.circle(frame, (cx2,cy2), 5, (255,0,0), 3)
            # Display the resulting frame
            #cv2.imshow('frame2',img2)
            #send2(armdata) #publish 
    return cx2, cy2

def findgreen(largest_contour):
    global cx, cy
    cx=0
    cy=0
    if largest_contour is not None:
        # Highlight the largest contour
        cv2.drawContours(frame, [largest_contour], -1, (0, 255, 0), 2)
        # Compute the center of the largest contour
        M = cv2.moments(largest_contour)
        #print('green')
        if M["m00"] != 0:
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
            xdata= cx
            ydata= cy
            #print(cx,cy)
            # Draw the track window on the frame
            img2 = cv2.circle(frame, (cx,cy), 5, (255,255,0), 3)
            # Display the resulting frame
            cv2.imshow('Feed',img2)
            send(xdata,ydata) #publish
            return cx, cy

# Read the video
cap = cv2.VideoCapture(0)

# Read the first frame
ret, frame = cap.read()

def detection(frame):
    ret, frame = cap.read()
    cv2.imshow('video',frame)
    global cx, cy, cx2, cy2, cx3, cy3
    b, g, r = cv2.split(frame)
    red = cv2.subtract(r, g)
    blue= cv2.subtract(b,r)
    green=cv2.subtract(g,r)
    cv2.imshow('frame', frame)
    # Blur the red channel image
    blurred = cv2.GaussianBlur(red, (5, 5), 0)
    blurred2 = cv2.GaussianBlur(blue, (5, 5), 0)
    blurred3 = cv2.GaussianBlur(green, (5, 5), 0)

    # Threshold the blurred image
    thresh = cv2.threshold(blurred, 125, 255, cv2.THRESH_BINARY)[1]
    thresh2 = cv2.threshold(blurred2, 50, 230, cv2.THRESH_BINARY)[1]
    thresh3 = cv2.threshold(blurred3, 55, 255, cv2.THRESH_BINARY)[1]
    # Find contours in the thresholded image
    cnts, _ = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts2, _ = cv2.findContours(thresh2.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts3, _ = cv2.findContours(thresh3.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    # Find the largest contour (if any)
    largest_contour = max(cnts, key=cv2.contourArea, default=None)
    largest_contour2 = max(cnts2, key=cv2.contourArea, default=None)
    largest_contour3 = max(cnts3, key=cv2.contourArea, default=None)
    findblue(largest_contour2)
    findgreen(largest_contour3)

    time.sleep(.5)
    return None
num=0
x1=[]
y1=[]
cat=0
while True:
    num=num+1
    detection(frame)
    if cx!=0:
        cat=cat+1
        if cat==1:
            x1=np.append(num,cx)
            y1=np.append(num,cy)
        print('Green is at', x1[1],y1[1])
    #if : #is it getting close?
        print('comparing',x1[1],cx)
        if x1[1]-75>cx or cx>x1[1]+75:
            print('We grabbed the bone!',cx,x1[1])
    if cx!=0 and cx2!=0:
        if cx+300>cx2>cx-300:
            print('We are close!',x1,cx2)
    

    if cv2.waitKey(1) == ord('q'): # Exit if the 'q' key is pressed
        break
cap.release() # Release the camera
cv2.destroyAllWindows() # Close all windows


#Notes: 0,0 should be opposite side of the arm apartus, opposite corner of servo. 8" from back of track, 11.5" from left eddge of the trackd, 13.5" from ground level
#feedback: is blue close to green, has green movedq