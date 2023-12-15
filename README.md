# ME35Final
This repository contains the code required to run the camera detection and movement processes for our ME35 Robotics final. Basic function is as follows:
1. Mount a phone above the gameboard (ideally an Apple iPhone so that the detection code can be run on a Mac)
2. Connect to the Raspberry Pi Pico via a micro USB cable and run the finalMain code. This code will first home the device to the edge of the game board, then wait for an MQTT signal from the detection code.
3. Run the detection code. The code will attempt to location the green "Wrenched Ankle" inside the game board, then send the coordinates to the Pico via MQTT.
4. Once the Pico receives the coordinates, it will convert those values to stepper motor turns and servo angle.
5. The stepper motor will run for the set number of turns, then the servo will actuate to the correct angle to lower the magnet over the board.
6. The magnet will pick up the piece, then the DC motor will run to extract the piece.
7. Finally, the detection code will check to see if the object is still in place, and will grade its own progress.
8. The grading scale is as follows:
   - 100% if the piece has moved (equates to the piece being picked up)
   - 50% ("we were close") if the blue indicator on the robot body is approximately in line with the piece (but the piece was not successfully picked up)
   - 0% otherwise

Libraries Required:
ColorDetection.py -- cv2, time, numpy, paho.mqtt, sys

finalMain.py -- machine, rp2, sys, time, struct, uasyncio, network, ubinascii, urequests, mqtt (included in repo)
