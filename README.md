# CUinSpace-payload-2024
<h2>This is a repository for all code and plans related to the 2023-2024 CUinSpace Payload. </h2>

<h3>Contents will include:</h3>

<h4>-Payload Parachute Test Platform (PPTP)</h4>

An arduino based platform to test the sensors, motors, and algorithms required for the payload. This is the upper part of the payload designed to be a bolt on recovery system for the bottom experiment part of the payload. The PPTP should be about a 1U cubesat or smaller. This test platform will be 3D printed and likely use an Arduino Uno and a Proto-shield. While this won't meet competition standards it is important to ensure the circuits, sensors, and code work properly before sending boards to PCB manufacturers. To be completed before the PDR.  
Sensors ordered:  
-SparkFun 9DoF IMU ICM-20948 (Magnetometer and accelerometer) (NOTE: magnetometer must be calibrated when any hardware changes are made or the payload is in a new environment.)
-SparkFun GPS SAM-M10Q (GPS)  
-Adafruit BMP388 (Altimeter/Thermometer) NOTE: Before test flights this should be calibrated with the local time/area sea level pressure for accurate altitudes.

<h4>-Payload Parachute Recovery System (PPRS) (I like acronyms sue me)</h4>

This will be the continuation of the PPTP moving into the fall semester. We will move to make everything smaller, swap from Arduino Uno to the Arduino Nano (or pi zero if required), create real PCBs, swap 3D printed construction to aluminum or stainless. The size should be about 0.5U or smaller (if possible). This will take place through the fall semester and into the start of the winter semester. Hopefully having everything assembled and tested by around February/March.


<h4>-Payload Biological Experiment (PBE?)</h4>

This will be designed and constructed simultaneously with the PPRS. It is the lower portion of the payload and contains the experiment. The current idea for the experiment is a RPi camera observing a cell culture through a microscope (I think) as the cells undergo the extreme G-forces of the rocket. The question remains of why we have the PPRS and are ejecting out of the rocket which we will need to explain to judges. Outer casing ands inner structure with the cells and microscope. Inner structure must not shake relative to itself. Inner structure should be insulated as much as possible to protect the cells. Hopefully having everything assembled by around February/March.




For next years payload what if we tried using live neurons to control the direction of the parachute
