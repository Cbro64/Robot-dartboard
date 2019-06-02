# Robotic Dartboard
Final year mechatronics engineering project

Semester 1 2019

Cameron Brown and James Rowe
Monash University Australia


This repository contains the code for our robotic ball catching project.

The system contains two Raspberry Pis running python code and one PSoC 5 (previously an Arduino).

Each Raspberry Pi has a Pi Camera (v2.1) to track the trajectory of a dart.

The Pis send trajectory predictions to the PSoC via Serial UART.

The PSoc drives the motors.
