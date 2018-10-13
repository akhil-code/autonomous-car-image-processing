# Autonomous driving based on Image processing
An image processing model takes camera view of the car, detects lanes and then steers the car based on the distance from the lanes. Developed using OpenCV libraray of python.

## Overview
When we drive, we use our eyes to decide where to go. The lines on the road that show us where the lanes are act as our constant reference for where to steer the vehicle. Naturally, one of the first things we would like to do in developing a self-driving car is to automatically detect lane lines using an algorithm. So, using few concepts of image processing, a model is developed that detects the lanes of the road and helps in steering the car.

## Setup instructions
These instructions will get you a copy of the project up and running on your local machine for development and testing purposes.
#### Prerequisites
+ [Python](https://www.python.org/downloads/) (with [numpy](https://docs.scipy.org/doc/numpy-1.15.1/user/install.html) and [OpenCV](https://docs.opencv.org/3.0-beta/doc/py_tutorials/py_setup/py_setup_in_windows/py_setup_in_windows.html) libraries installed)
#### Deployment
+ Clone the Repository `git clone https://github.com/akhil-code/autonomous-car-image-processing`
+ Change current directory to this repository
+ Run the application using command `python init.py`
+ Upon execution the models starts detecting the lanes and prints the directions on the console ouput.

## Authors
+ Akhil Guttula

## Learn more
+ [Building a lane detection system using Python 3 and OpenCV](https://medium.com/@galen.ballew/opencv-lanedetection-419361364fc0)
