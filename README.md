# Tracking-Parrot-Ar-Drone
tracking with parrot ar drone 2.0

#Dependencies
- ROS(indigo)
- ardroneAutono
- opencv(2.4)
- python(2.7)
- imutils
- zbar 
- PIL
- numpy

# Install depencies 
##Docker and ROS indigo 
`docker run -it \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    osrf/ros:indigo-desktop-full
export containerId=$(docker ps -l -q)`
[More info](http://wiki.ros.org/docker/Tutorials/GUI#The_simple_way)

##unzip 
`sudo apt-get install unzip`

##OpenCV 
run script `chmod +x opencv_install.sh && ./opencv_install.sh`

##imutils, zbar, pillow, flask, pymessenger, requests[security] 
`pip install -r requirements.txt`

## ignore libdc1394 
`sudo ln /dev/null /dev/raw1394`

##SSL lib for request from facebook
apt-get install libffi-dev libssl-dev



# To intall and run   

- `cd ~/catkin_ws/src`
- `git clone git@github.com:vito-dante/Tracking_Parrot_Ar_Drone.git`   
- `cd ~/catkin_ws/`
- `catkin_make`
- `roslaunch Tracking_Parrot_Ar_Drone tracking.launch`
- `./ngrok http 5000` -->  directory facebookMessenger 
- `./bot.py` 

## Permission error

- `chmod +x track.py` --> directory scripts
- `chmod +x bot.py` --> directory facebookMessenger


# License

The MIT License

Copyright (c) 2016 by the AUTHOR

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
