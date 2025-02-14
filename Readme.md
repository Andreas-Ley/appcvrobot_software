# AppCV Robot

## Dependencies

The following packages are needed on raspbian:

    sudo apt update
    sudo apt install \
            cmake gcc \
            libgpiod-dev \
            libboost-dev \
            libboost-system-dev \
            libboost-thread-dev \
            libboost-filesystem-dev \
            libasio-dev \
            libmagick++-dev \
            libeigen3-dev \
            libopencv-dev

Optional: Git to directly pull the repository, ccmake

Ssh into the robot. 

Copy or clone the pigpiod repository:

    git clone https://github.com/joan2937/pigpio 
    
Build and install:

    cd pigpio
    mkdir buildRel
    cd buildRel
    cmake .. -DCMAKE_BUILD_TYPE=Release
    make -j 4
    sudo make install

## Installation on Robot

Ssh into the robot. 

Copy or clone the software repository:

    git clone gitosis@andreas-ley.com:software_appcvrobot.git

Configure:

    mkdir buildRel
    cmake .. -DCMAKE_BUILD_TYPE=Release

Optional: Configure various options

    ccmake ./


## Installation of firmware

Remove microcontroller from socket and plug in programmer on the side pins.
In the arduino ide, select the "Arduino Pro Mini" in the variant "ATMega 328P 8MHz 3.3V".
Compile and upload the project in the firmware folder.
Disconnect programmer and plug microcontroller back into the socket.


## ffmpeg Video recording

ffmpeg -f v4l2 -input_format h264 -framerate 15 -i /dev/video0 -c:v copy outputFile.mp4

## ffmpeg Video streaming

    ffmpeg -f v4l2 -input_format h264 -framerate 15 -i /dev/video0 -c:v copy -fflags nobuffer -f mpegts udp://192.168.93.167:1337

    ffplay -fflags nobuffer -flags low_delay -framedrop -strict experimental -probesize 32 -analyzeduration 0 -sync ext udp://@:1337

Center crop for tweaking focus:
    ffplay -fflags nobuffer -flags low_delay -framedrop -strict experimental -probesize 32 -analyzeduration 0 -sync ext -vf "crop=640:480:640:480"  udp://@:1337


display and record on PC:g
ffmpeg -i udp://@:1337 -map 0 -c:v libx264 -crf 23 -f tee "test.mp4|[f=nut]pipe:" | ffplay pipe:

## Misc

use raspi-config to change GPU memory to 256MB (otherwise can't record in high resolutions)

    v4l2-ctl -L

Rotate, but breaks calibration:

    v4l2-ctl -c rotate=180

for video, trade noise for lower exposure times:

    v4l2-ctl -c iso_sensitivity_auto=0
    v4l2-ctl -c iso_sensitivity=4

not sure if it helps with exposure times

    v4l2-ctl -c scene_mode=11 
    
    v4l2-ctl -c video_bitrate=25000000
    
    v4l2-ctl --list-formats-ext
    v4l2-ctl --get-fmt-video
    v4l2-ctl --set-fmt-video=width=640,height=480,pixelformat=1

    v4l2-ctl --set-fmt-video=width=1920,height=1080,pixelformat=4


