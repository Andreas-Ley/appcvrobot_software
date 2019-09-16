# AppCV Robot

## Dependencies

The following packages are needed on raspbian:


    sudo apt install \
            cmake gcc \
            libgpiod-dev \
            libboost-dev \
            libboost-system-dev \
            libboost-thread-dev \
            libasio-dev \
            libmagick++-dev \
            libeigen3-dev

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




