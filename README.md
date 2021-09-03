# SpotMicroAI for Jetson

This repository is a fork of [https://gitlab.com/custom_robots/spotmicroai/basic-runtime.git](https://gitlab.com/custom_robots/spotmicroai/basic-runtime.git).  


## Requirements

* Jetson Nano
* PCA9685 0x40
* PCA9685 0x41
* SpotMicro Robot Dog


## Install

[base docker]
```
sudo docker run --runtime nvidia -it --network host -v /home/jetson/nvdli-data:/nvdli-nano/data --mount type=bind,source=/dev/,target=/dev/ --privileged nvcr.io/nvidia/dli/dli-nano-ai:v2.0.1-r32.6.1
```

[Install on docker container]
```
apt update -y
apt-get install -y git i2c-tools joystick
pip3 install smbus
pip3 install -U pip
pip3 install adafruit-circuitpython-pca9685
pip3 install adafruit-circuitpython-motor
pip3 install pick
pip3 install jmespath
pip3 install Jetson.GPIO

mkdir /nvdli-nano/github
cd /nvgithub/github
git clone https://github.com/naisy/spotmicroai-jetson
cd spotmicroai-jetson
cp spotmicroai.json ~/
```


[Calibration]
```
python3 calibration.py
```

[Run]
```
python3 main.py
```
