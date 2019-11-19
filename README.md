# ROSECHO
购买链接(Purchase link)：https://item.taobao.com/item.htm?id=607094712042

ROSECHO的唤醒词为 “波弟波弟”。
中文详细使用文档 doc.tianbot.com/rosecho

ROS drivers and examples for Tianbot ROSECHO, a smart speaker converse in Chinese. 
Wake-up-word: Bodhi Bodhi

## Installation Instructions
You can skip these steps if you purchase ROSECHO with a ROS2GO system 
- Install from debian package  
  To be released
- Install from source

Steps to install to catkin_ws. 
```
cd ~/catkin_ws/src/
git clone https://github.com/tianbot/rosecho.git
cd ~/catkin_ws && catkin_make
```
## Usage Instructions
Make sure the power supply and usb connection are correct.

### Start ROSECHO node
When first time use, remember to setup the wi-fi connection of ROSECHO.

```
roslaunch rosecho rosecho.launch ssid:=<wifi ssid> password:=<wifi password>
```

To enable wifi config, publish 'wifi' to /rosecho/cfg topic. Later this function may change to service.
```
rostopic pub /rosecho/cfg std_msgs/String "data: 'wifi'"
```

check the tts result
```
rosrun rosecho asr_echo.py
```

check the answer
```
rosrun rosecho answer_echo.py
```

send tts to rosecho
```
rostopic pub /rosecho/tts std_msgs/String "data: '你好机器人'"
```

### Simple Demo
Voice command test in turtlebot stage

```
roslaunch rosecho rosecho.launch
roslaunch turtlebot_stage turtlebot_in_stage.launch
roslaunch rosecho demo.py
```


## Topics

### Published  
- /rosecho/answer
The answer returned by iflyos
- /rosecho/asr
voice recognized result
- /rosecho/status
The ROSECHO operation status
- /rosecho/wakeup_pos
Position of the speaker when wakeup.

### Subscribed
- /rosecho/tts
The text to speech topic
- /rosecho/cfg
Config. Later will change to service.


## License
The package is under BSD 3-Clause License
