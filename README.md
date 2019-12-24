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
Launch the ROSECHO.

```
roslaunch rosecho rosecho.launch
```
The default serial port is /dev/ttyUSB0, if serial open failed, pls check the port used by ROSECHO
If it is /dev/ttyUSB1, you can change the launch file or launch use the command as follows
```
roslaunch rosecho rosecho.launch serial_port:=/dev/ttyUSB1
```

To enable wifi config, call the wifi_cfg service.
```
rosservice call /rosecho/wifi_cfg "ssid: 'tianbot' password: 'www.tianbot.com'"
```
You can use Tab to auto-complete the command and then fill in the ssid and password.

check the automated sound recognition result
```
rosrun rosecho asr_echo.py
```

check the answer
```
rosrun rosecho answer_echo.py
```

send tts to rosecho/tts/goal
```
rostopic pub /rosecho/tts/goal rosecho/ttsActionGoal "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
goal_id:
  stamp:
    secs: 0
    nsecs: 0
  id: ''
goal:
  text: '你好机器人'"
```
You can use Tab to auto-complete the command and then fill in the goal: text: '你好机器人'

### Simple Demo
Voice command test in turtlebot stage

```
roslaunch rosecho rosecho.launch
roslaunch turtlebot_stage turtlebot_in_stage.launch
rosrun rosecho demo.py
```


## Topics

### Published  
- /rosecho/answer
The answer returned by iflyos
- /rosecho/asr
voice recognized result
- /rosecho/wakeup_pos
Position of the speaker when wakeup.

## Actions
Actionlib also use topic to transfer data. 
- /rosecho/tts/feedback
- /rosecho/tts/result
- /rosecho/tts/status
The ROSECHO tts status

- /rosecho/tts/goal
The text to speech topic
- /rosecho/tts/cancel
Cancel goal

## Services
- /rosecho/disable
- /rosecho/enable
- /rosecho/sleep
- /rosecho/wakeup
- /rosecho/wifi_cfg


## License
The package is under BSD 3-Clause License
