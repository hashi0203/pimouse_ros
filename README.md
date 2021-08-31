[![Build Status](https://travis-ci.com/hashi0203/pimouse_ros.svg?branch=main)](https://travis-ci.com/hashi0203/pimouse_ros)

# pimouse_ros

## 0. Prepare

If you haven't create catkin workspace, execute the following commands.

```bash
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/src
$ source /opt/ros/melodic/setup.bash
$ catkin_init_workspace
$ cd ~/catkin_ws
$ catkin_make
```

## 1. Install

```bash
$ git clone https://github.com/hashi0203/pimouse_ros.git
$ cd ~/catkin_ws
$ catkin_make
```

## 2. Run

- Terminal 1

    Start roscore.

    ```bash
    $ roscore
    ```

### 2.1. Buzzer

- Terminal 2

    Run buzzer.

    ```bash
    $ rosrun pimous_ros buzzer.py
    ```

- Terminal 3

    Send commands to buzzer.

    ```bash
    $ rostopic pub -1 '/buzzer/' std_msgs/UInt16 1000 # start buzzer
    publishing and latching message for 3.0 seconds
    $ rostopic pub -1 '/buzzer/' std_msgs/UInt16 0 # stop buzzer
    publishing and latching message for 3.0 seconds
    $ roscd pimouse_ros # move directory
    $ ./misc/keikyu.bash # start music
    $ ./misc/keikyu_stop.bash # start music, but interrupted
    ```

### 2.2. Light Sensor

- Terminal 2

    Run light sensors.

    ```bash
    $ rosparam set /lightsensors_freq 1 # set sensor frequency at 1[Hz] (default 10[Hz])
    $ rosrun pimous_ros lightsensors.py
    ```

- Terminal 3

    Read sensor values.

    ```bash
    $ rostopic echo lightsensors # continue reading sensor values (enter ctrl-C to terminate)
    right_forward: 4
    right_side: 1
    left_side: 3
    left_forward: 4
    sum_all: 12
    sum_forward: 8
    ---
    right_forward: 4
    right_side: 3
    left_side: 6
    left_forward: 5
    sum_all: 18
    sum_forward: 9
    ---
    ...
    ```

    or

    ```bash
    $ rostopic echo lightsensors -n 1 # read sensor values only once
    right_forward: 8
    right_side: 3
    left_side: 5
    left_forward: 5
    sum_all: 21
    sum_forward: 13
    ---
    ```

### 2.3. Motor

- Terminal 2

    Run motors.

    ```bash
    $ rosrun pimouse_ros motors.py
    ```

- Terminal 3

    Send commands to motors.

    ```bash
    $ rosservice call /motor_on # start motors
    success: True
    message: "ON"
    $ rosservice call /timed_motion "left_hz: 300
    right_hz: -300
    duration_ms: 500" # move motors by specifing time and frequencies
    success: True
    $ rostopic pub /motor_raw pimouse_ros/MotorFreqs "left_hz: 400
    right_hz: 200" # keep moving motors by specifing frequencies
    publishing and latching message. Press ctrl-C to terminate
    $ rostopic pub /cmd_vel geometry_msgs/Twist "linear:
    x: 0.0
    y: 0.0
    z: 0.0
    angular:
    x: 0.0
    y: 0.0
    z: 1.0" # stop moving motors in 1 second
    publishing and latching message. Press ctrl-C to terminate
    $ rosservice call /motor_off # stop motors
    success: True
    message: "OFF"
    ```

## Reference

- 「[Raspberry Piで学ぶ ROSロボット入門](https://github.com/ryuichiueda/raspimouse_book_info)」