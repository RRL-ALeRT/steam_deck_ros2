# steam_deck_ros2

Thanks to https://github.com/ctu-vras/steam-deck-ros-controller
You may follow the initial setup, but just install ROS2 Humble using Robostack

## Setup

Copy spot_controls_toggle.py and spot.png to home directory.


Add following to crontab -e
```
@reboot sleep 10 && python3 /home/deck/spot_controls_toggle.py
```

## Only clone/copy msgs and description packages and spot_driver_plus from
```
https://github.com/RRL-ALeRT/alert_ros2/tree/nuc/spot_driver_plus
```

## SSH without password
First generate key with
```
ssh-keygen
```
Copy key to the on-board PC
```
ssh-copy-id user@host
```
