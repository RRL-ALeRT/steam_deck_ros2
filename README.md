# steam_deck_ros2

Game Mode
1. Settings -> Controller -> Desktop Layout (Edit).
"Change Official Layout for - Desktop Configuration" -> "Gamepad with Gyro" in Templates.

2. Settings -> System -> Enable Developer Mode.
Settings -> Developer -> Disable Wifi Power Management.

Desktop Mode
1. In Power Settings, change screen dim and suspen settings.

2. In terminal, type "passwd" to set sudo password.

3. In terminal,
```
sudo steamos-readonly disable
```

4. In terminal, enable ssh server
```
sudo systemctl enable sshd.service && sudo systemctl start sshd.service
```

5. https://github.com/ctu-vras/steam-deck-ros-controller?tab=readme-ov-file#make-the-arch-linux-os-package-manager-usable

6. In terminal,
```
sudo pacman -S screen
```

7. Install micromamba - https://mamba.readthedocs.io/en/latest/installation/micromamba-installation.html and source it
```
"${SHELL}" <(curl -L micro.mamba.pm/install.sh)
```

8. Robostack, Install ROS2 Humble packages - https://robostack.github.io/GettingStarted.html#__tabbed_1_2
```
# Create a ros-humble desktop environment
micromamba create -n ros_env -c conda-forge -c robostack-staging ros-humble-desktop

# Activate the environment
micromamba activate ros_env
```

```
# Install ROS2 dev tools
micromamba install -c conda-forge compilers cmake pkg-config make ninja colcon-common-extensions catkin_tools rosdep
```

9. In terminal,
```
cd
git clone https://github.com/RRL-ALeRT/steam_deck_ros2
echo "
source steam_deck_ros2/humble" >> ~/.bashrc
cat steam_deck_ros2/screenrc > .screenrc
```

10. Install crontab
```
sudo pacman -S cronie
systemctl enable --now cronie.service
crontab  -e
```

And add following to crontab -e
```
@reboot sleep 10 && python3 /home/deck/steam_deck_ros2/spot_controls_toggle.py >> /home/deck/toggle.log 2>&1
```

11. Spot controls toggle dependencies
```
sudo pacman -S pango python-gobject gtk3 libappindicator-gtk3
```

12. Build rqt plugins
```
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/RRL-ALeRT/alert_dashboard_rqt
cd ..
cb
```

Few required dependencies
```
micromamba install -c robostack-staging ros-humble-desktop-full
micromamba install -c robostack-staging ros-humble-octomap-msgs ros-humble-depth-image-proc ros-humble-control-msgs ros-humble-controller-manager-msgs ros-humble-moveit-msgs
pip3 install paramiko
```

Install paramiko in base OS
```
sudo pacman -S  python-pyqt5
```

Clone/copy msgs and description packages from Spot's onboard PC.
Also, clone ffmpeg_image_transport repos from https://github.com/RRL-ALeRT

Notes:
i. Delete all the packages which aren't needed keeping only robot descriptions and msgs.

13. Overwrite default rviz2 config with one in rviz2 folder.

14. ssh in NUC shoud work without password

First generate key with
```
ssh-keygen # Press all enters
```
Copy key to the on-board PC
```
ssh-copy-id user@host
```


Thanks to https://github.com/ctu-vras/steam-deck-ros-controller
