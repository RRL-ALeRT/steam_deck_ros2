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




# Troubleshooting FFmpeg VAAPI Initialization on Arch Linux

If you encounter issues initializing VAAPI with FFmpeg on Arch Linux, follow these steps to troubleshoot and resolve the problem.

```sh
# 1. Check Device Path
# Verify the available VAAPI device paths:
ls /dev/dri

# Ensure `renderD128` is present. If not, adjust the path in your FFmpeg command accordingly.

# 2. Install VAAPI Drivers
# Install the necessary VAAPI drivers. For Intel GPUs, you need `intel-media-driver` or `libva-intel-driver`. For AMD GPUs, use `libva-mesa-driver`.
sudo pacman -S libva-intel-driver libva-mesa-driver

# Choose the appropriate driver for your hardware.

# 3. Verify User Permissions
# Ensure the user has the correct permissions to access the VAAPI device:
sudo usermod -aG video $USER

# Log out and log back in for the changes to take effect.

# 4. Check VAAPI Functionality
# Install `vainfo` to verify VAAPI functionality:
sudo pacman -S libva-utils
vainfo

# The output should list available VAAPI profiles and entrypoints. If this fails, it indicates an issue with the VAAPI installation or configuration.

# 5. Ensure FFmpeg Supports VAAPI
# Ensure that your FFmpeg build supports VAAPI. Install FFmpeg with VAAPI support using:
sudo pacman -S ffmpeg

# Confirm VAAPI support by checking the configuration:
ffmpeg -hwaccels

# Look for `vaapi` in the list of supported hardware accelerations.

# 6. Simplify Initialization
# Instead of specifying the device directly, try to use auto-detection. Remove the explicit device path and use `-init_hw_device vaapi` to see if FFmpeg can auto-detect the correct device.
ffmpeg -init_hw_device vaapi -f lavfi -i testsrc=duration=5:size=1280x720:rate=30 -vf 'format=nv12,hwupload' -c:v h264_vaapi output.mp4

# 7. Check `libva` Environment Variables
# Sometimes setting the `LIBVA_DRIVER_NAME` and `LIBVA_DRIVERS_PATH` environment variables can help. For AMD, the driver name is usually `radeonsi`.
export LIBVA_DRIVER_NAME=radeonsi
export LIBVA_DRIVERS_PATH=/usr/lib/dri
ffmpeg -init_hw_device vaapi=foo:/dev/dri/renderD128 -f lavfi -i testsrc=duration=5:size=1280x720:rate=30 -vf 'format=nv12,hwupload' -c:v h264_vaapi output.mp4

# 8. Rebuild FFmpeg with Specific Flags
# If you built FFmpeg yourself, ensure all necessary flags and dependencies were correctly included. Use Arch Linux's official FFmpeg package for the best compatibility:
sudo pacman -S ffmpeg

# 9. Update System and Drivers
# Make sure your system is fully updated to include the latest kernel, drivers, and libraries.
sudo pacman -Syu

# 10. Verify Access to VAAPI Device
# Check that the device node `/dev/dri/renderD128` has appropriate permissions and that the `video` group has access.
ls -l /dev/dri/renderD128

# 11. Run FFmpeg with Debug Information
# To get more detailed output, run FFmpeg with the `-loglevel debug` flag to provide more insight into why the initialization is failing.
ffmpeg -loglevel debug -init_hw_device vaapi=foo:/dev/dri/renderD128 -f lavfi -i testsrc=duration=5:size=1280x720:rate=30 -vf 'format=nv12,hwupload' -c:v h264_vaapi output.mp4
