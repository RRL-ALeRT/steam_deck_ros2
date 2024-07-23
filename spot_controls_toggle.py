#!/usr/bin/env python3

# For steam deck, this has to be executed through crontab @reboot
# All the dependencies should be installed in arch
# sudo pacman -Sy pango python-gobject gtk3 libappindicator-gtk3

import os
import sys
import subprocess
import signal
from PyQt5.QtWidgets import QApplication
import gi
gi.require_version('AppIndicator3', '0.1')
from gi.repository import AppIndicator3 as appindicator, Gtk
import time
import subprocess

# from ament_index_python import get_package_share_directory

# pkg_path = get_package_share_directory("alert_dashboard_rqt")
pkg_path = "/home/deck/ros2_ws/src/alert_dashboard_rqt/"

rqt1 = f"{pkg_path}/resource/estop_rqt.perspective"
rqt2 = f"{pkg_path}/resource/dashboard_rqt.perspective"

MAX_USER = os.getenv("MAX_USER")
MAX_IP = os.getenv("MAX_IP")
MAX_USER = "max1"
MAX_IP = "192.168.50.10"

def switch_to_desktop(number):
    command = "qdbus org.kde.KWin /KWin org.kde.KWin.setCurrentDesktop {number}"
    subprocess.Popen(command, shell=True)


def open_ros_apps():
    commands = [
        # "killall steam", # else controller doesn't work
        "screen -dmS deck"
    ]
    for command in commands:
        subprocess.Popen(command, shell=True)
        time.sleep(2)

    time.sleep(1)  # Add a delay to allow the screen session to initialize

    switch_to_desktop(2)

    commands = {
        "rqt2": f"rqt --force-discover --perspective-file {rqt2}",
        "konsole": f"konsole -e 'ssh {MAX_USER}@{MAX_IP} -t sleep 5 ; tmux a'",
    }

    for tab, command in commands.items():
        subprocess.Popen(f"screen -S deck -p {tab} -X stuff \"{command}\n\"", shell=True)
        time.sleep(1)

    time.sleep(5)  # Add a delay to allow the screen session to initialize

    switch_to_desktop(1)

    commands = {
        "rqt1": f"rqt --force-discover --perspective-file {rqt1}",
        "rviz2": "rviz2",
        "controller": "ros2 launch spot_driver_plus controller_launch.py",
        "display": "ros2 launch image_display image_display_launch.py",
        "deck_capture": "ros2 run audio_capture audio_capture_node --ros-args -p format:=wave -r __ns:=/operator",
        "deck_play": "ros2 run audio_play audio_play_node --ros-args -p format:=wave -r __ns:=/nuc",
    }

    for tab, command in commands.items():
        subprocess.Popen(f"screen -S deck -p {tab} -X stuff \"{command}\n\"", shell=True)


def close_ros_apps():
    # Terminate the 'deck' screen session
    subprocess.Popen("screen -S deck -X quit", shell=True)


def toggle_ros_apps(_):
    global apps_running
    if apps_running:
        close_ros_apps()
        apps_running = False
    else:
        open_ros_apps()
        apps_running = True


apps_running = False

app = QApplication(sys.argv)
app.setQuitOnLastWindowClosed(False)

spot_icon = "/home/deck/steam_deck_ros2/spot.png"

indicator = appindicator.Indicator.new("Spot Control View", spot_icon, appindicator.IndicatorCategory.SYSTEM_SERVICES)
indicator.set_status(appindicator.IndicatorStatus.ACTIVE)

menu = Gtk.Menu()

toggle_item = Gtk.ImageMenuItem.new_with_label("Toggle Control View")
toggle_item.set_image(Gtk.Image.new_from_file(spot_icon))
toggle_item.set_always_show_image(True)
toggle_item.connect('activate', toggle_ros_apps)
menu.append(toggle_item)

menu.show_all()
indicator.set_menu(menu)

if __name__ == "__main__":
    signal.signal(signal.SIGINT, signal.SIG_DFL)  # Enable Ctrl+C handling
    Gtk.main()
