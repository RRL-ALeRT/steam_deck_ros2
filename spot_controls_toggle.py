#!/usr/bin/env python3

# For steam deck, this has to be executed through crontab @reboot
# All the dependencies should be installed in arch
# sudo pacman -Sy pango python-gobject gtk3 libappindicator-gtk3

import os
import sys
import subprocess
import signal
import time
from PyQt5.QtWidgets import QApplication
import gi
gi.require_version('AppIndicator3', '0.1')
from gi.repository import AppIndicator3 as appindicator, Gtk

pkg_path = "/home/deck/ros2_ws/src/alert_dashboard_rqt"

rqt1 = f"{pkg_path}/resource/estop_rqt.perspective"
rqt2 = f"{pkg_path}/resource/dashboard_rqt.perspective"

MAX_USER = "max1"
MAX_IP = "192.168.50.10"

def switch_to_desktop(number):
    command = f"qdbus org.kde.KWin /KWin org.kde.KWin.setCurrentDesktop {number}"
    subprocess.Popen(command, shell=True)

def open_ros_apps():
    session = "steamdeck"

    # Kill any existing tmux session
    subprocess.call(f"tmux kill-session -t {session}", shell=True)
    time.sleep(1)

    # Create new tmux session in detached mode with first window named 'rqt2'
    subprocess.call(f"tmux new-session -d -s {session} -n rqt2", shell=True)

    # First desktop: rqt2 and ssh console
    commands_desktop_2 = {
        "rqt2": f"rqt --force-discover --perspective-file {rqt2} --lock-perspective",
        "konsole": f"konsole -e 'ssh {MAX_USER}@{MAX_IP} -t sleep 5 ; tmux a'",
    }

    for i, (win_name, cmd) in enumerate(commands_desktop_2.items()):
        if i == 0:
            # Rename first window and send command keys
            subprocess.call(f"tmux rename-window -t {session}:0 {win_name}", shell=True)
            subprocess.call(f"tmux send-keys -t {session}:{win_name} '{cmd}' C-m", shell=True)
        else:
            # Create new empty window, then send command keys to run command interactively
            subprocess.call(f"tmux new-window -t {session} -n {win_name}", shell=True)
            time.sleep(0.5)
            subprocess.call(f"tmux send-keys -t {session}:{win_name} '{cmd}' C-m", shell=True)
        time.sleep(1)

    switch_to_desktop(1)
    time.sleep(1)

    # Second desktop: rqt1, rviz2, controller, deck_capture
    commands_desktop_1 = {
        "rqt1": f"rqt --force-discover --perspective-file {rqt1} --lock-perspective",
        "rviz2": "rviz2",
        "controller": "ros2 launch spot_driver_plus controller_launch.py",
    }

    for win_name, cmd in commands_desktop_1.items():
        # Create new empty window, then send command keys
        subprocess.call(f"tmux new-window -t {session} -n {win_name}", shell=True)
        time.sleep(0.5)
        subprocess.call(f"tmux send-keys -t {session}:{win_name} '{cmd}' C-m", shell=True)
        time.sleep(1)

def close_ros_apps():
    subprocess.call("tmux kill-session -t steamdeck", shell=True)

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
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    Gtk.main()
