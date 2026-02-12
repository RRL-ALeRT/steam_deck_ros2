#!/usr/bin/env python3

# For steam deck, this has to be executed through crontab @reboot
# All the dependencies should be installed in arch
# sudo pacman -Sy pango python-gobject gtk3 libappindicator-gtk3

import os
import sys
import subprocess
import signal
import time
import uuid
from PyQt5.QtWidgets import QApplication
import gi
gi.require_version('AppIndicator3', '0.1')
from gi.repository import AppIndicator3 as appindicator, Gtk

def is_steamos():
    try:
        with open("/etc/os-release", "r") as f:
            return "ID=steamos" in f.read()
    except:
        return False

def move_window_to_desktop(title, desktop_index):
    """Moves a window with the given title to a specific desktop index (0-indexed) using wmctrl.
    This is non-blocking to the main script.
    """
    # Desktops in wmctrl are 0-indexed.
    # We use -r to target by title and -t to move to desktop index.
    # Note: This does not switch the user's current desktop.
    try:
        # Give the window a moment to appear in the background
        cmd = f"sleep 3 && wmctrl -r '{title}' -t {desktop_index}"
        subprocess.Popen(cmd, shell=True)
    except Exception as e:
        print(f"Failed to initiate move for window '{title}': {e}")

def get_unique_perspective(original_path):
    """Creates a unique symlink for a perspective file to avoid rqt's buggy cleanup logic."""
    if not os.path.exists(original_path):
        return original_path
    
    try:
        unique_id = str(uuid.uuid4())[:8]
        tmp_dir = "/tmp/rqt_perspectives"
        os.makedirs(tmp_dir, exist_ok=True)
        
        file_name = os.path.basename(original_path)
        base, ext = os.path.splitext(file_name)
        # Unique name ensures rqt treats it as a 'new' import every time
        unique_file_name = f"{base}_{unique_id}{ext}"
        unique_path = os.path.join(tmp_dir, unique_file_name)
        
        if os.path.exists(unique_path):
            os.remove(unique_path)
        os.symlink(original_path, unique_path)
        return unique_path
    except Exception as e:
        print(f"Failed to create unique perspective symlink: {e}")
        return original_path

# Use home directory instead of hardcoded paths
home_dir = os.path.expanduser("~")
workspace_path = "ros2_ws/src/alert_dashboard_rqt"
pkg_path = os.path.join(home_dir, workspace_path)

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

    # First desktop (intended): rqt2 and ssh console
    # We will launch them and then move them to Desktop 2 (index 1)
    # Workaround: Use unique perspective path to avoid rqt crash
    rqt2_unique = get_unique_perspective(rqt2)
    
    commands_to_desktop_2 = {
        "dashboard": {
            "title": os.path.basename(rqt2_unique).replace(".perspective", ""), 
            "cmd": f"rqt --force-discover --perspective-file {rqt2_unique} --lock-perspective"
        },
        "console": {
            "title": "Spot Console",
            "cmd": f"konsole --title 'Spot Console' -e 'ssh {MAX_USER}@{MAX_IP} -t sleep 5 ; tmux a'"
        },
    }

    for i, (win_key, data) in enumerate(commands_to_desktop_2.items()):
        cmd = data["cmd"]
        title = data["title"]
        if i == 0:
            # Rename first window and send command keys
            subprocess.call(f"tmux rename-window -t {session}:0 {win_key}", shell=True)
            subprocess.call(f"tmux send-keys -t {session}:{win_key} '{cmd}' C-m", shell=True)
        else:
            # Create new empty window, then send command keys to run command interactively
            subprocess.call(f"tmux new-window -t {session} -n {win_key}", shell=True)
            time.sleep(0.5)
            subprocess.call(f"tmux send-keys -t {session}:{win_key} '{cmd}' C-m", shell=True)
        
        # Move to Desktop 2 (index 1) in background
        move_window_to_desktop(title, 1)
        time.sleep(1)

    # Removed intrusive desktop switching as requested
    # switch_to_desktop(1)
    # time.sleep(1)

    # Second desktop (intended): rqt1, rviz2, controller
    # We will move them to Desktop 1 (index 0)
    # Workaround: Use unique perspective path to avoid rqt crash
    rqt1_unique = get_unique_perspective(rqt1)
    
    commands_to_desktop_1 = {
        "estop": {
            "title": os.path.basename(rqt1_unique).replace(".perspective", ""),
            "cmd": f"rqt --force-discover --perspective-file {rqt1_unique} --lock-perspective"
        },
        "rviz2": {
            "title": "rviz2", # rviz2 typically sets its title to rviz2 or Rviz2
            "cmd": "rviz2"
        },
        "controller": {
            "title": "", # No window for controller
            "cmd": "ros2 launch spot_driver_plus controller_launch.py"
        },
    }

    for win_key, data in commands_to_desktop_1.items():
        cmd = data["cmd"]
        title = data["title"]
        # Create new empty window, then send command keys
        subprocess.call(f"tmux new-window -t {session} -n {win_key}", shell=True)
        time.sleep(0.5)
        subprocess.call(f"tmux send-keys -t {session}:{win_key} '{cmd}' C-m", shell=True)
        
        if title:
            # Move to Desktop 1 (index 0) in background
            move_window_to_desktop(title, 0)
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

spot_icon = os.path.join(home_dir, "steam_deck_ros2/spot.png")

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
