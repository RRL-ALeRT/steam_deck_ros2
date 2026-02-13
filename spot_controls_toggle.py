#!/usr/bin/python3

import os
import subprocess
import signal
import time
import uuid
import libtmux
import traceback
import gi
import threading

gi.require_version('AppIndicator3', '0.1')
from gi.repository import AppIndicator3 as appindicator, Gtk

# ---------------------------------------------------------------------------
# Config
# ---------------------------------------------------------------------------

home_dir   = os.path.expanduser("~")
script_dir = os.path.dirname(os.path.abspath(__file__))

def load_env_file(path):
    with open(path) as f:
        for line in f:
            line = line.strip()
            if line.startswith("export "):
                key, _, value = line[len("export "):].partition("=")
                os.environ.setdefault(key.strip(), value.strip())

log_path = os.path.join(script_dir, "crash_log.txt")

def log(msg):
    timestamp = time.strftime("%Y-%m-%d %H:%M:%S")
    with open(log_path, "a") as f:
        f.write(f"[{timestamp}] {msg}\n")

try:
    load_env_file(os.path.join(script_dir, "humble"))
    MAX_USER = os.environ["MAX_USER"]
    MAX_IP   = os.environ["MAX_IP"]
except Exception as e:
    log(f"FATAL: {e}")
    raise

workspace_path    = "ros2_ws/src/alert_dashboard_rqt"
pkg_path          = os.path.join(home_dir, workspace_path)
estop_perspective = f"{pkg_path}/resource/estop_rqt.perspective"

TMUX_SESSION = "steamdeck"
KONSOLE_TITLE = "spot_ssh"

# ---------------------------------------------------------------------------
# Window Management Logic
# ---------------------------------------------------------------------------

def snap_windows_to_desktops():
    """
    Wait for windows to appear and then move them using wmctrl.
    wmctrl desktop index: 0 = Desktop 1, 1 = Desktop 2
    """
    # Define mapping: (Partial Window Title/Class, Target Desktop Index)
    targets = [
        ("RViz", 0),       # rviz2 -> Desktop 1
        ("estop", 0),      # rqt estop -> Desktop 1
        ("rqt_gui", 1),    # dashboard -> Desktop 2
        (KONSOLE_TITLE, 1) # SSH Console -> Desktop 2
    ]

    # Try for up to 10 seconds to find and move all windows
    start_time = time.time()
    while time.time() - start_time < 10:
        found_all = True
        for title_part, desktop_idx in targets:
            try:
                # -r: target by title, -t: move to desktop index
                result = subprocess.run(
                    ["wmctrl", "-r", title_part, "-t", str(desktop_idx)],
                    capture_output=True
                )
                if result.returncode != 0:
                    found_all = False
            except Exception:
                found_all = False
        
        if found_all:
            break
        time.sleep(1.0) # Poll every second

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def get_unique_perspective(original_path):
    if not os.path.exists(original_path):
        raise FileNotFoundError(f"Perspective file not found: {original_path}")
    unique_id   = str(uuid.uuid4())[:8]
    tmp_dir     = "/tmp/rqt_perspectives"
    os.makedirs(tmp_dir, exist_ok=True)
    base, ext   = os.path.splitext(os.path.basename(original_path))
    unique_path = os.path.join(tmp_dir, f"{base}_{unique_id}{ext}")
    if os.path.exists(unique_path):
        os.remove(unique_path)
    os.symlink(original_path, unique_path)
    return unique_path

def get_session():
    server = libtmux.Server()
    sessions = server.sessions.filter(session_name=TMUX_SESSION)
    return sessions[0] if sessions else None

# ---------------------------------------------------------------------------
# Launch / close using libtmux
# ---------------------------------------------------------------------------

def open_ros_apps():
    server  = libtmux.Server()
    session = get_session() 
    
    if session:
        session.kill()
        time.sleep(0.3)

    session      = server.new_session(session_name=TMUX_SESSION, window_name="init", detach=True)
    estop_unique = get_unique_perspective(estop_perspective)

    commands = {
        "dashboard":  "ros2 run rqt_gui rqt_gui --standalone alert_dashboard_rqt.alert_dashboard_rqt.DashboardRqtPlugin",
        "console":    f"konsole --title {KONSOLE_TITLE} -e bash -c 'while true; do ssh {MAX_USER}@{MAX_IP} -t \"sleep 5; tmux new -A -s spot_remote\"; sleep 5; done'",
        "estop":      f"rqt --force-discover --perspective-file {estop_unique} --lock-perspective",
        "rviz2":      "rviz2",
        "controller": "ros2 launch spot_driver_plus controller_launch.py",
    }

    for i, (win_key, cmd) in enumerate(commands.items()):
        if i == 0:
            window = session.windows[0]
            window.rename_window(win_key)
        else:
            window = session.new_window(window_name=win_key)
        window.panes[0].send_keys(cmd, enter=True)

    # Launch the window snappper in a separate thread so the UI stays responsive
    threading.Thread(target=snap_windows_to_desktops, daemon=True).start()

def close_ros_apps():
    session = get_session()
    if session:
        session.kill()

# ---------------------------------------------------------------------------
# Tray indicator
# ---------------------------------------------------------------------------

def toggle_ros_apps(_):
    try:
        if get_session():
            close_ros_apps()
        else:
            open_ros_apps()
    except Exception as e:
        log(f"ERROR in toggle:\n{traceback.format_exc()}")

spot_icon = os.path.join(home_dir, "steam_deck_ros2/spot.png")

indicator = appindicator.Indicator.new(
    "Spot Control View",
    spot_icon,
    appindicator.IndicatorCategory.SYSTEM_SERVICES
)
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
