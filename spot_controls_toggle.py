#!/usr/bin/python3

# For steam deck, this has to be executed through crontab @reboot
# All the dependencies should be installed in arch
# sudo pacman -Sy pango python-gobject gtk3 libappindicator-gtk3 python-libtmux

import os
import subprocess
import signal
import time
import uuid
import configparser
import tempfile
import libtmux
import gi

gi.require_version('AppIndicator3', '0.1')
from gi.repository import AppIndicator3 as appindicator, Gtk

# ---------------------------------------------------------------------------
# Config — loaded from the humble file next to this script
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
    GEN3_IP  = os.environ["GEN3_IP"]
except Exception as e:
    log(f"FATAL: {e}")
    raise

workspace_path    = "ros2_ws/src/alert_dashboard_rqt"
pkg_path          = os.path.join(home_dir, workspace_path)
estop_perspective = f"{pkg_path}/resource/estop_rqt.perspective"

TMUX_SESSION = "steamdeck"

RQT_ESTOP_TITLE = "estop"
KONSOLE_TITLE   = "spot_ssh"

KWIN_RULES_PATH = os.path.join(home_dir, ".config", "kwinrulesrc")

# ---------------------------------------------------------------------------
# KWin rules
# ---------------------------------------------------------------------------

KWIN_RULES = {
    "rqt estop → desktop 1": {
        "titlematch":   "2",
        "title":        RQT_ESTOP_TITLE,
        "desktop":      "1",
        "desktopforce": "2",
    },
    "rviz2 → desktop 1": {
        "wmclass":      "rviz2",
        "wmclassmatch": "1",
        "desktop":      "1",
        "desktopforce": "2",
    },
    # dashboard uses --standalone so wmclass is stable, no title guessing needed
    "rqt dashboard → desktop 2": {
        "wmclass":      "rqt_gui",
        "wmclassmatch": "1",
        "desktop":      "2",
        "desktopforce": "2",
    },
    "konsole ssh → desktop 2": {
        "titlematch":   "2",
        "title":        KONSOLE_TITLE,
        "desktop":      "2",
        "desktopforce": "2",
    },
}

def setup_kwin_rules():
    """Merge our rules into kwinrulesrc atomically without touching existing rules."""
    config = configparser.RawConfigParser()
    config.optionxform = str  # preserve key case

    if os.path.exists(KWIN_RULES_PATH):
        config.read(KWIN_RULES_PATH)

    if not config.has_section("General"):
        config.add_section("General")

    existing_count = int(config.get("General", "count", fallback="0"))

    # Map existing descriptions to their group numbers so re-runs update in place
    desc_to_group = {}
    for section in config.sections():
        if section == "General":
            continue
        if config.has_option(section, "description"):
            desc_to_group[config.get(section, "description")] = section

    used_groups = {s for s in config.sections() if s != "General"}
    new_count = existing_count
    next_num  = existing_count + 1

    for description, rule in KWIN_RULES.items():
        if description in desc_to_group:
            group = desc_to_group[description]
        else:
            while str(next_num) in used_groups:
                next_num += 1
            group = str(next_num)
            used_groups.add(group)
            next_num  += 1
            new_count += 1

        if not config.has_section(group):
            config.add_section(group)

        config.set(group, "description", description)
        for key, value in rule.items():
            config.set(group, key, str(value))

    config.set("General", "count", str(new_count))

    # Atomic write: temp file in same dir then os.replace() to avoid corruption
    config_dir = os.path.dirname(KWIN_RULES_PATH)
    os.makedirs(config_dir, exist_ok=True)
    fd, temp_path = tempfile.mkstemp(dir=config_dir, text=True)
    try:
        with os.fdopen(fd, "w") as f:
            # Write manually to avoid configparser's blank lines between sections
            # which KWin does not tolerate
            for section in config.sections():
                f.write(f"[{section}]\n")
                for key, value in config.items(section):
                    f.write(f"{key}={value}\n")
        os.replace(temp_path, KWIN_RULES_PATH)
    except Exception as e:
        os.remove(temp_path)
        log(f"Failed to write KWin rules: {e}")
        raise

    subprocess.call("qdbus org.kde.KWin /KWin reconfigure", shell=True)

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def get_unique_perspective(original_path):
    """
    Creates a unique symlink to avoid rqt's buggy re-import crash.
    Only needed for estop — dashboard uses --standalone which has no such issue.
    """
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
    """Return the tmux session if it exists, else None."""
    server = libtmux.Server()
    return server.find_where({"session_name": TMUX_SESSION})

# ---------------------------------------------------------------------------
# Launch / close using libtmux
# ---------------------------------------------------------------------------

def open_ros_apps():
    setup_kwin_rules()

    server  = libtmux.Server()
    session = server.find_where({"session_name": TMUX_SESSION})
    if session:
        session.kill_session()
        time.sleep(0.3)  # wait for tmux to release the session name, not a GUI wait

    session      = server.new_session(session_name=TMUX_SESSION, window_name="init", detach=True)
    estop_unique = get_unique_perspective(estop_perspective)

    commands = {
        # dashboard: --standalone loads single plugin with stable wmclass, no perspective file needed
        "dashboard":  "ros2 run rqt_gui rqt_gui --standalone alert_dashboard_rqt.alert_dashboard_rqt.DashboardRqtPlugin",
        # while true reconnects automatically on SSH drop without needing a full restart
        "console":    f"konsole --title {KONSOLE_TITLE} -e bash -c 'while true; do ssh {MAX_USER}@{MAX_IP} -t \"sleep 5; tmux a\"; sleep 5; done'",
        # estop: loads multiple plugins so a perspective file is required
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
        # send_keys sends characters directly to the PTY — no shell quoting involved
        window.panes[0].send_keys(cmd, enter=True)

def close_ros_apps():
    session = get_session()
    if session:
        session.kill_session()

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
        log(f"ERROR in toggle: {e}")

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
