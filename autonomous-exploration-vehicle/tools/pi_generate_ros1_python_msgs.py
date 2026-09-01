#!/usr/bin/env python3
from pathlib import Path

from genmsg import MsgContext
from genmsg.msg_loader import load_msg_from_file
from genpy.generator import msg_generator


OUT_ROOT = Path("/home/pi/ros_genpy")
SEARCH_PATH = {
    "std_msgs": ["/usr/share/std_msgs/msg"],
    "sensor_msgs": ["/usr/share/sensor_msgs/msg"],
}
MESSAGES = [
    ("std_msgs/Header", "/usr/share/std_msgs/msg/Header.msg"),
    ("sensor_msgs/LaserScan", "/usr/share/sensor_msgs/msg/LaserScan.msg"),
]


def write_message(pkg, short_name, generated_lines):
    pkg_dir = OUT_ROOT / pkg / "msg"
    pkg_dir.mkdir(parents=True, exist_ok=True)
    (OUT_ROOT / pkg / "__init__.py").write_text("", encoding="utf-8")
    (pkg_dir / "__init__.py").write_text(
        f"from ._{short_name} import *\n", encoding="utf-8"
    )
    (pkg_dir / f"_{short_name}.py").write_text(
        "\n".join(generated_lines) + "\n", encoding="utf-8"
    )


def main():
    context = MsgContext.create_default()
    for full_name, msg_path in MESSAGES:
        pkg, short_name = full_name.split("/")
        spec = load_msg_from_file(context, msg_path, full_name)
        lines = list(msg_generator(context, spec, SEARCH_PATH))
        write_message(pkg, short_name, lines)

    print(f"Generated ROS Python message modules in {OUT_ROOT}")


if __name__ == "__main__":
    main()
