#!/usr/bin/env python3
"""
python3 rosbag_check.py -i /path/to/input_dir -o /path/to/output_dir -n ros2bag_name
"""
import argparse
import os
import shutil
import subprocess
import sys
from pathlib import Path
from typing import Dict, List, Tuple
import yaml

# Required topics for Livox setup
REQUIRED_TOPICS: List[str] = [
    "/livox/imu",
    "/livox/lidar",
]

# Target rates in Hz (messages or pictures per second) and tolerance (target_hz, tol_hz)
RATE_TARGETS: Dict[str, Tuple[float, float]] = {
    "/livox/imu": (200.0, 5.0),   # Expected ~200 Hz (range 195-205)
    "/livox/lidar": (10.0, 0.5),   # Expected ~10 Hz  (range 9.5-10.5)
    "camera": (10.0, 1.0),          # Expected ~10 Hz  (range 9-11)
}

IMAGE_EXTENSIONS = {".jpg", ".jpeg", ".png", ".bmp", ".pnm", ".tiff"}


def check_rosbag_integrity(bag_dir_path: Path, input_dir: Path, required_topics: List[str]) -> bool:
    """Verifies ROS 2 bag integrity, outputting duration (ns), topic frequencies (Hz), and camera picture frequencies (Hz)."""
    print(f"\nChecking ROS 2 Bag Directory: {bag_dir_path}")
    print("=" * 60)

    # 1. Check folder existence
    if not bag_dir_path.exists() or not bag_dir_path.is_dir():
        print(f"[FAIL] ROS 2 bag directory does not exist or is invalid: {bag_dir_path}")
        return False
    print("[OK] Bag folder exists.")

    # 2. Check metadata (.yaml) existence
    yaml_files = list(bag_dir_path.glob("*.yaml"))
    if not yaml_files:
        print(f"[FAIL] No YAML metadata file found in: {bag_dir_path}")
        return False

    yaml_path = bag_dir_path / "metadata.yaml"
    if not yaml_path.exists():
        yaml_path = yaml_files[0]

    print(f"[OK] Found metadata file: {yaml_path.name}")

    # Parse metadata YAML
    try:
        with open(yaml_path, "r", encoding="utf-8") as f:
            metadata = yaml.safe_load(f)
    except Exception as e:
        print(f"[FAIL] Failed to parse YAML file: {e}")
        return False

    info = metadata.get("rosbag2_bagfile_information", {})

    # Extract Duration
    duration_info = info.get("duration", {})
    duration_ns = duration_info.get("nanoseconds", 0) if isinstance(duration_info, dict) else 0
    duration_sec = duration_ns / 1e9 if duration_ns > 0 else 0.0

    print("-" * 60)
    print(f"Bag Duration: {duration_ns} nanoseconds ({duration_sec:.2f} seconds)")

    if duration_sec == 0:
        print("[FAIL] Duration is 0 seconds. Cannot compute message rates.")
        return False

    # 3. Check data files (.mcap / .db3)
    files_list = info.get("files", [])
    if not files_list:
        relative_paths = info.get("relative_file_paths", [])
        if relative_paths:
            files_list = [{"path": p} for p in relative_paths]

    if not files_list:
        print("[FAIL] No bag data files declared in metadata YAML.")
        return False

    bag_files_ok = True
    for file_entry in files_list:
        rel_path = file_entry.get("path", "")
        full_bag_path = bag_dir_path / rel_path

        if not full_bag_path.exists():
            print(f"[FAIL] Referenced bag file missing: {rel_path}")
            bag_files_ok = False
        else:
            file_size_mb = full_bag_path.stat().st_size / (1024 * 1024)
            print(f"[OK] Bag file exists: {rel_path} ({file_size_mb:.2f} MB)")

    if not bag_files_ok:
        return False

    # 4. Check topics, counts, and frequencies (Hz)
    topics_in_bag: Dict[str, int] = {}
    for entry in info.get("topics_with_message_count", []):
        topic_meta = entry.get("topic_metadata", {})
        topic_name = topic_meta.get("name")
        msg_count = entry.get("message_count", 0)
        if topic_name:
            topics_in_bag[topic_name] = msg_count

    print("-" * 60)
    print(f"Total topics recorded in bag: {len(topics_in_bag)}")

    all_checks_passed = True

    for req_topic in required_topics:
        if req_topic not in topics_in_bag:
            print(f"[FAIL] Required topic MISSING: {req_topic}")
            all_checks_passed = False
        else:
            count = topics_in_bag[req_topic]
            rate = count / duration_sec
            target_info = RATE_TARGETS.get(req_topic)

            if count == 0:
                print(f"[FAIL] {req_topic}: 0 messages recorded.")
                all_checks_passed = False
            elif target_info is not None:
                target_hz, tol_hz = target_info
                min_hz = target_hz - tol_hz
                max_hz = target_hz + tol_hz

                if min_hz <= rate <= max_hz:
                    print(
                        f"[OK] {req_topic}: {count} msgs | {rate:.2f} Hz "
                        f"(Target: ~{target_hz:.0f} Hz)"
                    )
                else:
                    print(
                        f"[WARN] {req_topic}: {count} msgs | {rate:.2f} Hz "
                        f"(Expected range: {min_hz:.0f}-{max_hz:.0f} Hz)"
                    )
            else:
                print(f"[OK] {req_topic}: {count} msgs | {rate:.2f} Hz")

    # 5. Check camera images and picture rates (Hz)
    print("-" * 60)
    print("Checking Camera Images:")
    camera_dir = input_dir / "camera"
    cam_target_hz, cam_tol_hz = RATE_TARGETS["camera"]
    cam_min_hz = cam_target_hz - cam_tol_hz
    cam_max_hz = cam_target_hz + cam_tol_hz

    for cam_side in ["left", "right"]:
        cam_dir = camera_dir / cam_side
        if not cam_dir.exists() or not cam_dir.is_dir():
            print(f"[FAIL] Camera subfolder missing: {cam_dir}")
            all_checks_passed = False
            continue

        img_files = [
            f for f in cam_dir.iterdir()
            if f.is_file() and f.suffix.lower() in IMAGE_EXTENSIONS
        ]
        img_count = len(img_files)
        img_rate = img_count / duration_sec

        if img_count == 0:
            print(f"[FAIL] Camera '{cam_side}': 0 pictures found.")
            all_checks_passed = False
        elif cam_min_hz <= img_rate <= cam_max_hz:
            print(
                f"[OK] Camera '{cam_side}': {img_count} pictures | {img_rate:.2f} Hz "
                f"(Target: ~{cam_target_hz:.0f} Hz)"
            )
        else:
            print(
                f"[WARN] Camera '{cam_side}': {img_count} pictures | {img_rate:.2f} Hz "
                f"(Expected range: {cam_min_hz:.0f}-{cam_max_hz:.0f} Hz)"
            )

    print("=" * 60)
    if all_checks_passed:
        print("Integrity Check PASSED: All files, topics, duration, and frequencies verified successfully.")
    else:
        print("Integrity Check FAILED: Missing topics, zero counts, or directory errors detected.")

    return all_checks_passed


def main():
    parser = argparse.ArgumentParser(
        description="Convert a ROS 1 bag directory into ROS 2 and verify topic/camera message frequencies."
    )
    parser.add_argument(
        "-i",
        "--input",
        required=True,
        type=str,
        help="Path to the source ROS 1 bag directory (containing 'data/*.bag' and 'camera/')",
    )
    parser.add_argument(
        "-o",
        "--output-dir",
        required=True,
        type=str,
        help="Directory path where the output ROS 2 bag will be saved",
    )
    parser.add_argument(
        "-n",
        "--output-name",
        required=True,
        type=str,
        help="Name of the output ROS 2 bag folder",
    )

    args = parser.parse_args()

    input_dir = Path(args.input).resolve()
    output_dir = Path(args.output_dir).resolve()
    output_path = output_dir / args.output_name

    # 1. Verify existence of ROS 1 directory structure
    if not input_dir.exists() or not input_dir.is_dir():
        print(f"Error: Input directory does not exist: '{input_dir}'", file=sys.stderr)
        sys.exit(1)

    data_dir = input_dir / "data"
    if not data_dir.exists() or not data_dir.is_dir():
        print(f"Error: Could not find 'data' subfolder at '{data_dir}'.", file=sys.stderr)
        sys.exit(1)

    bag_files = list(data_dir.glob("*.bag"))
    if not bag_files:
        print(f"Error: No '.bag' file found inside data directory: '{data_dir}'", file=sys.stderr)
        sys.exit(1)

    input_bag_file = bag_files[0]

    camera_dir = input_dir / "camera"
    if not camera_dir.exists() or not (camera_dir / "left").is_dir() or not (camera_dir / "right").is_dir():
        print(f"Error: Missing camera directory structure in '{camera_dir}' (requires left/ and right/).", file=sys.stderr)
        sys.exit(1)

    # 2. Check dependencies
    if not shutil.which("rosbags-convert"):
        print(
            "Error: 'rosbags-convert' command not found.\n"
            "Please install it using: pip install rosbags",
            file=sys.stderr,
        )
        sys.exit(1)

    output_dir.mkdir(parents=True, exist_ok=True)

    if output_path.exists():
        print(
            f"Error: Output path '{output_path}' already exists. Choose a different name or destination.",
            file=sys.stderr,
        )
        sys.exit(1)

    # 3. Convert ROS 1 bag to ROS 2
    print(f"Found ROS 1 bag file: {input_bag_file}")
    print(f"Converting ROS 1 bag to ROS 2 folder at: {output_path}")

    try:
        cmd = [
            "rosbags-convert",
            "--src",
            str(input_bag_file),
            "--dst",
            str(output_path),
        ]
        subprocess.run(cmd, check=True)
        print(f"\nSuccessfully converted to ROS 2 bag at: {output_path}")
    except subprocess.CalledProcessError as e:
        print(f"Error during bag conversion: {e}", file=sys.stderr)
        sys.exit(e.returncode)

    # 4. Verify output ROS 2 bag integrity, duration, and frequencies
    success = check_rosbag_integrity(output_path, input_dir, REQUIRED_TOPICS)
    sys.exit(0 if success else 1)


if __name__ == "__main__":
    main()
