#!/usr/bin/env python3
"""Stop ROS processes owned by the current user without touching Unitree services."""

import argparse
import os
import signal
import time
from pathlib import Path


def cmdline(pid):
    try:
        return Path(f'/proc/{pid}/cmdline').read_bytes().replace(b'\0', b' ').decode(
            errors='replace')
    except (FileNotFoundError, PermissionError, ProcessLookupError):
        return ''


def parent(pid):
    try:
        fields = Path(f'/proc/{pid}/stat').read_text().split()
        return int(fields[3])
    except (FileNotFoundError, PermissionError, ProcessLookupError, ValueError):
        return 0


def ancestors():
    result = set()
    pid = os.getpid()
    while pid > 1 and pid not in result:
        result.add(pid)
        pid = parent(pid)
    return result


def alive(pid):
    try:
        return Path(f'/proc/{pid}/stat').read_text().split()[2] != 'Z'
    except (FileNotFoundError, PermissionError, ProcessLookupError, IndexError):
        return False


def find_targets(workspace):
    excluded = ancestors()
    uid = os.getuid()
    patterns = (
        f'{workspace.rstrip("/")}/install/',
        '/opt/ros/jazzy/lib/',
        '/opt/ros/humble/lib/',
        '/opt/ros/jazzy/bin/ros2',
        '/opt/ros/humble/bin/ros2',
        'ros2cli.daemon',
    )
    targets = {}
    for entry in Path('/proc').iterdir():
        if not entry.name.isdigit():
            continue
        pid = int(entry.name)
        try:
            if pid in excluded or entry.stat().st_uid != uid:
                continue
        except (FileNotFoundError, PermissionError, ProcessLookupError):
            continue
        command = cmdline(pid)
        if command and any(pattern in command for pattern in patterns):
            targets[pid] = command
    return targets


def send(targets, sig):
    for pid in targets:
        try:
            os.kill(pid, sig)
        except (ProcessLookupError, PermissionError):
            pass


def wait_for_exit(targets, timeout):
    deadline = time.monotonic() + timeout
    remaining = set(targets)
    while remaining and time.monotonic() < deadline:
        remaining = {pid for pid in remaining if alive(pid)}
        if remaining:
            time.sleep(0.1)
    return {pid for pid in remaining if alive(pid)}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--workspace', default='/opt/G1/lighting_ws')
    args = parser.parse_args()

    targets = find_targets(args.workspace)
    if not targets:
        print('No matching ROS processes found.')
        return
    for pid, command in sorted(targets.items()):
        print(f'Stopping PID {pid}: {command}')

    send(targets, signal.SIGINT)
    remaining = wait_for_exit(targets, 3.0)
    if remaining:
        send(remaining, signal.SIGTERM)
        remaining = wait_for_exit(remaining, 2.0)
    if remaining:
        send(remaining, signal.SIGKILL)
        wait_for_exit(remaining, 1.0)
    print('ROS process cleanup complete.')


if __name__ == '__main__':
    main()
