import os
import signal
import subprocess
import asyncio
import threading
import shlex
import rclpy
from rclpy.node import Node
from aid_robot_msgs.srv import ControlLaunch, QueryLaunchStatus


class LaunchManagerNode(Node):
    def __init__(self):
        super().__init__('aid_robot_launch_manager')

        self.declare_parameter('launch_files', [])
        self.launch_files = self.get_parameter(
            'launch_files').get_parameter_value().string_array_value
        self.use_sim_time = self.get_parameter(
            'use_sim_time').get_parameter_value().bool_value
        self._processes = []
        self._lock = threading.Lock()

        self.create_service(ControlLaunch, 'start_launch',
                            self.start_launch_callback)
        self.create_service(ControlLaunch, 'stop_launch',
                            self.stop_launch_callback)
        self.create_service(
            QueryLaunchStatus, 'query_launch_status', self.query_launch_status_callback)

        for launch_file in self.launch_files:
            if not os.path.isfile(launch_file):
                self.get_logger().error(
                    f"Launch file '{launch_file}' does not exist.")
                continue
            self._start_process(launch_file)

        self._loop = asyncio.get_event_loop()
        self._loop.create_task(self._monitor_processes())

    async def _monitor_processes(self):
        # Asynchronous monitoring of the output and status of all processes
        while rclpy.ok():
            to_remove = []
            with self._lock:
                for launch_file, process in self._processes:
                    if process.poll() is not None:  # Process has exited
                        to_remove.append((launch_file, process))
                    else:
                        await self._read_stream_output(launch_file, process)
                for item in to_remove:
                    self._processes.remove(item)
            await asyncio.sleep(0.1)

    def _read_stream_output_thread(self, launch_file, process, stream):
        """Real-time output reads for the thread version"""
        while True:
            line = stream.readline()
            if not line:
                break
            line = line.strip()
            if line:
                # self.get_logger().info(f"{launch_file}: {line}")
                self.get_logger().info(f"{line}")

    def _start_process(self, launch_file, params=None):
        """Start a new ROS2 launch process and start the output read thread"""
        cmd = ['ros2', 'launch', launch_file]

        if self.use_sim_time:
            cmd.append('use_sim_time:=true')
        else:
            cmd.append('use_sim_time:=false')

        if params:
            cmd.extend(shlex.split(params))
        try:
            process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                start_new_session=True,
                bufsize=1,
                universal_newlines=True
            )
            with self._lock:

                self._processes.append((launch_file, process))

            # Start a thread to read standard output and error output
            threading.Thread(target=self._read_stream_output_thread, args=(
                launch_file, process, process.stdout), daemon=True).start()
            threading.Thread(target=self._read_stream_output_thread, args=(
                launch_file, process, process.stderr), daemon=True).start()

            self.get_logger().info(
                f"Started process for {launch_file} {params}")
        except Exception as e:
            self.get_logger().error(
                f"Failed to start {launch_file} {params}: {e}")

    def start_launch_callback(self, request, response):
        """Launch the specified ROS2 launch file"""
        launch_file = request.launch_file
        params = request.parameter

        with self._lock:
            # 已自行退出（崩溃/被外部杀掉）的记录要先清掉，否则会永远"already running"
            self._processes = [(lf, p) for lf, p in self._processes if p.poll() is None]
            if any(launch_file == lf for lf, _ in self._processes):
                response.success = False
                response.message = f"{launch_file} is already running."
                return response

        self._start_process(launch_file, params)
        response.success = True
        response.message = f"Started {launch_file}."
        return response

    def stop_launch_callback(self, request, response):
        """Stop the specified ROS2 launch file"""
        launch_file = request.launch_file
        with self._lock:
            for lf, process in self._processes:
                if launch_file == lf:
                    self._terminate_process(process)
                    self._processes.remove((lf, process))
                    response.success = True
                    response.message = f"Stopped {launch_file}."
                    return response

        response.success = False
        response.message = f"{launch_file} is not running."
        return response

    def query_launch_status_callback(self, request, response):
        """Query the running status of a specified ROS2 launch file"""
        launch_file = request.launch_file
        with self._lock:
            for lf, process in self._processes:
                if launch_file == lf:
                    response.success = True
                    response.message = 'running' if process.poll() is None else 'exited'
                    return response

        response.success = True
        response.message = 'not found'
        return response

    def _terminate_process(self, process):
        """Terminate the child process"""
        pgid = None
        try:
            pgid = os.getpgid(process.pid)
        except ProcessLookupError:
            pass
        # SIGINT 让 ros2 launch 正常收尾；超时后对整个进程组 SIGTERM，再超时由下方 SIGKILL 清场。
        # 总时长有上界（约 13 s），调用方（robot_status_manager）据此设置等待时间。
        try:
            process.send_signal(signal.SIGINT)
            process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            try:
                if pgid is not None:
                    os.killpg(pgid, signal.SIGTERM)
                process.wait(timeout=3)
            except (subprocess.TimeoutExpired, ProcessLookupError, PermissionError):
                pass
        finally:
            # ros2 launch 退出后仍可能留下同进程组的子节点（孤儿），
            # 会与下一次启动的同名节点冲突，统一清理整个进程组。
            if pgid is not None:
                try:
                    os.killpg(pgid, signal.SIGKILL)
                except (ProcessLookupError, PermissionError):
                    pass
            output, error = process.communicate()
            self.get_logger().info(
                f"Process terminated. Output: {output}, Error: {error}")

    def shutdown_callback(self):
        """Clean up all child processes when the node is shut down"""
        self.get_logger().info("Shutting down. Terminating all processes.")
        with self._lock:
            for _, process in self._processes:
                self._terminate_process(process)
            self._processes.clear()

    def run(self):
        try:
            rclpy.spin(self)
        except KeyboardInterrupt:
            pass
        finally:
            self.shutdown_callback()
            rclpy.shutdown()


def main():
    rclpy.init()
    node = LaunchManagerNode()
    try:
        node.run()
    finally:
        asyncio.get_event_loop().close()


if __name__ == '__main__':
    main()
