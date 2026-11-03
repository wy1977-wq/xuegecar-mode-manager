
import os
import signal
import subprocess
import time

from pathlib import Path

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


class ModeManager(Node):

    def __init__(self):
        super().__init__('xuegecar_mode_manager')

        # 需要加载的地图，可以在运行期间修改
        self.declare_parameter(
            'map_file',
            '/home/chuyun/maps/test_map.yaml'
        )

        # 当前运行模式
        self.mode = 'idle'
        self.process = None

        # 用于检查是否存在旧的建图或导航节点
        self.mapping_nodes = {'slam_gmapping', 'map_saver'}
        self.navigation_nodes = {
            'amcl',
            'map_server',
            'planner_server',
            'bt_navigator'
        }

        # ROS 2 操作服务
        self.create_service(
            Trigger,
            '/xuegecar/start_mapping',
            self.start_mapping_callback
        )

        self.create_service(
            Trigger,
            '/xuegecar/start_navigation',
            self.start_navigation_callback
        )

        self.create_service(
            Trigger,
            '/xuegecar/stop_mode',
            self.stop_callback
        )

        self.create_service(
            Trigger,
            '/xuegecar/get_mode',
            self.status_callback
        )

        self.get_logger().info('Mode Manager started')

    def existing_nodes(self):
        """获取当前 ROS 2 图中的节点名称"""
        return {
            name
            for name, namespace
            in self.get_node_names_and_namespaces()
        }

    def conflicting_nodes(self):
        """查找可能重复启动的 SLAM/Nav2 节点"""
        return self.existing_nodes() & (
            self.mapping_nodes | self.navigation_nodes
        )

    def wait_nodes_gone(self, names, timeout=8.0):
        """等待上一模式的 ROS 2 节点退出"""
        deadline = time.monotonic() + timeout

        while time.monotonic() < deadline:
            if not (self.existing_nodes() & names):
                return True
            time.sleep(0.2)

        return not (self.existing_nodes() & names)

    def refresh_state(self):
        """检查所管理的 launch 是否已经退出"""
        if self.process is not None:
            if self.process.poll() is not None:
                self.get_logger().warning(
                    f'{self.mode} launch has exited'
                )
                self.process = None
                self.mode = 'idle'

    def stop_current(self):
        """只停止本管理器启动的进程组"""
        self.refresh_state()

        if self.process is None:
            self.mode = 'idle'
            return True

        old_mode = self.mode

        self.get_logger().info(
            f'Stopping mode: {old_mode}'
        )

        try:
            if self.process.poll() is None:
                # 通知 ros2 launch 正常退出
                os.killpg(
                    self.process.pid,
                    signal.SIGINT
                )

                try:
                    self.process.wait(timeout=15.0)
                except subprocess.TimeoutExpired:
                    self.get_logger().error(
                        'Launch did not stop within 15 seconds'
                    )
                    return False

            self.process = None
            self.mode = 'idle'

            # 等待上一模式的节点从 ROS 图中消失
            old_nodes = (
                self.mapping_nodes
                if old_mode == 'mapping'
                else self.navigation_nodes
            )

            if not self.wait_nodes_gone(old_nodes):
                self.get_logger().error(
                    'Old ROS nodes are still visible'
                )
                return False

            self.get_logger().info(
                f'Mode stopped: {old_mode}'
            )
            return True

        except (OSError, Exception) as exc:
            self.get_logger().error(
                f'Failed to stop launch: {exc}'
            )
            return False

    def start_launch(self, command, target_mode):
        """启动目标模式"""
        try:
            self.process = subprocess.Popen(
                command,
                start_new_session=True
            )
            self.mode = target_mode

            # 检查启动命令是否立即失败
            time.sleep(1.0)

            if self.process.poll() is not None:
                code = self.process.returncode
                self.process = None
                self.mode = 'idle'
                return False, (
                    f'Launch exited early, code={code}'
                )

            self.get_logger().info(
                f'{target_mode} launch started'
            )

            return True, (
                f'{target_mode} launch started; '
                'waiting for ROS nodes to initialize'
            )

        except Exception as exc:
            self.process = None
            self.mode = 'idle'
            return False, str(exc)

    def start_mapping_callback(self, request, response):
        """启动建图模式"""
        self.refresh_state()

        if self.process is not None:
            if self.mode == 'mapping':
                response.success = True
                response.message = 'Mapping already running'
            else:
                response.success = False
                response.message = (
                    'Stop navigation first, then start mapping'
                )
            return response

        conflicts = self.conflicting_nodes()
        if conflicts:
            response.success = False
            response.message = (
                'Existing nodes detected: '
                + ', '.join(sorted(conflicts))
            )
            return response

        command = [
            'ros2', 'launch',
            'xuegecar_navigation2',
            'slam_nav2.launch.py',
            'use_sim_time:=false',
            'use_rviz:=false'
        ]

        response.success, response.message = (
            self.start_launch(command, 'mapping')
        )

        return response

    def start_navigation_callback(self, request, response):
        """停止建图并启动静态地图导航"""
        self.refresh_state()

        map_file = Path(
            str(self.get_parameter('map_file').value)
        ).expanduser().resolve()

        # 必须先确认地图文件存在
        if not map_file.is_file():
            response.success = False
            response.message = (
                f'Map file not found: {map_file}'
            )
            return response

        if map_file.suffix.lower() != '.yaml':
            response.success = False
            response.message = 'Map file must be YAML'
            return response

        if self.process is not None:
            if self.mode == 'navigation':
                response.success = True
                response.message = 'Navigation already running'
                return response

            if not self.stop_current():
                response.success = False
                response.message = (
                    'Mapping did not stop cleanly. '
                    'Navigation not started.'
                )
                return response

        conflicts = self.conflicting_nodes()
        if conflicts:
            response.success = False
            response.message = (
                'Existing nodes detected: '
                + ', '.join(sorted(conflicts))
            )
            return response

        command = [
            'ros2', 'launch',
            'xuegecar_navigation2',
            'navigation2.launch.py',
            f'map:={map_file}',
            'use_sim_time:=false'
        ]

        response.success, response.message = (
            self.start_launch(command, 'navigation')
        )

        return response

    def stop_callback(self, request, response):
        """停止当前管理的模式"""
        self.refresh_state()

        if self.process is None:
            response.success = True
            response.message = 'Already idle'
            return response

        response.success = self.stop_current()
        response.message = (
            'Mode stopped'
            if response.success
            else 'Failed to stop mode; check logs'
        )

        return response

    def status_callback(self, request, response):
        """查询当前模式"""
        self.refresh_state()

        response.success = True
        response.message = self.mode
        return response


def main(args=None):
    rclpy.init(args=args)

    node = ModeManager()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_current()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()