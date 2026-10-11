#!/usr/bin/env python3
"""
Test Runner Node — ROS-native hardware test orchestration.

Exposes /hw_test/run (chess_interfaces/action/RunHardwareTest) so any
client (Chess OS, ros2 action send_goal, a future CLI) can kick off a
hardware test category/subtest and stream its output as action feedback.

Internally this still shells out to run_hw_test.sh exactly as Chess OS's
old in-process _TestRunner did — only the *ownership* of the subprocess
moves into a persistent ROS node, so test output survives a Chess OS
restart and the run is visible via normal ROS tooling
(ros2 action list / ros2 action info / ros2 action send_goal).
"""

import os
import queue
import signal
import subprocess
import tempfile
import threading
from pathlib import Path

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from chess_interfaces.action import RunHardwareTest


class TestRunnerNode(Node):

    def __init__(self):
        super().__init__('test_runner_node')

        self.declare_parameter(
            'test_script_path',
            str(Path.home() / 'dev' / 'smart_chess_board' / 'run_hw_test.sh'))

        self._busy = threading.Lock()

        self._action_server = ActionServer(
            self,
            RunHardwareTest,
            '/hw_test/run',
            execute_callback=self.execute_callback,
            goal_callback=self._goal_cb,
            cancel_callback=self._cancel_cb,
        )

        self.get_logger().info('Test Runner Node ready (/hw_test/run)')

    def _goal_cb(self, goal_request):
        if self._busy.locked():
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _cancel_cb(self, goal_handle):
        self.get_logger().info('Test run cancel requested')
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        category = goal_handle.request.category
        subtest  = goal_handle.request.subtest
        result   = RunHardwareTest.Result()

        script = Path(self.get_parameter('test_script_path').value)
        if script.exists():
            cmd = ['bash', str(script), '--category', category, '--subtest', subtest]
            cwd = str(script.parent)
        else:
            cmd = ['python3', '-m', 'chess_hw_interface.testing.test_runner',
                   '--category', category, '--subtest', subtest]
            cwd = None

        # The test subprocess normally runs as root (via sudo in
        # run_hw_test.sh, see setup/smart-chess-hw-tests.sudoers), so
        # os.killpg() below is a best-effort SIGTERM that the kernel will
        # silently reject once it reaches the root-owned process (regular
        # users cannot signal a more-privileged process regardless of
        # process group). The killswitch file is the reliable cancel path:
        # test_runner.py polls for it between test steps and exits itself.
        cancel_file = tempfile.mktemp(prefix='chess_hw_test_cancel_')
        env = os.environ.copy()
        env['CHESS_HW_TEST_CANCEL_FILE'] = cancel_file

        with self._busy:
            self.get_logger().info(f'Starting test: {category}/{subtest}')
            try:
                proc = subprocess.Popen(
                    cmd, cwd=cwd, env=env,
                    stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                    text=True, bufsize=1, preexec_fn=os.setsid)
            except Exception as e:
                goal_handle.abort()
                result.success     = False
                result.return_code = -1
                result.message     = f'Failed to start: {e}'
                return result

            # Reader thread decouples blocking stdout reads from the
            # cancel-polling loop below.
            line_q = queue.Queue()

            def _reader():
                for line in proc.stdout:
                    line_q.put(line.rstrip())
                line_q.put(None)  # sentinel: stdout closed

            threading.Thread(target=_reader, daemon=True).start()

            cancelled = False
            while True:
                if goal_handle.is_cancel_requested:
                    cancelled = True
                    try:
                        os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
                    except Exception:
                        pass
                    try:
                        Path(cancel_file).touch()
                    except Exception:
                        pass
                try:
                    line = line_q.get(timeout=0.2)
                except queue.Empty:
                    if proc.poll() is not None:
                        break
                    continue
                if line is None:
                    break
                fb = RunHardwareTest.Feedback()
                fb.line = line
                goal_handle.publish_feedback(fb)

            try:
                # Longer grace period than a plain SIGTERM would need — the
                # subprocess only checks the killswitch file between test
                # steps, and a step waiting on operator input can itself
                # take up to that step's own timeout to unblock.
                proc.wait(timeout=15)
            except subprocess.TimeoutExpired:
                # proc is normally root-owned (see cancel_file comment
                # above) — kill() would raise PermissionError in that case,
                # which we can't do anything about from here.
                try:
                    proc.kill()
                except Exception:
                    pass
            try:
                Path(cancel_file).unlink(missing_ok=True)
            except Exception:
                pass
            rc = proc.returncode

            if cancelled:
                goal_handle.canceled()
                result.success     = False
                result.return_code = rc if rc is not None else -1
                result.message     = 'Cancelled'
            elif rc == 0:
                goal_handle.succeed()
                result.success     = True
                result.return_code = 0
                result.message     = 'Completed'
            else:
                goal_handle.abort()
                result.success     = False
                result.return_code = rc if rc is not None else -1
                result.message     = f'Exited with code {rc}'

            self.get_logger().info(f'Test finished: {category}/{subtest} — {result.message}')
            return result


def main(args=None):
    rclpy.init(args=args)
    node = TestRunnerNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
