#!/usr/bin/env python3
"""Exercise every RB-Y1 controller against a running driver (simulator core) and report numbers.

  python3 validate.py <test> [<test> ...]     tests: state jtc stream impedance gravity base cartesian
                                              cartesian_impedance impedance_pose optimal group switch stop realtime
Results are printed as one JSON line per test (prefix RESULT).
"""

import json
import math
import sys
import threading
import time

import rclpy
from control_msgs.action import FollowJointTrajectory
from controller_manager_msgs.srv import ListControllers, LoadController, SwitchController
from geometry_msgs.msg import PoseArray, PoseStamped, Twist, PointStamped
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener
from trajectory_msgs.msg import JointTrajectoryPoint

ARM = {s: [f"{s}_arm_{i}" for i in range(7)] for s in ("right", "left")}
TORSO = [f"torso_{i}" for i in range(6)]
HEAD = ["head_0", "head_1"]
BODY = TORSO + ARM["right"] + ARM["left"]


class Validator(Node):
    def __init__(self):
        super().__init__("rby1_sim_validator")
        self.lock = threading.Lock()
        self.samples = []  # (t, {name: (pos, vel, eff)}, stamp_age)
        self.latest = None
        self.create_subscription(JointState, "/joint_states", self.on_js, 50)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.switch_client = self.create_client(SwitchController, "/controller_manager/switch_controller")
        self.list_client = self.create_client(ListControllers, "/controller_manager/list_controllers")
        self.load_client = self.create_client(LoadController, "/controller_manager/load_controller")
        self.odom = None
        self.create_subscription(Odometry, "/base_velocity_controller/odom", lambda m: setattr(self, "odom", m), 10)
        self.robot_state = None
        try:
            from rby1_msgs.msg import RobotState

            self.create_subscription(RobotState, "/rby1/robot_state", lambda m: setattr(self, "robot_state", m), 10)
        except ImportError:
            pass

    # ------------------------------------------------------------------ state
    def on_js(self, msg):
        now = self.get_clock().now().nanoseconds
        stamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        d = {n: (p, v, e) for n, p, v, e in zip(msg.name, msg.position, msg.velocity, msg.effort)}
        with self.lock:
            self.latest = d
            self.samples.append((time.time(), d, (now - stamp) * 1e-9))
            if len(self.samples) > 20000:
                del self.samples[:10000]

    def q(self, names):
        with self.lock:
            return [self.latest[n][0] for n in names]

    def record(self):
        with self.lock:
            return len(self.samples)

    def since(self, index):
        with self.lock:
            return list(self.samples[index:])

    def wait_state(self, timeout=30.0):
        end = time.time() + timeout
        while time.time() < end:
            if self.latest is not None:
                return True
            time.sleep(0.1)
        return False

    # ------------------------------------------------------------ controllers
    def call(self, client, request, timeout=20.0):
        if not client.wait_for_service(timeout_sec=timeout):
            raise RuntimeError(f"service {client.srv_name} not available")
        future = client.call_async(request)
        end = time.time() + timeout
        while not future.done() and time.time() < end:
            time.sleep(0.01)
        if not future.done():
            raise RuntimeError(f"{client.srv_name} timed out")
        return future.result()

    def active_controllers(self):
        r = self.call(self.list_client, ListControllers.Request())
        return [c.name for c in r.controller if c.state == "active" and c.name != "joint_state_broadcaster"]

    def switch(self, activate=(), deactivate=None):
        active = self.active_controllers()
        if deactivate is None:
            deactivate = [c for c in active if c not in activate]
        activate = [c for c in activate if c not in active]
        if not activate and not deactivate:
            return True
        req = SwitchController.Request()
        req.activate_controllers = list(activate)
        req.deactivate_controllers = [c for c in deactivate if c not in activate]
        req.strictness = SwitchController.Request.STRICT
        ok = self.call(self.switch_client, req).ok
        time.sleep(0.3)
        return ok

    def load(self, name):
        req = LoadController.Request()
        req.name = name
        return self.call(self.load_client, req).ok

    def trajectory(self, controller, names, target, duration):
        client = ActionClient(self, FollowJointTrajectory, f"/{controller}/follow_joint_trajectory")
        if not client.wait_for_server(timeout_sec=10.0):
            raise RuntimeError(f"{controller} action not available")
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = names
        point = JointTrajectoryPoint()
        point.positions = list(target)
        point.time_from_start.sec = int(duration)
        point.time_from_start.nanosec = int((duration % 1) * 1e9)
        goal.trajectory.points = [point]
        send = client.send_goal_async(goal)
        end = time.time() + 10.0
        while not send.done() and time.time() < end:
            time.sleep(0.01)
        handle = send.result()
        if handle is None or not handle.accepted:
            return {"accepted": False}
        result = handle.get_result_async()
        end = time.time() + duration + 15.0
        while not result.done() and time.time() < end:
            time.sleep(0.01)
        r = result.result()
        return {"accepted": True, "status": r.status if r else None,
                "error_code": r.result.error_code if r else None}

    def tf(self, parent, child):
        end = time.time() + 5.0
        while time.time() < end:
            try:
                t = self.tf_buffer.lookup_transform(parent, child, rclpy.time.Time())
                return t.transform
            except Exception:
                time.sleep(0.05)
        raise RuntimeError(f"no tf {parent}->{child}")


def max_abs(xs):
    return max(abs(x) for x in xs) if xs else 0.0


def report(name, **values):
    print("RESULT " + json.dumps({"test": name, **values}, default=lambda x: round(x, 5)), flush=True)


def settle(v, names, seconds=1.0):
    """Peak speed of `names` over the next `seconds`."""
    i = v.record()
    time.sleep(seconds)
    s = v.since(i)
    return max((abs(d[n][1]) for _, d, _ in s for n in names), default=0.0)


# =============================================================================================== tests

def t_state(v):
    i = v.record()
    time.sleep(3.0)
    s = v.since(i)
    ages = sorted(a for _, _, a in s)
    rate = (len(s) - 1) / (s[-1][0] - s[0][0]) if len(s) > 1 else 0
    finite = all(math.isfinite(p) for _, d, _ in s for p, _, _ in d.values())
    report("state", rate_hz=rate, stamp_age_ms_median=1e3 * ages[len(ages) // 2],
           stamp_age_ms_p99=1e3 * ages[int(len(ages) * 0.99)], finite=finite,
           torso=v.q(TORSO), right_arm=v.q(ARM["right"]))


def t_jtc(v):
    results = {}
    for controller, names, delta in (
        ("right_arm_controller", ARM["right"], [0.2, -0.2, 0.1, -0.3, 0.1, 0.2, 0.1]),
        ("left_arm_controller", ARM["left"], [0.2, 0.2, -0.1, -0.3, -0.1, 0.2, -0.1]),
        ("torso_controller", TORSO, [0.0, 0.05, -0.1, 0.05, 0.0, 0.05]),
        ("head_controller", HEAD, [0.2, 0.1]),
    ):
        assert v.switch([controller]), f"cannot activate {controller}"
        q0 = v.q(names)
        target = [a + b for a, b in zip(q0, delta)]
        t0 = time.time()
        r = v.trajectory(controller, names, target, 2.0)
        elapsed = time.time() - t0
        time.sleep(0.5)
        err = [a - b for a, b in zip(v.q(names), target)]
        # back
        v.trajectory(controller, names, q0, 2.0)
        results[controller] = {**r, "elapsed_s": elapsed, "final_error_max_rad": max_abs(err)}
    v.switch([])
    report("jtc", **results)


def stream_test(v, controller, names, delta, name, publish_hz=30.0, seconds=3.0, stiffness_check=False):
    assert v.switch([controller]), f"cannot activate {controller}"
    pub = v.create_publisher(JointState, f"/{controller}/joint_commands", 1)
    time.sleep(0.5)
    q0 = v.q(names)
    target = [a + b for a, b in zip(q0, delta)]
    i = v.record()
    t_start = time.time()
    while time.time() - t_start < seconds:
        m = JointState()
        m.header.stamp = v.get_clock().now().to_msg()
        m.name = names
        m.position = target
        pub.publish(m)
        time.sleep(1.0 / publish_hz)
    s = v.since(i)
    err = [a - b for a, b in zip(v.q(names), target)]
    # rise time of the largest step joint: time to 90 %
    k = max(range(len(delta)), key=lambda j: abs(delta[j]))
    rise = None
    for t, d, _ in s:
        if abs(d[names[k]][0] - q0[k]) >= 0.9 * abs(delta[k]):
            rise = t - t_start
            break
    peak = max((abs(d[names[k]][0] - q0[k]) for _, d, _ in s), default=0.0)
    overshoot = max(0.0, (peak - abs(delta[k])) / abs(delta[k]))
    # return to start
    t_start = time.time()
    while time.time() - t_start < 2.0:
        m = JointState()
        m.name = names
        m.position = q0
        pub.publish(m)
        time.sleep(1.0 / publish_hz)
    v.destroy_publisher(pub)
    v.switch([])
    report(name, final_error_max_rad=max_abs(err), rise_90_s=rise, overshoot=overshoot,
           step_joint=names[k], step_rad=delta[k])


def t_stream(v):
    stream_test(v, "right_arm_position_stream_controller", ARM["right"], [0.0, 0.0, 0.0, -0.3, 0.0, 0.0, 0.0],
                "stream_position")


def t_impedance(v):
    stream_test(v, "right_arm_impedance_controller", ARM["right"], [0.0, 0.0, 0.0, -0.3, 0.0, 0.0, 0.0],
                "impedance_right_arm")
    stream_test(v, "torso_impedance_controller", TORSO, [0.0, 0.0, 0.0, 0.0, 0.0, 0.05], "impedance_torso")


def t_gravity(v):
    q0 = v.q(ARM["right"])
    assert v.switch(["right_arm_gravity_compensation_controller"])
    time.sleep(3.0)
    drift = [a - b for a, b in zip(v.q(ARM["right"]), q0)]
    torso_drift = 0.0
    v.switch([])
    report("gravity_compensation", drift_3s_max_rad=max_abs(drift), drift=drift, torso_drift=torso_drift)


def t_base(v):
    results = {}
    assert v.switch(["base_velocity_controller"])
    pub = v.create_publisher(Twist, "/base_velocity_controller/cmd_vel", 1)
    time.sleep(1.0)
    o0 = v.odom
    t0 = time.time()
    while time.time() - t0 < 3.0:
        m = Twist()
        m.linear.x = 0.1
        m.angular.z = 0.2
        pub.publish(m)
        time.sleep(0.05)
    time.sleep(1.0)
    o1 = v.odom
    if o0 and o1:
        dx = o1.pose.pose.position.x - o0.pose.pose.position.x
        dy = o1.pose.pose.position.y - o0.pose.pose.position.y
        results["se2_distance_m"] = math.hypot(dx, dy)
    results["se2_odom_received"] = o1 is not None
    v.destroy_publisher(pub)
    wheels = ["right_wheel", "left_wheel"]
    from std_msgs.msg import Float64MultiArray

    assert v.switch(["wheel_velocity_controller"])
    pub = v.create_publisher(Float64MultiArray, "/wheel_velocity_controller/commands", 1)
    w0 = v.q(wheels)
    t0 = time.time()
    while time.time() - t0 < 2.0:
        pub.publish(Float64MultiArray(data=[1.0, 1.0]))
        time.sleep(0.05)
    pub.publish(Float64MultiArray(data=[0.0, 0.0]))
    w1 = v.q(wheels)
    results["wheel_rotation_rad"] = [b - a for a, b in zip(w0, w1)]
    v.destroy_publisher(pub)
    v.switch([])
    report("base", **results)


def pose_of(tr):
    return [tr.translation.x, tr.translation.y, tr.translation.z], [tr.rotation.x, tr.rotation.y, tr.rotation.z, tr.rotation.w]


def task_test(v, controller, ref, link, offset, name, extra_targets=None, com=False):
    assert v.switch([controller]), f"cannot activate {controller}"
    p0, q0 = pose_of(v.tf(ref, link))
    pub = v.create_publisher(PoseArray if extra_targets else PoseStamped,
                             f"/{controller}/target_poses" if extra_targets else f"/{controller}/target_pose", 1)
    com_pub = v.create_publisher(PointStamped, f"/{controller}/com_target", 1) if com else None
    time.sleep(0.5)
    target = [a + b for a, b in zip(p0, offset)]
    others = []
    for r, l in extra_targets or []:
        others.append(pose_of(v.tf(r, l)))
    com0 = list(v.robot_state.center_of_mass) if (com and v.robot_state) else None
    t0 = time.time()
    while time.time() - t0 < 4.0:
        if extra_targets:
            m = PoseArray()
            m.header.frame_id = ""
            from geometry_msgs.msg import Pose

            for pos, ori in [(target, q0)] + others:
                p = Pose()
                p.position.x, p.position.y, p.position.z = pos
                p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = ori
                m.poses.append(p)
            pub.publish(m)
        else:
            m = PoseStamped()
            m.header.frame_id = ref
            m.pose.position.x, m.pose.position.y, m.pose.position.z = target
            m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z, m.pose.orientation.w = q0
            pub.publish(m)
        if com_pub and com0:
            c = PointStamped()
            c.point.x, c.point.y, c.point.z = com0
            com_pub.publish(c)
        time.sleep(0.03)
    p1, _ = pose_of(v.tf(ref, link))
    moved = [b - a for a, b in zip(p0, p1)]
    err = math.dist(p1, target)
    v.destroy_publisher(pub)
    v.switch([])
    report(name, moved=moved, commanded=offset, final_position_error_m=err)


def t_cartesian(v):
    task_test(v, "right_arm_cartesian_controller", "base", "ee_right", [0.0, 0.0, 0.05], "cartesian_right_arm")


def t_cartesian_impedance(v):
    task_test(v, "right_arm_cartesian_impedance_controller", "base", "ee_right", [0.0, 0.0, 0.05],
              "cartesian_impedance_right_arm")


def t_impedance_pose(v):
    task_test(v, "right_arm_impedance_pose_controller", "base", "ee_right", [0.0, 0.0, 0.05], "impedance_pose_right_arm")


def t_optimal(v):
    task_test(v, "body_optimal_control_controller", "base", "link_torso_5", [0.0, 0.0, 0.0], "optimal_control_body",
              extra_targets=[("base", "ee_right"), ("base", "ee_left")], com=True)


def t_group(v):
    # JTC over a subset of torso joints (rby1_bringup torso_group_controller) -> JointGroupPositionCommand
    assert v.switch(["torso_group_controller"]), "cannot activate torso_group_controller"
    names = ["torso_4", "torso_5"]
    q0 = v.q(names)
    target = [q0[0], q0[1] + 0.1]
    r = v.trajectory("torso_group_controller", names, target, 2.0)
    time.sleep(0.5)
    err = [a - b for a, b in zip(v.q(names), target)]
    v.trajectory("torso_group_controller", names, q0, 2.0)
    v.switch([])
    report("joint_group_position", **r, final_error_max_rad=max_abs(err))


WORK = {
    "right": [0.0, -0.32, 0.25, -1.87, -0.15, 1.30, -0.34],
    "left": [0.0, 0.32, -0.25, -1.87, 0.15, 1.30, 0.34],
}
TORSO_READY = [0.0, 0.1, -0.2, 0.1, 0.0, 0.0]


def t_ready(v):
    """Arms to the work pose and torso to the ready pose (away from singular configurations)."""
    out = {}
    for controller, names, target in (("torso_controller", TORSO, TORSO_READY),
                                      ("right_arm_controller", ARM["right"], WORK["right"]),
                                      ("left_arm_controller", ARM["left"], WORK["left"])):
        assert v.switch([controller]), f"cannot activate {controller}"
        out[controller] = v.trajectory(controller, names, target, 4.0)
    v.switch([])
    report("ready", **out)


def t_switch(v):
    """Joint jumps when switching modes on the right arm while it holds still."""
    results = {}
    pairs = [
        ("right_arm_controller", "right_arm_impedance_controller"),
        ("right_arm_impedance_controller", "right_arm_controller"),
        ("right_arm_controller", "right_arm_position_stream_controller"),
        ("right_arm_controller", "right_arm_gravity_compensation_controller"),
        ("right_arm_gravity_compensation_controller", "right_arm_controller"),
        ("right_arm_controller", None),
    ]
    for a, b in pairs:
        assert v.switch([a])
        time.sleep(1.0)
        q0 = v.q(ARM["right"])
        i = v.record()
        ok = v.switch([b] if b else [])
        time.sleep(1.5)
        s = v.since(i)
        jump = max((abs(d[n][0] - q0[k]) for _, d, _ in s for k, n in enumerate(ARM["right"])), default=0.0)
        speed = max((abs(d[n][1]) for _, d, _ in s for n in ARM["right"]), default=0.0)
        results[f"{a}->{b or 'hold'}"] = {"ok": ok, "max_deviation_rad": jump, "peak_speed_rad_s": speed}
    v.switch([])
    report("switch", **results)


def t_stop(v):
    """Manager stop while the impedance controller streams."""
    stop = v.create_client(Trigger, "/rby1/stop")
    assert v.switch(["right_arm_impedance_controller"])
    time.sleep(1.0)
    import subprocess

    q0 = v.q(ARM["right"])
    r = v.call(stop, Trigger.Request())
    time.sleep(2.0)
    hw = subprocess.run(["ros2", "control", "list_hardware_components"], capture_output=True, text=True).stdout
    state = [l.strip() for l in hw.splitlines() if "state" in l.lower()]
    # does the driver still command the arm? (a resumed stream would keep it on its target)
    active = v.active_controllers()
    v.switch([])
    report("stop", stop_success=r.success, message=r.message, active_after=active, hardware=state,
           arm_moved_after_stop=max_abs([a - b for a, b in zip(v.q(ARM["right"]), q0)]))


def t_realtime(v):
    stream_test(v, "body_realtime_controller", BODY,
                [0.0] * 6 + [0.0, 0.0, 0.0, -0.3, 0.0, 0.0, 0.0] + [0.0] * 7, "realtime_body", publish_hz=100.0)


TESTS = {
    "state": t_state, "jtc": t_jtc, "stream": t_stream, "impedance": t_impedance, "gravity": t_gravity,
    "base": t_base, "cartesian": t_cartesian, "cartesian_impedance": t_cartesian_impedance,
    "impedance_pose": t_impedance_pose, "optimal": t_optimal, "group": t_group, "switch": t_switch,
    "stop": t_stop, "realtime": t_realtime, "ready": t_ready,
}


def main():
    rclpy.init()
    v = Validator()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(v)
    threading.Thread(target=executor.spin, daemon=True).start()
    if not v.wait_state():
        print("RESULT " + json.dumps({"test": "setup", "error": "no /joint_states"}))
        return 1
    time.sleep(2.0)
    for name in sys.argv[1:]:
        try:
            TESTS[name](v)
        except Exception as e:  # report and continue with the next test
            report(name, error=str(e))
            try:
                v.switch([])
            except Exception:
                pass
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
