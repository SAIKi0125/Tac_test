import os
import time

import numpy as np
import pybullet as p
import pybullet_data
import tacto
from omegaconf import OmegaConf

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ROBOT_URDF = os.path.join(
    PROJECT_ROOT,
    "assets",
    "robot",
    "ur5_rg2_tac3d_description",
    "urdf",
    "ur5_rg2_tac3d.urdf",
)
SPHERE_URDF = os.path.join(PROJECT_ROOT, "assets", "object", "sphere_small.urdf")
TACTO_DEMO_CONFIG = os.path.join(PROJECT_ROOT, "conf", "tac3d_demo.yaml")

ARM_JOINT_NAMES = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]
ARM_HOME = [0.0, -1.20, 1.85, -2.20, -1.57, 0.0]
ACTIVE_GRIPPER_JOINTS = {
    "gripper_joint": 1.0,
    "l_finger_2_joint": -1.0,
    "l_finger_passive_joint": 1.0,
    "r_finger_1_joint": -1.0,
    "r_finger_2_joint": 1.0,
    "r_finger_passive_joint": -1.0,
}




def load_demo_cfg():
    cfg = OmegaConf.load(TACTO_DEMO_CONFIG)
    cfg = OmegaConf.to_container(cfg, resolve=True)
    config_path = cfg["tacto"]["config_path"]
    if not os.path.isabs(config_path):
        cfg["tacto"]["config_path"] = os.path.join(PROJECT_ROOT, config_path)
    return cfg

class SimpleBody:
    def __init__(self, urdf_path, body_id, global_scaling=1.0):
        self.urdf_path = urdf_path
        self.id = body_id
        self.global_scaling = global_scaling


def get_joint_map(body_id):
    joint_map = {}
    for joint_idx in range(p.getNumJoints(body_id)):
        info = p.getJointInfo(body_id, joint_idx)
        joint_map[info[1].decode("utf-8")] = joint_idx
    return joint_map


def find_link_id(body_id, link_name):
    for joint_idx in range(p.getNumJoints(body_id)):
        info = p.getJointInfo(body_id, joint_idx)
        child_link = info[12].decode("utf-8")
        if child_link == link_name:
            return joint_idx
    raise ValueError("Link not found: {}".format(link_name))


def print_joint_summary(body_id):
    print("Loaded joints:")
    for joint_idx in range(p.getNumJoints(body_id)):
        info = p.getJointInfo(body_id, joint_idx)
        joint_name = info[1].decode("utf-8")
        link_name = info[12].decode("utf-8")
        joint_type = info[2]
        print(
            "  idx={:2d} type={} joint={} child_link={}".format(
                joint_idx, joint_type, joint_name, link_name
            )
        )


def set_arm_targets(body_id, joint_map, targets, force=220.0):
    for joint_name, target in zip(ARM_JOINT_NAMES, targets):
        joint_idx = joint_map[joint_name]
        p.setJointMotorControl2(
            bodyUniqueId=body_id,
            jointIndex=joint_idx,
            controlMode=p.POSITION_CONTROL,
            targetPosition=target,
            force=force,
        )


def reset_arm_pose(body_id, joint_map, targets):
    for joint_name, target in zip(ARM_JOINT_NAMES, targets):
        joint_idx = joint_map[joint_name]
        p.resetJointState(body_id, joint_idx, target)
    set_arm_targets(body_id, joint_map, targets)


def set_gripper_opening(body_id, joint_map, opening, force=10.0):
    for joint_name, multiplier in ACTIVE_GRIPPER_JOINTS.items():
        joint_idx = joint_map[joint_name]
        p.setJointMotorControl2(
            bodyUniqueId=body_id,
            jointIndex=joint_idx,
            controlMode=p.POSITION_CONTROL,
            targetPosition=multiplier * opening,
            force=force,
        )


def get_midpoint_between_links(body_id, right_link_id, left_link_id):
    right_state = p.getLinkState(body_id, right_link_id, computeForwardKinematics=True)
    left_state = p.getLinkState(body_id, left_link_id, computeForwardKinematics=True)
    right_pos = np.array(right_state[0])
    left_pos = np.array(left_state[0])
    return 0.5 * (right_pos + left_pos)




def summarize_force(force_map):
    return float(sum(force_map.values())) if force_map else 0.0


def create_arm_sliders(default_targets):
    sliders = {}
    for joint_name, target in zip(ARM_JOINT_NAMES, default_targets):
        sliders[joint_name] = p.addUserDebugParameter(joint_name, -6.28, 6.28, target)
    return sliders


def read_arm_sliders(sliders):
    return [p.readUserDebugParameter(sliders[joint_name]) for joint_name in ARM_JOINT_NAMES]


def main():
    cfg = load_demo_cfg()

    client = p.connect(p.GUI)
    if client < 0:
        raise RuntimeError("Failed to connect to PyBullet GUI")

    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1)
    p.loadURDF("plane.urdf")

    tacto_kwargs = dict(cfg["tacto"])
    tacto_kwargs["background"] = None
    digits = tacto.Sensor(**tacto_kwargs)

    robot_id = p.loadURDF(
        ROBOT_URDF,
        basePosition=[0.0, 0.0, 0.0],
        baseOrientation=[0.0, 0.0, 0.0, 1.0],
        useFixedBase=True,
        flags=p.URDF_USE_INERTIA_FROM_FILE,
    )

    joint_map = get_joint_map(robot_id)
    print_joint_summary(robot_id)
    reset_arm_pose(robot_id, joint_map, ARM_HOME)
    initial_opening = 0.35
    set_gripper_opening(robot_id, joint_map, initial_opening)

    right_sensor_link = find_link_id(robot_id, "tac3d_dl1_sensor_right")
    left_sensor_link = find_link_id(robot_id, "tac3d_dl1_sensor_left")
    digits.add_camera(robot_id, [right_sensor_link, left_sensor_link])

    for _ in range(120):
        p.stepSimulation()

    sphere_position = get_midpoint_between_links(robot_id, right_sensor_link, left_sensor_link)
    sphere_position = sphere_position + np.array([0.0, 0.0, 0.01])
    sphere_scaling = 0.18
    sphere_id = p.loadURDF(
        SPHERE_URDF,
        basePosition=sphere_position.tolist(),
        baseOrientation=[0.0, 0.0, 0.0, 1.0],
        globalScaling=sphere_scaling,
        useFixedBase=True,
    )
    digits.add_body(SimpleBody(SPHERE_URDF, sphere_id, global_scaling=sphere_scaling))

    p.resetDebugVisualizerCamera(
        cameraDistance=0.12,
        cameraYaw=90.0,
        cameraPitch=-20.0,
        cameraTargetPosition=sphere_position.tolist(),
    )

    arm_sliders = create_arm_sliders(ARM_HOME)
    gripper_slider = p.addUserDebugParameter("gripper_opening", -0.45, 1.0, initial_opening)
    gripper_force_slider = p.addUserDebugParameter("gripper_force", 0.0, 50.0, 50.0)
    reset_button = p.addUserDebugParameter("reset_arm_pose", 1, 0, 0)

    last_reset_value = p.readUserDebugParameter(reset_button)
    step_count = 0

    print("PyBullet mouse controls remain available: left drag rotate, middle drag pan, wheel zoom.")
    print("The ball is fixed between the Tac3D fingers so you can focus on arm, gripper, and tactile response.")

    try:
        while p.isConnected():
            arm_targets = read_arm_sliders(arm_sliders)
            gripper_opening = p.readUserDebugParameter(gripper_slider)
            gripper_force = p.readUserDebugParameter(gripper_force_slider)
            reset_value = p.readUserDebugParameter(reset_button)

            if reset_value != last_reset_value:
                reset_arm_pose(robot_id, joint_map, ARM_HOME)
                set_gripper_opening(robot_id, joint_map, initial_opening)
                last_reset_value = reset_value

            set_arm_targets(robot_id, joint_map, arm_targets)
            set_gripper_opening(robot_id, joint_map, gripper_opening, force=gripper_force)

            p.stepSimulation()

            color, depth = digits.render()
            digits.updateGUI(color, depth)

            if step_count % 30 == 0:
                right_force = summarize_force(digits.get_force("cam0"))
                left_force = summarize_force(digits.get_force("cam1"))
                print("contact_force right={:.3f}N left={:.3f}N total={:.3f}N".format(
                    right_force, left_force, right_force + left_force
                ))

            step_count += 1
            time.sleep(1.0 / 240.0)
    finally:
        p.disconnect()


if __name__ == "__main__":
    main()
