import logging
import os
from pathlib import Path

import hydra
import pybullet as p
import pybulletX as px
import tacto

log = logging.getLogger(__name__)
PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
RG2_PKG_DIR = os.path.join(PROJECT_ROOT, "assets", "robot", "rg2_tac3d_description")
RG2_URDF = os.path.join(RG2_PKG_DIR, "urdf", "rg2_tac3d_dual.urdf")
SPHERE_URDF = os.path.join(PROJECT_ROOT, "assets", "object", "sphere_small.urdf")


def resolve_project_path(path_str):
    if os.path.isabs(path_str):
        return path_str
    return os.path.join(PROJECT_ROOT, path_str)


def resolve_tacto_config_path(cfg):
    cfg.tacto.config_path = resolve_project_path(cfg.tacto.config_path)
    if "object" in cfg and "urdf_path" in cfg.object:
        cfg.object.urdf_path = resolve_project_path(cfg.object.urdf_path)
    return cfg


def _prepare_rg2_urdf():
    text = Path(RG2_URDF).read_text()
    text = text.replace('package://rg2_tac3d_description', RG2_PKG_DIR)
    text = text.replace('../meshes/', os.path.join(RG2_PKG_DIR, 'meshes') + '/')
    tmp_path = os.path.join(RG2_PKG_DIR, 'urdf', '_rg2_tac3d_dual_pybullet_tmp.urdf')
    Path(tmp_path).write_text(text)
    return tmp_path




def _lock_all_joints(body_id):
    for joint_idx in range(p.getNumJoints(body_id)):
        info = p.getJointInfo(body_id, joint_idx)
        joint_type = info[2]
        if joint_type in (p.JOINT_REVOLUTE, p.JOINT_PRISMATIC):
            p.setJointMotorControl2(
                bodyUniqueId=body_id,
                jointIndex=joint_idx,
                controlMode=p.POSITION_CONTROL,
                targetPosition=0.0,
                force=200.0,
            )

def _find_link_id(body_id, link_name):
    for joint_idx in range(p.getNumJoints(body_id)):
        info = p.getJointInfo(body_id, joint_idx)
        child_link = info[12].decode('utf-8')
        if child_link == link_name:
            return joint_idx
    raise ValueError('Link not found: {}'.format(link_name))


@hydra.main(config_path='../conf', config_name='tac3d_demo')
def main(cfg):
    cfg = resolve_tacto_config_path(cfg)
    digits = tacto.Sensor(**cfg.tacto, background=None)

    log.info('Initializing world')
    px.init()
    p.resetDebugVisualizerCamera(**cfg.pybullet_camera)

    rg2_urdf = _prepare_rg2_urdf()
    rg2_id = p.loadURDF(rg2_urdf, [0, 0, 0], [0, 0, 0, 1], useFixedBase=True)
    _lock_all_joints(rg2_id)

    right_link = _find_link_id(rg2_id, 'tac3d_dl1_sensor_right')
    left_link = _find_link_id(rg2_id, 'tac3d_dl1_sensor_left')
    digits.add_camera(rg2_id, [right_link, left_link])

    obj = px.Body(
        urdf_path=SPHERE_URDF,
        base_position=[0.10, 0.0, 0.0],
        global_scaling=0.15,
    )
    digits.add_body(obj)

    panel = px.gui.PoseControlPanel(obj, **cfg.object_control_panel)
    panel.start()
    log.info('Move the object to test the Tac3D sensors mounted on the RG2 gripper')

    t = px.utils.SimulationThread(real_time_factor=1.0)
    t.start()

    while True:
        color, depth = digits.render()
        digits.updateGUI(color, depth)

    t.stop()


if __name__ == '__main__':
    main()
