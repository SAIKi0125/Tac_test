import logging
import os

import hydra
import numpy as np
import pybullet as p
import pybulletX as px
import tacto
from omegaconf import OmegaConf

log = logging.getLogger(__name__)
PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
TAC3D_URDF = os.path.join(PROJECT_ROOT, "assets", "sensor", "Tac3D", "urdf", "Tac3D_DL1_URDF.urdf")




def resolve_project_path(path_str):
    if os.path.isabs(path_str):
        return path_str
    return os.path.join(PROJECT_ROOT, path_str)

def resolve_tacto_config_path(cfg):
    cfg.tacto.config_path = resolve_project_path(cfg.tacto.config_path)
    if "digit" in cfg and "urdf_path" in cfg.digit:
        cfg.digit.urdf_path = resolve_project_path(cfg.digit.urdf_path)
    if "object" in cfg and "urdf_path" in cfg.object:
        cfg.object.urdf_path = resolve_project_path(cfg.object.urdf_path)
    if "digits" in cfg:
        for key in cfg.digits:
            if "urdf_path" in cfg.digits[key]:
                cfg.digits[key].urdf_path = resolve_project_path(cfg.digits[key].urdf_path)
    return cfg


def _quat_to_matrix(quat):
    return np.array(p.getMatrixFromQuaternion(quat)).reshape(3, 3)


def add_gel_visual(sensor_cfg_path, body_cfg):
    if not os.path.isabs(sensor_cfg_path):
        sensor_cfg_path = os.path.join(PROJECT_ROOT, sensor_cfg_path)
    sensor_cfg = OmegaConf.load(sensor_cfg_path)
    gel = sensor_cfg.sensor.gel

    half_extents = [0.001, gel.width / 2.0, gel.height / 2.0]
    visual_shape = p.createVisualShape(
        shapeType=p.GEOM_BOX,
        halfExtents=half_extents,
        rgbaColor=[0.2, 0.8, 0.9, 0.35],
    )

    base_position = np.array(body_cfg["base_position"])
    base_orientation = body_cfg["base_orientation"]
    rotation = _quat_to_matrix(base_orientation)
    gel_center = base_position + rotation @ np.array(gel.origin)

    p.createMultiBody(
        baseMass=0,
        baseVisualShapeIndex=visual_shape,
        basePosition=gel_center.tolist(),
        baseOrientation=base_orientation,
    )


@hydra.main(config_path="../conf", config_name="tac3d_demo")
def main(cfg):
    cfg = resolve_tacto_config_path(cfg)
    # Initialize digits
    digits = tacto.Sensor(**cfg.tacto, background=None)

    # Initialize World
    log.info("Initializing world")
    px.init()
    p.setAdditionalSearchPath(os.path.join(PROJECT_ROOT, "assets", "sensor"))

    p.resetDebugVisualizerCamera(**cfg.pybullet_camera)

    # Create and initialize DIGIT
    digit_cfg = OmegaConf.to_container(cfg.digit, resolve=True)
    digit_cfg["urdf_path"] = TAC3D_URDF
    digit_body = px.Body(**digit_cfg)
    digits.add_camera(digit_body.id, [-1])
    add_gel_visual(cfg.tacto.config_path, digit_cfg)

    # Add object to pybullet and tacto simulator
    obj_cfg = OmegaConf.to_container(cfg.object, resolve=True)
    obj = px.Body(**obj_cfg)
    digits.add_body(obj)

    # Create control panel to control the 6DoF pose of the object
    panel = px.gui.PoseControlPanel(obj, **cfg.object_control_panel)
    panel.start()
    log.info("Use the slides to move the object until in contact with the Tac3D")

    # run p.stepSimulation in another thread
    t = px.utils.SimulationThread(real_time_factor=1.0)
    t.start()

    while True:
        color, depth = digits.render()
        digits.updateGUI(color, depth)

    t.stop()


if __name__ == "__main__":
    main()
