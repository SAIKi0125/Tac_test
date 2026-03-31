import logging
import os

import hydra
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


def _make_sensor_cfg(sensor_cfg):
    sensor_cfg = OmegaConf.to_container(sensor_cfg, resolve=True)
    sensor_cfg["urdf_path"] = TAC3D_URDF
    return sensor_cfg


@hydra.main(config_path="../conf", config_name="tac3d_rolling")
def main(cfg):
    cfg = resolve_tacto_config_path(cfg)
    digits = tacto.Sensor(**cfg.tacto, background=None)

    log.info("Initializing world")
    px.init()
    p.setAdditionalSearchPath(os.path.join(PROJECT_ROOT, "assets", "sensor"))
    p.resetDebugVisualizerCamera(**cfg.pybullet_camera)

    tac3d_top = px.Body(**_make_sensor_cfg(cfg.digits.top))
    tac3d_bottom = px.Body(**_make_sensor_cfg(cfg.digits.bottom))

    digits.add_camera(tac3d_top.id, [-1])
    digits.add_camera(tac3d_bottom.id, [-1])

    obj_cfg = OmegaConf.to_container(cfg.object, resolve=True)
    obj = px.Body(**obj_cfg)
    digits.add_body(obj)

    panel = px.gui.PoseControlPanel(tac3d_top, **cfg.object_control_panel)
    panel.start()
    log.info("Use the slides to move the Tac3D rolling setup")

    t = px.utils.SimulationThread(real_time_factor=1.0)
    t.start()

    while True:
        (_, _, z), _ = obj.get_base_pose()
        if z <= cfg.reset_z_threshold:
            obj.reset()
            tac3d_top.reset()

        color, depth = digits.render()
        digits.updateGUI(color, depth)

    t.stop()


if __name__ == "__main__":
    main()
