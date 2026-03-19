from mjlab.envs import ManagerBasedRlEnvCfg
from mjlab.envs.mdp.actions import JointPositionActionCfg
from mjlab.managers.termination_manager import TerminationTermCfg
from mjlab.sensor import ContactMatch, ContactSensorCfg, RayCastSensorCfg
from mjlab.tasks.velocity import mdp
from mjlab.tasks.velocity.mdp import UniformVelocityCommandCfg
from mjlab.managers.observation_manager import ObservationGroupCfg, ObservationTermCfg
from mjlab.tasks.velocity.velocity_env_cfg import make_velocity_env_cfg, Unoise, SceneEntityCfg, RewardTermCfg
import math

from ip2_quadruped_robot_velocity.quad_bot.quad_bot_constants import (
    QUAD_BOT_ACTION_SCALE,
    get_quad_bot_cfg,
)
def quad_bot_flat_env_cfg(play: bool = False) -> ManagerBasedRlEnvCfg:
    """Configuración limpia para evitar el error de mismatch de tensores."""
    cfg = make_velocity_env_cfg()
    cfg.sim.mujoco.ccd_iterations = 500
    cfg.sim.contact_sensor_maxmatch = 500
    cfg.sim.nconmax = 1024 # Subido para evitar errores
    # 1. Asignar el robot
    cfg.scene.entities = {"robot": get_quad_bot_cfg()}

    # 2. Definir nombres (Tus nombres del XML)
    site_names = ("FL", "FR", "RL", "RR")
    geom_names = ("FL_foot_collision", "FR_foot_collision", "RL_foot_collision", "RR_foot_collision")

    # 3. Sensores (Formato ANYmal C)
    cfg.scene.sensors = (
        ContactSensorCfg(
            name="feet_ground_contact",
            primary=ContactMatch(mode="geom", pattern=geom_names, entity="robot"),
            secondary=ContactMatch(mode="body", pattern="terrain"),
            fields=("found", "force"),
            reduce="netforce",
            num_slots=1,
            track_air_time=True,
        ),
        ContactSensorCfg(
            name="nonfoot_ground_touch",
            primary=ContactMatch(
                mode="geom", 
                entity="robot",
                pattern=r".*_collision\d*$",
                exclude=tuple(geom_names),
            ),
            secondary=ContactMatch(mode="body", pattern="terrain"),
            fields=("found",),
            reduce="none",
            num_slots=1,
        ),
    )

    #####################
    ###  Observations ###
    #####################

    policy_terms = {
        "base_ang_vel": ObservationTermCfg(
        func=mdp.builtin_sensor,
        params={"sensor_name": "robot/imu_ang_vel"},
        noise=Unoise(n_min=-0.2, n_max=0.2),
        ),
        "projected_gravity": ObservationTermCfg(
        func=mdp.projected_gravity,
        noise=Unoise(n_min=-0.05, n_max=0.05),
        ),
        "joint_pos": ObservationTermCfg(
        func=mdp.joint_pos_rel,
        noise=Unoise(n_min=-0.01, n_max=0.01),
        ),
        "joint_vel": ObservationTermCfg( # TODO pendiente de validacion
        func=mdp.joint_vel_rel,
        noise=Unoise(n_min=-1.5, n_max=1.5),
        ),
        "actions": ObservationTermCfg(func=mdp.last_action),
        "command": ObservationTermCfg(
        func=mdp.generated_commands,
        params={"command_name": "twist"},
        ),
    }

    critic_terms = {
        **policy_terms,
        "foot_height": ObservationTermCfg(
        func=mdp.foot_height,
        params={"asset_cfg": SceneEntityCfg("robot", site_names=site_names),},  # Set per-robot.
        ),
        "foot_air_time": ObservationTermCfg(
        func=mdp.foot_air_time,
        params={"sensor_name": "feet_ground_contact"},
        ),
        "foot_contact": ObservationTermCfg(
        func=mdp.foot_contact,
        params={"sensor_name": "feet_ground_contact"},
        ),
        "foot_contact_forces": ObservationTermCfg(
        func=mdp.foot_contact_forces,
        params={"sensor_name": "feet_ground_contact"},
        ),
    }

    cfg.observations = {    # para usar la default, quitar el cfg.
        "policy": ObservationGroupCfg(
        terms=policy_terms,
        concatenate_terms=True,
        enable_corruption=True,
        ),
        "critic": ObservationGroupCfg(
        terms=critic_terms,
        concatenate_terms=True,
        enable_corruption=False,
        ),
    }


    cfg.events["foot_friction"].params[
        "asset_cfg"
    ].geom_names = geom_names  # que partes necesitan friccion, los pies
    cfg.events["base_com"].params["asset_cfg"].body_names = ("trunk",)

    # # 4. LIMPIEZA AGRESIVA DE RECOMPENSAS (Rewards)
    # # Eliminamos las que causan el error de tensores (12 vs 0) #solucionat afegint std rewards
    # # y las que buscan nombres de sitios o joints del Go1
    # problematic_rewards = ["pose", "foot_clearance", "foot_swing_height", "foot_slip", "joint_pos", "air_time"]
    # for r in problematic_rewards:
    #     if r in cfg.rewards:
    #         cfg.rewards.pop(r)

    #TODO: experimentar amb rewards
    cfg.rewards = {
    "track_linear_velocity": RewardTermCfg(
      func=mdp.track_linear_velocity,
      weight=5.0,
      params={"command_name": "twist", "std": math.sqrt(0.25)},
    ),
    "track_angular_velocity": RewardTermCfg( #do not spin for the moment
      func=mdp.track_angular_velocity,
      weight=0.0,
      params={"command_name": "twist", "std": math.sqrt(0.5)},
    ),
    "upright": RewardTermCfg(
      func=mdp.flat_orientation,
      weight=0.5,
      params={
        "std": math.sqrt(0.2),
        "asset_cfg": SceneEntityCfg("robot", body_names=("trunk",)),  # Set per-robot.
      },
    ),
    "pose": RewardTermCfg(
      func=mdp.variable_posture,
      weight=0.0,
      params={
        "asset_cfg": SceneEntityCfg("robot", joint_names=(".*",)),
        "command_name": "twist",
        "std_standing": { ".*_hip_roll": 0.05, ".*_hip_pitch": 0.05, ".*_knee": 0.1, },  # Set per-robot.
        "std_walking": { ".*_hip_roll": 0.3, ".*_hip_pitch": 0.3, ".*_knee": 0.6, },  # Set per-robot.
        "std_running": { ".*_hip_roll": 0.3, ".*_hip_pitch": 0.3, ".*_knee": 0.6, }, # Set per-robot.
        "walking_threshold": 0.05,
        "running_threshold": 1.5,
      },
    ),
    "body_ang_vel": RewardTermCfg(
      func=mdp.body_angular_velocity_penalty,
      weight=0.0,  # Override per-robot
      params={"asset_cfg": SceneEntityCfg("robot", body_names=("trunk",))},  # Set per-robot.
    ),
    # "angular_momentum": RewardTermCfg(
    #   func=mdp.angular_momentum_penalty,
    #   weight=0.0,  # Override per-robot
    #   params={"sensor_name": "robot/root_angmom"},
    # ),
    "dof_pos_limits": RewardTermCfg(func=mdp.joint_pos_limits, weight=0.0), #was negative
    "action_rate_l2": RewardTermCfg(func=mdp.action_rate_l2, weight=-0.1),
    "air_time": RewardTermCfg(
      func=mdp.feet_air_time,
      weight=1.0,  # Override per-robot. deafult 0.0
      params={
        "sensor_name": "feet_ground_contact",
        "threshold_min": 0.05,
        "threshold_max": 0.5,
        "command_name": "twist",
        "command_threshold": 0.5,
      },
    ),
    # "foot_clearance": RewardTermCfg(
    #   func=mdp.feet_clearance,
    #   weight=0.0,
    #   params={
    #     "target_height": 0.1,
    #     "command_name": "twist",
    #     "command_threshold": 0.05,
    #     "asset_cfg": SceneEntityCfg("robot", site_names=site_names),  # Set per-robot.
    #   },
    # ),
    # "foot_swing_height": RewardTermCfg(
    #   func=mdp.feet_swing_height,
    #   weight=0.0, #was negative
    #   params={
    #     "sensor_name": "feet_ground_contact",
    #     "target_height": 0.1,
    #     "command_name": "twist",
    #     "command_threshold": 0.05,
    #     "asset_cfg": SceneEntityCfg("robot", site_names=site_names),  # Set per-robot.
    #   },
    # ),
    "foot_slip": RewardTermCfg(
      func=mdp.feet_slip,
      weight=0.0, #was negative
      params={
        "sensor_name": "feet_ground_contact",
        "command_name": "twist",
        "command_threshold": 0.05,
        "asset_cfg": SceneEntityCfg("robot", site_names=site_names),  # Set per-robot.
      },
    ),
    "soft_landing": RewardTermCfg(
      func=mdp.soft_landing,
      weight=-1e-5,
      params={
        "sensor_name": "feet_ground_contact",
        "command_name": "twist",
        "command_threshold": 0.05,
      },
    ),
  }

    # 5. ACCIONES (Mapeo de motores)
    joint_pos_action = cfg.actions["joint_pos"]
    assert isinstance(
        joint_pos_action, JointPositionActionCfg
    )  # comprovar si la accion es posicion
    joint_pos_action.scale = QUAD_BOT_ACTION_SCALE

    # 6. TERMINACIONES (Muerte del robot)
    cmd = cfg.commands["twist"]
    assert isinstance(cmd, UniformVelocityCommandCfg)  # envia un twist para entrenar
    cmd.viz.z_offset = 0.5  # lo ves a 0,5 m del robot
    cmd.ranges.lin_vel_x = (0.0, 1.0)
    cmd.ranges.lin_vel_y = (0.0, 1.0)
    cmd.ranges.ang_vel_z = (0.0, 0.0)
    cfg.terminations["nonfoot_contact"] = TerminationTermCfg(
        func=mdp.illegal_contact,
        params={"sensor_name": "nonfoot_ground_touch"},
        time_out=False,
    )

    # 7. AJUSTES DE ESCENA
    cfg.viewer.body_name = "trunk"
    cfg.viewer.distance = 2.0
    cfg.viewer.elevation = -10.0
    cfg.scene.entities["robot"].init_state.pos = (0.0, 0.0, 0.17)

    # flat terrain
    if cfg.scene.terrain is not None:
        cfg.scene.terrain.terrain_type = "plane"
        cfg.scene.terrain.terrain_generator = None

    cfg.scene.sensors = tuple(
        s for s in (cfg.scene.sensors or ()) if s.name != "terrain_scan"
    )
    cfg.observations["policy"].terms.pop("height_scan", None)
    cfg.observations["critic"].terms.pop("height_scan", None)

    if hasattr(cfg, "curriculum"):
         cfg.curriculum.pop("terrain_levels", None) #no curriculim

    if play:
        cfg.episode_length_s = int(1e9)  # en modo play no termina

        cfg.observations["policy"].enable_corruption = False  # quitamos ruido sensores
        cfg.events.pop("push_robot", None)  # para evitar empujones

    return cfg