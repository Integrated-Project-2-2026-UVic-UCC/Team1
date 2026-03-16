from pathlib import Path

import mujoco
from mjlab.actuator import BuiltinPositionActuatorCfg, DcMotorActuatorCfg, DelayedActuatorCfg
from mjlab.entity import EntityArticulationInfoCfg, EntityCfg
from mjlab.utils.actuator import ElectricActuator, reflected_inertia
from mjlab.utils.os import update_assets
from mjlab.utils.spec_config import CollisionCfg

QUAD_XML: Path = Path(__file__).parent / "xmls" / "quad_bot.xml"
assert QUAD_XML.exists()

def get_spec() -> mujoco.MjSpec:
    return mujoco.MjSpec.from_file(str(QUAD_XML))
#NIco: changed to dcMOtor (parametres pendents de tunejar)
STALL_TORQUE_NM = 2.45
VELOCITY_LIMIT = 12.0 #10.0
STIFFNESS = 150.0#12.0
DAMPING = 5.0#0.5

base_motor_cfg= DcMotorActuatorCfg(
    target_names_expr=(
        ".*_hip_roll",
        ".*_hip_pitch",
        ".*_knee",
    ),
    stiffness=STIFFNESS,
    damping=DAMPING,
    saturation_effort=STALL_TORQUE_NM,
    effort_limit=STALL_TORQUE_NM,
    velocity_limit=VELOCITY_LIMIT,
    armature=0.01,
    frictionloss=0.5#0.02
)

QUAD_BOT_ACTUATOR_CFG = DelayedActuatorCfg( #NICO: afegir delay per tenir en compte a la simulació (latencia)
    base_cfg=base_motor_cfg,
    delay_min_lag=1, #unos 70ms
    delay_max_lag=14, #timestep 0.005s, por 14 son 70ms
    delay_target="position"  # El delay se aplica a la orden de posición enviada al servo [6]
)

INIT_STATE = EntityCfg.InitialStateCfg(
    pos=(0.0, 0.0, 0.3),
    joint_pos={
        ".*_hip_roll": 0.0,
        ".*_hip_pitch": 0.7,
        ".*_knee": -1.4,
       },
)

# NICO:collision config
_foot_regex = r".*_foot_collision$"
FULL_COLLISION = CollisionCfg(
    geom_names_expr=(r".*_collision$", _foot_regex),
    condim=3,
    friction=(0.8,),
    solimp={_foot_regex: (0.005, 1.0, 0.015)}, # suaviza el impacto del pie
)

QUAD_BOT_ARTICULATION_INFO_CFG = EntityArticulationInfoCfg(
    actuators=(QUAD_BOT_ACTUATOR_CFG,),
    soft_joint_pos_limit_factor=0.9,  # Ns: para evitar que el robot intente ir a los limites de las articulaciones
)

QUAD_BOT_ACTION_SCALE: dict[str, float] = {    
    ".*_hip_roll": 0.5,
    ".*_hip_pitch": 0.8,
    ".*_knee": 0.8,
    }

def get_quad_bot_cfg() -> EntityCfg:
    return EntityCfg(
        spec_fn=get_spec,
        init_state=INIT_STATE,
        collisions=(FULL_COLLISION,),  # tupla
        articulation=QUAD_BOT_ARTICULATION_INFO_CFG,
    )

if __name__ == "__main__":
    import mujoco.viewer as viewer
    from mjlab.entity.entity import Entity

    robot = Entity(get_quad_bot_cfg())
    viewer.launch(robot.spec.compile())