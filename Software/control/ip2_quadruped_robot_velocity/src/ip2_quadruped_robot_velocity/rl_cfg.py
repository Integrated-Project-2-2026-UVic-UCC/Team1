from mjlab.rl import (
    RslRlOnPolicyRunnerCfg,
    RslRlPpoAlgorithmCfg,
    RslRlPpoActorCriticCfg,  
)

def quad_bot_ppo_runner_cfg():
    return RslRlOnPolicyRunnerCfg(
        num_steps_per_env=24,
        max_iterations=3000,
        save_interval=50,
        experiment_name="quad_bot_velocity",
        empirical_normalization=True,
        policy={
            "class_name": "ActorCritic",
            "activation": "elu",
            "actor_hidden_dims": [512, 256, 128],
            "critic_hidden_dims": [512, 256, 128],
        },
        algorithm={
            "class_name": "PPO",
            "value_loss_coef": 1.0,
            "use_clipped_value_loss": True,
            "clip_param": 0.2,
            "entropy_coef": 0.01,
            "learning_rate": 1e-3,
            "num_learning_epochs": 5,
            "num_mini_batches": 4,
        },
    )
#Nico
def quad_bot2_ppo_runner_cfg():
    return RslRlOnPolicyRunnerCfg(
        policy=RslRlPpoActorCriticCfg(
            init_noise_std=1.0, 
            actor_hidden_dims=(512, 256, 128),
            critic_hidden_dims=(512, 256, 128),
            activation="elu",  # tipical function value
        ),
        algorithm=RslRlPpoAlgorithmCfg(
            entropy_coef=0.01,
        ),
        experiment_name="quadruped_robot_velocity",
        max_iterations=30000,
        save_interval=50, 
    )