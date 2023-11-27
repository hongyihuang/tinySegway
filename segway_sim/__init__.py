from gymnasium.envs.registration import register

register(
    id='segway_sim/InvertedPendulum-v0',
    entry_point='segway_sim.envs:InvertedPendulumEnv',
)