from gym.envs.registration import register

for reward_type in ['sparse']:
    suffix = 'Dense' if reward_type == 'dense' else ''
    kwargs = {
        'reward_type': reward_type,
        'other_args': { 
        }   
    }

register(
    id='HectorQuadReach-v1',
    entry_point='gym_hectorquad.envs:HectorQuadReachEnv',
    kwargs=kwargs,
    max_episode_steps=50,
)

register(
    id='HectorQuadPayload-v1',
    entry_point='gym_hectorquad.envs:HectorQuadPayloadEnv',
    kwargs=kwargs,
    max_episode_steps=100,
)

register(
    id='HectorQuadForest-v1',
    entry_point='gym_hectorquad.envs:HectorQuadForestEnv',
    kwargs=kwargs,
    max_episode_steps=100,
)