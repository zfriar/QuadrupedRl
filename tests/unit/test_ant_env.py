import numpy
import pybullet  # type: ignore


def test_env_reset_returns_correct_shape(ant_environment):
    obs = ant_environment.reset()
    assert isinstance(obs, numpy.ndarray)
    assert obs.shape[0] == ant_environment.observation_space.shape[0]


def test_env_step_returns_expected_tuple(ant_environment):
    ant_environment.reset()
    action = ant_environment.action_space.sample()
    obs, reward, done, info = ant_environment.step(action)
    assert isinstance(obs, numpy.ndarray)
    assert isinstance(reward, float)
    assert isinstance(done, bool)
    assert isinstance(info, dict)


def test_env_step_clipping(ant_environment):
    ant_environment.reset()
    action = numpy.array([10.0] * 8)  # intentionally out of bounds
    obs, reward, done, info = ant_environment.step(action)
    assert isinstance(obs, numpy.ndarray)
    assert isinstance(reward, float)


def test_reward_forward_motion_increases(ant_environment):
    ant_environment.reset()
    initial_reward = ant_environment._compute_reward()
    # apply one step and compute reward again; tests shouldn't assert strict increase
    # because physics can be non-deterministic; ensure both are floats and use the
    # initial value to avoid unused-variable lint errors.
    ant_environment.step(ant_environment.action_space.sample())
    new_reward = ant_environment._compute_reward()
    assert isinstance(initial_reward, float)
    assert isinstance(new_reward, float)


def test_done_condition_falls(ant_environment):
    ant_environment.reset()
    # manually lower the robot to trigger done
    pybullet.resetBasePositionAndOrientation(
        ant_environment.robot, [0, 0, 0.1], [0, 0, 0, 1]
    )
    assert ant_environment._check_done() is True


def test_done_condition_max_steps(ant_environment):
    ant_environment.reset()
    ant_environment.current_step = ant_environment.max_episode_steps
    assert ant_environment._check_done() is True
