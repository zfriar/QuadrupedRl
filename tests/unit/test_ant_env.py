import unittest.mock

import numpy
import pybullet

import envs.ant_env


def test_env_reset_returns_correct_shape(ant_environment: envs.ant_env.AntEnv):
    """Test that the reset method returns observation and info with correct types."""
    obs, info = ant_environment.reset()
    assert isinstance(obs, numpy.ndarray)
    assert isinstance(info, dict)
    assert obs.shape[0] == ant_environment.observation_space.shape[0]


def test_env_step_returns_expected_tuple(ant_environment: envs.ant_env.AntEnv):
    """Test that the step method returns the expected tuple."""
    ant_environment.reset()
    action = ant_environment.action_space.sample()
    obs, reward, done, _, info = ant_environment.step(action)
    assert isinstance(obs, numpy.ndarray)
    assert isinstance(reward, float)
    assert isinstance(done, bool)
    assert isinstance(info, dict)


def test_env_step_clipping(ant_environment: envs.ant_env.AntEnv):
    """Test that actions are clipped to the action space bounds."""
    ant_environment.reset()
    action = numpy.array([10.0] * 8)  # intentionally out of bounds
    obs, reward, _, _, _ = ant_environment.step(action)
    assert isinstance(obs, numpy.ndarray)
    assert isinstance(reward, float)


def test_reward_forward_motion_increases(ant_environment: envs.ant_env.AntEnv):
    """Test that the reward increases with forward motion."""
    ant_environment.reset()
    initial_reward = ant_environment._compute_reward()
    ant_environment.step(ant_environment.action_space.sample())
    new_reward = ant_environment._compute_reward()
    assert isinstance(initial_reward, float)
    assert isinstance(new_reward, float)


def test_done_condition_falls(ant_environment: envs.ant_env.AntEnv):
    """Test that the done condition triggers when the robot falls below threshold."""
    ant_environment.reset()
    # manually lower the robot to trigger done
    pybullet.resetBasePositionAndOrientation(
        ant_environment.robot, [0, 0, 0.1], [0, 0, 0, 1]
    )
    assert ant_environment._check_done() is True


def test_done_condition_max_steps(ant_environment: envs.ant_env.AntEnv):
    """Test that the done condition triggers when max steps are reached."""
    ant_environment.reset()
    ant_environment.current_step = ant_environment.max_episode_steps
    assert ant_environment._check_done() is True


def test_close_disconnects_if_client_exists(ant_environment: envs.ant_env.AntEnv):
    """Test that close calls pybullet.disconnect when physics_client exists."""
    # Ensure physics_client is set
    assert ant_environment.physics_client is not None

    with unittest.mock.patch("pybullet.disconnect") as mock_disconnect:
        ant_environment.close()
        mock_disconnect.assert_called_once_with(ant_environment.physics_client)


def test_close_does_nothing_if_client_none():
    """Test that close does not call pybullet.disconnect when physics_client is None."""
    env = envs.ant_env.AntEnv(gui=False)
    env.physics_client = None  # simulate no client

    with unittest.mock.patch("pybullet.disconnect") as mock_disconnect:
        env.close()
        mock_disconnect.assert_not_called()
