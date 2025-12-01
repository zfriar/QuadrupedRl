import pybullet
import pytest

import envs.ant_env


@pytest.fixture
def ant_environment():
    """Provide an AntEnv instance for testing."""
    # Ensure fresh connection context
    if pybullet.isConnected():
        pybullet.disconnect()

    env = envs.ant_env.AntEnv(gui=False)
    yield env
    env.close()
