import pytest

import envs.ant_env


@pytest.fixture
def ant_environment():
    env = envs.ant_env.AntEnv(gui=False)
    yield env
    env.close()
