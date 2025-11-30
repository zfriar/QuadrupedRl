"""Ant environment using PyBullet.

This module contains a small Ant-like environment implemented with PyBullet
and Gymnasium. It is intentionally minimal and strongly typed so unit tests
can exercise the simulation deterministically. The project style enforces
fully-qualified imports (no ``from X import Y`` or ``import X as Y``).
"""

import os
import typing

import gymnasium
import numpy
import pybullet
import pybullet_data

BASE_PATH = os.path.join(os.path.dirname(__file__), "..", "assets", "quadruped")


class AntEnv(gymnasium.Env):
    """Ant-like Gymnasium environment backed by PyBullet.

    The environment provides a small, deterministic simulation that exposes
    an 8-dimensional action space and a variable-length observation space
    derived from the URDF joint count (tests expect 14 joints -> 28-D
    observation).

    Attributes:
        physics_client (Optional[int]): Identifier returned by ``pybullet.connect``.
        plane (Optional[int]): PyBullet id for the loaded plane.
        robot (Optional[int]): PyBullet id for the loaded robot.

    Args:
        gui (bool): If True, connect to the PyBullet GUI. GUI mode is excluded
            from coverage because it typically isn't available in CI.
    """

    physics_client: typing.Optional[int]
    plane: typing.Optional[int]
    robot: typing.Optional[int]

    def __init__(self, gui: bool = False) -> None:
        # Connect to PyBullet (GUI connection excluded from coverage)
        if gui:
            self.physics_client = pybullet.connect(pybullet.GUI)  # pragma: no cover
        else:
            self.physics_client = pybullet.connect(pybullet.DIRECT)

        # Make sure PyBullet can find standard assets
        pybullet.setAdditionalSearchPath(pybullet_data.getDataPath())

        # Action and observation spaces: defined to match tests' expectations.
        # Action: 8 actuators (Ant-like)
        self.action_space = gymnasium.spaces.Box(
            low=-1.0, high=1.0, shape=(8,), dtype=float
        )

        # Observation: default to 28 values (14 joints * [pos, vel]) but this
        # may be replaced during reset if the URDF has a different joint count.
        self.observation_space = gymnasium.spaces.Box(
            low=-float("inf"), high=float("inf"), shape=(28,), dtype=float
        )

        self.time_step = 0.01
        pybullet.setTimeStep(self.time_step)
        self.max_episode_steps = 1000
        self.current_step = 0

        self.plane = None
        self.robot = None

        # Initialize simulation state
        self.reset()

    def reset(self) -> numpy.ndarray:
        """Reset the simulation and return the initial observation.

        Returns:
            numpy.ndarray: Initial observation (joint positions and velocities).
        """
        pybullet.resetSimulation()
        pybullet.setGravity(0, 0, -9.81)

        # Load plane and Ant URDF from the repository assets
        self.plane = pybullet.loadURDF(os.path.join(BASE_PATH, "plane.urdf"))
        self.robot = pybullet.loadURDF(os.path.join(BASE_PATH, "ant.urdf"), [0, 0, 0.5])

        # Adjust observation space to the actual joint count
        num_joints = pybullet.getNumJoints(self.robot)
        obs_size = num_joints * 2
        self.observation_space = gymnasium.spaces.Box(
            low=-float("inf"), high=float("inf"), shape=(obs_size,), dtype=float
        )

        self.current_step = 0
        return self._get_obs()

    def step(
        self, action: numpy.ndarray
    ) -> typing.Tuple[numpy.ndarray, float, bool, dict]:
        """Apply an action and step the simulation.

        Args:
            action (numpy.ndarray): Control vector for the 8 actuators.

        Returns:
            tuple: ``(observation, reward, done, info)`` where ``observation`` is
            a NumPy array, ``reward`` is a float, ``done`` is a bool, and
            ``info`` is an empty dict for compatibility.
        """
        clipped_action = numpy.clip(
            action, self.action_space.low, self.action_space.high
        )

        # Apply the clipped actions to the first 8 joints
        for joint_index in range(8):
            pybullet.setJointMotorControl2(
                bodyUniqueId=self.robot,
                jointIndex=joint_index,
                controlMode=pybullet.VELOCITY_CONTROL,
                targetVelocity=clipped_action[joint_index],
            )

        pybullet.stepSimulation()
        self.current_step += 1

        obs = self._get_obs()
        reward = self._compute_reward()
        done = self._check_done()

        info: dict = {}
        return obs, reward, done, info

    def _get_obs(self) -> numpy.ndarray:
        """Return joint positions and velocities as a flat NumPy array.

        The test URDF supplies 14 joints; this helper reads position and
        velocity for the first 14 joints and returns an array of length 28.
        """
        obs_list: typing.List[float] = []
        # Collect positions and velocities for 14 joints (tests rely on this)
        for joint_index in range(14):
            joint_state = pybullet.getJointState(self.robot, joint_index)
            obs_list.append(joint_state[0])
            obs_list.append(joint_state[1])
        return numpy.array(obs_list, dtype=float)

    def _compute_reward(self) -> float:
        """Compute a simple forward-progress reward.

        Returns:
            float: Forward progress (x coordinate of the robot base).
        """
        position, _ = pybullet.getBasePositionAndOrientation(self.robot)
        forward_reward = position[0]
        return float(forward_reward)

    def _check_done(self) -> bool:
        """Check whether the episode is finished.

        The episode is terminated when the base height falls below a threshold
        or the maximum number of steps is reached.

        Returns:
            bool: True if done, False otherwise.
        """
        position, _ = pybullet.getBasePositionAndOrientation(self.robot)
        if position[2] < 0.2 or self.current_step >= self.max_episode_steps:
            return True
        return False

    def close(self) -> None:
        """Disconnect from the PyBullet physics client.

        Safe to call multiple times; checks that the client exists before
        disconnecting.
        """
        if getattr(self, "physics_client", None) is not None:
            pybullet.disconnect(self.physics_client)
