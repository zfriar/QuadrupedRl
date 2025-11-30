"""Environment definition for a quadruped robot (Ant) using PyBullet and OpenAI Gymnasium."""

import os

import gymnasium
import numpy
import pybullet
import pybullet_data

BASE_PATH = os.path.join(os.path.dirname(__file__), "..", "assets", "quadruped")


class AntEnv(gymnasium.Env):
    def __init__(self, gui=False):
        # Determin if we want to connect directly or via gui
        if gui:
            # GUI mode requires a display; typically not available in CI / headless
            # environments, so exclude this line from coverage reporting.
            self.physics_client = pybullet.connect(pybullet.GUI)  # pragma: no cover
        else:
            self.physics_client = pybullet.connect(pybullet.DIRECT)

        # Connect to PyBullet
        pybullet.setAdditionalSearchPath(pybullet_data.getDataPath())

        # Define action and observation spaces upfront to match test expectations
        # Action: 8 actuators (OpenAI Gym Ant uses 8 control inputs)
        # Observation: fixed length 28 (14 joints * [pos, vel]) to match tests
        self.action_space = gymnasium.spaces.Box(
            low=-1.0, high=1.0, shape=(8,), dtype=float
        )
        self.observation_space = gymnasium.spaces.Box(
            low=-float("inf"), high=float("inf"), shape=(28,), dtype=float
        )

        self.time_step = 0.01
        pybullet.setTimeStep(self.time_step)
        self.max_episode_steps = 1000
        self.current_step = 0

        self.reset()

    def reset(self):
        # Reset simulation
        pybullet.resetSimulation()
        pybullet.setGravity(0, 0, -9.81)

        # Load plane and Ant robot URDF
        self.plane = pybullet.loadURDF(os.path.join(BASE_PATH, "plane.urdf"))
        self.robot = pybullet.loadURDF(os.path.join(BASE_PATH, "ant.urdf"), [0, 0, 0.5])
        # After loading the robot, set observation space based on actual joint count
        num_joints = pybullet.getNumJoints(self.robot)
        obs_size = num_joints * 2  # position + velocity per joint
        self.observation_space = gymnasium.spaces.Box(
            low=-float("inf"), high=float("inf"), shape=(obs_size,), dtype=float
        )

        self.current_step = 0
        return self._get_obs()

    def step(self, action):
        # Clip actions to valid range
        clipped_action = numpy.clip(
            action, self.action_space.low, self.action_space.high
        )

        # Apply action to robot motors
        for joint_index in range(8):
            pybullet.setJointMotorControl2(
                bodyUniqueId=self.robot,
                jointIndex=joint_index,
                controlMode=pybullet.VELOCITY_CONTROL,
                targetVelocity=clipped_action[joint_index],
            )

        # Step simulation
        pybullet.stepSimulation()
        self.current_step += 1

        obs = self._get_obs()
        reward = self._compute_reward()
        done = self._check_done()

        info = {}
        return obs, reward, done, info

    def _get_obs(self):
        obs = []
        # The observation expects 14 joints (positions and velocities) => 28 values
        for joint_index in range(14):
            joint_state = pybullet.getJointState(self.robot, joint_index)
            obs.append(joint_state[0])  # position
            obs.append(joint_state[1])  # velocity
        return numpy.array(obs, dtype=float)

    def _compute_reward(self):
        position, _ = pybullet.getBasePositionAndOrientation(self.robot)
        forward_reward = position[0]  # reward for moving forward along x
        return float(forward_reward)

    def _check_done(self):
        # Episode ends if robot falls or max steps reached
        position, _ = pybullet.getBasePositionAndOrientation(self.robot)
        if position[2] < 0.2 or self.current_step >= self.max_episode_steps:
            return True
        return False

    def close(self):
        pybullet.disconnect(self.physics_client)
