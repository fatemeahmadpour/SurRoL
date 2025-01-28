"""
SutureThreadManagement Environment
This environment simulates a bimanual surgical task where two PSM arms need to
cooperatively manage a suture thread. The goal is to maintain proper tension
while moving the thread to target positions.

a suture thread. The task involves maintaining proper tension while moving the 
thread endpoints to target positions.

Author: [Fatemeh Ahmadpour]
"""

import os
import numpy as np
import pybullet as p
from surrol.tasks.psm_env import PsmsEnv
from surrol.utils.pybullet_utils import get_link_pose
from surrol.const import ASSET_DIR_PATH

class SutureThreadManagement(PsmsEnv):
    ACTION_MODE = 'yaw'
    WORKSPACE_LIMITS1 = ((0.55, 0.6), (0.01, 0.08), (0.695, 0.745))
    WORKSPACE_LIMITS2 = ((0.55, 0.6), (-0.08, -0.01), (0.695, 0.745))
    SCALING = 5.
    DISTANCE_THRESHOLD = 0.005

    def __init__(self, render_mode=None):
        super().__init__(render_mode)
        self.thread_start = None
        self.thread_end = None
        self.thread_constraint = None
        self.has_object = True
        self._waypoint_goal = True
        
    def _env_setup(self):
        super()._env_setup()
        self._setup_arms()
        self._setup_thread()
        
    def _setup_arms(self):
        # Setup PSM1
        pos1 = np.mean(np.array(self.WORKSPACE_LIMITS1), axis=1) * self.SCALING
        pos1[2] += 0.05 * self.SCALING
        orn = p.getQuaternionFromEuler([0, -np.pi/2, -np.pi/2])
        joints1 = self.psm1.inverse_kinematics((pos1, orn), self.psm1.EEF_LINK_INDEX)
        self.psm1.reset_joint(joints1)

        # Setup PSM2  
        pos2 = np.mean(np.array(self.WORKSPACE_LIMITS2), axis=1) * self.SCALING
        pos2[2] += 0.05 * self.SCALING
        joints2 = self.psm2.inverse_kinematics((pos2, orn), self.psm2.EEF_LINK_INDEX)
        self.psm2.reset_joint(joints2)
        
    def _setup_thread(self):
        # Create thread endpoints
        pos1 = np.mean(np.array(self.WORKSPACE_LIMITS1), axis=1) * self.SCALING
        pos2 = np.mean(np.array(self.WORKSPACE_LIMITS2), axis=1) * self.SCALING
        
        # Create spheres for thread ends
        self.thread_start = p.loadURDF(
            os.path.join(ASSET_DIR_PATH, 'sphere/sphere.urdf'),
            pos1,
            globalScaling=0.01 * self.SCALING
        )
        self.thread_end = p.loadURDF(
            os.path.join(ASSET_DIR_PATH, 'sphere/sphere.urdf'),
            pos2,  
            globalScaling=0.01 * self.SCALING
        )
        
        # Set thread properties
        for thread_end in [self.thread_start, self.thread_end]:
            p.changeDynamics(
                thread_end,
                -1,
                mass=0.1 * self.SCALING,
                lateralFriction=0.5,
                spinningFriction=0.1,
                rollingFriction=0.1,
                restitution=0.1,
                linearDamping=0.1,
                angularDamping=0.1
            )
            
        # Create constraint between ends
        self.thread_constraint = p.createConstraint(
            self.thread_start,
            -1,
            self.thread_end, 
            -1,
            p.JOINT_POINT2POINT,
            [0, 0, 0],
            [0, 0, 0],
            [0, 0, 0]
        )
        p.changeConstraint(self.thread_constraint, maxForce=20 * self.SCALING)

    def _get_obs(self):
        psm1_state = self._get_robot_state(0)
        psm2_state = self._get_robot_state(1)
        
        thread_start_pos = np.array(p.getBasePositionAndOrientation(self.thread_start)[0])
        thread_end_pos = np.array(p.getBasePositionAndOrientation(self.thread_end)[0])
        
        thread_tension = np.linalg.norm(thread_start_pos - thread_end_pos)
        
        observation = np.concatenate([
            psm1_state,
            psm2_state, 
            thread_start_pos,
            thread_end_pos,
            [thread_tension]
        ])
        
        achieved_goal = np.concatenate([thread_start_pos, thread_end_pos])
        
        return {
            'observation': observation.copy(),
            'achieved_goal': achieved_goal.copy(), 
            'desired_goal': self.goal.copy()
        }

    def _sample_goal(self):
        limits1 = np.array(self.WORKSPACE_LIMITS1) * self.SCALING
        limits2 = np.array(self.WORKSPACE_LIMITS2) * self.SCALING
        
        goal_start = np.mean(limits1, axis=1) + np.random.uniform(-0.02, 0.02, 3) * self.SCALING
        goal_end = np.mean(limits2, axis=1) + np.random.uniform(-0.02, 0.02, 3) * self.SCALING
        
        return np.concatenate([goal_start, goal_end])

    def compute_reward(self, achieved_goal, desired_goal, info):
        start_pos = achieved_goal[:3]
        end_pos = achieved_goal[3:]
        goal_start = desired_goal[:3]
        goal_end = desired_goal[3:]

        start_dist = np.linalg.norm(start_pos - goal_start)
        end_dist = np.linalg.norm(end_pos - goal_end)
        
        current_tension = np.linalg.norm(start_pos - end_pos)
        target_tension = np.linalg.norm(goal_start - goal_end)
        tension_error = abs(current_tension - target_tension)
        
        return float(
            start_dist < self.DISTANCE_THRESHOLD and
            end_dist < self.DISTANCE_THRESHOLD and  
            tension_error < self.DISTANCE_THRESHOLD
        ) - 1

    def _is_success(self, achieved_goal, desired_goal):
        return self.compute_reward(achieved_goal, desired_goal, None) == 0