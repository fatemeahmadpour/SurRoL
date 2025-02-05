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
    """
    Environment for simulating suture thread management using two PSM arms.
    
    State Space:
        - PSM1 and PSM2 states (position, orientation, gripper)
        - Thread endpoints positions 
        - Thread tension
        
    Action Space:
        - PSM1: [dx, dy, dz, dyaw, gripper]
        - PSM2: [dx, dy, dz, dyaw, gripper]
    """
    
    # Environment configuration
    ACTION_MODE = 'yaw'  # Using yaw rotation for better thread control
    WORKSPACE_LIMITS1 = ((0.55, 0.6), (0.01, 0.08), (0.695, 0.745))  # PSM1 workspace
    WORKSPACE_LIMITS2 = ((0.55, 0.6), (-0.08, -0.01), (0.695, 0.745))  # PSM2 workspace
    SCALING = 5.  # Scaling factor for better physics simulation
    DISTANCE_THRESHOLD = 0.005  # Success threshold for positions and tension
    
    def __init__(self, render_mode=None):
        """Initialize environment components"""
        super(SutureThreadManagement, self).__init__(render_mode)
        self.thread_start = None
        self.thread_end = None
        self.thread_constraint = None
        self.has_object = True
        self._waypoint_goal = True
        self.goal = None

    def _array_to_position(self, arr):
        """Convert numpy array to 3D position list for PyBullet"""
        pos = list(arr)
        if len(pos) != 3:
            raise ValueError("Position must have 3 coordinates [x,y,z]")
        return pos

    def _setup_arms(self):
        """Setup initial robot arm positions and orientations"""
        try:
            # Convert workspace limits to numpy arrays and scale them
            limits1 = np.asarray(self.WORKSPACE_LIMITS1) * self.SCALING
            limits2 = np.asarray(self.WORKSPACE_LIMITS2) * self.SCALING
            
            # Calculate mean positions with safety margins
            pos1 = limits1.mean(axis=1)
            pos2 = limits2.mean(axis=1)
            
            # Add safety height offset
            pos1[2] += 0.05 * self.SCALING  # Add 5cm height offset
            pos2[2] += 0.05 * self.SCALING
            
            # Convert to list format for PyBullet
            pos1 = list(pos1)
            pos2 = list(pos2)
            
            # Use fixed orientation that's known to work
            orn = p.getQuaternionFromEuler([0, -np.pi/2, -np.pi/2])
            
            # Set initial pose for PSM1
            try:
                joint_positions = self.psm1.inverse_kinematics(
                    (pos1, orn),
                    self.psm1.EEF_LINK_INDEX
                )
                # Clip joint positions to limits
                if self.psm1.limits is not None:
                    joint_positions = np.clip(
                        joint_positions,
                        self.psm1.limits['lower'][:self.psm1.DoF],
                        self.psm1.limits['upper'][:self.psm1.DoF]
                    )
                self.psm1.reset_joint(joint_positions)
            except Exception as e:
                print(f"Error setting up PSM1: {e}")
                
            # Set initial pose for PSM2
            try:
                joint_positions = self.psm2.inverse_kinematics(
                    (pos2, orn),
                    self.psm2.EEF_LINK_INDEX
                )
                # Clip joint positions to limits
                if self.psm2.limits is not None:
                    joint_positions = np.clip(
                        joint_positions,
                        self.psm2.limits['lower'][:self.psm2.DoF],
                        self.psm2.limits['upper'][:self.psm2.DoF]
                    )
                self.psm2.reset_joint(joint_positions)
            except Exception as e:
                print(f"Error setting up PSM2: {e}")

        except Exception as e:
            print(f"Error in setup_arms: {e}")

    def _setup_simple_thread(self):
        """Create a simple thread with two endpoints"""
        try:
            # Convert workspace limits to numpy arrays and scale them
            limits1 = np.asarray(self.WORKSPACE_LIMITS1) * self.SCALING
            limits2 = np.asarray(self.WORKSPACE_LIMITS2) * self.SCALING
            
            # Calculate positions with offset
            start_center = limits1.mean(axis=1)
            end_center = limits2.mean(axis=1)
            
            # Add height offset and convert to list
            start_pos = list(limits1.mean(axis=1) + np.array([0, 0, 0.02 * self.SCALING]))
            end_pos = list(limits2.mean(axis=1) + np.array([0, 0, 0.02 * self.SCALING]))
            
            # Verify position format
            if len(start_pos) != 3 or len(end_pos) != 3:
                raise ValueError("Invalid position format")
                
            # Create thread endpoints
            self.thread_start = self._create_thread_end(start_pos, color=(0.8, 0.8, 0.8, 1))
            self.thread_end = self._create_thread_end(end_pos, color=(0.8, 0.8, 0.8, 1))
            
            # Connect thread ends only if both ends were created successfully
            if self.thread_start is not None and self.thread_end is not None:
                self._connect_thread_ends()
            else:
                print("Failed to create thread ends")
                
        except Exception as e:
            print(f"Error in setup_simple_thread: {e}")

    def _connect_thread_ends(self):
        """Connect thread ends with constraints and set dynamic parameters"""
        # Create point-to-point constraint between thread ends
        self.thread_constraint = p.createConstraint(
            parentBodyUniqueId=self.thread_start,
            parentLinkIndex=-1,
            childBodyUniqueId=self.thread_end,
            childLinkIndex=-1,
            jointType=p.JOINT_POINT2POINT,
            jointAxis=[0, 0, 0],
            parentFramePosition=[0, 0, 0],
            childFramePosition=[0, 0, 0]
        )

        # Set constraint force for thread tension simulation
        p.changeConstraint(
            self.thread_constraint,
            maxForce=20.0 * self.SCALING
        )

        # Set dynamic parameters for realistic thread behavior
        dynamics_params = {
            'linearDamping': 0.1,
            'angularDamping': 0.1,
            'restitution': 0.1,
            'lateralFriction': 0.5,
            'spinningFriction': 0.1,
            'rollingFriction': 0.1
        }

        # Apply dynamics to both thread ends
        for thread_end in [self.thread_start, self.thread_end]:
            p.changeDynamics(
                thread_end,
                -1,
                mass=0.1 * self.SCALING,
                **dynamics_params
            )

    def _env_setup(self):
        """Initialize environment"""
        super(SutureThreadManagement, self)._env_setup()
        self.has_object = True
        self._waypoint_goal = True
        
        # Create spheres for visualization
        obj_id = p.loadURDF(os.path.join(ASSET_DIR_PATH, 'sphere/sphere.urdf'),
                            globalScaling=self.SCALING)
        self.obj_ids['fixed'].append(obj_id)  # For start position
        
        obj_id = p.loadURDF(os.path.join(ASSET_DIR_PATH, 'sphere/sphere.urdf'),
                            globalScaling=self.SCALING)
        self.obj_ids['fixed'].append(obj_id)  # For end position
        
        self._setup_arms()
        self._setup_simple_thread()

    def _sample_goal_callback(self):
        """Visualize goal positions"""
        if self.goal is not None:
            start_pos = self.goal[:3]
            end_pos = self.goal[3:]
            
            # Visualize start position
            p.resetBasePositionAndOrientation(
                self.obj_ids['fixed'][0],
                start_pos, 
                (0, 0, 0, 1)
            )
            
            # Visualize end position
            p.resetBasePositionAndOrientation(
                self.obj_ids['fixed'][1],
                end_pos,
                (0, 0, 0, 1) 
            )

    def get_oracle_action(self, obs) -> np.ndarray:
        """Expert strategy for thread management"""
        if isinstance(obs, dict):
            current_start = obs['achieved_goal'][:3]
            current_end = obs['achieved_goal'][3:]
            target_start = obs['desired_goal'][:3] 
            target_end = obs['desired_goal'][3:]
            
            # Calculate position errors
            delta_pos1 = (target_start - current_start) / 0.01 / self.SCALING
            delta_pos2 = (target_end - current_end) / 0.01 / self.SCALING
            
            # Scale actions
            if np.abs(delta_pos1).max() > 1:
                delta_pos1 /= np.abs(delta_pos1).max()
            if np.abs(delta_pos2).max() > 1:
                delta_pos2 /= np.abs(delta_pos2).max()
                
            delta_pos1 *= 0.3
            delta_pos2 *= 0.3
            
            # Combine actions for both arms
            action = np.array([
                delta_pos1[0], delta_pos1[1], delta_pos1[2], 0, -0.5,  # PSM1
                delta_pos2[0], delta_pos2[1], delta_pos2[2], 0, -0.5   # PSM2
            ])
            
            return action
            
        return np.zeros(10)  # Default action

    def _get_obs(self):
        """Get current environment state"""
        # Get robot states
        psm1_state = self._get_robot_state(0)
        psm2_state = self._get_robot_state(1)
        
        # Get thread endpoint positions
        thread_start_pos = np.array(p.getBasePositionAndOrientation(self.thread_start)[0])
        thread_end_pos = np.array(p.getBasePositionAndOrientation(self.thread_end)[0])
        
        # Calculate thread tension
        thread_tension = np.linalg.norm(thread_start_pos - thread_end_pos)
        
        # Combine all observations
        observation = np.concatenate([
            psm1_state,
            psm2_state,
            thread_start_pos,
            thread_end_pos,
            [thread_tension]
        ])
        
        # Define achieved goal as thread endpoint positions
        achieved_goal = np.concatenate([thread_start_pos, thread_end_pos])
        
        return {
            'observation': observation.copy(),
            'achieved_goal': achieved_goal.copy(),
            'desired_goal': self.goal.copy()
        }

    def _sample_goal(self):
        """Generate random goal state"""
        limits1 = np.asarray(self.WORKSPACE_LIMITS1)
        limits2 = np.asarray(self.WORKSPACE_LIMITS2) 
        
        goal_start = limits1.mean(axis=1) + np.random.uniform(-0.02, 0.02, size=3)
        goal_end = limits2.mean(axis=1) + np.random.uniform(-0.02, 0.02, size=3)
        
        self.goal = np.concatenate([goal_start, goal_end])
        return self.goal.copy()
    
    def _create_thread_end(self, position, color=(0.8, 0.8, 0.8, 1)):
        """Create a thread endpoint as a small sphere"""
        obj_id = p.loadURDF(
            os.path.join(ASSET_DIR_PATH, 'sphere/sphere.urdf'),
            position,
            p.getQuaternionFromEuler([0, 0, 0]),
            useFixedBase=False,
            globalScaling=0.01 * self.SCALING
        )
        p.changeVisualShape(obj_id, -1, rgbaColor=color)
        return obj_id

    def compute_reward(self, achieved_goal, desired_goal, info):
        """Calculate reward based on current state"""
        # Extract current and target positions
        current_start = achieved_goal[:3]
        current_end = achieved_goal[3:]
        target_start = desired_goal[:3]
        target_end = desired_goal[3:]
        
        # Calculate distances to targets
        start_distance = np.linalg.norm(current_start - target_start)
        end_distance = np.linalg.norm(current_end - target_end)
        
        # Calculate tension error
        current_tension = np.linalg.norm(current_start - current_end)
        target_tension = np.linalg.norm(target_start - target_end)
        tension_error = abs(current_tension - target_tension)
        
        # Determine success
        success = (
            start_distance < self.DISTANCE_THRESHOLD and
            end_distance < self.DISTANCE_THRESHOLD and
            tension_error < self.DISTANCE_THRESHOLD
        )
        
        return float(success) - 1.0

    def _is_success(self, achieved_goal, desired_goal):
        """Check if current state meets success criteria"""
        return self.compute_reward(achieved_goal, desired_goal, None) == 0.0
