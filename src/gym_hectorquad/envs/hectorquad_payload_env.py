import numpy as np
import rospy
import os
import subprocess

from gym import utils, spaces
from gym_hectorquad.envs import hectorquad_env

from geometry_msgs.msg import Twist, Vector3, PoseStamped

class HectorQuadPayloadEnv(hectorquad_env.HectorQuadEnv):
    
    def __init__(self,reward_type='sparse', other_args=None):
        self.reward_type = reward_type
        self.n_actions = 2
        self.publish_rate = rospy.Rate(40)
        self.publish_count = 6
        self.target_name = 'target_area'

        # Payload
        self.has_payload = False
        self.payload_pos = np.zeros(3)
        self.payload_thresh = 0.5
        self.payload_pos_pub = None




        # Action Space
        self.distance_threshold = 0.5
        self.action_space = spaces.Box(low=np.array([0, -1.0]), high=np.array([1, 1.0]), dtype=np.float32)


        super(HectorQuadPayloadEnv, self).__init__()
        

    def goal_distance(self,goal_a, goal_b):
        assert goal_a.shape == goal_b.shape
        return np.linalg.norm(goal_a - goal_b, axis=-1)
    
    def _is_success(self, achieved_goal, desired_goal):
        d = self.goal_distance(achieved_goal, desired_goal)
        return (d < self.distance_threshold).astype(np.float32)

    def compute_reward(self, achieved_goal, goal, info=None):
        # Compute distance between goal and the achieved goal.
        ret = 0
        self.reward_type = 'dense'
        d = self.goal_distance(achieved_goal, goal)
        if self.reward_type == 'sparse':
            ret = -1*(d > self.distance_threshold).astype(np.float32)
        else:
            ret = -1*d
        return ret

    def check_payload(self,drone_pos):
        d = self.goal_distance(drone_pos, self.payload_pos)
        grabbed =  (d < self.payload_thresh).astype(np.float32)
        return grabbed

    def remove_payload(self):
        payload_remove_cmd = "gz model -m payload -d"
        subprocess.call(payload_remove_cmd,shell=True)

    def _sample_goal(self):
        """Samples a new goal and returns it.
        """
        goal_pos = self.np_random.uniform(-2.0,2.0,size=3)
        goal_pos[2] = self.target_height

        goal_vector = Vector3()
        goal_vector.x = goal_pos[0]
        goal_vector.y = goal_pos[1]
        goal_vector.z = goal_pos[2]
        self.goal_position_pub.publish(goal_vector)

        self.payload_pos = self.np_random.uniform(-2.0,2.0,size=3)
        self.payload_pos[2] = self.target_height

        payload_vector = Vector3()
        payload_vector.x = self.payload_pos[0]
        payload_vector.y = self.payload_pos[1]
        payload_vector.z = self.payload_pos[2]

        if self.payload_pos_pub == None:
            self.payload_pos_pub = rospy.Publisher(self.namespace+'/payload_position',Vector3 ,queue_size=1,latch=True)
            self.payload_path = os.path.join(self.rospack.get_path('gym_hectorquad'),"models/payload/model.sdf")

        self.payload_pos_pub.publish(payload_vector)

        return goal_pos

    def spawn_target(self):
        """ Override spawn_target from hectorquad_env to also spawn payload
        """
        target_spawn_cmd = "gz model -f " + self.target_path + " -m " + self.target_name + " -x {} -y {} -z {}".format(*self.goal)
        subprocess.call(target_spawn_cmd,shell=True)

        payload_spawn_cmd = "gz model -f " + self.payload_path + " -m payload -x {} -y {} -z {}".format(*self.payload_pos)
        subprocess.call(payload_spawn_cmd,shell=True)


    def reset_target_position(self):
        """ Override reset_target_position from hectorquad_env to also spawn payload
        """
        self.goal = self._sample_goal()
        target_cmd = "gz model -m " + self.target_name + " -x {} -y {} -z {}".format(*self.goal)
        subprocess.call(target_cmd,shell=True)

        # if self.has_payload:
        #     self.remove_payload()
        #     payload_cmd = "gz model -f " + self.payload_path + " -m payload -x {} -y {} -z {}".format(*self.payload_pos)
        #     self.has_payload = None
        # else:
        #     payload_cmd = "gz model -m payload -x {} -y {} -z {}".format(*self.payload_pos)
        # payload_cmd = "gz model -m payload -x {} -y {} -z {}".format(*self.payload_pos)
        self.has_payload = False
        
        # subprocess.call(payload_cmd,shell=True)

    def _get_obs(self):
        """ Override _get_obs from hectorquad_env to also spawn payload
        """
        rospy.wait_for_message(self.namespace+'/ground_truth_to_tf/pose',PoseStamped)
        drone_pos = self.pos_data
        drone_euler = self.attitude
        if not self.has_payload:
            self.has_payload = self.check_payload(drone_pos)
            payload_pos = self.payload_pos
        else:
            self.payload_pos = drone_pos

            payload_vector = Vector3()
            payload_vector.x = self.payload_pos[0]
            payload_vector.y = self.payload_pos[1]
            payload_vector.z = self.payload_pos[2]
            self.payload_pos_pub.publish(payload_vector)

        payload_params = np.array([self.goal_distance(drone_pos,self.payload_pos),int(self.has_payload)])

        obs = np.append(drone_pos,drone_euler)
        obs = np.concatenate([obs,self.payload_pos,payload_params],axis=0)

        achieved_goal = self.payload_pos.copy()

        return {
            'observation': obs.copy(),
            'achieved_goal': achieved_goal.copy(),
            'desired_goal': self.goal.copy(),
        }

    def step(self,action):
        action = np.clip(action, self.action_space.low, self.action_space.high)

        self._set_action(action)

        # rospy.sleep(self.step_delay)

        obs = self._get_obs()

        info = {
            'is_success': self._is_success(obs['achieved_goal'], self.goal),
            'has_payload':self.has_payload,
        }
        reward = self.compute_reward(obs['achieved_goal'], self.goal)

        done = self._is_success(obs['achieved_goal'], self.goal)
        # done = False

        # print(self.goal_distance(obs['achieved_goal'],self.goal))
        # print(self._is_success(obs['achieved_goal'], self.goal))

        return obs, reward, done, info

    def _set_action(self, action):
        """Applies the given action to the simulation.
        """
        # action = np.resize(action,self.n_actions)
        # assert action.shape == (self.n_actions,)

        twist_msg = Twist()
        twist_msg.linear.x = action[0]*0.5
        twist_msg.angular.z = action[1]

        for _ in range(self.publish_count):
            self.publish_rate.sleep()
            self.cmd_pub.publish(twist_msg)
