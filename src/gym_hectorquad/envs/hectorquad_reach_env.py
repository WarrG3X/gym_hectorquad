import numpy as np
import rospy

from gym import utils, spaces
from gym_hectorquad.envs import hectorquad_env

from geometry_msgs.msg import Twist, Vector3

class HectorQuadReachEnv(hectorquad_env.HectorQuadEnv):
    
    def __init__(self,reward_type='sparse', other_args=None):
        self.reward_type = reward_type
        self.n_actions = 2
        self.publish_rate = rospy.Rate(40)
        self.publish_count = 6
        self.target_name = 'target_area'



        # Action Space
        self.distance_threshold = 0.5
        self.action_space = spaces.Box(low=np.array([0, -1.0]), high=np.array([1, 1.0]), dtype=np.float32)


        super(HectorQuadReachEnv, self).__init__()
        

    def goal_distance(self,goal_a, goal_b):
        assert goal_a.shape == goal_b.shape
        return np.linalg.norm(goal_a - goal_b, axis=-1)
    
    def _is_success(self, achieved_goal, desired_goal):
        d = self.goal_distance(achieved_goal, desired_goal)
        return (d < self.distance_threshold).astype(np.float32)

    def compute_reward(self, achieved_goal, goal, info=None):
        # Compute distance between goal and the achieved goal.
        ret = 0
        d = self.goal_distance(achieved_goal, goal)
        if self.reward_type == 'sparse':
            ret = -1*(d > self.distance_threshold).astype(np.float32)
        else:
            ret = -1*d
        return ret

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

        return goal_pos

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


