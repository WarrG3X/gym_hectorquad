import numpy as np
import rospy
import os
import subprocess

from gym import utils, spaces
from gym_hectorquad.envs import hectorquad_env

from gazebo_msgs.srv import SpawnModel,DeleteModel,SetModelState
from gazebo_msgs.msg import ModelState 
from geometry_msgs.msg import Twist, Vector3, Pose

class HectorQuadForestEnv(hectorquad_env.HectorQuadEnv):
    
    def __init__(self,reward_type='sparse', other_args=None):
        self.reward_type = reward_type
        self.n_actions = 2
        self.publish_rate = rospy.Rate(40)
        self.publish_count = 6
        self.target_name = 'target_area'



        # Action Space
        self.distance_threshold = 0.5
        self.action_space = spaces.Box(low=np.array([0, -1.0]), high=np.array([1, 1.0]), dtype=np.float32)


        super(HectorQuadForestEnv, self).__init__()
        

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

    def generate_forest(self):
        """ Randomly place trees around center area to generate a forest
        """

        initial_pose = Pose()
        model_path = os.path.join(self.rospack.get_path('gym_hectorquad'),"models/tree/model.sdf")
        with open(model_path,'r') as f:
            sdf = f.read() 
        spawn_model_proxy = rospy.ServiceProxy('gazebo/spawn_sdf_model', SpawnModel)

        W = 10
        D = 10
        density = 0.4
        NW = int(W*density)
        ND = int(D*density) 
        trees = [(int(W*i/NW)-W/2+np.random.uniform()*W/NW,int(D*j/ND)-D/2+np.random.uniform()*D/ND) for i in range(NW) for j in range(ND)]

        counter = 1
        center_radius = 1
        for t in trees:
            if abs(t[0])< center_radius and abs(t[1]) < center_radius:
                continue
            initial_pose.position.x = t[0]
            initial_pose.position.y = t[1]
            initial_pose.position.z = 1
            rospy.wait_for_service('gazebo/spawn_sdf_model')
            spawn_model_proxy("tree{}".format(counter), sdf, "", initial_pose, "world")
            counter +=1
        
        self.total_trees = counter

    def _env_setup(self):
        """Override _env_setup from hectorquad_env to also call generate forest
        """
        self.set_gazebo_physics()
        self.generate_forest()


    def reset_target_position(self):
        """ Override reset_target_position from hectorquad_env to also delete trees
        """
        self.goal = self._sample_goal()
        target_cmd = "gz model -m " + self.target_name + " -x {} -y {} -z {}".format(*self.goal)
        subprocess.call(target_cmd,shell=True)

        # delete_model_proxy = rospy.ServiceProxy('gazebo/delete_model', DeleteModel)
        # print("Removing Forest")

        # for i in range(self.total_trees):
        #     rospy.wait_for_service('gazebo/delete_model')
        #     delete_model_proxy("tree{}".format(i+1))

        # print("Respawning Forest")
        # self.generate_forest()

        W = 10
        D = 10
        density = 0.4
        NW = int(W*density)
        ND = int(D*density) 
        trees = [(int(W*i/NW)-W/2+np.random.uniform()*W/NW,int(D*j/ND)-D/2+np.random.uniform()*D/ND) for i in range(NW) for j in range(ND)]

        state_msg = ModelState()

        set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)


        print("Moving Trees")

        for i in range(self.total_trees):
            rospy.wait_for_service('/gazebo/set_model_state')

            state_msg.model_name = 'tree{}'.format(i+1)
            state_msg.pose.position.x = trees[i][0]
            state_msg.pose.position.y = trees[i][1]
            state_msg.pose.position.z = 1
            set_state(state_msg)

