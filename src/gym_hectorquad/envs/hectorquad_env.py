import os
import subprocess
import time
import numpy as np
import rospy
import rospkg
import pdb
import tf.transformations as tft

import gym
from gym import error, spaces
from gym.utils import seeding

from std_msgs.msg import Empty
from geometry_msgs.msg import Twist, PoseStamped, Vector3
from gazebo_msgs.msg import ODEPhysics
from gazebo_msgs.srv import SetPhysicsProperties



class HectorQuadEnv(gym.GoalEnv):
    def __init__(self):
        self.seed()
        self.rospack = rospkg.RosPack()
        self.namespace = ''
        self.motors_enabled = False
        self.real_time_factor = 0 # Value Between 0~5. Set 0 for max.
        self.takeoff_counter_limit = 20
        self.takeoff_height = float(rospy.get_param('starting_height'))
        self.dist_eps = 0.1
        self.target_path = os.path.join(self.rospack.get_path('gym_hectorquad'),"models/{}/model.sdf".format(self.target_name))
        self.target_height = self.takeoff_height

        # Target/Goal Params
        if self.target_name == 'sphere_target':
            self.distance_threshold = 0.2
        elif self.target_name == 'target_area':
            self.distance_threshold = 0.5
        else:
            raise RuntimeError("Unknown Target Object")

        self.cmd_pub = rospy.Publisher('process_cmd_vel',Twist,queue_size=1)
        rospy.Subscriber(self.namespace+'/ground_truth_to_tf/pose',PoseStamped,self.pose_callback)
        self.goal_position_pub = rospy.Publisher(self.namespace+'/goal_position',Vector3 ,queue_size=1,latch=True)


        # Sample Goal
        self.goal = self._sample_goal()
        rospy.loginfo("[GYM] Goal = {} {} {}".format(*self.goal))
        print("Goal Sampled")

        # Spawn Target
        self.spawn_target()
        print("Target Ready")

        # Observation Space
        obs = self._get_obs()
        print("Obs Ready")
        self.observation_space = spaces.Dict(dict(
            desired_goal=spaces.Box(-np.inf, np.inf, shape=obs['achieved_goal'].shape, dtype='float32'),
            achieved_goal=spaces.Box(-np.inf, np.inf, shape=obs['achieved_goal'].shape, dtype='float32'),
            observation=spaces.Box(-np.inf, np.inf, shape=obs['observation'].shape, dtype='float32'),
        ))


        # Setup Environment (Set physics and update rate)
        self._env_setup()

        print("Env Ready!")


    @property
    def dt(self):
        raise NotImplementedError()

    # Env methods
    # ----------------------------

    def pose_callback(self,data):
        self.pos_data = np.array([data.pose.position.x,data.pose.position.y,data.pose.position.z])
        self.attitude = tft.euler_from_quaternion([data.pose.orientation.w,data.pose.orientation.x,data.pose.orientation.y,data.pose.orientation.z])[0]
        

    def seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        print(seed)
        return [seed]


    def spawn_target(self):
        target_spawn_cmd = "gz model -f " + self.target_path + " -m " + self.target_name + " -x {} -y {} -z {}".format(*self.goal)
        subprocess.call(target_spawn_cmd,shell=True)

    
    def takeoff(self):
        takeoff_counter = 0
        rate = rospy.Rate(20)
        while True:
            lower_bound = self.takeoff_height - self.dist_eps
            upper_bound = self.takeoff_height + self.dist_eps
            while takeoff_counter < self.takeoff_counter_limit:
                height = self.pos_data[2]
                if height < lower_bound:
                    delta_height = +0.7
                    takeoff_counter = 0
                elif height > upper_bound:
                    delta_height = -0.7 
                    takeoff_counter = 0
                else:
                    delta_height = 0
                    takeoff_counter +=1
            
                data = Twist()
                data.linear.z = delta_height
                self.cmd_pub.publish(data)
                rate.sleep()


            # print("Takeoff Done")
            return

    def step(self,action):
        action = np.clip(action, self.action_space.low, self.action_space.high)

        self._set_action(action)

        # rospy.sleep(self.step_delay)

        obs = self._get_obs()

        info = {
            'is_success': self._is_success(obs['achieved_goal'], self.goal),
        }
        reward = self.compute_reward(obs['achieved_goal'], self.goal)

        done = self._is_success(obs['achieved_goal'], self.goal)
        # done = False

        # print(self.goal_distance(obs['achieved_goal'],self.goal))
        # print(self._is_success(obs['achieved_goal'], self.goal))

        return obs, reward, done, info

    def reset_target_position(self):
        self.goal = self._sample_goal()
        target_cmd = "gz model -m " + self.target_name + " -x {} -y {} -z {}".format(*self.goal)
        subprocess.call(target_cmd,shell=True)

    def reset(self):
        self._unpause_sim()
        rospy.loginfo("[GYM] Resetting Drone")
        self._reset_sim()
        self.reset_target_position()
        try:
            self.takeoff()
        # except rospy.ROSInterruptException:
        except:
            rospy.signal_shutdown("Shutting Down")
            exit("done")

        obs = self._get_obs()
        return obs

    def close(self):
        raise NotImplementedError()

    def render(self):
        raise NotImplementedError()

    def _get_viewer(self):
        raise NotImplementedError()

    # Extension methods
    # ----------------------------

    def _reset_sim(self):
        self._toggle_motors(False)
        subprocess.call('rosservice call /gazebo/reset_simulation',shell=True)
        self._unpause_sim()
        time.sleep(0.1)
        self._toggle_motors(True)

    def _pause_sim(self):
        subprocess.call('rosservice call /gazebo/unpause_physics',shell=True)

    def _unpause_sim(self):
        subprocess.call('rosservice call /gazebo/unpause_physics',shell=True)

    def _toggle_motors(self,arg):
        if arg == True:
            subprocess.call('rosservice call /enable_motors "enable: true"',shell=True,stdout=subprocess.DEVNULL,stderr=subprocess.STDOUT)
            self.motors_enabled = True
        else:
            subprocess.call('rosservice call /enable_motors "enable: false"',shell=True,stdout=subprocess.DEVNULL,stderr=subprocess.STDOUT)
            self.motors_enabled = False

    def set_gazebo_physics(self):
        rospy.wait_for_service('/gazebo/set_physics_properties')
        set_physics = rospy.ServiceProxy('/gazebo/set_physics_properties', SetPhysicsProperties)
        time_step = 0.001
        max_update_rate = self.real_time_factor * 1000
        gravity = Vector3(0,0,-9.8)
        #Args order = ['auto_disable_bodies', 'sor_pgs_precon_iters', 'sor_pgs_iters', 'sor_pgs_w', 'sor_pgs_rms_error_tol', 'contact_surface_layer', 'contact_max_correcting_vel', 'cfm', 'erp', 'max_contacts']
        ode_config = ODEPhysics(False,0,50,1.3,0.0,0.001,100.0,0.0,0.2,20)
        try:
            resp = set_physics(time_step,max_update_rate,gravity,ode_config)
            print("Physics Set :",resp.success)
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))

    def _get_obs(self):
        """Returns the observation.
        """
        rospy.wait_for_message(self.namespace+'/ground_truth_to_tf/pose',PoseStamped)
        drone_pos = self.pos_data
        drone_euler = self.attitude
        obs = np.append(drone_pos,drone_euler)
        achieved_goal = drone_pos.copy()

        return {
            'observation': obs.copy(),
            'achieved_goal': achieved_goal.copy(),
            'desired_goal': self.goal.copy(),
        }


    def _set_action(self, action):
        """Applies the given action to the simulation.
        """
        raise NotImplementedError()

    def _is_success(self, achieved_goal, desired_goal):
        """Indicates whether or not the achieved goal successfully achieved the desired goal.
        """
        raise NotImplementedError()

    def _sample_goal(self):
        """Samples a new goal and returns it.
        """
        raise NotImplementedError()

    def _env_setup(self):
        """Initial configuration of the environment. Can be used to configure initial state
        and extract information from the simulation.
        """
        self.set_gazebo_physics()

    def _viewer_setup(self):
        """Initial configuration of the viewer. Can be used to set the camera position,
        for example.
        """
        pass

    def _render_callback(self):
        """A custom callback that is called before rendering. Can be used
        to implement custom visualizations.
        """
        pass

    def _step_callback(self):
        """A custom callback that is called after stepping the simulation. Can be used
        to enforce additional constraints on the simulation state.
        """
        pass
