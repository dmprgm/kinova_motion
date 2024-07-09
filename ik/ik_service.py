#! /usr/bin/env python3
import rospy 
import ikpy.urdf.utils
import ikpy.chain
import pathlib
import urdf_cleaner

from IPython import display
import ipywidgets as widgets
import ikpy.utils.plot as plot_utils
import matplotlib.pyplot as plt
import numpy as np
import rospy

from omic_project.srv import Forward, Inverse, ForwardResponse, InverseResponse
from sensor_msgs.msg import JointState
import hello_helpers.hello_misc as hm



class IKSolver(hm.HelloNode):
    def __init__(self):
        hm.HelloNode.__init__(self)
        hm.HelloNode.main(self, 'kinematics_service', 'kinematics_service', wait_for_first_pointcloud=False)
        cleaner = urdf_cleaner.URDFConverter()
        cleaner.convertUrdf()
        iktuturdf_path = "/tmp/iktutorial/stretch.urdf"
        self.chain = ikpy.chain.Chain.from_urdf_file(iktuturdf_path)
        self.tool = 'tool_stretch_dex_wrist_gripper'
        self.joint_states=None

    def joint_callback(self, msg):
        self.joint_states = msg

    #Executes Stretch to [target]
    def inverseKinematicsReq(self, request):
        target_point = [request.x_position, request.y_position, request.z_position]
        target_orientation = ikpy.utils.geometry.rpy_matrix(0.0, 0.0, 0.0)
        pretarget_orientation = ikpy.utils.geometry.rpy_matrix(0.0, 0.0, 0.0)
        #Solve for poses
        q_init = self.get_current_configuration()
        q_mid = self.chain.inverse_kinematics(target_point, initial_position=q_init)
        q_soln = self.chain.inverse_kinematics(target_point,  initial_position=q_init)
        #Plot
        #fig, ax = plot_utils.init_3d_figure()
        #plt.xlim(-0.75, 0.2)
        #plt.ylim(-0.75, 0.2)
        #ax.set_zlim(0.0, 1.0)
        #self.chain.plot(q_soln, ax, target=target_point, show=True)
        
        #if self.isValidMove(target_point, q_soln):
        return self.toJson(q_soln)
        #else:
        #    rospy.loginfo("Invalid Coordinates: cannot reach")
        #    return InverseResponse(JointState())
        
    def forwardKinematics(self):
        q = self.get_current_configuration()
        #Plot
        #fig, ax = plot_utils.init_3d_figure()
        #plt.xlim(-0.75, 0.2)
        #plt.ylim(-0.75, 0.2)
        #ax.set_zlim(0.0, 1.0)
        #self.chain.plot(q_soln, ax, target=target_point, show=True)
        return self.chain.forward_kinematics(q)[:3, 3]
    
    def forwardKinematicsReq(self, request):
        q = self.get_current_configuration()
        pos = self.chain.forward_kinematics(q)[:3, 3]
        
        return ForwardResponse(pos[0],pos[1],pos[2])
    
    def isValidMove(self, target_point, q_soln):
        return np.linalg.norm(self.chain.forward_kinematics(q_soln)[:3, 3] - target_point) < 0.01



    #Pushes commands to change the pose of the robot
    def toJson(self, q):
        joint_state = JointState()
        if self.tool == 'tool_stretch_gripper':
          
            q_lift = q[2]
            q_arm = q[4] + q[5] + q[6] + q[7]
            q_yaw = q[8]-(np.pi/2)

        elif self.tool == 'tool_stretch_dex_wrist' or self.tool == 'tool_stretch_dex_wrist_gripper':
            q_lift = q[2]
            q_arm = q[4] + q[5] + q[6] + q[7]
            q_yaw = q[8]
            q_pitch = q[10]
            q_roll = q[11]
        
        dict = {'joint_lift':q_lift, 'wrist_extension': q_arm, 'joint_wrist_yaw': q_yaw, 
                'joint_wrist_pitch':q_pitch, 'joint_wrist_roll':q_roll}
        vertical = {'joint_lift':q_lift}

        remaining_move = {'wrist_extension': q_arm, 'joint_wrist_yaw': q_yaw, 
                'joint_wrist_pitch':q_pitch, 'joint_wrist_roll':q_roll}
        

        self.move_to_pose(vertical)
        
        self.move_to_pose(remaining_move)

        for key, value in dict.items():
            joint_state.name.append(key)
            joint_state.position.append(value)
        return InverseResponse(joints=joint_state)

    def jointsToDict(self):
        joint_positions = {}
        for joint in ['joint_lift', 'wrist_extension', 'joint_wrist_yaw', 
                'joint_wrist_pitch', 'joint_wrist_roll']:
            if joint == "wrist_extension":
                index = self.joint_states.name.index('joint_arm_l0')
                joint_positions['wrist_extension'] = 4*self.joint_states.position[index]
                continue
            index = self.joint_states.name.index(joint)
            joint_positions[joint] = self.joint_states.position[index]
        return joint_positions

    #Gets the current pose of the robot
    def get_current_configuration(self):
        def bound_range(name, value):
            names = [l.name for l in self.chain.links]
            print(names)
            index = names.index(name)
            bounds = self.chain.links[index].bounds
            if name == "joint_wrist_yaw":
                print(value, bounds[0], bounds[1])
            return min(max(value, bounds[0]), bounds[1])
        joints = self.jointsToDict()
        if self.tool == 'tool_stretch_gripper':
            q_lift = bound_range('joint_lift', joints['joint_lift'])
            q_arml = bound_range('joint_arm_l0', joints['wrist_extension'] / 4.0)
            q_yaw = bound_range('joint_wrist_yaw', joints['joint_wrist_yaw'])
            return [0.0, 0.0, q_lift, 0.0, q_arml, q_arml, q_arml, q_arml, q_yaw, 0.0, 0.0]
        elif self.tool == 'tool_stretch_dex_wrist' or 'tool_stretch_dex_wrist_gripper':
            q_lift = bound_range('joint_lift', joints['joint_lift'])
            q_arml = bound_range('joint_arm_l0', joints['wrist_extension']  / 4.0)
            q_yaw = bound_range('joint_wrist_yaw', joints['joint_wrist_yaw'])
            q_pitch = bound_range('joint_wrist_pitch', joints['joint_wrist_pitch'])
            q_roll = bound_range('joint_wrist_roll', joints['joint_wrist_roll'])
        return [0.0, 0.0, q_lift, 0.0, q_arml, q_arml, q_arml, q_arml, q_yaw, 0.0, q_pitch, q_roll, 0.0, 0.0]
    


    def servers(self):
        self.sub = rospy.Subscriber('joint_states', JointState, self.joint_callback)
        rospy.Service('kinematics_service/forward', Forward, self.forwardKinematicsReq)
        rospy.Service('kinematics_service/inverse', Inverse, self.inverseKinematicsReq)
        rospy.loginfo("IK and FK Server Start")
        rospy.spin()
    
    def main(self):
        self.servers()

if __name__ == "__main__":
    ik = IKSolver()
    ik.main()



       

#ik_test = IKSolver()
#print(ik_test.forwardKinematics())
#robot.stow()
#ik_test.inverseKinematicsReq(target_point=[-0.01, -0.3,  1.2])
##ik_test.inverseKinematicsReq(target_point=[0.01, -0.7,  1.2])
#ik_test.inverseKinematicsReq(target_point=[-0.01, -0.3,  1.2])