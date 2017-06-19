'''In this exercise you need to implemente inverse kinematics for NAO's legs

* Tasks:
    1. solve inverse kinemtatics for NAO's legs by using analytical or numerical method.
       You may need documentation of NAO's leg:
       http://doc.aldebaran.com/2-1/family/nao_h21/joints_h21.html
       http://doc.aldebaran.com/2-1/family/nao_h21/links_h21.html
    2. use the results of inverse kinemtatics to control NAO's legs (in InverseKinematicsAgent.set_transforms)
       and test your inverse kinemtatics implementation.
'''


from forward_kinematics import ForwardKinematicsAgent
from numpy.matlib import identity

import numpy as np


class InverseKinematicsAgent(ForwardKinematicsAgent):
    def inverse_kinematics(self, effector_name, transform):
        '''solve the inverse kinematics

        :param str effector_name: name of end effector, e.g. LLeg, RLeg
        :param transform: 4x4 transform matrix
        :return: list of joint angles
        '''

        '''jointss = {'HeadYaw': 0.0, 'RHipPitch': -0.0, 'RElbowYaw': -0.0, 'RShoulderPitch': 0.0, 'LShoulderPitch': 0.0, 'LKneePitch': -0.0, 'RAnkleRoll': 0.0, 'LShoulderRoll': 0.008726646259971648, 'LHipPitch': -0.0, 'LElbowYaw': 0.0, 'LAnklePitch': -0.0, 'RHipYawPitch': -0.0, 'HeadPitch': 0.0, 'LElbowRoll': -0.0066322511575784525, 'RShoulderRoll': -0.0066322511575784525, 'LAnkleRoll': -0.0, 'LHipYawPitch': -0.0, 'RAnklePitch': -0.0, 'LHipRoll': -0.0, 'RHipRoll': 0.0, 'RElbowRoll': 0.008726646259971648, 'RKneePitch': -0.0}'''
        
        jointss = (self.perception.joint).copy()
        lambda_scale = 0.01
        e = 0.01
        djoints = np.zeros((6,1))
        joints = self.chains[effector_name]
        last_joint = joints[-1]
        #print last_joint
        x_desired = np.array(transform)[0][3]
        y_desired = np.array(transform)[1][3]
        z_desired= np.array(transform)[2][3]
        thetax_d = 0
        thetay_d = 0
        thetaz_d = 0
        joint_angles = []
        for i in xrange(len(joints)):
            joint_angles.append(0)
        #print joint_angles
        
        self.forward_kinematics(jointss)
        x_cur = np.array(self.transforms.get(joints[-1]))[0][3]
        y_cur = np.array(self.transforms.get(joints[-1]))[1][3]
        z_cur = np.array(self.transforms.get(joints[-1]))[2][3]
        #print(x_cur, y_cur, z_cur)        
        while(np.abs(x_desired - x_cur) > e  or np.abs(y_desired - y_cur) > e or np.abs(z_desired - z_cur) > e):
            print (x_desired - x_cur, y_desired - y_cur, z_desired - z_cur)
            #print(transform)
            #Goal
            #print (xE,yE,zE)
            J = np.zeros((6,len(joints)))
            #print(self.transforms)
            for (i, joint) in enumerate(joints):
                xc = np.array(self.transforms.get(joint))[0][3]
                yc = np.array(self.transforms.get(joint))[1][3]
                zc = np.array(self.transforms.get(joint))[2][3]
               
                J[0, i] = x_cur - xc
                J[1, i] = y_cur - yc
                J[2, i] = z_cur - zc


                if (joint in ["LElbowRoll", "RElbowRoll", "LShoulderRoll", "LHipRoll", "RShoulderRoll", "RHipRoll", "LAnkleRoll", "RAnkleRoll"]):
                        J[3,i] = 1


                if (joint in ["HeadPitch", "LShoulderPitch", "LHipYawPitch", "LKneePitch", "LAnklePitch", "LHipPitch", "RShoulderPitch", "RHipYawPitch", "RKneePitch", "RAnklePitch", "RHipPitch"]):
                    if ("R" in joint):
                        J[4,i] = -1
                    else:
                        J[4,i] = 1

                if (joint in ["HeadYaw", "LHipYawPitch", "LElbowYaw", "RHipYawPitch", "RElbowYaw"]):
                    if("Hip" in joint):
                        J[5,i] = -1
                    else:
                        J[5,i] = 1
            
  

            Jplus = np.linalg.pinv(J)    
            #Jplus = np.dot(np.linalg.pinv(np.dot(J.T,J)),J.T)
            #print (len(Jplus),len(Jplus[0]))

            d = np.ones((6,1))
            #print(joints)
            d[0] = x_desired - np.array(self.transforms.get(joints[-1]))[0][3]
            d[1] = y_desired - np.array(self.transforms.get(joints[-1]))[1][3]
            d[2] = z_desired - np.array(self.transforms.get(joints[-1]))[2][3]
            d[3] = thetax_d - self.perception.joint.get(joints[-1])#x theta
            d[4] = thetay_d - self.perception.joint.get(joints[-1])#y theta
            d[5] = thetaz_d - self.perception.joint.get(joints[-1])#z theta

            djoints = lambda_scale * np.dot(Jplus,d)
            #print djoints
            for (i, j) in enumerate(joints):
                jointss[j] += djoints[i, 0]
                joint_angles[i] += djoints[i, 0]
            
            self.forward_kinematics(jointss)
            
            x_cur = np.array(self.transforms.get(joints[-1]))[0][3]
            y_cur = np.array(self.transforms.get(joints[-1]))[1][3]
            z_cur = np.array(self.transforms.get(joints[-1]))[2][3]
        
        
        print("inverse done")
        return joint_angles

    def set_transforms(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''
        # YOUR CODE HERE
        v = self.inverse_kinematics(effector_name,transform)
        for i, joint in enumerate(self.chains[effector_name]):
	      	self.target_joints[joint] = v[i]
        self.keyframes = ([], [], [])  # the result joint angles have to fill in

if __name__ == '__main__':
    agent = InverseKinematicsAgent()
    # test inverse kinematics
    T = identity(4)
    T[1, -1] = 0.05
    T[2, -1] = -0.26
    agent.set_transforms('LLeg', T)
    agent.run()
