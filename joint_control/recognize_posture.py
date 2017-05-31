'''In this exercise you need to use the learned classifier to recognize current posture of robot

* Tasks:
    1. load learned classifier in `PostureRecognitionAgent.__init__`
    2. recognize current posture in `PostureRecognitionAgent.recognize_posture`

* Hints:
    Let the robot execute different keyframes, and recognize these postures.

'''


from angle_interpolation import AngleInterpolationAgent
from keyframes import * 
import pickle

ROBOT_POSE_CLF = 'robot_pose.pkl'
INDEX_TO_NAMES = {0: 'Frog', 1: 'Knee', 2: 'Crouch', 3: 'StandInit', 4: 'HeadBack', 5: 'Right', 6: 'Back', 7: 'Stand', 8: 'Sit', 9: 'Belly', 10: 'Left'}

class PostureRecognitionAgent(AngleInterpolationAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(PostureRecognitionAgent, self).__init__(simspark_ip,
                                                      simspark_port,
                                                      teamname,
                                                      player_id,
                                                      sync_mode)
        self.posture = 'unknown'
        self.posture_classifier = pickle.load(open(ROBOT_POSE_CLF))
        self.features = ['LHipYawPitch', 'LHipRoll', 'LHipPitch', 'LKneePitch', 'RHipYawPitch', 'RHipRoll', 'RHipPitch', 'RKneePitch','AngleX', 'AngleY']

    def think(self, perception):
        self.posture = self.recognize_posture(perception)
        #print self.posture
        return super(PostureRecognitionAgent, self).think(perception)

    def recognize_posture(self, perception):
        posture = 'unknown'
        # YOUR CODE HERE
        data = []
        
        for feature in self.features:
            if feature == 'AngleX':
                data.append(perception.imu[0])
            elif feature == 'AngleY':
                data.append(perception.imu[1])
            else:
                data.append(perception.joint[feature])

        predicted = self.posture_classifier.predict(data)
        posture = INDEX_TO_NAMES[predicted[0]]
        return posture

if __name__ == '__main__':
    agent = PostureRecognitionAgent()
    agent.keyframes = hello()  
    agent.run()


