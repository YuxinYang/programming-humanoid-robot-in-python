'''
In this exercise you need to know how to get sensor data.

* Task: get the current joint angle and temperature of joint HeadYaw

* Hint: The current sensor data of robot are store in perception (class Perception in spark_agent.py)

'''

from spark_agent import SparkAgent
from spark_agent import Perception


class MyAgent(SparkAgent):
    def think(self, perception):

        angle = perception.imu
        tempeture = perception.joint_temperature['HeadYaw']
        # YOUR CODE HERE
        # set angle and tempeture to current data of joint HeadYaw

        print 'HeadYaw angle: ' + str(angle) + ' tempeture: ' + str(tempeture)
        return super(MyAgent, self).think(perception)

if '__main__' == __name__:
    agent = MyAgent()
    agent.run()
