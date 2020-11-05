#! /usr/bin/env python
import rospy
import os
from ur_dashboard_msgs.srv import GetRobotMode
from std_srvs.srv import Trigger
from requests import post, get

class InitializerNH:
    def __init__(self):
        rospy.init_node('initializer', anonymous=True)

        self.status = "OFF"
        self.rate = rospy.Rate(3)  # 3 hz

        self.robot_ip = "http://192.168.171"

        self.instantiate_the_robot()

    def check_ping(self):
        hostname = self.robot_ip
        ping_status = False
        while not ping_status:
            response = get(hostname)
            # and then check the response...
            print(response.status_code)
            if response.status_code == 200:
                ping_status = True
            else:
                ping_status = False

            break
        return ping_status

    def instantiate_the_robot(self):
        self.robot_power_status = self.power_on()

        if "Powering on" in self.robot_power_status.message:
            print(self.robot_power_status.message)
            self.status = "Powering on"

        while self.status == "Powering on":
            self.robot_power_status = self.wait_for_status()
            print(self.robot_power_status)
            if "IDLE" in self.robot_power_status.answer:
                print(self.robot_power_status.answer)
                self.status = "IDLE"

        rospy.sleep(1.0)
        self.robot_power_status = self.brake_release()
        if "IDLE" in self.robot_power_status.answer:
            print(self.robot_power_status.answer)
            self.status = "IDLE"
        message = "Robot Initialized"

        os.system("pkill rosmaster && pkill roscore")

        return True, message


    def brake_release(self):
        rospy.wait_for_service('/ur_hardware_interface/dashboard/brake_release', timeout=10.0)

        try:
            service = rospy.ServiceProxy('/ur_hardware_interface/dashboard/brake_release', GetRobotMode)
            resp = service()
            return resp
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)

    def wait_for_status(self):
        rospy.wait_for_service('/ur_hardware_interface/dashboard/get_robot_mode', timeout=10.0)

        try:
            service = rospy.ServiceProxy('/ur_hardware_interface/dashboard/get_robot_mode', GetRobotMode)
            resp = service()
            return resp
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)

    def power_on(self):
        rospy.wait_for_service('/ur_hardware_interface/dashboard/power_on', timeout=10.0)

        try:
            service = rospy.ServiceProxy('/ur_hardware_interface/dashboard/power_on', Trigger)
            resp = service()
            return resp
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)


if __name__ == "__main__":
    g = InitializerNH()
