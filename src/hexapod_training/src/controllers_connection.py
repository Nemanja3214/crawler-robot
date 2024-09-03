#!/usr/bin/python3.8

import rospy
from controller_manager_msgs.srv import SwitchController,UnloadController, UnloadControllerRequest, LoadController, LoadControllerRequest, SwitchControllerRequest, SwitchControllerResponse
import rospkg
import rosparam

def make_name(part, side, num):
            return part + "_joint_" + side + str(num) +"_position_controller"

class ControllersConnection():
    
    def __init__(self, namespace):

        self.switch_service_name = '/'+namespace+'/controller_manager/switch_controller'
        self.switch_service = rospy.ServiceProxy(self.switch_service_name, SwitchController)

        self.load_service_name = '/'+namespace+'/controller_manager/load_controller'
        self.load_service = rospy.ServiceProxy(self.load_service_name, LoadController)

        self.unload_service_name = '/'+namespace+'/controller_manager/unload_controller'
        self.unload_service = rospy.ServiceProxy(self.unload_service_name, UnloadController)

    def controllers_list(self):
        parts = ["coxa", "tibia", "femur"]
        sides = ["l", "r"]
        nums = [1, 2, 3]
        controllers = ["joint_state_controller"]
        for part in parts:
            for side in sides:
                for num in nums:
                    name = make_name(part, side, num)
                    controllers.append(name)
        return controllers

    def unload(self):
        for controller in self.controllers_list():
            req = UnloadControllerRequest()
            req.name = controller
            rospy.wait_for_service(self.unload_service_name)
            unload_ok = self.unload_service(req)
            rospy.logdebug("UNLOAD IS>>>>>"+str(unload_ok))

    def stop(self):
        self.switch_controllers(controllers_on=[],
                                                    controllers_off=self.controllers_list())

    def start(self):
        self.switch_controllers(controllers_on=self.controllers_list(),
                                                    controllers_off=[])


               

    def load(self):
        for controller in self.controllers_list():
            req = LoadControllerRequest()
            req.name = controller
            rospy.wait_for_service(self.load_service_name)
            load_ok = self.load_service(req)
            rospy.logdebug("UNLOAD IS>>>>>"+str(load_ok))

    def switch_controllers(self, controllers_on, controllers_off, strictness=1):
        """
        Give the controllers you wan to switch on or off.
        :param controllers_on: ["name_controler_1", "name_controller2",...,"name_controller_n"]
        :param controllers_off: ["name_controler_1", "name_controller2",...,"name_controller_n"]
        :return:
        """
        rospy.logdebug("WAITING FOR SWITCH SERVICE")
        rospy.wait_for_service(self.switch_service_name)

        try:
            switch_request_object = SwitchControllerRequest()
            switch_request_object.start_controllers = controllers_on
            switch_request_object.stop_controllers = controllers_off
            switch_request_object.strictness = strictness
            switch_request_object.start_asap = True
            switch_request_object.timeout = 120
            rospy.loginfo(switch_request_object)
            switch_result = self.switch_service(switch_request_object)
            """
            [controller_manager_msgs/SwitchController]
            int32 BEST_EFFORT=1
            int32 STRICT=2
            string[] start_controllers
            string[] stop_controllers
            int32 strictness
            ---
            bool ok
            """
            rospy.loginfo("Switch Result==>"+str(switch_result.ok))

            return switch_result.ok

        except rospy.ServiceException as e:
            print (self.switch_service_name+" service call failed")

            return None

    def reset_controllers(self, controllers_reset):
        """
        We turn on and off the given controllers
        :param controllers_reset: ["name_controler_1", "name_controller2",...,"name_controller_n"]
        :return:
        """
        reset_result = False

        rospy.logdebug("TURNING OFF CONTROLLERS>>>"+str(controllers_reset))
        result_off_ok = self.switch_controllers(controllers_on = [],
                                controllers_off = controllers_reset)
        if result_off_ok:
            rospy.logdebug("TURNING ON CONTROLLERS>>>"+str(controllers_reset))

            result_on_ok = self.switch_controllers(controllers_on=controllers_reset,
                                                    controllers_off=[])
            if result_on_ok:
                rospy.logdebug("Controllers Reseted==>"+str(controllers_reset))
                reset_result = True
            else:
                rospy.logdebug("result_on_ok==>" + str(result_on_ok))
        else:
            rospy.logdebug("result_off_ok==>" + str(result_off_ok))

        return reset_result

    def reset_hexapod_joint_controllers(self):
        self.reset_controllers(self.controllers_list())
