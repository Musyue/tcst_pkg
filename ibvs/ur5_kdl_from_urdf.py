#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Import the module
from urdf_parser_py.urdf import URDF
from pykdl_utils.kdl_parser import kdl_tree_from_urdf_model
from pykdl_utils.kdl_kinematics import KDLKinematics


# robot = URDF.from_xml_file("../urdf/test_robot.urdf")
# robot = URDF.from_xml_file("../urdf/ur3.urdf")

class UR_robot:
    def __init__(self):
        # rospy.init_node("import_ur3_from_urdf")
        self.robot = self.init_robot("/data/ros/yue_ws_201903/src/tcst_pkg/urdf/ur5.urdf")
        self.kdl_kin = KDLKinematics(self.robot, "base_link", "ee_link")
        self.tree = kdl_tree_from_urdf_model(self.robot)
        self.chain = self.tree.getChain("base_link", "ee_link")
        # safe angle of UR3

        self.safe_q = [0.20741444444444448, -1.8128266666666668, -1.2626288888888888, -1.66891, 1.6191933333333333, 3.204893333333333]
        self.q = self.safe_q

    def init_robot(self, filename):
        # print("here")
        robot = URDF.from_xml_file(filename)
        # print("outhere")
        return robot

    def set_q(self, q_list):
        self.q = q_list

    def get_fk_pose(self):
        q = self.q
        pose = self.kdl_kin.forward(q)  # forward kinematics (returns homogeneous 4x4 matrix)
        return pose

    def get_chain(self):
        self.tree = kdl_tree_from_urdf_model(self.robot)
        self.chain = self.tree.getChain("base_link", "ee_link")

    def get_jacobian(self):
        J = self.kdl_kin.jacobian(self.q)
        return J

    # def get_fk_pose(self):
    def get_ik_pose(self,pose):
        q_ik = self.kdl_kin.inverse(pose)  # inverse kinematics
        print "q_ik", q_ik
    def test_fk_ik_jacob(self):
        q = self.q
        pose = self.kdl_kin.forward(q)  # forward kinematics (returns homogeneous 4x4 matrix)
        print pose

        q_ik = self.kdl_kin.inverse(pose, q)  # inverse kinematics
        print "q_ik", q_ik

        if q_ik is not None:
            pose_sol = self.kdl_kin.forward(q_ik)  # should equal pose
            print pose_sol

        J = self.kdl_kin.jacobian(q)
        print 'J:', J

def main():
    q = [0,0,1,0,1,0]
    ur3 = UR_robot()
    ur3.set_q(q)
    ur3.test_fk_ik_jacob()

if __name__ == "__main__":
    main()