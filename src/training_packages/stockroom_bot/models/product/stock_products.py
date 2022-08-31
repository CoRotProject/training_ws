#!/home/hvh/miniconda3/envs/MRTA/bin/python

import rospy, tf

from gazebo_msgs.srv import *
from geometry_msgs.msg import *

if __name__ == '__main__':
    rospy.init_node("stock_products")
    rospy.wait_for_service("/gazebo/delete_model")
    rospy.wait_for_service("/gazebo/spawn_sdf_model")
    delete_model = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)
    s = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)
    orient = Quaternion(*tf.transformations.quaternion_from_euler(0, 0, 0))
    with open("./src/training_packages/stockroom_bot/models/product/model.sdf", "r") as f:
        product_xml = f.read()
    for product_num in xrange(0, 12):
        item_name = "product_{0}_0".format(product_num)
        delete_model(item_name)
    for product_num in xrange(0, 12):
        bin_y = 2.8 * (product_num / 6) - 1.4
        bin_x = 0.5 * (product_num % 6) - 1.5
        item_name = "product_{0}_0".format(product_num)
        item_pose = Pose(Point(x=bin_x, y=bin_y, z=2), orient)
        s(item_name, product_xml, "", item_pose, "world")
