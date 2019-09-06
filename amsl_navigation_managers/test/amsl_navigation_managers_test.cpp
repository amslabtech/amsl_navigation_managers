#include <gtest/gtest.h>

#include <ros/ros.h>

#include "amsl_navigation_managers/node_edge_map_interface.h"

bool callback_flag = false;
amsl_navigation_msgs::NodeEdgeMap map;

void map_callback(const amsl_navigation_msgs::NodeEdgeMapConstPtr& msg)
{
    std::cout << "callback" << std::endl;
    map = *msg;
    callback_flag = true;
}

TEST(TestSuite, test_calculation1)
{
    ros::NodeHandle nh;
    ros::Subscriber map_sub = nh.subscribe("/node_edge_map/map", 1, &map_callback);
    ros::Duration(1.0).sleep();
    ros::spinOnce();
    EXPECT_TRUE(callback_flag);
    NodeEdgeMapInterface nemi;
    nemi.set_map(map);
    int edge_index = nemi.get_reversed_edge_index_from_edge_index(0);
    EXPECT_EQ(1, edge_index);
    edge_index = nemi.get_reversed_edge_index_from_edge_index(1);
    EXPECT_EQ(0, edge_index);
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);

    ros::init(argc, argv, "amsl_navigation_managers_test");

    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::Duration(3.0).sleep();

    int r_e_t = RUN_ALL_TESTS();

    spinner.stop();

    ros::shutdown();

    return r_e_t;
}
