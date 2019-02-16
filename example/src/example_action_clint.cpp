#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>

#include <stdio.h>
#include <thread>

#include <data/TestAction.h>

int a = 0;

void ReadThread(void *arg)
{
    actionlib::SimpleActionClient<data::TestAction> *ac_i = (actionlib::SimpleActionClient<data::TestAction> *)arg;
    if (getchar() == 'a')
    {
        ac_i->stopTrackingGoal();
    }
}

void donecb(const actionlib::SimpleClientGoalState &state, const data::TestResult::ConstPtr &result)
{
    ROS_INFO("Done");
    ros::shutdown();
}

void activecb()
{
    ROS_INFO("active");
}

void feedbackcb(const data::TestFeedback::ConstPtr &feedback)
{
    ROS_INFO("is run: %d", feedback->progress);
    a++;
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "testAction_clint");
    actionlib::SimpleActionClient<data::TestAction> ac("testAction", true);
    // std::thread thread1(ReadThread,(void*)&ac);

    ROS_INFO("Waiting for action server to start");

    ac.waitForServer();
    ROS_INFO("Action server is started");
    data::TestGoal goal;
    goal.goal = 10;
    ac.sendGoal(goal, &donecb, &activecb, &feedbackcb);
    //thread1.join();
    //ros::spin();
    while (ros::ok)
    {
        if (a == 5)
        {
            ac.cancelGoal();
            break;
        }
        ros::spinOnce();
    }

    return 0;
}