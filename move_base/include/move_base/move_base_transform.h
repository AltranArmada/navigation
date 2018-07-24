/*********************************************************************
*
* Author: Livia Parente
*********************************************************************/
#ifndef NAV_MOVE_BASE_TRANSFORM_H_
#define NAV_MOVE_BASE_TRANSFORM_H_

#include <vector>
#include <string>

#include <ros/ros.h>


#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Twist.h>


  /**
   * @class MoveBaseTransform
   * @brief A class that transforms message of geometry_msgs/Twist type to geometry_msgs/TwistStamped.
   */
  class MoveBaseTransform {
    public:
      /**
       * @brief  Constructor for the actions
       * @param name The name of the action
       */
      MoveBaseTransform();

      /**
       * @brief  Destructor - Cleans up
       */
      virtual ~MoveBaseTransform();

      /**
       * @brief  Performs a control cycle
       * @param cmd_vel_ The velocity command sent
       */
      void callBack(const geometry_msgs::Twist& cmd_vel_);
      void executeCycle();
      


    private:

      ros::Publisher vel_transform_pub_;
      ros::Publisher att_transform_pub_;
      ros::Subscriber vel_sub_;
	 // Messages to be published
      geometry_msgs::TwistStamped msg_vel_;
      geometry_msgs::TwistStamped msg_att_;
      double controller_frequency_;

  };

#endif

