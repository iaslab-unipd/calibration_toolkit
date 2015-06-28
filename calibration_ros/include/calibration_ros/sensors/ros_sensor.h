/*
 *  Copyright (c) 2015-, Filippo Basso <bassofil@gmail.com>
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *     1. Redistributions of source code must retain the above copyright
 *        notice, this list of conditions and the following disclaimer.
 *     2. Redistributions in binary form must reproduce the above copyright
 *        notice, this list of conditions and the following disclaimer in the
 *        documentation and/or other materials provided with the distribution.
 *     3. Neither the name of the copyright holder(s) nor the
 *        names of its contributors may be used to endorse or promote products
 *        derived from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 *  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 *  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
 *  DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 *  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 *  ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 *  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef UNIPD_CALIBRATION_CALIBRATION_ROS_SENSORS_ROS_SENSOR_H_
#define UNIPD_CALIBRATION_CALIBRATION_ROS_SENSORS_ROS_SENSOR_H_

#include <ros/ros.h>

namespace unipd
{
namespace calib
{

template <typename TopicT_, typename MessagesT_, typename DataT_>
  class ROSSensor
  {
  public:

    using Topic = TopicT_;
    using Messages = MessagesT_;
    using Data = DataT_;
    
    ROSSensor ()
      : topic_mask_(Topic::ALL),
        received_messages_(Topic::NONE)
    {
      // Do nothing
    }

    virtual
    ~ROSSensor () {}

    virtual void
    subscribe (int topic) = 0;
    
    bool
    allMessagesReceived () const
    {
      return topic_mask_ == received_messages_;
    }

    bool
    isMessagesReceived (int topic) const
    {
      return topic & received_messages_;
    }

    bool
    waitForMessage (int topic,
                    ros::Rate rate = ros::Rate(1.0)) const
    {
      if (!(topic & topic_mask_))
        return true;
      
      while (ros::ok() and not isMessagesReceived(topic))
      {
        ros::spinOnce();
        rate.sleep();
      }
      return isMessagesReceived(topic);
    }
    
    bool
    waitForAllMessages (ros::Rate rate = ros::Rate(1.0)) const
    {
      while (ros::ok() and not allMessagesReceived())
      {
        ros::spinOnce();
        rate.sleep();
      }
      return allMessagesReceived();
    }
    
    virtual const Messages &
    lastMessages () const = 0;

    virtual Data
    convertMessages (const Messages & messages) const = 0;

  protected:

    void
    setTopicMask (int topic_mask)
    {
      topic_mask_ = topic_mask;
    }

    void
    newMessageReceived (int topic) const
    {
      received_messages_ |= topic;
    }

    void
    resetReceivedMessages () const
    {
      received_messages_ = Topic::NONE;
    }
    
    int topic_mask_;
    std::string log_;
    
  private:

    mutable int received_messages_;

  };

} // namespace calib
} // namespace unipd
#endif // UNIPD_CALIBRATION_CALIBRATION_ROS_SENSORS_ROS_SENSOR_H_
