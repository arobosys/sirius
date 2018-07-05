#ifndef _ROS_malish_Obstacle_h
#define _ROS_malish_Obstacle_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "ros/time.h"
#include "geometry_msgs/Pose.h"

namespace malish
{

  class Obstacle : public ros::Msg
  {
    public:
      typedef ros::Time _timestamp_type;
      _timestamp_type timestamp;
      typedef bool _alert_type;
      _alert_type alert;
      typedef geometry_msgs::Pose _pos_type;
      _pos_type pos;
      typedef float _area_type;
      _area_type area;

    Obstacle():
      timestamp(),
      alert(0),
      pos(),
      area(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->timestamp.sec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->timestamp.sec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->timestamp.sec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->timestamp.sec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->timestamp.sec);
      *(outbuffer + offset + 0) = (this->timestamp.nsec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->timestamp.nsec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->timestamp.nsec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->timestamp.nsec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->timestamp.nsec);
      union {
        bool real;
        uint8_t base;
      } u_alert;
      u_alert.real = this->alert;
      *(outbuffer + offset + 0) = (u_alert.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->alert);
      offset += this->pos.serialize(outbuffer + offset);
      union {
        float real;
        uint32_t base;
      } u_area;
      u_area.real = this->area;
      *(outbuffer + offset + 0) = (u_area.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_area.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_area.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_area.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->area);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->timestamp.sec =  ((uint32_t) (*(inbuffer + offset)));
      this->timestamp.sec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->timestamp.sec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->timestamp.sec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->timestamp.sec);
      this->timestamp.nsec =  ((uint32_t) (*(inbuffer + offset)));
      this->timestamp.nsec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->timestamp.nsec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->timestamp.nsec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->timestamp.nsec);
      union {
        bool real;
        uint8_t base;
      } u_alert;
      u_alert.base = 0;
      u_alert.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->alert = u_alert.real;
      offset += sizeof(this->alert);
      offset += this->pos.deserialize(inbuffer + offset);
      union {
        float real;
        uint32_t base;
      } u_area;
      u_area.base = 0;
      u_area.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_area.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_area.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_area.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->area = u_area.real;
      offset += sizeof(this->area);
     return offset;
    }

    const char * getType(){ return "malish/Obstacle"; };
    const char * getMD5(){ return "2fae2feed5cce8ea46cabef05e90417a"; };

  };

}
#endif