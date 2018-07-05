#ifndef _ROS_malish_NewTwist_h
#define _ROS_malish_NewTwist_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace malish
{

  class NewTwist : public ros::Msg
  {
    public:
      typedef float _angle_vel_type;
      _angle_vel_type angle_vel;
      typedef float _orient_type;
      _orient_type orient;
      typedef int16_t _linear_vel_type;
      _linear_vel_type linear_vel;

    NewTwist():
      angle_vel(0),
      orient(0),
      linear_vel(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_angle_vel;
      u_angle_vel.real = this->angle_vel;
      *(outbuffer + offset + 0) = (u_angle_vel.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_angle_vel.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_angle_vel.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_angle_vel.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->angle_vel);
      union {
        float real;
        uint32_t base;
      } u_orient;
      u_orient.real = this->orient;
      *(outbuffer + offset + 0) = (u_orient.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_orient.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_orient.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_orient.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->orient);
      union {
        int16_t real;
        uint16_t base;
      } u_linear_vel;
      u_linear_vel.real = this->linear_vel;
      *(outbuffer + offset + 0) = (u_linear_vel.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_linear_vel.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->linear_vel);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_angle_vel;
      u_angle_vel.base = 0;
      u_angle_vel.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_angle_vel.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_angle_vel.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_angle_vel.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->angle_vel = u_angle_vel.real;
      offset += sizeof(this->angle_vel);
      union {
        float real;
        uint32_t base;
      } u_orient;
      u_orient.base = 0;
      u_orient.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_orient.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_orient.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_orient.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->orient = u_orient.real;
      offset += sizeof(this->orient);
      union {
        int16_t real;
        uint16_t base;
      } u_linear_vel;
      u_linear_vel.base = 0;
      u_linear_vel.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_linear_vel.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->linear_vel = u_linear_vel.real;
      offset += sizeof(this->linear_vel);
     return offset;
    }

    const char * getType(){ return "malish/NewTwist"; };
    const char * getMD5(){ return "df1f173b299ee3c106eb0e1e0cde2189"; };

  };

}
#endif