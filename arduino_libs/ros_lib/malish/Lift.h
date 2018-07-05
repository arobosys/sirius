#ifndef _ROS_malish_Lift_h
#define _ROS_malish_Lift_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace malish
{

  class Lift : public ros::Msg
  {
    public:
      typedef bool _dio1_type;
      _dio1_type dio1;
      typedef bool _dio2_type;
      _dio2_type dio2;
      typedef bool _dio3_type;
      _dio3_type dio3;

    Lift():
      dio1(0),
      dio2(0),
      dio3(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_dio1;
      u_dio1.real = this->dio1;
      *(outbuffer + offset + 0) = (u_dio1.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->dio1);
      union {
        bool real;
        uint8_t base;
      } u_dio2;
      u_dio2.real = this->dio2;
      *(outbuffer + offset + 0) = (u_dio2.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->dio2);
      union {
        bool real;
        uint8_t base;
      } u_dio3;
      u_dio3.real = this->dio3;
      *(outbuffer + offset + 0) = (u_dio3.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->dio3);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_dio1;
      u_dio1.base = 0;
      u_dio1.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->dio1 = u_dio1.real;
      offset += sizeof(this->dio1);
      union {
        bool real;
        uint8_t base;
      } u_dio2;
      u_dio2.base = 0;
      u_dio2.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->dio2 = u_dio2.real;
      offset += sizeof(this->dio2);
      union {
        bool real;
        uint8_t base;
      } u_dio3;
      u_dio3.base = 0;
      u_dio3.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->dio3 = u_dio3.real;
      offset += sizeof(this->dio3);
     return offset;
    }

    const char * getType(){ return "malish/Lift"; };
    const char * getMD5(){ return "593b4ab3bf1bbb6d11257e5e6ed6da25"; };

  };

}
#endif