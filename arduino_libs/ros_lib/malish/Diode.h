#ifndef _ROS_malish_Diode_h
#define _ROS_malish_Diode_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace malish
{

  class Diode : public ros::Msg
  {
    public:
      typedef int16_t _red_type;
      _red_type red;
      typedef int16_t _green_type;
      _green_type green;
      typedef int16_t _blue_type;
      _blue_type blue;

    Diode():
      red(0),
      green(0),
      blue(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int16_t real;
        uint16_t base;
      } u_red;
      u_red.real = this->red;
      *(outbuffer + offset + 0) = (u_red.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_red.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->red);
      union {
        int16_t real;
        uint16_t base;
      } u_green;
      u_green.real = this->green;
      *(outbuffer + offset + 0) = (u_green.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_green.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->green);
      union {
        int16_t real;
        uint16_t base;
      } u_blue;
      u_blue.real = this->blue;
      *(outbuffer + offset + 0) = (u_blue.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_blue.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->blue);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int16_t real;
        uint16_t base;
      } u_red;
      u_red.base = 0;
      u_red.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_red.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->red = u_red.real;
      offset += sizeof(this->red);
      union {
        int16_t real;
        uint16_t base;
      } u_green;
      u_green.base = 0;
      u_green.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_green.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->green = u_green.real;
      offset += sizeof(this->green);
      union {
        int16_t real;
        uint16_t base;
      } u_blue;
      u_blue.base = 0;
      u_blue.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_blue.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->blue = u_blue.real;
      offset += sizeof(this->blue);
     return offset;
    }

    const char * getType(){ return "malish/Diode"; };
    const char * getMD5(){ return "a530e893cf8706bc8d67c5b969d07eb5"; };

  };

}
#endif