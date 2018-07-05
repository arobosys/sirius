#ifndef _ROS_malish_JoyCMD_h
#define _ROS_malish_JoyCMD_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace malish
{

  class JoyCMD : public ros::Msg
  {
    public:
      typedef bool _gogogo_type;
      _gogogo_type gogogo;
      typedef bool _restart_type;
      _restart_type restart;

    JoyCMD():
      gogogo(0),
      restart(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_gogogo;
      u_gogogo.real = this->gogogo;
      *(outbuffer + offset + 0) = (u_gogogo.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->gogogo);
      union {
        bool real;
        uint8_t base;
      } u_restart;
      u_restart.real = this->restart;
      *(outbuffer + offset + 0) = (u_restart.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->restart);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_gogogo;
      u_gogogo.base = 0;
      u_gogogo.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->gogogo = u_gogogo.real;
      offset += sizeof(this->gogogo);
      union {
        bool real;
        uint8_t base;
      } u_restart;
      u_restart.base = 0;
      u_restart.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->restart = u_restart.real;
      offset += sizeof(this->restart);
     return offset;
    }

    const char * getType(){ return "malish/JoyCMD"; };
    const char * getMD5(){ return "2d3b4a3db3ca77922893be883406d430"; };

  };

}
#endif