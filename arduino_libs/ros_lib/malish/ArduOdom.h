#ifndef _ROS_malish_ArduOdom_h
#define _ROS_malish_ArduOdom_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "ros/time.h"

namespace malish
{

  class ArduOdom : public ros::Msg
  {
    public:
      typedef float _wfl_type;
      _wfl_type wfl;
      typedef float _wfr_type;
      _wfr_type wfr;
      typedef float _wrl_type;
      _wrl_type wrl;
      typedef float _wrr_type;
      _wrr_type wrr;
      typedef ros::Time _timestamp_type;
      _timestamp_type timestamp;

    ArduOdom():
      wfl(0),
      wfr(0),
      wrl(0),
      wrr(0),
      timestamp()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_wfl;
      u_wfl.real = this->wfl;
      *(outbuffer + offset + 0) = (u_wfl.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_wfl.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_wfl.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_wfl.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->wfl);
      union {
        float real;
        uint32_t base;
      } u_wfr;
      u_wfr.real = this->wfr;
      *(outbuffer + offset + 0) = (u_wfr.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_wfr.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_wfr.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_wfr.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->wfr);
      union {
        float real;
        uint32_t base;
      } u_wrl;
      u_wrl.real = this->wrl;
      *(outbuffer + offset + 0) = (u_wrl.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_wrl.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_wrl.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_wrl.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->wrl);
      union {
        float real;
        uint32_t base;
      } u_wrr;
      u_wrr.real = this->wrr;
      *(outbuffer + offset + 0) = (u_wrr.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_wrr.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_wrr.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_wrr.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->wrr);
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
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_wfl;
      u_wfl.base = 0;
      u_wfl.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_wfl.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_wfl.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_wfl.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->wfl = u_wfl.real;
      offset += sizeof(this->wfl);
      union {
        float real;
        uint32_t base;
      } u_wfr;
      u_wfr.base = 0;
      u_wfr.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_wfr.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_wfr.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_wfr.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->wfr = u_wfr.real;
      offset += sizeof(this->wfr);
      union {
        float real;
        uint32_t base;
      } u_wrl;
      u_wrl.base = 0;
      u_wrl.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_wrl.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_wrl.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_wrl.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->wrl = u_wrl.real;
      offset += sizeof(this->wrl);
      union {
        float real;
        uint32_t base;
      } u_wrr;
      u_wrr.base = 0;
      u_wrr.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_wrr.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_wrr.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_wrr.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->wrr = u_wrr.real;
      offset += sizeof(this->wrr);
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
     return offset;
    }

    const char * getType(){ return "malish/ArduOdom"; };
    const char * getMD5(){ return "d16800db7bd2d132886a035a0c85c142"; };

  };

}
#endif