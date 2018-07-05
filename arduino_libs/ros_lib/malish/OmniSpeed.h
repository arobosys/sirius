#ifndef _ROS_malish_OmniSpeed_h
#define _ROS_malish_OmniSpeed_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace malish
{

  class OmniSpeed : public ros::Msg
  {
    public:
      typedef uint16_t _wfl_type;
      _wfl_type wfl;
      typedef uint16_t _wfr_type;
      _wfr_type wfr;
      typedef uint16_t _wrl_type;
      _wrl_type wrl;
      typedef uint16_t _wrr_type;
      _wrr_type wrr;

    OmniSpeed():
      wfl(0),
      wfr(0),
      wrl(0),
      wrr(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->wfl >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->wfl >> (8 * 1)) & 0xFF;
      offset += sizeof(this->wfl);
      *(outbuffer + offset + 0) = (this->wfr >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->wfr >> (8 * 1)) & 0xFF;
      offset += sizeof(this->wfr);
      *(outbuffer + offset + 0) = (this->wrl >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->wrl >> (8 * 1)) & 0xFF;
      offset += sizeof(this->wrl);
      *(outbuffer + offset + 0) = (this->wrr >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->wrr >> (8 * 1)) & 0xFF;
      offset += sizeof(this->wrr);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->wfl =  ((uint16_t) (*(inbuffer + offset)));
      this->wfl |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->wfl);
      this->wfr =  ((uint16_t) (*(inbuffer + offset)));
      this->wfr |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->wfr);
      this->wrl =  ((uint16_t) (*(inbuffer + offset)));
      this->wrl |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->wrl);
      this->wrr =  ((uint16_t) (*(inbuffer + offset)));
      this->wrr |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->wrr);
     return offset;
    }

    const char * getType(){ return "malish/OmniSpeed"; };
    const char * getMD5(){ return "a2a0dc8b40238216924727e0786d8b0a"; };

  };

}
#endif