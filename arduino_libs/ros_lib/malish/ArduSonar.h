#ifndef _ROS_malish_ArduSonar_h
#define _ROS_malish_ArduSonar_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "ros/time.h"

namespace malish
{

  class ArduSonar : public ros::Msg
  {
    public:
      typedef uint16_t _sonLeft_type;
      _sonLeft_type sonLeft;
      typedef uint16_t _sonRight_type;
      _sonRight_type sonRight;
      typedef uint16_t _sonFront_type;
      _sonFront_type sonFront;
      typedef uint16_t _sonRear_type;
      _sonRear_type sonRear;
      typedef ros::Time _timestamp_type;
      _timestamp_type timestamp;

    ArduSonar():
      sonLeft(0),
      sonRight(0),
      sonFront(0),
      sonRear(0),
      timestamp()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->sonLeft >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->sonLeft >> (8 * 1)) & 0xFF;
      offset += sizeof(this->sonLeft);
      *(outbuffer + offset + 0) = (this->sonRight >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->sonRight >> (8 * 1)) & 0xFF;
      offset += sizeof(this->sonRight);
      *(outbuffer + offset + 0) = (this->sonFront >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->sonFront >> (8 * 1)) & 0xFF;
      offset += sizeof(this->sonFront);
      *(outbuffer + offset + 0) = (this->sonRear >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->sonRear >> (8 * 1)) & 0xFF;
      offset += sizeof(this->sonRear);
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
      this->sonLeft =  ((uint16_t) (*(inbuffer + offset)));
      this->sonLeft |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->sonLeft);
      this->sonRight =  ((uint16_t) (*(inbuffer + offset)));
      this->sonRight |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->sonRight);
      this->sonFront =  ((uint16_t) (*(inbuffer + offset)));
      this->sonFront |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->sonFront);
      this->sonRear =  ((uint16_t) (*(inbuffer + offset)));
      this->sonRear |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->sonRear);
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

    const char * getType(){ return "malish/ArduSonar"; };
    const char * getMD5(){ return "84abc8d4b4a6e2829cd7c00a09e502f5"; };

  };

}
#endif