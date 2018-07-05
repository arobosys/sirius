#ifndef _ROS_core_msgs_ChildProcess_h
#define _ROS_core_msgs_ChildProcess_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace core_msgs
{

  class ChildProcess : public ros::Msg
  {
    public:
      typedef int32_t _id_type;
      _id_type id;
      typedef const char* _name_type;
      _name_type name;
      typedef const char* _filepath_type;
      _filepath_type filepath;

    ChildProcess():
      id(0),
      name(""),
      filepath("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_id;
      u_id.real = this->id;
      *(outbuffer + offset + 0) = (u_id.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_id.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_id.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_id.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->id);
      uint32_t length_name = strlen(this->name);
      varToArr(outbuffer + offset, length_name);
      offset += 4;
      memcpy(outbuffer + offset, this->name, length_name);
      offset += length_name;
      uint32_t length_filepath = strlen(this->filepath);
      varToArr(outbuffer + offset, length_filepath);
      offset += 4;
      memcpy(outbuffer + offset, this->filepath, length_filepath);
      offset += length_filepath;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_id;
      u_id.base = 0;
      u_id.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_id.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_id.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_id.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->id = u_id.real;
      offset += sizeof(this->id);
      uint32_t length_name;
      arrToVar(length_name, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_name-1]=0;
      this->name = (char *)(inbuffer + offset-1);
      offset += length_name;
      uint32_t length_filepath;
      arrToVar(length_filepath, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_filepath; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_filepath-1]=0;
      this->filepath = (char *)(inbuffer + offset-1);
      offset += length_filepath;
     return offset;
    }

    const char * getType(){ return "core_msgs/ChildProcess"; };
    const char * getMD5(){ return "18536d64d708f426ebbbcbeb580edd62"; };

  };

}
#endif