#ifndef _ROS_core_msgs_LogMessage_h
#define _ROS_core_msgs_LogMessage_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "core_msgs/KeyValue.h"

namespace core_msgs
{

  class LogMessage : public ros::Msg
  {
    public:
      typedef const char* _processName_type;
      _processName_type processName;
      uint32_t data_length;
      typedef core_msgs::KeyValue _data_type;
      _data_type st_data;
      _data_type * data;

    LogMessage():
      processName(""),
      data_length(0), data(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_processName = strlen(this->processName);
      varToArr(outbuffer + offset, length_processName);
      offset += 4;
      memcpy(outbuffer + offset, this->processName, length_processName);
      offset += length_processName;
      *(outbuffer + offset + 0) = (this->data_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->data_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->data_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->data_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->data_length);
      for( uint32_t i = 0; i < data_length; i++){
      offset += this->data[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_processName;
      arrToVar(length_processName, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_processName; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_processName-1]=0;
      this->processName = (char *)(inbuffer + offset-1);
      offset += length_processName;
      uint32_t data_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      data_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      data_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      data_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->data_length);
      if(data_lengthT > data_length)
        this->data = (core_msgs::KeyValue*)realloc(this->data, data_lengthT * sizeof(core_msgs::KeyValue));
      data_length = data_lengthT;
      for( uint32_t i = 0; i < data_length; i++){
      offset += this->st_data.deserialize(inbuffer + offset);
        memcpy( &(this->data[i]), &(this->st_data), sizeof(core_msgs::KeyValue));
      }
     return offset;
    }

    const char * getType(){ return "core_msgs/LogMessage"; };
    const char * getMD5(){ return "f1ff24394838bdcb2a235fd55bc521ab"; };

  };

}
#endif