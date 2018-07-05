#ifndef _ROS_core_msgs_LL_Params_h
#define _ROS_core_msgs_LL_Params_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "core_msgs/KeyValue.h"

namespace core_msgs
{

  class LL_Params : public ros::Msg
  {
    public:
      typedef int32_t _ID_Proc_type;
      _ID_Proc_type ID_Proc;
      typedef int32_t _ID_TL_type;
      _ID_TL_type ID_TL;
      uint32_t key_value_length;
      typedef core_msgs::KeyValue _key_value_type;
      _key_value_type st_key_value;
      _key_value_type * key_value;
      uint32_t path_length;
      typedef int32_t _path_type;
      _path_type st_path;
      _path_type * path;

    LL_Params():
      ID_Proc(0),
      ID_TL(0),
      key_value_length(0), key_value(NULL),
      path_length(0), path(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_ID_Proc;
      u_ID_Proc.real = this->ID_Proc;
      *(outbuffer + offset + 0) = (u_ID_Proc.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_ID_Proc.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_ID_Proc.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_ID_Proc.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->ID_Proc);
      union {
        int32_t real;
        uint32_t base;
      } u_ID_TL;
      u_ID_TL.real = this->ID_TL;
      *(outbuffer + offset + 0) = (u_ID_TL.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_ID_TL.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_ID_TL.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_ID_TL.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->ID_TL);
      *(outbuffer + offset + 0) = (this->key_value_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->key_value_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->key_value_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->key_value_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->key_value_length);
      for( uint32_t i = 0; i < key_value_length; i++){
      offset += this->key_value[i].serialize(outbuffer + offset);
      }
      *(outbuffer + offset + 0) = (this->path_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->path_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->path_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->path_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->path_length);
      for( uint32_t i = 0; i < path_length; i++){
      union {
        int32_t real;
        uint32_t base;
      } u_pathi;
      u_pathi.real = this->path[i];
      *(outbuffer + offset + 0) = (u_pathi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_pathi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_pathi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_pathi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->path[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_ID_Proc;
      u_ID_Proc.base = 0;
      u_ID_Proc.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_ID_Proc.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_ID_Proc.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_ID_Proc.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->ID_Proc = u_ID_Proc.real;
      offset += sizeof(this->ID_Proc);
      union {
        int32_t real;
        uint32_t base;
      } u_ID_TL;
      u_ID_TL.base = 0;
      u_ID_TL.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_ID_TL.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_ID_TL.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_ID_TL.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->ID_TL = u_ID_TL.real;
      offset += sizeof(this->ID_TL);
      uint32_t key_value_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      key_value_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      key_value_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      key_value_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->key_value_length);
      if(key_value_lengthT > key_value_length)
        this->key_value = (core_msgs::KeyValue*)realloc(this->key_value, key_value_lengthT * sizeof(core_msgs::KeyValue));
      key_value_length = key_value_lengthT;
      for( uint32_t i = 0; i < key_value_length; i++){
      offset += this->st_key_value.deserialize(inbuffer + offset);
        memcpy( &(this->key_value[i]), &(this->st_key_value), sizeof(core_msgs::KeyValue));
      }
      uint32_t path_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      path_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      path_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      path_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->path_length);
      if(path_lengthT > path_length)
        this->path = (int32_t*)realloc(this->path, path_lengthT * sizeof(int32_t));
      path_length = path_lengthT;
      for( uint32_t i = 0; i < path_length; i++){
      union {
        int32_t real;
        uint32_t base;
      } u_st_path;
      u_st_path.base = 0;
      u_st_path.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_path.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_path.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_path.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_path = u_st_path.real;
      offset += sizeof(this->st_path);
        memcpy( &(this->path[i]), &(this->st_path), sizeof(int32_t));
      }
     return offset;
    }

    const char * getType(){ return "core_msgs/LL_Params"; };
    const char * getMD5(){ return "12fd81664583ad4104d49d14379d571c"; };

  };

}
#endif