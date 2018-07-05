#ifndef _ROS_core_msgs_ProcessData_h
#define _ROS_core_msgs_ProcessData_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "core_msgs/LL_Params.h"
#include "core_msgs/ChildProcess.h"

namespace core_msgs
{

  class ProcessData : public ros::Msg
  {
    public:
      typedef int32_t _ID_type;
      _ID_type ID;
      typedef const char* _name_type;
      _name_type name;
      typedef const char* _command_type;
      _command_type command;
      typedef const char* _filepath_type;
      _filepath_type filepath;
      typedef int32_t _priority_type;
      _priority_type priority;
      uint32_t params_length;
      typedef core_msgs::LL_Params _params_type;
      _params_type st_params;
      _params_type * params;
      uint32_t processes_length;
      typedef core_msgs::ChildProcess _processes_type;
      _processes_type st_processes;
      _processes_type * processes;

    ProcessData():
      ID(0),
      name(""),
      command(""),
      filepath(""),
      priority(0),
      params_length(0), params(NULL),
      processes_length(0), processes(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_ID;
      u_ID.real = this->ID;
      *(outbuffer + offset + 0) = (u_ID.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_ID.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_ID.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_ID.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->ID);
      uint32_t length_name = strlen(this->name);
      varToArr(outbuffer + offset, length_name);
      offset += 4;
      memcpy(outbuffer + offset, this->name, length_name);
      offset += length_name;
      uint32_t length_command = strlen(this->command);
      varToArr(outbuffer + offset, length_command);
      offset += 4;
      memcpy(outbuffer + offset, this->command, length_command);
      offset += length_command;
      uint32_t length_filepath = strlen(this->filepath);
      varToArr(outbuffer + offset, length_filepath);
      offset += 4;
      memcpy(outbuffer + offset, this->filepath, length_filepath);
      offset += length_filepath;
      union {
        int32_t real;
        uint32_t base;
      } u_priority;
      u_priority.real = this->priority;
      *(outbuffer + offset + 0) = (u_priority.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_priority.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_priority.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_priority.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->priority);
      *(outbuffer + offset + 0) = (this->params_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->params_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->params_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->params_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->params_length);
      for( uint32_t i = 0; i < params_length; i++){
      offset += this->params[i].serialize(outbuffer + offset);
      }
      *(outbuffer + offset + 0) = (this->processes_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->processes_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->processes_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->processes_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->processes_length);
      for( uint32_t i = 0; i < processes_length; i++){
      offset += this->processes[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_ID;
      u_ID.base = 0;
      u_ID.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_ID.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_ID.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_ID.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->ID = u_ID.real;
      offset += sizeof(this->ID);
      uint32_t length_name;
      arrToVar(length_name, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_name-1]=0;
      this->name = (char *)(inbuffer + offset-1);
      offset += length_name;
      uint32_t length_command;
      arrToVar(length_command, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_command; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_command-1]=0;
      this->command = (char *)(inbuffer + offset-1);
      offset += length_command;
      uint32_t length_filepath;
      arrToVar(length_filepath, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_filepath; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_filepath-1]=0;
      this->filepath = (char *)(inbuffer + offset-1);
      offset += length_filepath;
      union {
        int32_t real;
        uint32_t base;
      } u_priority;
      u_priority.base = 0;
      u_priority.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_priority.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_priority.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_priority.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->priority = u_priority.real;
      offset += sizeof(this->priority);
      uint32_t params_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      params_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      params_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      params_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->params_length);
      if(params_lengthT > params_length)
        this->params = (core_msgs::LL_Params*)realloc(this->params, params_lengthT * sizeof(core_msgs::LL_Params));
      params_length = params_lengthT;
      for( uint32_t i = 0; i < params_length; i++){
      offset += this->st_params.deserialize(inbuffer + offset);
        memcpy( &(this->params[i]), &(this->st_params), sizeof(core_msgs::LL_Params));
      }
      uint32_t processes_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      processes_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      processes_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      processes_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->processes_length);
      if(processes_lengthT > processes_length)
        this->processes = (core_msgs::ChildProcess*)realloc(this->processes, processes_lengthT * sizeof(core_msgs::ChildProcess));
      processes_length = processes_lengthT;
      for( uint32_t i = 0; i < processes_length; i++){
      offset += this->st_processes.deserialize(inbuffer + offset);
        memcpy( &(this->processes[i]), &(this->st_processes), sizeof(core_msgs::ChildProcess));
      }
     return offset;
    }

    const char * getType(){ return "core_msgs/ProcessData"; };
    const char * getMD5(){ return "bda42f75a56200bd7d3ee4c202f9a3c0"; };

  };

}
#endif