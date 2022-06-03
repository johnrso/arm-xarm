// Auto-generated. Do not edit!

// (in-package xarm_msgs.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class MoveVelocityRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.speeds = null;
      this.is_sync = null;
      this.is_tool_coord = null;
      this.duration = null;
    }
    else {
      if (initObj.hasOwnProperty('speeds')) {
        this.speeds = initObj.speeds
      }
      else {
        this.speeds = [];
      }
      if (initObj.hasOwnProperty('is_sync')) {
        this.is_sync = initObj.is_sync
      }
      else {
        this.is_sync = false;
      }
      if (initObj.hasOwnProperty('is_tool_coord')) {
        this.is_tool_coord = initObj.is_tool_coord
      }
      else {
        this.is_tool_coord = false;
      }
      if (initObj.hasOwnProperty('duration')) {
        this.duration = initObj.duration
      }
      else {
        this.duration = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type MoveVelocityRequest
    // Serialize message field [speeds]
    bufferOffset = _arraySerializer.float32(obj.speeds, buffer, bufferOffset, null);
    // Serialize message field [is_sync]
    bufferOffset = _serializer.bool(obj.is_sync, buffer, bufferOffset);
    // Serialize message field [is_tool_coord]
    bufferOffset = _serializer.bool(obj.is_tool_coord, buffer, bufferOffset);
    // Serialize message field [duration]
    bufferOffset = _serializer.float32(obj.duration, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type MoveVelocityRequest
    let len;
    let data = new MoveVelocityRequest(null);
    // Deserialize message field [speeds]
    data.speeds = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [is_sync]
    data.is_sync = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [is_tool_coord]
    data.is_tool_coord = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [duration]
    data.duration = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 4 * object.speeds.length;
    return length + 10;
  }

  static datatype() {
    // Returns string type for a service object
    return 'xarm_msgs/MoveVelocityRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '3ff5ed26eb25726c663dac7efa05cc61';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    
    
    
    
    
    
    
    float32[] speeds
    
    
    
    bool is_sync
    
    
    
    bool is_tool_coord
    
    
    
    
    
    
    float32 duration
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new MoveVelocityRequest(null);
    if (msg.speeds !== undefined) {
      resolved.speeds = msg.speeds;
    }
    else {
      resolved.speeds = []
    }

    if (msg.is_sync !== undefined) {
      resolved.is_sync = msg.is_sync;
    }
    else {
      resolved.is_sync = false
    }

    if (msg.is_tool_coord !== undefined) {
      resolved.is_tool_coord = msg.is_tool_coord;
    }
    else {
      resolved.is_tool_coord = false
    }

    if (msg.duration !== undefined) {
      resolved.duration = msg.duration;
    }
    else {
      resolved.duration = 0.0
    }

    return resolved;
    }
};

class MoveVelocityResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.ret = null;
      this.message = null;
    }
    else {
      if (initObj.hasOwnProperty('ret')) {
        this.ret = initObj.ret
      }
      else {
        this.ret = 0;
      }
      if (initObj.hasOwnProperty('message')) {
        this.message = initObj.message
      }
      else {
        this.message = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type MoveVelocityResponse
    // Serialize message field [ret]
    bufferOffset = _serializer.int16(obj.ret, buffer, bufferOffset);
    // Serialize message field [message]
    bufferOffset = _serializer.string(obj.message, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type MoveVelocityResponse
    let len;
    let data = new MoveVelocityResponse(null);
    // Deserialize message field [ret]
    data.ret = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [message]
    data.message = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.message.length;
    return length + 6;
  }

  static datatype() {
    // Returns string type for a service object
    return 'xarm_msgs/MoveVelocityResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '76c68a2c5e109f4d6a4dc1cfc355f405';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    
    
    
    
    int16 ret
    
    string message
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new MoveVelocityResponse(null);
    if (msg.ret !== undefined) {
      resolved.ret = msg.ret;
    }
    else {
      resolved.ret = 0
    }

    if (msg.message !== undefined) {
      resolved.message = msg.message;
    }
    else {
      resolved.message = ''
    }

    return resolved;
    }
};

module.exports = {
  Request: MoveVelocityRequest,
  Response: MoveVelocityResponse,
  md5sum() { return 'ba6e5e4cb25baf01712a273c753becc6'; },
  datatype() { return 'xarm_msgs/MoveVelocity'; }
};
