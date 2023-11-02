// Auto-generated. Do not edit!

// (in-package trans.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class ecu {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.motor = null;
      this.steer = null;
      this.brake = null;
      this.cur_speed = null;
      this.speed_ratio = null;
      this.shift = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('motor')) {
        this.motor = initObj.motor
      }
      else {
        this.motor = 0.0;
      }
      if (initObj.hasOwnProperty('steer')) {
        this.steer = initObj.steer
      }
      else {
        this.steer = 0.0;
      }
      if (initObj.hasOwnProperty('brake')) {
        this.brake = initObj.brake
      }
      else {
        this.brake = false;
      }
      if (initObj.hasOwnProperty('cur_speed')) {
        this.cur_speed = initObj.cur_speed
      }
      else {
        this.cur_speed = 0.0;
      }
      if (initObj.hasOwnProperty('speed_ratio')) {
        this.speed_ratio = initObj.speed_ratio
      }
      else {
        this.speed_ratio = 0.0;
      }
      if (initObj.hasOwnProperty('shift')) {
        this.shift = initObj.shift
      }
      else {
        this.shift = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ecu
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [motor]
    bufferOffset = _serializer.float32(obj.motor, buffer, bufferOffset);
    // Serialize message field [steer]
    bufferOffset = _serializer.float32(obj.steer, buffer, bufferOffset);
    // Serialize message field [brake]
    bufferOffset = _serializer.bool(obj.brake, buffer, bufferOffset);
    // Serialize message field [cur_speed]
    bufferOffset = _serializer.float32(obj.cur_speed, buffer, bufferOffset);
    // Serialize message field [speed_ratio]
    bufferOffset = _serializer.float32(obj.speed_ratio, buffer, bufferOffset);
    // Serialize message field [shift]
    bufferOffset = _serializer.uint8(obj.shift, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ecu
    let len;
    let data = new ecu(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [motor]
    data.motor = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [steer]
    data.steer = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [brake]
    data.brake = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [cur_speed]
    data.cur_speed = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [speed_ratio]
    data.speed_ratio = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [shift]
    data.shift = _deserializer.uint8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 18;
  }

  static datatype() {
    // Returns string type for a message object
    return 'trans/ecu';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '59bf7eba7675d619ecb8c5ad9e66f08b';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    
    float32 motor # 目标速度
    float32 steer # 转向
    bool brake # 紧急停车
    float32 cur_speed # 当前速度
    float32 speed_ratio # 控制速度比率的
    
    uint8 shift # 档位
    uint8 SHIFT_UNKNOWN = 0 
    uint8 SHIFT_D = 1 #前进档位
    uint8 SHIFT_N = 2 #停止档位
    uint8 SHIFT_R = 3 #后退档位
    uint8 SHIFT_T = 9 #遥控
    
    ================================================================================
    MSG: std_msgs/Header
    # Standard metadata for higher-level stamped data types.
    # This is generally used to communicate timestamped data 
    # in a particular coordinate frame.
    # 
    # sequence ID: consecutively increasing ID 
    uint32 seq
    #Two-integer timestamp that is expressed as:
    # * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
    # * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
    # time-handling sugar is provided by the client library
    time stamp
    #Frame this data is associated with
    string frame_id
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ecu(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.motor !== undefined) {
      resolved.motor = msg.motor;
    }
    else {
      resolved.motor = 0.0
    }

    if (msg.steer !== undefined) {
      resolved.steer = msg.steer;
    }
    else {
      resolved.steer = 0.0
    }

    if (msg.brake !== undefined) {
      resolved.brake = msg.brake;
    }
    else {
      resolved.brake = false
    }

    if (msg.cur_speed !== undefined) {
      resolved.cur_speed = msg.cur_speed;
    }
    else {
      resolved.cur_speed = 0.0
    }

    if (msg.speed_ratio !== undefined) {
      resolved.speed_ratio = msg.speed_ratio;
    }
    else {
      resolved.speed_ratio = 0.0
    }

    if (msg.shift !== undefined) {
      resolved.shift = msg.shift;
    }
    else {
      resolved.shift = 0
    }

    return resolved;
    }
};

// Constants for message
ecu.Constants = {
  SHIFT_UNKNOWN: 0,
  SHIFT_D: 1,
  SHIFT_N: 2,
  SHIFT_R: 3,
  SHIFT_T: 9,
}

module.exports = ecu;
