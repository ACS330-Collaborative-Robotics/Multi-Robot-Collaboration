// Auto-generated. Do not edit!

// (in-package cpr_robot.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class RobotState {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.Header = null;
      this.Override = null;
      this.StatusFlags = null;
    }
    else {
      if (initObj.hasOwnProperty('Header')) {
        this.Header = initObj.Header
      }
      else {
        this.Header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('Override')) {
        this.Override = initObj.Override
      }
      else {
        this.Override = 0.0;
      }
      if (initObj.hasOwnProperty('StatusFlags')) {
        this.StatusFlags = initObj.StatusFlags
      }
      else {
        this.StatusFlags = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type RobotState
    // Serialize message field [Header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.Header, buffer, bufferOffset);
    // Serialize message field [Override]
    bufferOffset = _serializer.float64(obj.Override, buffer, bufferOffset);
    // Serialize message field [StatusFlags]
    bufferOffset = _serializer.uint32(obj.StatusFlags, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type RobotState
    let len;
    let data = new RobotState(null);
    // Deserialize message field [Header]
    data.Header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [Override]
    data.Override = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [StatusFlags]
    data.StatusFlags = _deserializer.uint32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.Header);
    return length + 12;
  }

  static datatype() {
    // Returns string type for a message object
    return 'cpr_robot/RobotState';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '9f1e4c5a63d074be8d818e2683b635ac';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Header Header
    float64 Override
    uint32 StatusFlags
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
    const resolved = new RobotState(null);
    if (msg.Header !== undefined) {
      resolved.Header = std_msgs.msg.Header.Resolve(msg.Header)
    }
    else {
      resolved.Header = new std_msgs.msg.Header()
    }

    if (msg.Override !== undefined) {
      resolved.Override = msg.Override;
    }
    else {
      resolved.Override = 0.0
    }

    if (msg.StatusFlags !== undefined) {
      resolved.StatusFlags = msg.StatusFlags;
    }
    else {
      resolved.StatusFlags = 0
    }

    return resolved;
    }
};

module.exports = RobotState;
