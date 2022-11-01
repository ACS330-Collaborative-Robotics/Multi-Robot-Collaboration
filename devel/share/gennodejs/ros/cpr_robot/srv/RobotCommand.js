// Auto-generated. Do not edit!

// (in-package cpr_robot.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class RobotCommandRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.Sender = null;
      this.CommandId = null;
      this.PayloadFloat = null;
      this.PayloadInt = null;
    }
    else {
      if (initObj.hasOwnProperty('Sender')) {
        this.Sender = initObj.Sender
      }
      else {
        this.Sender = '';
      }
      if (initObj.hasOwnProperty('CommandId')) {
        this.CommandId = initObj.CommandId
      }
      else {
        this.CommandId = 0;
      }
      if (initObj.hasOwnProperty('PayloadFloat')) {
        this.PayloadFloat = initObj.PayloadFloat
      }
      else {
        this.PayloadFloat = 0.0;
      }
      if (initObj.hasOwnProperty('PayloadInt')) {
        this.PayloadInt = initObj.PayloadInt
      }
      else {
        this.PayloadInt = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type RobotCommandRequest
    // Serialize message field [Sender]
    bufferOffset = _serializer.string(obj.Sender, buffer, bufferOffset);
    // Serialize message field [CommandId]
    bufferOffset = _serializer.uint32(obj.CommandId, buffer, bufferOffset);
    // Serialize message field [PayloadFloat]
    bufferOffset = _serializer.float64(obj.PayloadFloat, buffer, bufferOffset);
    // Serialize message field [PayloadInt]
    bufferOffset = _serializer.int64(obj.PayloadInt, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type RobotCommandRequest
    let len;
    let data = new RobotCommandRequest(null);
    // Deserialize message field [Sender]
    data.Sender = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [CommandId]
    data.CommandId = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [PayloadFloat]
    data.PayloadFloat = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [PayloadInt]
    data.PayloadInt = _deserializer.int64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.Sender);
    return length + 24;
  }

  static datatype() {
    // Returns string type for a service object
    return 'cpr_robot/RobotCommandRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'b4ae46840bac87549d3c8ef9dbcd298d';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string Sender
    uint32 CommandId
    float64 PayloadFloat
    int64 PayloadInt
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new RobotCommandRequest(null);
    if (msg.Sender !== undefined) {
      resolved.Sender = msg.Sender;
    }
    else {
      resolved.Sender = ''
    }

    if (msg.CommandId !== undefined) {
      resolved.CommandId = msg.CommandId;
    }
    else {
      resolved.CommandId = 0
    }

    if (msg.PayloadFloat !== undefined) {
      resolved.PayloadFloat = msg.PayloadFloat;
    }
    else {
      resolved.PayloadFloat = 0.0
    }

    if (msg.PayloadInt !== undefined) {
      resolved.PayloadInt = msg.PayloadInt;
    }
    else {
      resolved.PayloadInt = 0
    }

    return resolved;
    }
};

class RobotCommandResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type RobotCommandResponse
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type RobotCommandResponse
    let len;
    let data = new RobotCommandResponse(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'cpr_robot/RobotCommandResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd41d8cd98f00b204e9800998ecf8427e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new RobotCommandResponse(null);
    return resolved;
    }
};

module.exports = {
  Request: RobotCommandRequest,
  Response: RobotCommandResponse,
  md5sum() { return 'b4ae46840bac87549d3c8ef9dbcd298d'; },
  datatype() { return 'cpr_robot/RobotCommand'; }
};
