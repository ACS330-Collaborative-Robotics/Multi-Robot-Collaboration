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

class GetJointInfoRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.Sender = null;
      this.JointId = null;
    }
    else {
      if (initObj.hasOwnProperty('Sender')) {
        this.Sender = initObj.Sender
      }
      else {
        this.Sender = '';
      }
      if (initObj.hasOwnProperty('JointId')) {
        this.JointId = initObj.JointId
      }
      else {
        this.JointId = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetJointInfoRequest
    // Serialize message field [Sender]
    bufferOffset = _serializer.string(obj.Sender, buffer, bufferOffset);
    // Serialize message field [JointId]
    bufferOffset = _serializer.uint32(obj.JointId, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetJointInfoRequest
    let len;
    let data = new GetJointInfoRequest(null);
    // Deserialize message field [Sender]
    data.Sender = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [JointId]
    data.JointId = _deserializer.uint32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.Sender);
    return length + 8;
  }

  static datatype() {
    // Returns string type for a service object
    return 'cpr_robot/GetJointInfoRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '41358db89ee0b7be7f8dd1bad552b58e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string Sender
    uint32 JointId
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetJointInfoRequest(null);
    if (msg.Sender !== undefined) {
      resolved.Sender = msg.Sender;
    }
    else {
      resolved.Sender = ''
    }

    if (msg.JointId !== undefined) {
      resolved.JointId = msg.JointId;
    }
    else {
      resolved.JointId = 0
    }

    return resolved;
    }
};

class GetJointInfoResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.JointName = null;
      this.JointType = null;
    }
    else {
      if (initObj.hasOwnProperty('JointName')) {
        this.JointName = initObj.JointName
      }
      else {
        this.JointName = '';
      }
      if (initObj.hasOwnProperty('JointType')) {
        this.JointType = initObj.JointType
      }
      else {
        this.JointType = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetJointInfoResponse
    // Serialize message field [JointName]
    bufferOffset = _serializer.string(obj.JointName, buffer, bufferOffset);
    // Serialize message field [JointType]
    bufferOffset = _serializer.uint32(obj.JointType, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetJointInfoResponse
    let len;
    let data = new GetJointInfoResponse(null);
    // Deserialize message field [JointName]
    data.JointName = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [JointType]
    data.JointType = _deserializer.uint32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.JointName);
    return length + 8;
  }

  static datatype() {
    // Returns string type for a service object
    return 'cpr_robot/GetJointInfoResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '2e062f5953b81fb4b736cd37870663cd';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string JointName
    uint32 JointType
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetJointInfoResponse(null);
    if (msg.JointName !== undefined) {
      resolved.JointName = msg.JointName;
    }
    else {
      resolved.JointName = ''
    }

    if (msg.JointType !== undefined) {
      resolved.JointType = msg.JointType;
    }
    else {
      resolved.JointType = 0
    }

    return resolved;
    }
};

module.exports = {
  Request: GetJointInfoRequest,
  Response: GetJointInfoResponse,
  md5sum() { return '06bfa666ccf7ae403dfc3460222a104e'; },
  datatype() { return 'cpr_robot/GetJointInfo'; }
};
