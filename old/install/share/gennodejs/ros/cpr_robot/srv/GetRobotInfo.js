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

class GetRobotInfoRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.Sender = null;
    }
    else {
      if (initObj.hasOwnProperty('Sender')) {
        this.Sender = initObj.Sender
      }
      else {
        this.Sender = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetRobotInfoRequest
    // Serialize message field [Sender]
    bufferOffset = _serializer.string(obj.Sender, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetRobotInfoRequest
    let len;
    let data = new GetRobotInfoRequest(null);
    // Deserialize message field [Sender]
    data.Sender = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.Sender);
    return length + 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'cpr_robot/GetRobotInfoRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'bf661c295c93387e71dbd70441d1cc44';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string Sender
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetRobotInfoRequest(null);
    if (msg.Sender !== undefined) {
      resolved.Sender = msg.Sender;
    }
    else {
      resolved.Sender = ''
    }

    return resolved;
    }
};

class GetRobotInfoResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.Model = null;
      this.CountJoints = null;
      this.InputChannels = null;
      this.OutputChannels = null;
    }
    else {
      if (initObj.hasOwnProperty('Model')) {
        this.Model = initObj.Model
      }
      else {
        this.Model = '';
      }
      if (initObj.hasOwnProperty('CountJoints')) {
        this.CountJoints = initObj.CountJoints
      }
      else {
        this.CountJoints = 0;
      }
      if (initObj.hasOwnProperty('InputChannels')) {
        this.InputChannels = initObj.InputChannels
      }
      else {
        this.InputChannels = [];
      }
      if (initObj.hasOwnProperty('OutputChannels')) {
        this.OutputChannels = initObj.OutputChannels
      }
      else {
        this.OutputChannels = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetRobotInfoResponse
    // Serialize message field [Model]
    bufferOffset = _serializer.string(obj.Model, buffer, bufferOffset);
    // Serialize message field [CountJoints]
    bufferOffset = _serializer.uint32(obj.CountJoints, buffer, bufferOffset);
    // Serialize message field [InputChannels]
    bufferOffset = _arraySerializer.string(obj.InputChannels, buffer, bufferOffset, null);
    // Serialize message field [OutputChannels]
    bufferOffset = _arraySerializer.string(obj.OutputChannels, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetRobotInfoResponse
    let len;
    let data = new GetRobotInfoResponse(null);
    // Deserialize message field [Model]
    data.Model = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [CountJoints]
    data.CountJoints = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [InputChannels]
    data.InputChannels = _arrayDeserializer.string(buffer, bufferOffset, null)
    // Deserialize message field [OutputChannels]
    data.OutputChannels = _arrayDeserializer.string(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.Model);
    object.InputChannels.forEach((val) => {
      length += 4 + _getByteLength(val);
    });
    object.OutputChannels.forEach((val) => {
      length += 4 + _getByteLength(val);
    });
    return length + 16;
  }

  static datatype() {
    // Returns string type for a service object
    return 'cpr_robot/GetRobotInfoResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '06ada5bff31d9ffb1d84dbd4ca91405e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string Model
    uint32 CountJoints
    string[] InputChannels
    string[] OutputChannels
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetRobotInfoResponse(null);
    if (msg.Model !== undefined) {
      resolved.Model = msg.Model;
    }
    else {
      resolved.Model = ''
    }

    if (msg.CountJoints !== undefined) {
      resolved.CountJoints = msg.CountJoints;
    }
    else {
      resolved.CountJoints = 0
    }

    if (msg.InputChannels !== undefined) {
      resolved.InputChannels = msg.InputChannels;
    }
    else {
      resolved.InputChannels = []
    }

    if (msg.OutputChannels !== undefined) {
      resolved.OutputChannels = msg.OutputChannels;
    }
    else {
      resolved.OutputChannels = []
    }

    return resolved;
    }
};

module.exports = {
  Request: GetRobotInfoRequest,
  Response: GetRobotInfoResponse,
  md5sum() { return 'e575ccf5296d82bf7ce2a23f1cc16a78'; },
  datatype() { return 'cpr_robot/GetRobotInfo'; }
};
