// Auto-generated. Do not edit!

// (in-package ur10_picking.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class vacuum_switchRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.input = null;
    }
    else {
      if (initObj.hasOwnProperty('input')) {
        this.input = initObj.input
      }
      else {
        this.input = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type vacuum_switchRequest
    // Serialize message field [input]
    bufferOffset = _serializer.bool(obj.input, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type vacuum_switchRequest
    let len;
    let data = new vacuum_switchRequest(null);
    // Deserialize message field [input]
    data.input = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'ur10_picking/vacuum_switchRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '2b64ae4a7ed5de74b5f183194512b62f';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool input
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new vacuum_switchRequest(null);
    if (msg.input !== undefined) {
      resolved.input = msg.input;
    }
    else {
      resolved.input = false
    }

    return resolved;
    }
};

class vacuum_switchResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.output = null;
    }
    else {
      if (initObj.hasOwnProperty('output')) {
        this.output = initObj.output
      }
      else {
        this.output = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type vacuum_switchResponse
    // Serialize message field [output]
    bufferOffset = _serializer.string(obj.output, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type vacuum_switchResponse
    let len;
    let data = new vacuum_switchResponse(null);
    // Deserialize message field [output]
    data.output = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.output.length;
    return length + 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'ur10_picking/vacuum_switchResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '0825d95fdfa2c8f4bbb4e9c74bccd3fd';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string output
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new vacuum_switchResponse(null);
    if (msg.output !== undefined) {
      resolved.output = msg.output;
    }
    else {
      resolved.output = ''
    }

    return resolved;
    }
};

module.exports = {
  Request: vacuum_switchRequest,
  Response: vacuum_switchResponse,
  md5sum() { return '2d82a0d3cd91c1b1995bd03ea0dc40fd'; },
  datatype() { return 'ur10_picking/vacuum_switch'; }
};
