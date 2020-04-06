// Auto-generated. Do not edit!

// (in-package dataspeed_can_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class CanMessage {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.data = null;
      this.id = null;
      this.extended = null;
      this.dlc = null;
    }
    else {
      if (initObj.hasOwnProperty('data')) {
        this.data = initObj.data
      }
      else {
        this.data = new Array(8).fill(0);
      }
      if (initObj.hasOwnProperty('id')) {
        this.id = initObj.id
      }
      else {
        this.id = 0;
      }
      if (initObj.hasOwnProperty('extended')) {
        this.extended = initObj.extended
      }
      else {
        this.extended = false;
      }
      if (initObj.hasOwnProperty('dlc')) {
        this.dlc = initObj.dlc
      }
      else {
        this.dlc = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type CanMessage
    // Check that the constant length array field [data] has the right length
    if (obj.data.length !== 8) {
      throw new Error('Unable to serialize array field data - length must be 8')
    }
    // Serialize message field [data]
    bufferOffset = _arraySerializer.uint8(obj.data, buffer, bufferOffset, 8);
    // Serialize message field [id]
    bufferOffset = _serializer.uint32(obj.id, buffer, bufferOffset);
    // Serialize message field [extended]
    bufferOffset = _serializer.bool(obj.extended, buffer, bufferOffset);
    // Serialize message field [dlc]
    bufferOffset = _serializer.uint8(obj.dlc, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type CanMessage
    let len;
    let data = new CanMessage(null);
    // Deserialize message field [data]
    data.data = _arrayDeserializer.uint8(buffer, bufferOffset, 8)
    // Deserialize message field [id]
    data.id = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [extended]
    data.extended = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [dlc]
    data.dlc = _deserializer.uint8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 14;
  }

  static datatype() {
    // Returns string type for a message object
    return 'dataspeed_can_msgs/CanMessage';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '2866d426ed29ed9fab7f393d3ece69b0';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    uint8[8] data
    uint32 id
    bool extended
    uint8 dlc
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new CanMessage(null);
    if (msg.data !== undefined) {
      resolved.data = msg.data;
    }
    else {
      resolved.data = new Array(8).fill(0)
    }

    if (msg.id !== undefined) {
      resolved.id = msg.id;
    }
    else {
      resolved.id = 0
    }

    if (msg.extended !== undefined) {
      resolved.extended = msg.extended;
    }
    else {
      resolved.extended = false
    }

    if (msg.dlc !== undefined) {
      resolved.dlc = msg.dlc;
    }
    else {
      resolved.dlc = 0
    }

    return resolved;
    }
};

module.exports = CanMessage;
