// Auto-generated. Do not edit!

// (in-package hexapod_training.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class ResultMsg {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.order = null;
      this.reward = null;
      this.actions = null;
    }
    else {
      if (initObj.hasOwnProperty('order')) {
        this.order = initObj.order
      }
      else {
        this.order = 0;
      }
      if (initObj.hasOwnProperty('reward')) {
        this.reward = initObj.reward
      }
      else {
        this.reward = 0.0;
      }
      if (initObj.hasOwnProperty('actions')) {
        this.actions = initObj.actions
      }
      else {
        this.actions = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ResultMsg
    // Serialize message field [order]
    bufferOffset = _serializer.int8(obj.order, buffer, bufferOffset);
    // Serialize message field [reward]
    bufferOffset = _serializer.float64(obj.reward, buffer, bufferOffset);
    // Serialize message field [actions]
    bufferOffset = _arraySerializer.float64(obj.actions, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ResultMsg
    let len;
    let data = new ResultMsg(null);
    // Deserialize message field [order]
    data.order = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [reward]
    data.reward = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [actions]
    data.actions = _arrayDeserializer.float64(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 8 * object.actions.length;
    return length + 13;
  }

  static datatype() {
    // Returns string type for a message object
    return 'hexapod_training/ResultMsg';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '68ca254f3ffbf2a1605af7da24eb91b4';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # ResultMsg.msg
    int8 order
    float64 reward
    float64[] actions
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ResultMsg(null);
    if (msg.order !== undefined) {
      resolved.order = msg.order;
    }
    else {
      resolved.order = 0
    }

    if (msg.reward !== undefined) {
      resolved.reward = msg.reward;
    }
    else {
      resolved.reward = 0.0
    }

    if (msg.actions !== undefined) {
      resolved.actions = msg.actions;
    }
    else {
      resolved.actions = []
    }

    return resolved;
    }
};

module.exports = ResultMsg;
