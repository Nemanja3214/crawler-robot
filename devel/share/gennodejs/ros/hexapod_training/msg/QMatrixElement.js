// Auto-generated. Do not edit!

// (in-package hexapod_training.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let StateActionPair = require('./StateActionPair.js');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class QMatrixElement {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.pair = null;
      this.reward = null;
    }
    else {
      if (initObj.hasOwnProperty('pair')) {
        this.pair = initObj.pair
      }
      else {
        this.pair = new StateActionPair();
      }
      if (initObj.hasOwnProperty('reward')) {
        this.reward = initObj.reward
      }
      else {
        this.reward = new std_msgs.msg.Float64();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type QMatrixElement
    // Serialize message field [pair]
    bufferOffset = StateActionPair.serialize(obj.pair, buffer, bufferOffset);
    // Serialize message field [reward]
    bufferOffset = std_msgs.msg.Float64.serialize(obj.reward, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type QMatrixElement
    let len;
    let data = new QMatrixElement(null);
    // Deserialize message field [pair]
    data.pair = StateActionPair.deserialize(buffer, bufferOffset);
    // Deserialize message field [reward]
    data.reward = std_msgs.msg.Float64.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += StateActionPair.getMessageSize(object.pair);
    return length + 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'hexapod_training/QMatrixElement';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '39b6677d577d14397b6281d04d889aab';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # QMatrixElement.msg
    hexapod_training/StateActionPair pair
    std_msgs/Float64 reward
    
    ================================================================================
    MSG: hexapod_training/StateActionPair
    # StateActionPair.msg
    string state
    int32 action
    ================================================================================
    MSG: std_msgs/Float64
    float64 data
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new QMatrixElement(null);
    if (msg.pair !== undefined) {
      resolved.pair = StateActionPair.Resolve(msg.pair)
    }
    else {
      resolved.pair = new StateActionPair()
    }

    if (msg.reward !== undefined) {
      resolved.reward = std_msgs.msg.Float64.Resolve(msg.reward)
    }
    else {
      resolved.reward = new std_msgs.msg.Float64()
    }

    return resolved;
    }
};

module.exports = QMatrixElement;
