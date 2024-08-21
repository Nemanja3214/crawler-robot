// Auto-generated. Do not edit!

// (in-package hexapod_training.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let QMatrixElement = require('./QMatrixElement.js');

//-----------------------------------------------------------

class QMatrix {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.elements = null;
    }
    else {
      if (initObj.hasOwnProperty('elements')) {
        this.elements = initObj.elements
      }
      else {
        this.elements = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type QMatrix
    // Serialize message field [elements]
    // Serialize the length for message field [elements]
    bufferOffset = _serializer.uint32(obj.elements.length, buffer, bufferOffset);
    obj.elements.forEach((val) => {
      bufferOffset = QMatrixElement.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type QMatrix
    let len;
    let data = new QMatrix(null);
    // Deserialize message field [elements]
    // Deserialize array length for message field [elements]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.elements = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.elements[i] = QMatrixElement.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    object.elements.forEach((val) => {
      length += QMatrixElement.getMessageSize(val);
    });
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'hexapod_training/QMatrix';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'fdb7bde857788bc279933d9d650b6350';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    #QMatrix.msg
    hexapod_training/QMatrixElement[] elements
    
    ================================================================================
    MSG: hexapod_training/QMatrixElement
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
    const resolved = new QMatrix(null);
    if (msg.elements !== undefined) {
      resolved.elements = new Array(msg.elements.length);
      for (let i = 0; i < resolved.elements.length; ++i) {
        resolved.elements[i] = QMatrixElement.Resolve(msg.elements[i]);
      }
    }
    else {
      resolved.elements = []
    }

    return resolved;
    }
};

module.exports = QMatrix;
