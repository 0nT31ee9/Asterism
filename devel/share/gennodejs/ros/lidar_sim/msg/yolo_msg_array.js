// Auto-generated. Do not edit!

// (in-package lidar_sim.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let yolo_msg = require('./yolo_msg.js');

//-----------------------------------------------------------

class yolo_msg_array {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.yolo_array = null;
    }
    else {
      if (initObj.hasOwnProperty('yolo_array')) {
        this.yolo_array = initObj.yolo_array
      }
      else {
        this.yolo_array = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type yolo_msg_array
    // Serialize message field [yolo_array]
    // Serialize the length for message field [yolo_array]
    bufferOffset = _serializer.uint32(obj.yolo_array.length, buffer, bufferOffset);
    obj.yolo_array.forEach((val) => {
      bufferOffset = yolo_msg.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type yolo_msg_array
    let len;
    let data = new yolo_msg_array(null);
    // Deserialize message field [yolo_array]
    // Deserialize array length for message field [yolo_array]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.yolo_array = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.yolo_array[i] = yolo_msg.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    object.yolo_array.forEach((val) => {
      length += yolo_msg.getMessageSize(val);
    });
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'lidar_sim/yolo_msg_array';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'a085b1524497b4e27e72f595045ab303';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    yolo_msg[] yolo_array
    
    
    ================================================================================
    MSG: lidar_sim/yolo_msg
    string label
    int32 x1
    int32 y1
    int32 x2
    int32 y2
    int32 center_x
    int32 center_y
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new yolo_msg_array(null);
    if (msg.yolo_array !== undefined) {
      resolved.yolo_array = new Array(msg.yolo_array.length);
      for (let i = 0; i < resolved.yolo_array.length; ++i) {
        resolved.yolo_array[i] = yolo_msg.Resolve(msg.yolo_array[i]);
      }
    }
    else {
      resolved.yolo_array = []
    }

    return resolved;
    }
};

module.exports = yolo_msg_array;
