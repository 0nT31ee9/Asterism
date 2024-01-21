// Auto-generated. Do not edit!

// (in-package lidar_sim.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class shot {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.shot = null;
    }
    else {
      if (initObj.hasOwnProperty('shot')) {
        this.shot = initObj.shot
      }
      else {
        this.shot = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type shot
    // Serialize message field [shot]
    bufferOffset = _serializer.int8(obj.shot, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type shot
    let len;
    let data = new shot(null);
    // Deserialize message field [shot]
    data.shot = _deserializer.int8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a message object
    return 'lidar_sim/shot';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '08639d6a3e086b1cfe56bdf224f3fbc8';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int8 shot
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new shot(null);
    if (msg.shot !== undefined) {
      resolved.shot = msg.shot;
    }
    else {
      resolved.shot = 0
    }

    return resolved;
    }
};

module.exports = shot;
