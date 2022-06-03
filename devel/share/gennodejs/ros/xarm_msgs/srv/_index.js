
"use strict";

let GetControllerDigitalIO = require('./GetControllerDigitalIO.js')
let GripperConfig = require('./GripperConfig.js')
let SetAxis = require('./SetAxis.js')
let SetDigitalIO = require('./SetDigitalIO.js')
let Move = require('./Move.js')
let SetMultipleInts = require('./SetMultipleInts.js')
let SetToolModbus = require('./SetToolModbus.js')
let MoveVelo = require('./MoveVelo.js')
let MoveVelocity = require('./MoveVelocity.js')
let PlayTraj = require('./PlayTraj.js')
let ConfigToolModbus = require('./ConfigToolModbus.js')
let GripperMove = require('./GripperMove.js')
let SetControllerAnalogIO = require('./SetControllerAnalogIO.js')
let GetFloat32List = require('./GetFloat32List.js')
let SetString = require('./SetString.js')
let GetInt32 = require('./GetInt32.js')
let TCPOffset = require('./TCPOffset.js')
let GetAnalogIO = require('./GetAnalogIO.js')
let ClearErr = require('./ClearErr.js')
let GetDigitalIO = require('./GetDigitalIO.js')
let GetErr = require('./GetErr.js')
let SetLoad = require('./SetLoad.js')
let GripperState = require('./GripperState.js')
let MoveAxisAngle = require('./MoveAxisAngle.js')
let SetFloat32 = require('./SetFloat32.js')
let SetInt16 = require('./SetInt16.js')

module.exports = {
  GetControllerDigitalIO: GetControllerDigitalIO,
  GripperConfig: GripperConfig,
  SetAxis: SetAxis,
  SetDigitalIO: SetDigitalIO,
  Move: Move,
  SetMultipleInts: SetMultipleInts,
  SetToolModbus: SetToolModbus,
  MoveVelo: MoveVelo,
  MoveVelocity: MoveVelocity,
  PlayTraj: PlayTraj,
  ConfigToolModbus: ConfigToolModbus,
  GripperMove: GripperMove,
  SetControllerAnalogIO: SetControllerAnalogIO,
  GetFloat32List: GetFloat32List,
  SetString: SetString,
  GetInt32: GetInt32,
  TCPOffset: TCPOffset,
  GetAnalogIO: GetAnalogIO,
  ClearErr: ClearErr,
  GetDigitalIO: GetDigitalIO,
  GetErr: GetErr,
  SetLoad: SetLoad,
  GripperState: GripperState,
  MoveAxisAngle: MoveAxisAngle,
  SetFloat32: SetFloat32,
  SetInt16: SetInt16,
};
