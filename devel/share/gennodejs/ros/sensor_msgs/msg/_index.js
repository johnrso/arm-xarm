
"use strict";

let RelativeHumidity = require('./RelativeHumidity.js');
let RegionOfInterest = require('./RegionOfInterest.js');
let Imu = require('./Imu.js');
let CameraInfo = require('./CameraInfo.js');
let Image = require('./Image.js');
let PointCloud2 = require('./PointCloud2.js');
let JoyFeedbackArray = require('./JoyFeedbackArray.js');
let LaserEcho = require('./LaserEcho.js');
let LaserScan = require('./LaserScan.js');
let MagneticField = require('./MagneticField.js');
let Temperature = require('./Temperature.js');
let JoyFeedback = require('./JoyFeedback.js');
let CompressedImage = require('./CompressedImage.js');
let PointField = require('./PointField.js');
let PointCloud = require('./PointCloud.js');
let NavSatStatus = require('./NavSatStatus.js');
let Illuminance = require('./Illuminance.js');
let Range = require('./Range.js');
let JointState = require('./JointState.js');
let MultiEchoLaserScan = require('./MultiEchoLaserScan.js');
let BatteryState = require('./BatteryState.js');
let Joy = require('./Joy.js');
let ChannelFloat32 = require('./ChannelFloat32.js');
let FluidPressure = require('./FluidPressure.js');
let TimeReference = require('./TimeReference.js');
let NavSatFix = require('./NavSatFix.js');
let MultiDOFJointState = require('./MultiDOFJointState.js');

module.exports = {
  RelativeHumidity: RelativeHumidity,
  RegionOfInterest: RegionOfInterest,
  Imu: Imu,
  CameraInfo: CameraInfo,
  Image: Image,
  PointCloud2: PointCloud2,
  JoyFeedbackArray: JoyFeedbackArray,
  LaserEcho: LaserEcho,
  LaserScan: LaserScan,
  MagneticField: MagneticField,
  Temperature: Temperature,
  JoyFeedback: JoyFeedback,
  CompressedImage: CompressedImage,
  PointField: PointField,
  PointCloud: PointCloud,
  NavSatStatus: NavSatStatus,
  Illuminance: Illuminance,
  Range: Range,
  JointState: JointState,
  MultiEchoLaserScan: MultiEchoLaserScan,
  BatteryState: BatteryState,
  Joy: Joy,
  ChannelFloat32: ChannelFloat32,
  FluidPressure: FluidPressure,
  TimeReference: TimeReference,
  NavSatFix: NavSatFix,
  MultiDOFJointState: MultiDOFJointState,
};
