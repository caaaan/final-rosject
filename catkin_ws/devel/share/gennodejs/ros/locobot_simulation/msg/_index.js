
"use strict";

let Model = require('./Model.js');
let DetectedObjects = require('./DetectedObjects.js');
let BoundingBoxes = require('./BoundingBoxes.js');
let BoundingBox = require('./BoundingBox.js');
let LogicalImage = require('./LogicalImage.js');
let DetectedObject = require('./DetectedObject.js');

module.exports = {
  Model: Model,
  DetectedObjects: DetectedObjects,
  BoundingBoxes: BoundingBoxes,
  BoundingBox: BoundingBox,
  LogicalImage: LogicalImage,
  DetectedObject: DetectedObject,
};
