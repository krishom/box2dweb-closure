goog.provide('box2d.dynamics.joints.RopeJointDef');

goog.require('box2d.dynamics.joints.JointDef');
goog.require('box2d.dynamics.joints.RopeJoint');

/**
 * @constructor
 * @extends {box2d.dynamics.joints.JointDef}
 * @param {!box2d.dynamics.Body} bA
 * @param {!box2d.dynamics.Body} bB
 * @param {!box2d.common.math.Vec2} anchorB
 * @param {!box2d.common.math.Vec2} anchorA
 * @param {!number} maxlength
 */
box2d.dynamics.joints.RopeJointDef = function(bA, bB, anchorA, anchorB, maxlength) {
    goog.base(this);

    var dX = anchorB.x - anchorA.x,
        dY = anchorB.y - anchorA.y;

    this.type = box2d.dynamics.joints.Joint.e_ropeJoint;
    this.bodyA = bA;
    this.bodyB = bB;

    /**
     * @type {!box2d.common.math.Vec2}
     */
    this.localAnchorA = bA.getLocalPoint(anchorA);

    /**
     * @type {!box2d.common.math.Vec2}
     */
    this.localAnchorB = bA.getLocalPoint(anchorA);

    /**
     * @type {number}
     */
    this.maxlength = maxlength;

    /**
     * @type {number}
     */
    this.length = Math.sqrt(dX * dX + dY * dY);
};
goog.inherits(box2d.dynamics.joints.RopeJointDef, box2d.dynamics.joints.JointDef);

box2d.dynamics.joints.RopeJointDef.prototype.create = function() {
    return new box2d.dynamics.joints.RopeJoint(this);
};
