/*
 * Copyright (c) 2006-2007 Erin Catto http://www.gphysics.com
 *
 * This software is provided 'as-is', without any express or implied
 * warranty.  In no event will the authors be held liable for any damages
 * arising from the use of this software.
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 * 1. The origin of this software must not be misrepresented; you must not
 * claim that you wrote the original software. If you use this software
 * in a product, an acknowledgment in the product documentation would be
 * appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 * misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 *
 * Ported to AS3 by Allan Bishop http://allanbishop.com
 * Ported to google closure box2d by Mads Jon Nielsen
 */

goog.provide('box2d.dynamics.joints.RopeJointDef');

goog.require('box2d.dynamics.joints.JointDef');
goog.require('box2d.dynamics.joints.RopeJoint');

/**
 * @constructor
 * @extends {box2d.dynamics.joints.JointDef}
 * @param {!box2d.dynamics.Body} bA Body a.
 * @param {!box2d.dynamics.Body} bB Body b.
 * @param {!box2d.common.math.Vec2} anchorA Anchor a.
 * @param {!box2d.common.math.Vec2} anchorB Anchor b.
 * @param {!number} maxlength Max length of the rope.
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

/**
 * Creates a rope joint.
 * @return {box2d.dynamics.joints.RopeJoint} A rope joint.
 */
box2d.dynamics.joints.RopeJointDef.prototype.create = function() {
    return new box2d.dynamics.joints.RopeJoint(this);
};
