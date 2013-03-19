/*
 * copyright (c) 2006-2007 Erin Catto http://www.gphysics.com
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
 */
/*
 * Original Box2D created by Erin Catto
 * http://www.gphysics.com
 * http://box2d.org/
 *
 * Box2D was converted to Flash by Boris the Brave, Matt Bush, and John Nesky as Box2DFlash
 * http://www.box2dflash.org/
 *
 * Box2DFlash was converted from Flash to Javascript by Uli Hecht as box2Dweb
 * http://code.google.com/p/box2dweb/
 *
 * box2Dweb was modified to utilize Google Closure, as well as other bug fixes, optimizations, and tweaks by Illandril
 * https://github.com/illandril/box2dweb-closure
 */

goog.provide('box2d.dynamics.joints.PulleyJointDef');

goog.require('box2d.common.math.Vec2');
goog.require('box2d.dynamics.joints.Joint');
goog.require('box2d.dynamics.joints.JointDef');
goog.require('box2d.dynamics.joints.PulleyJoint');

/**
 * @constructor
 * @extends {box2d.dynamics.joints.JointDef}
 */
box2d.dynamics.joints.PulleyJointDef = function() {
    goog.base(this);

    this.groundAnchorA = box2d.common.math.Vec2.get(0, 0);
    this.groundAnchorB = box2d.common.math.Vec2.get(0, 0);
    this.localAnchorA = box2d.common.math.Vec2.get(0, 0);
    this.localAnchorB = box2d.common.math.Vec2.get(0, 0);
    this.type = box2d.dynamics.joints.Joint.e_pulleyJoint;
    this.groundAnchorA.set((-1.0), 1.0);
    this.groundAnchorB.set(1.0, 1.0);
    this.localAnchorA.set((-1.0), 0.0);
    this.localAnchorB.set(1.0, 0.0);
    this.lengthA = 0.0;
    this.maxlengthA = 0.0;
    this.lengthB = 0.0;
    this.maxlengthB = 0.0;
    this.ratio = 1.0;
    this.collideConnected = true;
};
goog.inherits(box2d.dynamics.joints.PulleyJointDef, box2d.dynamics.joints.JointDef);

box2d.dynamics.joints.PulleyJointDef.prototype.initialize = function(bA, bB, gaA, gaB, anchorA, anchorB, r) {
    if (r === undefined) r = 0;
    this.bodyA = bA;
    this.bodyB = bB;
    this.groundAnchorA.setV(gaA);
    this.groundAnchorB.setV(gaB);
    this.localAnchorA = this.bodyA.getLocalPoint(anchorA);
    this.localAnchorB = this.bodyB.getLocalPoint(anchorB);
    var d1X = anchorA.x - gaA.x;
    var d1Y = anchorA.y - gaA.y;
    this.lengthA = Math.sqrt(d1X * d1X + d1Y * d1Y);
    var d2X = anchorB.x - gaB.x;
    var d2Y = anchorB.y - gaB.y;
    this.lengthB = Math.sqrt(d2X * d2X + d2Y * d2Y);
    this.ratio = r;
    var C = this.lengthA + this.ratio * this.lengthB;
    this.maxlengthA = C - this.ratio * box2d.dynamics.joints.PulleyJoint.minPulleylength;
    this.maxlengthB = (C - box2d.dynamics.joints.PulleyJoint.minPulleylength) / this.ratio;
};

box2d.dynamics.joints.PulleyJointDef.prototype.create = function() {
    return new box2d.dynamics.joints.PulleyJoint(this);
};
