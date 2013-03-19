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

goog.provide('box2d.dynamics.joints.DistanceJointDef');

goog.require('box2d.common.math.Vec2');
goog.require('box2d.dynamics.joints.DistanceJoint');
goog.require('box2d.dynamics.joints.Joint');
goog.require('box2d.dynamics.joints.JointDef');

/**
 * @constructor
 * @extends {box2d.dynamics.joints.JointDef}
 */
box2d.dynamics.joints.DistanceJointDef = function() {
    goog.base(this);

    this.localAnchorA = box2d.common.math.Vec2.get(0, 0);
    this.localAnchorB = box2d.common.math.Vec2.get(0, 0);
    this.type = box2d.dynamics.joints.Joint.e_distanceJoint;
    this.length = 1.0;
    this.frequencyHz = 0.0;
    this.dampingRatio = 0.0;
};
goog.inherits(box2d.dynamics.joints.DistanceJointDef, box2d.dynamics.joints.JointDef);

box2d.dynamics.joints.DistanceJointDef.prototype.initialize = function(bA, bB, anchorA, anchorB) {
    this.bodyA = bA;
    this.bodyB = bB;
    this.localAnchorA.setV(this.bodyA.getLocalPoint(anchorA));
    this.localAnchorB.setV(this.bodyB.getLocalPoint(anchorB));
    var dX = anchorB.x - anchorA.x;
    var dY = anchorB.y - anchorA.y;
    this.length = Math.sqrt(dX * dX + dY * dY);
    this.frequencyHz = 0.0;
    this.dampingRatio = 0.0;
};

box2d.dynamics.joints.DistanceJointDef.prototype.create = function() {
    return new box2d.dynamics.joints.DistanceJoint(this);
};
