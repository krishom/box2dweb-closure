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

goog.provide('box2d.dynamics.joints.Joint');

goog.require('box2d.common.Settings');
goog.require('box2d.common.math.Math');
goog.require('box2d.common.math.Vec2');
goog.require('box2d.dynamics.joints.JointEdge');

/**
 * @param {!box2d.dynamics.joints.JointDef} def
 * @constructor
 */
box2d.dynamics.joints.Joint = function(def) {
    this.m_edgeA = new box2d.dynamics.joints.JointEdge();
    this.m_edgeB = new box2d.dynamics.joints.JointEdge();
    this.m_localCenterA = box2d.common.math.Vec2.get(0, 0);
    this.m_localCenterB = box2d.common.math.Vec2.get(0, 0);
    box2d.common.Settings.assert(def.bodyA != def.bodyB);
    this.m_type = def.type;
    this.m_prev = null;
    this.m_next = null;
    this.m_bodyA = def.bodyA;
    this.m_bodyB = def.bodyB;
    this.m_collideConnected = def.collideConnected;
};

box2d.dynamics.joints.Joint.prototype.getType = function() {
    return this.m_type;
};

box2d.dynamics.joints.Joint.prototype.getAnchorA = function() {
    return null;
};

box2d.dynamics.joints.Joint.prototype.getAnchorB = function() {
    return null;
};

box2d.dynamics.joints.Joint.prototype.getReactionForce = function(inv_dt) {
    if (inv_dt === undefined) inv_dt = 0;
    return null;
};

box2d.dynamics.joints.Joint.prototype.getReactionTorque = function(inv_dt) {
    if (inv_dt === undefined) inv_dt = 0;
    return 0.0;
};

box2d.dynamics.joints.Joint.prototype.getBodyA = function() {
    return this.m_bodyA;
};

box2d.dynamics.joints.Joint.prototype.getBodyB = function() {
    return this.m_bodyB;
};

box2d.dynamics.joints.Joint.prototype.getNext = function() {
    return this.m_next;
};

box2d.dynamics.joints.Joint.prototype.isActive = function() {
    return this.m_bodyA.isActive() && this.m_bodyB.isActive();
};

box2d.dynamics.joints.Joint.create = function(def) {
    return def.create();
};

box2d.dynamics.joints.Joint.prototype.initVelocityConstraints = function(step) {
};

box2d.dynamics.joints.Joint.prototype.solveVelocityConstraints = function(step) {
};

box2d.dynamics.joints.Joint.prototype.finalizeVelocityConstraints = function() {
};

box2d.dynamics.joints.Joint.prototype.solvePositionConstraints = function(step) {
    return false;
};

box2d.dynamics.joints.Joint.e_unknownJoint = 0;
box2d.dynamics.joints.Joint.e_revoluteJoint = 1;
box2d.dynamics.joints.Joint.e_prismaticJoint = 2;
box2d.dynamics.joints.Joint.e_distanceJoint = 3;
box2d.dynamics.joints.Joint.e_pulleyJoint = 4;
box2d.dynamics.joints.Joint.e_mouseJoint = 5;
box2d.dynamics.joints.Joint.e_gearJoint = 6;
box2d.dynamics.joints.Joint.e_lineJoint = 7;
box2d.dynamics.joints.Joint.e_weldJoint = 8;
box2d.dynamics.joints.Joint.e_ropeJoint = 9;
box2d.dynamics.joints.Joint.e_frictionJoint = 9;
box2d.dynamics.joints.Joint.e_inactiveLimit = 0;
box2d.dynamics.joints.Joint.e_atLowerLimit = 1;
box2d.dynamics.joints.Joint.e_atUpperLimit = 2;
box2d.dynamics.joints.Joint.e_equalLimits = 3;
