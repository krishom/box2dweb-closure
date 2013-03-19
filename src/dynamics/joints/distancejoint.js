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

goog.provide('box2d.dynamics.joints.DistanceJoint');

goog.require('box2d.common.Settings');
goog.require('box2d.common.math.Math');
goog.require('box2d.common.math.Vec2');
goog.require('box2d.dynamics.joints.Joint');

/**
 * @param {!box2d.dynamics.joints.DistanceJointDef} def
 * @constructor
 * @extends {box2d.dynamics.joints.Joint}
 */
box2d.dynamics.joints.DistanceJoint = function(def) {
    goog.base(this, def);

    this.m_localAnchor1 = box2d.common.math.Vec2.get(0, 0);
    this.m_localAnchor2 = box2d.common.math.Vec2.get(0, 0);
    this.m_u = box2d.common.math.Vec2.get(0, 0);
    this.m_localAnchor1.setV(def.localAnchorA);
    this.m_localAnchor2.setV(def.localAnchorB);
    this.m_length = def.length;
    this.m_frequencyHz = def.frequencyHz;
    this.m_dampingRatio = def.dampingRatio;
    this.m_impulse = 0.0;
    this.m_gamma = 0.0;
    this.m_bias = 0.0;
};
goog.inherits(box2d.dynamics.joints.DistanceJoint, box2d.dynamics.joints.Joint);

box2d.dynamics.joints.DistanceJoint.prototype.getAnchorA = function() {
    return this.m_bodyA.getWorldPoint(this.m_localAnchor1);
};

box2d.dynamics.joints.DistanceJoint.prototype.getAnchorB = function() {
    return this.m_bodyB.getWorldPoint(this.m_localAnchor2);
};

/**
 * @param {number} inv_dt
 */
box2d.dynamics.joints.DistanceJoint.prototype.getReactionForce = function(inv_dt) {
    return box2d.common.math.Vec2.get(inv_dt * this.m_impulse * this.m_u.x, inv_dt * this.m_impulse * this.m_u.y);
};

/**
 * @param {number} inv_dt
 */
box2d.dynamics.joints.DistanceJoint.prototype.getReactionTorque = function(inv_dt) {
    return 0.0;
};

box2d.dynamics.joints.DistanceJoint.prototype.getLength = function() {
    return this.m_length;
};

/**
 * @param {number} length
 */
box2d.dynamics.joints.DistanceJoint.prototype.setlength = function(length) {
    this.m_length = length;
};

box2d.dynamics.joints.DistanceJoint.prototype.getFrequency = function() {
    return this.m_frequencyHz;
};

/**
 * @param {number} hz
 */
box2d.dynamics.joints.DistanceJoint.prototype.setFrequency = function(hz) {
    this.m_frequencyHz = hz;
};

box2d.dynamics.joints.DistanceJoint.prototype.getDampingRatio = function() {
    return this.m_dampingRatio;
};

/**
 * @param {number} ratio
 */
box2d.dynamics.joints.DistanceJoint.prototype.setDampingRatio = function(ratio) {
    this.m_dampingRatio = ratio;
};

box2d.dynamics.joints.DistanceJoint.prototype.initVelocityConstraints = function(step) {
    var tMat;
    var tX = 0;
    var bA = this.m_bodyA;
    var bB = this.m_bodyB;
    tMat = bA.m_xf.R;
    var r1X = this.m_localAnchor1.x - bA.m_sweep.localCenter.x;
    var r1Y = this.m_localAnchor1.y - bA.m_sweep.localCenter.y;
    tX = (tMat.col1.x * r1X + tMat.col2.x * r1Y);
    r1Y = (tMat.col1.y * r1X + tMat.col2.y * r1Y);
    r1X = tX;
    tMat = bB.m_xf.R;
    var r2X = this.m_localAnchor2.x - bB.m_sweep.localCenter.x;
    var r2Y = this.m_localAnchor2.y - bB.m_sweep.localCenter.y;
    tX = (tMat.col1.x * r2X + tMat.col2.x * r2Y);
    r2Y = (tMat.col1.y * r2X + tMat.col2.y * r2Y);
    r2X = tX;
    this.m_u.x = bB.m_sweep.c.x + r2X - bA.m_sweep.c.x - r1X;
    this.m_u.y = bB.m_sweep.c.y + r2Y - bA.m_sweep.c.y - r1Y;
    var length = Math.sqrt(this.m_u.x * this.m_u.x + this.m_u.y * this.m_u.y);
    if (length > box2d.common.Settings.linearSlop) {
        this.m_u.multiply(1.0 / length);
    } else {
        this.m_u.setZero();
    }
    var cr1u = (r1X * this.m_u.y - r1Y * this.m_u.x);
    var cr2u = (r2X * this.m_u.y - r2Y * this.m_u.x);
    var invMass = bA.m_invMass + bA.m_invI * cr1u * cr1u + bB.m_invMass + bB.m_invI * cr2u * cr2u;
    this.m_mass = invMass != 0.0 ? 1.0 / invMass : 0.0;
    if (this.m_frequencyHz > 0.0) {
        var C = length - this.m_length;
        var omega = 2.0 * Math.PI * this.m_frequencyHz;
        var d = 2.0 * this.m_mass * this.m_dampingRatio * omega;
        var k = this.m_mass * omega * omega;
        this.m_gamma = step.dt * (d + step.dt * k);
        this.m_gamma = this.m_gamma != 0.0 ? 1 / this.m_gamma : 0.0;
        this.m_bias = C * step.dt * k * this.m_gamma;
        this.m_mass = invMass + this.m_gamma;
        this.m_mass = this.m_mass != 0.0 ? 1.0 / this.m_mass : 0.0;
    }
    if (step.warmStarting) {
        this.m_impulse *= step.dtRatio;
        var PX = this.m_impulse * this.m_u.x;
        var PY = this.m_impulse * this.m_u.y;
        bA.m_linearVelocity.x -= bA.m_invMass * PX;
        bA.m_linearVelocity.y -= bA.m_invMass * PY;
        bA.m_angularVelocity -= bA.m_invI * (r1X * PY - r1Y * PX);
        bB.m_linearVelocity.x += bB.m_invMass * PX;
        bB.m_linearVelocity.y += bB.m_invMass * PY;
        bB.m_angularVelocity += bB.m_invI * (r2X * PY - r2Y * PX);
    } else {
        this.m_impulse = 0.0;
    }
};

box2d.dynamics.joints.DistanceJoint.prototype.solveVelocityConstraints = function(step) {
    var r1X = this.m_localAnchor1.x - this.m_bodyA.m_sweep.localCenter.x;
    var r1Y = this.m_localAnchor1.y - this.m_bodyA.m_sweep.localCenter.y;
    var tX = (this.m_bodyA.m_xf.R.col1.x * r1X + this.m_bodyA.m_xf.R.col2.x * r1Y);
    r1Y = (this.m_bodyA.m_xf.R.col1.y * r1X + this.m_bodyA.m_xf.R.col2.y * r1Y);
    r1X = tX;
    var r2X = this.m_localAnchor2.x - this.m_bodyB.m_sweep.localCenter.x;
    var r2Y = this.m_localAnchor2.y - this.m_bodyB.m_sweep.localCenter.y;
    tX = (this.m_bodyB.m_xf.R.col1.x * r2X + this.m_bodyB.m_xf.R.col2.x * r2Y);
    r2Y = (this.m_bodyB.m_xf.R.col1.y * r2X + this.m_bodyB.m_xf.R.col2.y * r2Y);
    r2X = tX;
    var v1X = this.m_bodyA.m_linearVelocity.x - this.m_bodyA.m_angularVelocity * r1Y;
    var v1Y = this.m_bodyA.m_linearVelocity.y + this.m_bodyA.m_angularVelocity * r1X;
    var v2X = this.m_bodyB.m_linearVelocity.x - this.m_bodyB.m_angularVelocity * r2Y;
    var v2Y = this.m_bodyB.m_linearVelocity.y + this.m_bodyB.m_angularVelocity * r2X;
    var Cdot = (this.m_u.x * (v2X - v1X) + this.m_u.y * (v2Y - v1Y));
    var impulse = -this.m_mass * (Cdot + this.m_bias + this.m_gamma * this.m_impulse);
    this.m_impulse += impulse;
    var PX = impulse * this.m_u.x;
    var PY = impulse * this.m_u.y;
    this.m_bodyA.m_linearVelocity.x -= this.m_bodyA.m_invMass * PX;
    this.m_bodyA.m_linearVelocity.y -= this.m_bodyA.m_invMass * PY;
    this.m_bodyA.m_angularVelocity -= this.m_bodyA.m_invI * (r1X * PY - r1Y * PX);
    this.m_bodyB.m_linearVelocity.x += this.m_bodyB.m_invMass * PX;
    this.m_bodyB.m_linearVelocity.y += this.m_bodyB.m_invMass * PY;
    this.m_bodyB.m_angularVelocity += this.m_bodyB.m_invI * (r2X * PY - r2Y * PX);
};

/**
 * @param {number} baumgarte
 */
box2d.dynamics.joints.DistanceJoint.prototype.solvePositionConstraints = function(baumgarte) {
    if (this.m_frequencyHz > 0.0) {
        return true;
    }
    var r1X = this.m_localAnchor1.x - this.m_bodyA.m_sweep.localCenter.x;
    var r1Y = this.m_localAnchor1.y - this.m_bodyA.m_sweep.localCenter.y;
    var tX = (this.m_bodyA.m_xf.R.col1.x * r1X + this.m_bodyA.m_xf.R.col2.x * r1Y);
    r1Y = (this.m_bodyA.m_xf.R.col1.y * r1X + this.m_bodyA.m_xf.R.col2.y * r1Y);
    r1X = tX;
    var r2X = this.m_localAnchor2.x - this.m_bodyB.m_sweep.localCenter.x;
    var r2Y = this.m_localAnchor2.y - this.m_bodyB.m_sweep.localCenter.y;
    tX = (this.m_bodyB.m_xf.R.col1.x * r2X + this.m_bodyB.m_xf.R.col2.x * r2Y);
    r2Y = (this.m_bodyB.m_xf.R.col1.y * r2X + this.m_bodyB.m_xf.R.col2.y * r2Y);
    r2X = tX;
    var dX = this.m_bodyB.m_sweep.c.x + r2X - this.m_bodyA.m_sweep.c.x - r1X;
    var dY = this.m_bodyB.m_sweep.c.y + r2Y - this.m_bodyA.m_sweep.c.y - r1Y;
    var length = Math.sqrt(dX * dX + dY * dY);
    dX /= length;
    dY /= length;
    var C = box2d.common.math.Math.clamp(length - this.m_length, -box2d.common.Settings.bmaxLinearCorrection, box2d.common.Settings.bmaxLinearCorrection);
    var impulse = -this.m_mass * C;
    this.m_u.set(dX, dY);
    var PX = impulse * this.m_u.x;
    var PY = impulse * this.m_u.y;
    this.m_bodyA.m_sweep.c.x -= this.m_bodyA.m_invMass * PX;
    this.m_bodyA.m_sweep.c.y -= this.m_bodyA.m_invMass * PY;
    this.m_bodyA.m_sweep.a -= this.m_bodyA.m_invI * (r1X * PY - r1Y * PX);
    this.m_bodyB.m_sweep.c.x += this.m_bodyB.m_invMass * PX;
    this.m_bodyB.m_sweep.c.y += this.m_bodyB.m_invMass * PY;
    this.m_bodyB.m_sweep.a += this.m_bodyB.m_invI * (r2X * PY - r2Y * PX);
    this.m_bodyA.synchronizeTransform();
    this.m_bodyB.synchronizeTransform();
    return Math.abs(C) < box2d.common.Settings.linearSlop;
};
