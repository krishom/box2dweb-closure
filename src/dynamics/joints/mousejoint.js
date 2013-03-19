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

goog.provide('box2d.dynamics.joints.MouseJoint');

goog.require('box2d.common.math.Mat22');
goog.require('box2d.common.math.Math');
goog.require('box2d.common.math.Vec2');
goog.require('box2d.dynamics.joints.Joint');

/**
 * @param {!box2d.dynamics.joints.MouseJointDef} def
 * @constructor
 * @extends {box2d.dynamics.joints.Joint}
 */
box2d.dynamics.joints.MouseJoint = function(def) {
    goog.base(this, def);

    this.K = box2d.common.math.Mat22.get();
    this.K1 = box2d.common.math.Mat22.get();
    this.K2 = box2d.common.math.Mat22.get();
    this.m_localAnchor = box2d.common.math.Vec2.get(0, 0);
    this.m_target = box2d.common.math.Vec2.get(0, 0);
    this.m_impulse = box2d.common.math.Vec2.get(0, 0);
    this.m_mass = box2d.common.math.Mat22.get();
    this.m_C = box2d.common.math.Vec2.get(0, 0);
    this.m_target.setV(def.target);
    var tX = this.m_target.x - this.m_bodyB.m_xf.position.x;
    var tY = this.m_target.y - this.m_bodyB.m_xf.position.y;
    var tMat = this.m_bodyB.m_xf.R;
    this.m_localAnchor.x = (tX * tMat.col1.x + tY * tMat.col1.y);
    this.m_localAnchor.y = (tX * tMat.col2.x + tY * tMat.col2.y);
    this.m_maxForce = def.maxForce;
    this.m_impulse.setZero();
    this.m_frequencyHz = def.frequencyHz;
    this.m_dampingRatio = def.dampingRatio;
    this.m_beta = 0.0;
    this.m_gamma = 0.0;
};
goog.inherits(box2d.dynamics.joints.MouseJoint, box2d.dynamics.joints.Joint);

box2d.dynamics.joints.MouseJoint.prototype.getAnchorA = function() {
    return box2d.common.math.Vec2.get(this.m_target.x, this.m_target.y);
};

box2d.dynamics.joints.MouseJoint.prototype.getAnchorB = function() {
    return this.m_bodyB.getWorldPoint(this.m_localAnchor);
};

box2d.dynamics.joints.MouseJoint.prototype.getReactionForce = function(inv_dt) {
    if (inv_dt === undefined) inv_dt = 0;
    return box2d.common.math.Vec2.get(inv_dt * this.m_impulse.x, inv_dt * this.m_impulse.y);
};

box2d.dynamics.joints.MouseJoint.prototype.getReactionTorque = function(inv_dt) {
    if (inv_dt === undefined) inv_dt = 0;
    return 0.0;
};

box2d.dynamics.joints.MouseJoint.prototype.getTarget = function() {
    return this.m_target;
};

box2d.dynamics.joints.MouseJoint.prototype.setTarget = function(target) {
    if (this.m_bodyB.isAwake() == false) {
        this.m_bodyB.setAwake(true);
    }
    this.m_target = target;
};

box2d.dynamics.joints.MouseJoint.prototype.getMaxForce = function() {
    return this.m_maxForce;
};

box2d.dynamics.joints.MouseJoint.prototype.setMaxForce = function(maxForce) {
    if (maxForce === undefined) maxForce = 0;
    this.m_maxForce = maxForce;
};

box2d.dynamics.joints.MouseJoint.prototype.getFrequency = function() {
    return this.m_frequencyHz;
};

box2d.dynamics.joints.MouseJoint.prototype.setFrequency = function(hz) {
    if (hz === undefined) hz = 0;
    this.m_frequencyHz = hz;
};

box2d.dynamics.joints.MouseJoint.prototype.getDampingRatio = function() {
    return this.m_dampingRatio;
};

box2d.dynamics.joints.MouseJoint.prototype.setDampingRatio = function(ratio) {
    if (ratio === undefined) ratio = 0;
    this.m_dampingRatio = ratio;
};

box2d.dynamics.joints.MouseJoint.prototype.initVelocityConstraints = function(step) {
    var b = this.m_bodyB;
    var mass = b.getMass();
    var omega = 2.0 * Math.PI * this.m_frequencyHz;
    var d = 2.0 * mass * this.m_dampingRatio * omega;
    var k = mass * omega * omega;
    this.m_gamma = step.dt * (d + step.dt * k);
    this.m_gamma = this.m_gamma != 0 ? 1 / this.m_gamma : 0.0;
    this.m_beta = step.dt * k * this.m_gamma;
    var tMat;
    tMat = b.m_xf.R;
    var rX = this.m_localAnchor.x - b.m_sweep.localCenter.x;
    var rY = this.m_localAnchor.y - b.m_sweep.localCenter.y;
    var tX = (tMat.col1.x * rX + tMat.col2.x * rY);
    rY = (tMat.col1.y * rX + tMat.col2.y * rY);
    rX = tX;
    var invMass = b.m_invMass;
    var invI = b.m_invI;
    this.K1.col1.x = invMass;
    this.K1.col2.x = 0.0;
    this.K1.col1.y = 0.0;
    this.K1.col2.y = invMass;
    this.K2.col1.x = invI * rY * rY;
    this.K2.col2.x = (-invI * rX * rY);
    this.K2.col1.y = (-invI * rX * rY);
    this.K2.col2.y = invI * rX * rX;
    this.K.setM(this.K1);
    this.K.addM(this.K2);
    this.K.col1.x += this.m_gamma;
    this.K.col2.y += this.m_gamma;
    this.K.getInverse(this.m_mass);
    this.m_C.x = b.m_sweep.c.x + rX - this.m_target.x;
    this.m_C.y = b.m_sweep.c.y + rY - this.m_target.y;
    b.m_angularVelocity *= 0.98;
    this.m_impulse.x *= step.dtRatio;
    this.m_impulse.y *= step.dtRatio;
    b.m_linearVelocity.x += invMass * this.m_impulse.x;
    b.m_linearVelocity.y += invMass * this.m_impulse.y;
    b.m_angularVelocity += invI * (rX * this.m_impulse.y - rY * this.m_impulse.x);
};

box2d.dynamics.joints.MouseJoint.prototype.solveVelocityConstraints = function(step) {
    var b = this.m_bodyB;
    var tMat;
    var tX = 0;
    var tY = 0;
    tMat = b.m_xf.R;
    var rX = this.m_localAnchor.x - b.m_sweep.localCenter.x;
    var rY = this.m_localAnchor.y - b.m_sweep.localCenter.y;
    tX = (tMat.col1.x * rX + tMat.col2.x * rY);
    rY = (tMat.col1.y * rX + tMat.col2.y * rY);
    rX = tX;
    var CdotX = b.m_linearVelocity.x + ((-b.m_angularVelocity * rY));
    var CdotY = b.m_linearVelocity.y + (b.m_angularVelocity * rX);
    tMat = this.m_mass;
    tX = CdotX + this.m_beta * this.m_C.x + this.m_gamma * this.m_impulse.x;
    tY = CdotY + this.m_beta * this.m_C.y + this.m_gamma * this.m_impulse.y;
    var impulseX = (-(tMat.col1.x * tX + tMat.col2.x * tY));
    var impulseY = (-(tMat.col1.y * tX + tMat.col2.y * tY));
    var oldImpulseX = this.m_impulse.x;
    var oldImpulseY = this.m_impulse.y;
    this.m_impulse.x += impulseX;
    this.m_impulse.y += impulseY;
    var maxImpulse = step.dt * this.m_maxForce;
    if (this.m_impulse.lengthSquared() > maxImpulse * maxImpulse) {
        this.m_impulse.multiply(maxImpulse / this.m_impulse.length());
    }
    impulseX = this.m_impulse.x - oldImpulseX;
    impulseY = this.m_impulse.y - oldImpulseY;
    b.m_linearVelocity.x += b.m_invMass * impulseX;
    b.m_linearVelocity.y += b.m_invMass * impulseY;
    b.m_angularVelocity += b.m_invI * (rX * impulseY - rY * impulseX);
};

box2d.dynamics.joints.MouseJoint.prototype.solvePositionConstraints = function(baumgarte) {
    return true;
};
