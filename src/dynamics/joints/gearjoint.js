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

goog.provide('box2d.dynamics.joints.GearJoint');

goog.require('box2d.common.Settings');
goog.require('box2d.common.math.Vec2');
goog.require('box2d.dynamics.joints.Jacobian');
goog.require('box2d.dynamics.joints.Joint');

/**
 * @param {!box2d.dynamics.joints.GearJointDef} def
 * @constructor
 * @extends {box2d.dynamics.joints.Joint}
 */
box2d.dynamics.joints.GearJoint = function(def) {
    goog.base(this, def);

    this.m_groundAnchor1 = box2d.common.math.Vec2.get(0, 0);
    this.m_groundAnchor2 = box2d.common.math.Vec2.get(0, 0);
    this.m_localAnchor1 = box2d.common.math.Vec2.get(0, 0);
    this.m_localAnchor2 = box2d.common.math.Vec2.get(0, 0);
    this.m_J = new box2d.dynamics.joints.Jacobian();
    var type1 = def.joint1.m_type;
    var type2 = def.joint2.m_type;
    this.m_revolute1 = null;
    this.m_prismatic1 = null;
    this.m_revolute2 = null;
    this.m_prismatic2 = null;
    var coordinate1 = 0;
    var coordinate2 = 0;
    this.m_ground1 = def.joint1.getBodyA();
    this.m_bodyA = def.joint1.getBodyB();
    if (type1 == box2d.dynamics.joints.Joint.e_revoluteJoint) {
        this.m_revolute1 = def.joint1;
        this.m_groundAnchor1.setV(this.m_revolute1.m_localAnchor1);
        this.m_localAnchor1.setV(this.m_revolute1.m_localAnchor2);
        coordinate1 = this.m_revolute1.getJointAngle();
    } else {
        this.m_prismatic1 = def.joint1;
        this.m_groundAnchor1.setV(this.m_prismatic1.m_localAnchor1);
        this.m_localAnchor1.setV(this.m_prismatic1.m_localAnchor2);
        coordinate1 = this.m_prismatic1.getJointTranslation();
    }
    this.m_ground2 = def.joint2.getBodyA();
    this.m_bodyB = def.joint2.getBodyB();
    if (type2 == box2d.dynamics.joints.Joint.e_revoluteJoint) {
        this.m_revolute2 = def.joint2;
        this.m_groundAnchor2.setV(this.m_revolute2.m_localAnchor1);
        this.m_localAnchor2.setV(this.m_revolute2.m_localAnchor2);
        coordinate2 = this.m_revolute2.getJointAngle();
    } else {
        this.m_prismatic2 = def.joint2;
        this.m_groundAnchor2.setV(this.m_prismatic2.m_localAnchor1);
        this.m_localAnchor2.setV(this.m_prismatic2.m_localAnchor2);
        coordinate2 = this.m_prismatic2.getJointTranslation();
    }
    this.m_ratio = def.ratio;
    this.m_constant = coordinate1 + this.m_ratio * coordinate2;
    this.m_impulse = 0.0;
};
goog.inherits(box2d.dynamics.joints.GearJoint, box2d.dynamics.joints.Joint);

box2d.dynamics.joints.GearJoint.prototype.getAnchorA = function() {
    return this.m_bodyA.getWorldPoint(this.m_localAnchor1);
};

box2d.dynamics.joints.GearJoint.prototype.getAnchorB = function() {
    return this.m_bodyB.getWorldPoint(this.m_localAnchor2);
};

box2d.dynamics.joints.GearJoint.prototype.getReactionForce = function(inv_dt) {
    if (inv_dt === undefined) inv_dt = 0;
    return box2d.common.math.Vec2.get(inv_dt * this.m_impulse * this.m_J.linearB.x, inv_dt * this.m_impulse * this.m_J.linearB.y);
};

box2d.dynamics.joints.GearJoint.prototype.getReactionTorque = function(inv_dt) {
    if (inv_dt === undefined) inv_dt = 0;
    var tMat = this.m_bodyB.m_xf.R;
    var rX = this.m_localAnchor1.x - this.m_bodyB.m_sweep.localCenter.x;
    var rY = this.m_localAnchor1.y - this.m_bodyB.m_sweep.localCenter.y;
    var tX = tMat.col1.x * rX + tMat.col2.x * rY;
    rY = tMat.col1.y * rX + tMat.col2.y * rY;
    rX = tX;
    var PX = this.m_impulse * this.m_J.linearB.x;
    var PY = this.m_impulse * this.m_J.linearB.y;
    return inv_dt * (this.m_impulse * this.m_J.angularB - rX * PY + rY * PX);
};

box2d.dynamics.joints.GearJoint.prototype.getRatio = function() {
    return this.m_ratio;
};

box2d.dynamics.joints.GearJoint.prototype.setRatio = function(ratio) {
    if (ratio === undefined) ratio = 0;
    this.m_ratio = ratio;
};

box2d.dynamics.joints.GearJoint.prototype.initVelocityConstraints = function(step) {
    var g1 = this.m_ground1;
    var g2 = this.m_ground2;
    var bA = this.m_bodyA;
    var bB = this.m_bodyB;
    var ugX = 0;
    var ugY = 0;
    var rX = 0;
    var rY = 0;
    var tMat;
    var tVec;
    var crug = 0;
    var tX = 0;
    var K = 0.0;
    this.m_J.setZero();
    if (this.m_revolute1) {
        this.m_J.angularA = (-1.0);
        K += bA.m_invI;
    } else {
        tMat = g1.m_xf.R;
        tVec = this.m_prismatic1.m_localXAxis1;
        ugX = tMat.col1.x * tVec.x + tMat.col2.x * tVec.y;
        ugY = tMat.col1.y * tVec.x + tMat.col2.y * tVec.y;
        tMat = bA.m_xf.R;
        rX = this.m_localAnchor1.x - bA.m_sweep.localCenter.x;
        rY = this.m_localAnchor1.y - bA.m_sweep.localCenter.y;
        tX = tMat.col1.x * rX + tMat.col2.x * rY;
        rY = tMat.col1.y * rX + tMat.col2.y * rY;
        rX = tX;
        crug = rX * ugY - rY * ugX;
        this.m_J.linearA.set((-ugX), (-ugY));
        this.m_J.angularA = (-crug);
        K += bA.m_invMass + bA.m_invI * crug * crug;
    }
    if (this.m_revolute2) {
        this.m_J.angularB = (-this.m_ratio);
        K += this.m_ratio * this.m_ratio * bB.m_invI;
    } else {
        tMat = g2.m_xf.R;
        tVec = this.m_prismatic2.m_localXAxis1;
        ugX = tMat.col1.x * tVec.x + tMat.col2.x * tVec.y;
        ugY = tMat.col1.y * tVec.x + tMat.col2.y * tVec.y;
        tMat = bB.m_xf.R;
        rX = this.m_localAnchor2.x - bB.m_sweep.localCenter.x;
        rY = this.m_localAnchor2.y - bB.m_sweep.localCenter.y;
        tX = tMat.col1.x * rX + tMat.col2.x * rY;
        rY = tMat.col1.y * rX + tMat.col2.y * rY;
        rX = tX;
        crug = rX * ugY - rY * ugX;
        this.m_J.linearB.set((-this.m_ratio * ugX), (-this.m_ratio * ugY));
        this.m_J.angularB = (-this.m_ratio * crug);
        K += this.m_ratio * this.m_ratio * (bB.m_invMass + bB.m_invI * crug * crug);
    }
    this.m_mass = K > 0.0 ? 1.0 / K : 0.0;
    if (step.warmStarting) {
        bA.m_linearVelocity.x += bA.m_invMass * this.m_impulse * this.m_J.linearA.x;
        bA.m_linearVelocity.y += bA.m_invMass * this.m_impulse * this.m_J.linearA.y;
        bA.m_angularVelocity += bA.m_invI * this.m_impulse * this.m_J.angularA;
        bB.m_linearVelocity.x += bB.m_invMass * this.m_impulse * this.m_J.linearB.x;
        bB.m_linearVelocity.y += bB.m_invMass * this.m_impulse * this.m_J.linearB.y;
        bB.m_angularVelocity += bB.m_invI * this.m_impulse * this.m_J.angularB;
    } else {
        this.m_impulse = 0.0;
    }
};

box2d.dynamics.joints.GearJoint.prototype.solveVelocityConstraints = function(step) {
    var bA = this.m_bodyA;
    var bB = this.m_bodyB;
    var Cdot = this.m_J.compute(bA.m_linearVelocity, bA.m_angularVelocity, bB.m_linearVelocity, bB.m_angularVelocity);
    var impulse = (-this.m_mass * Cdot);
    this.m_impulse += impulse;
    bA.m_linearVelocity.x += bA.m_invMass * impulse * this.m_J.linearA.x;
    bA.m_linearVelocity.y += bA.m_invMass * impulse * this.m_J.linearA.y;
    bA.m_angularVelocity += bA.m_invI * impulse * this.m_J.angularA;
    bB.m_linearVelocity.x += bB.m_invMass * impulse * this.m_J.linearB.x;
    bB.m_linearVelocity.y += bB.m_invMass * impulse * this.m_J.linearB.y;
    bB.m_angularVelocity += bB.m_invI * impulse * this.m_J.angularB;
};

box2d.dynamics.joints.GearJoint.prototype.solvePositionConstraints = function(baumgarte) {
    if (baumgarte === undefined) baumgarte = 0;
    var linearError = 0.0;
    var bA = this.m_bodyA;
    var bB = this.m_bodyB;
    var coordinate1 = 0;
    var coordinate2 = 0;
    if (this.m_revolute1) {
        coordinate1 = this.m_revolute1.getJointAngle();
    } else {
        coordinate1 = this.m_prismatic1.getJointTranslation();
    }
    if (this.m_revolute2) {
        coordinate2 = this.m_revolute2.getJointAngle();
    } else {
        coordinate2 = this.m_prismatic2.getJointTranslation();
    }
    var C = this.m_constant - (coordinate1 + this.m_ratio * coordinate2);
    var impulse = (-this.m_mass * C);
    bA.m_sweep.c.x += bA.m_invMass * impulse * this.m_J.linearA.x;
    bA.m_sweep.c.y += bA.m_invMass * impulse * this.m_J.linearA.y;
    bA.m_sweep.a += bA.m_invI * impulse * this.m_J.angularA;
    bB.m_sweep.c.x += bB.m_invMass * impulse * this.m_J.linearB.x;
    bB.m_sweep.c.y += bB.m_invMass * impulse * this.m_J.linearB.y;
    bB.m_sweep.a += bB.m_invI * impulse * this.m_J.angularB;
    bA.synchronizeTransform();
    bB.synchronizeTransform();
    return linearError < box2d.common.Settings.linearSlop;
};
