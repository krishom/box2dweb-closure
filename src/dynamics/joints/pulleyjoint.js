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

goog.provide('box2d.dynamics.joints.PulleyJoint');

goog.require('box2d.common.Settings');
goog.require('box2d.common.math.Math');
goog.require('box2d.common.math.Vec2');
goog.require('box2d.dynamics.joints.Joint');

/**
 * @param {!box2d.dynamics.joints.PulleyJointDef} def
 * @constructor
 * @extends {box2d.dynamics.joints.Joint}
 */
box2d.dynamics.joints.PulleyJoint = function(def) {
    goog.base(this, def);

    this.m_groundAnchor1 = box2d.common.math.Vec2.get(0, 0);
    this.m_groundAnchor2 = box2d.common.math.Vec2.get(0, 0);
    this.m_localAnchor1 = box2d.common.math.Vec2.get(0, 0);
    this.m_localAnchor2 = box2d.common.math.Vec2.get(0, 0);
    this.m_u1 = box2d.common.math.Vec2.get(0, 0);
    this.m_u2 = box2d.common.math.Vec2.get(0, 0);
    this.m_ground = this.m_bodyA.m_world.m_groundBody;
    this.m_groundAnchor1.x = def.groundAnchorA.x - this.m_ground.m_xf.position.x;
    this.m_groundAnchor1.y = def.groundAnchorA.y - this.m_ground.m_xf.position.y;
    this.m_groundAnchor2.x = def.groundAnchorB.x - this.m_ground.m_xf.position.x;
    this.m_groundAnchor2.y = def.groundAnchorB.y - this.m_ground.m_xf.position.y;
    this.m_localAnchor1.setV(def.localAnchorA);
    this.m_localAnchor2.setV(def.localAnchorB);
    this.m_ratio = def.ratio;
    this.m_constant = def.lengthA + this.m_ratio * def.lengthB;
    this.m_maxlength1 = Math.min(def.maxlengthA, this.m_constant - this.m_ratio * box2d.dynamics.joints.PulleyJoint.minPulleylength);
    this.m_maxlength2 = Math.min(def.maxlengthB, (this.m_constant - box2d.dynamics.joints.PulleyJoint.minPulleylength) / this.m_ratio);
    this.m_impulse = 0.0;
    this.m_limitImpulse1 = 0.0;
    this.m_limitImpulse2 = 0.0;
};
goog.inherits(box2d.dynamics.joints.PulleyJoint, box2d.dynamics.joints.Joint);

box2d.dynamics.joints.PulleyJoint.prototype.getAnchorA = function() {
    return this.m_bodyA.getWorldPoint(this.m_localAnchor1);
};

box2d.dynamics.joints.PulleyJoint.prototype.getAnchorB = function() {
    return this.m_bodyB.getWorldPoint(this.m_localAnchor2);
};

box2d.dynamics.joints.PulleyJoint.prototype.getReactionForce = function(inv_dt) {
    if (inv_dt === undefined) inv_dt = 0;
    return box2d.common.math.Vec2.get(inv_dt * this.m_impulse * this.m_u2.x, inv_dt * this.m_impulse * this.m_u2.y);
};

box2d.dynamics.joints.PulleyJoint.prototype.getReactionTorque = function(inv_dt) {
    if (inv_dt === undefined) inv_dt = 0;
    return 0.0;
};

box2d.dynamics.joints.PulleyJoint.prototype.getGroundAnchorA = function() {
    var a = this.m_ground.m_xf.position.copy();
    a.add(this.m_groundAnchor1);
    return a;
};

box2d.dynamics.joints.PulleyJoint.prototype.getGroundAnchorB = function() {
    var a = this.m_ground.m_xf.position.copy();
    a.add(this.m_groundAnchor2);
    return a;
};

box2d.dynamics.joints.PulleyJoint.prototype.getLength1 = function() {
    var p = this.m_bodyA.getWorldPoint(this.m_localAnchor1);
    var sX = this.m_ground.m_xf.position.x + this.m_groundAnchor1.x;
    var sY = this.m_ground.m_xf.position.y + this.m_groundAnchor1.y;
    var dX = p.x - sX;
    var dY = p.y - sY;
    return Math.sqrt(dX * dX + dY * dY);
};

box2d.dynamics.joints.PulleyJoint.prototype.getLength2 = function() {
    var p = this.m_bodyB.getWorldPoint(this.m_localAnchor2);
    var sX = this.m_ground.m_xf.position.x + this.m_groundAnchor2.x;
    var sY = this.m_ground.m_xf.position.y + this.m_groundAnchor2.y;
    var dX = p.x - sX;
    var dY = p.y - sY;
    return Math.sqrt(dX * dX + dY * dY);
};

box2d.dynamics.joints.PulleyJoint.prototype.getRatio = function() {
    return this.m_ratio;
};

box2d.dynamics.joints.PulleyJoint.prototype.initVelocityConstraints = function(step) {
    var bA = this.m_bodyA;
    var bB = this.m_bodyB;
    var tMat;
    tMat = bA.m_xf.R;
    var r1X = this.m_localAnchor1.x - bA.m_sweep.localCenter.x;
    var r1Y = this.m_localAnchor1.y - bA.m_sweep.localCenter.y;
    var tX = (tMat.col1.x * r1X + tMat.col2.x * r1Y);
    r1Y = (tMat.col1.y * r1X + tMat.col2.y * r1Y);
    r1X = tX;
    tMat = bB.m_xf.R;
    var r2X = this.m_localAnchor2.x - bB.m_sweep.localCenter.x;
    var r2Y = this.m_localAnchor2.y - bB.m_sweep.localCenter.y;
    tX = (tMat.col1.x * r2X + tMat.col2.x * r2Y);
    r2Y = (tMat.col1.y * r2X + tMat.col2.y * r2Y);
    r2X = tX;
    var p1X = bA.m_sweep.c.x + r1X;
    var p1Y = bA.m_sweep.c.y + r1Y;
    var p2X = bB.m_sweep.c.x + r2X;
    var p2Y = bB.m_sweep.c.y + r2Y;
    var s1X = this.m_ground.m_xf.position.x + this.m_groundAnchor1.x;
    var s1Y = this.m_ground.m_xf.position.y + this.m_groundAnchor1.y;
    var s2X = this.m_ground.m_xf.position.x + this.m_groundAnchor2.x;
    var s2Y = this.m_ground.m_xf.position.y + this.m_groundAnchor2.y;
    this.m_u1.set(p1X - s1X, p1Y - s1Y);
    this.m_u2.set(p2X - s2X, p2Y - s2Y);
    var length1 = this.m_u1.length();
    var length2 = this.m_u2.length();
    if (length1 > box2d.common.Settings.linearSlop) {
        this.m_u1.multiply(1.0 / length1);
    } else {
        this.m_u1.setZero();
    }
    if (length2 > box2d.common.Settings.linearSlop) {
        this.m_u2.multiply(1.0 / length2);
    } else {
        this.m_u2.setZero();
    }
    var C = this.m_constant - length1 - this.m_ratio * length2;
    if (C > 0.0) {
        this.m_state = box2d.dynamics.joints.Joint.e_inactiveLimit;
        this.m_impulse = 0.0;
    } else {
        this.m_state = box2d.dynamics.joints.Joint.e_atUpperLimit;
    }
    if (length1 < this.m_maxlength1) {
        this.m_limitState1 = box2d.dynamics.joints.Joint.e_inactiveLimit;
        this.m_limitImpulse1 = 0.0;
    } else {
        this.m_limitState1 = box2d.dynamics.joints.Joint.e_atUpperLimit;
    }
    if (length2 < this.m_maxlength2) {
        this.m_limitState2 = box2d.dynamics.joints.Joint.e_inactiveLimit;
        this.m_limitImpulse2 = 0.0;
    } else {
        this.m_limitState2 = box2d.dynamics.joints.Joint.e_atUpperLimit;
    }
    var cr1u1 = r1X * this.m_u1.y - r1Y * this.m_u1.x;
    var cr2u2 = r2X * this.m_u2.y - r2Y * this.m_u2.x;
    this.m_limitMass1 = bA.m_invMass + bA.m_invI * cr1u1 * cr1u1;
    this.m_limitMass2 = bB.m_invMass + bB.m_invI * cr2u2 * cr2u2;
    this.m_pulleyMass = this.m_limitMass1 + this.m_ratio * this.m_ratio * this.m_limitMass2;
    this.m_limitMass1 = 1.0 / this.m_limitMass1;
    this.m_limitMass2 = 1.0 / this.m_limitMass2;
    this.m_pulleyMass = 1.0 / this.m_pulleyMass;
    if (step.warmStarting) {
        this.m_impulse *= step.dtRatio;
        this.m_limitImpulse1 *= step.dtRatio;
        this.m_limitImpulse2 *= step.dtRatio;
        var P1X = ((-this.m_impulse) - this.m_limitImpulse1) * this.m_u1.x;
        var P1Y = ((-this.m_impulse) - this.m_limitImpulse1) * this.m_u1.y;
        var P2X = ((-this.m_ratio * this.m_impulse) - this.m_limitImpulse2) * this.m_u2.x;
        var P2Y = ((-this.m_ratio * this.m_impulse) - this.m_limitImpulse2) * this.m_u2.y;
        bA.m_linearVelocity.x += bA.m_invMass * P1X;
        bA.m_linearVelocity.y += bA.m_invMass * P1Y;
        bA.m_angularVelocity += bA.m_invI * (r1X * P1Y - r1Y * P1X);
        bB.m_linearVelocity.x += bB.m_invMass * P2X;
        bB.m_linearVelocity.y += bB.m_invMass * P2Y;
        bB.m_angularVelocity += bB.m_invI * (r2X * P2Y - r2Y * P2X);
    } else {
        this.m_impulse = 0.0;
        this.m_limitImpulse1 = 0.0;
        this.m_limitImpulse2 = 0.0;
    }
};

box2d.dynamics.joints.PulleyJoint.prototype.solveVelocityConstraints = function(step) {
    var bA = this.m_bodyA;
    var bB = this.m_bodyB;
    var tMat;
    tMat = bA.m_xf.R;
    var r1X = this.m_localAnchor1.x - bA.m_sweep.localCenter.x;
    var r1Y = this.m_localAnchor1.y - bA.m_sweep.localCenter.y;
    var tX = (tMat.col1.x * r1X + tMat.col2.x * r1Y);
    r1Y = (tMat.col1.y * r1X + tMat.col2.y * r1Y);
    r1X = tX;
    tMat = bB.m_xf.R;
    var r2X = this.m_localAnchor2.x - bB.m_sweep.localCenter.x;
    var r2Y = this.m_localAnchor2.y - bB.m_sweep.localCenter.y;
    tX = (tMat.col1.x * r2X + tMat.col2.x * r2Y);
    r2Y = (tMat.col1.y * r2X + tMat.col2.y * r2Y);
    r2X = tX;
    var v1X = 0;
    var v1Y = 0;
    var v2X = 0;
    var v2Y = 0;
    var P1X = 0;
    var P1Y = 0;
    var P2X = 0;
    var P2Y = 0;
    var Cdot = 0;
    var impulse = 0;
    var oldImpulse = 0;
    if (this.m_state == box2d.dynamics.joints.Joint.e_atUpperLimit) {
        v1X = bA.m_linearVelocity.x + ((-bA.m_angularVelocity * r1Y));
        v1Y = bA.m_linearVelocity.y + (bA.m_angularVelocity * r1X);
        v2X = bB.m_linearVelocity.x + ((-bB.m_angularVelocity * r2Y));
        v2Y = bB.m_linearVelocity.y + (bB.m_angularVelocity * r2X);
        Cdot = (-(this.m_u1.x * v1X + this.m_u1.y * v1Y)) - this.m_ratio * (this.m_u2.x * v2X + this.m_u2.y * v2Y);
        impulse = this.m_pulleyMass * ((-Cdot));
        oldImpulse = this.m_impulse;
        this.m_impulse = Math.max(0.0, this.m_impulse + impulse);
        impulse = this.m_impulse - oldImpulse;
        P1X = (-impulse * this.m_u1.x);
        P1Y = (-impulse * this.m_u1.y);
        P2X = (-this.m_ratio * impulse * this.m_u2.x);
        P2Y = (-this.m_ratio * impulse * this.m_u2.y);
        bA.m_linearVelocity.x += bA.m_invMass * P1X;
        bA.m_linearVelocity.y += bA.m_invMass * P1Y;
        bA.m_angularVelocity += bA.m_invI * (r1X * P1Y - r1Y * P1X);
        bB.m_linearVelocity.x += bB.m_invMass * P2X;
        bB.m_linearVelocity.y += bB.m_invMass * P2Y;
        bB.m_angularVelocity += bB.m_invI * (r2X * P2Y - r2Y * P2X);
    }
    if (this.m_limitState1 == box2d.dynamics.joints.Joint.e_atUpperLimit) {
        v1X = bA.m_linearVelocity.x + ((-bA.m_angularVelocity * r1Y));
        v1Y = bA.m_linearVelocity.y + (bA.m_angularVelocity * r1X);
        Cdot = (-(this.m_u1.x * v1X + this.m_u1.y * v1Y));
        impulse = (-this.m_limitMass1 * Cdot);
        oldImpulse = this.m_limitImpulse1;
        this.m_limitImpulse1 = Math.max(0.0, this.m_limitImpulse1 + impulse);
        impulse = this.m_limitImpulse1 - oldImpulse;
        P1X = (-impulse * this.m_u1.x);
        P1Y = (-impulse * this.m_u1.y);
        bA.m_linearVelocity.x += bA.m_invMass * P1X;
        bA.m_linearVelocity.y += bA.m_invMass * P1Y;
        bA.m_angularVelocity += bA.m_invI * (r1X * P1Y - r1Y * P1X);
    }
    if (this.m_limitState2 == box2d.dynamics.joints.Joint.e_atUpperLimit) {
        v2X = bB.m_linearVelocity.x + ((-bB.m_angularVelocity * r2Y));
        v2Y = bB.m_linearVelocity.y + (bB.m_angularVelocity * r2X);
        Cdot = (-(this.m_u2.x * v2X + this.m_u2.y * v2Y));
        impulse = (-this.m_limitMass2 * Cdot);
        oldImpulse = this.m_limitImpulse2;
        this.m_limitImpulse2 = Math.max(0.0, this.m_limitImpulse2 + impulse);
        impulse = this.m_limitImpulse2 - oldImpulse;
        P2X = (-impulse * this.m_u2.x);
        P2Y = (-impulse * this.m_u2.y);
        bB.m_linearVelocity.x += bB.m_invMass * P2X;
        bB.m_linearVelocity.y += bB.m_invMass * P2Y;
        bB.m_angularVelocity += bB.m_invI * (r2X * P2Y - r2Y * P2X);
    }
};

box2d.dynamics.joints.PulleyJoint.prototype.solvePositionConstraints = function(baumgarte) {
    if (baumgarte === undefined) baumgarte = 0;
    var bA = this.m_bodyA;
    var bB = this.m_bodyB;
    var tMat;
    var s1X = this.m_ground.m_xf.position.x + this.m_groundAnchor1.x;
    var s1Y = this.m_ground.m_xf.position.y + this.m_groundAnchor1.y;
    var s2X = this.m_ground.m_xf.position.x + this.m_groundAnchor2.x;
    var s2Y = this.m_ground.m_xf.position.y + this.m_groundAnchor2.y;
    var r1X = 0;
    var r1Y = 0;
    var r2X = 0;
    var r2Y = 0;
    var p1X = 0;
    var p1Y = 0;
    var p2X = 0;
    var p2Y = 0;
    var length1 = 0;
    var length2 = 0;
    var C = 0;
    var impulse = 0;
    var oldImpulse = 0;
    var oldLimitPositionImpulse = 0;
    var tX = 0;
    var linearError = 0.0;
    if (this.m_state == box2d.dynamics.joints.Joint.e_atUpperLimit) {
        tMat = bA.m_xf.R;
        r1X = this.m_localAnchor1.x - bA.m_sweep.localCenter.x;
        r1Y = this.m_localAnchor1.y - bA.m_sweep.localCenter.y;
        tX = (tMat.col1.x * r1X + tMat.col2.x * r1Y);
        r1Y = (tMat.col1.y * r1X + tMat.col2.y * r1Y);
        r1X = tX;
        tMat = bB.m_xf.R;
        r2X = this.m_localAnchor2.x - bB.m_sweep.localCenter.x;
        r2Y = this.m_localAnchor2.y - bB.m_sweep.localCenter.y;
        tX = (tMat.col1.x * r2X + tMat.col2.x * r2Y);
        r2Y = (tMat.col1.y * r2X + tMat.col2.y * r2Y);
        r2X = tX;
        p1X = bA.m_sweep.c.x + r1X;
        p1Y = bA.m_sweep.c.y + r1Y;
        p2X = bB.m_sweep.c.x + r2X;
        p2Y = bB.m_sweep.c.y + r2Y;
        this.m_u1.set(p1X - s1X, p1Y - s1Y);
        this.m_u2.set(p2X - s2X, p2Y - s2Y);
        length1 = this.m_u1.length();
        length2 = this.m_u2.length();
        if (length1 > box2d.common.Settings.linearSlop) {
            this.m_u1.multiply(1.0 / length1);
        } else {
            this.m_u1.setZero();
        }
        if (length2 > box2d.common.Settings.linearSlop) {
            this.m_u2.multiply(1.0 / length2);
        } else {
            this.m_u2.setZero();
        }
        C = this.m_constant - length1 - this.m_ratio * length2;
        linearError = Math.max(linearError, (-C));
        C = box2d.common.math.Math.clamp(C + box2d.common.Settings.linearSlop, (-box2d.common.Settings.bmaxLinearCorrection), 0.0);
        impulse = (-this.m_pulleyMass * C);
        p1X = (-impulse * this.m_u1.x);
        p1Y = (-impulse * this.m_u1.y);
        p2X = (-this.m_ratio * impulse * this.m_u2.x);
        p2Y = (-this.m_ratio * impulse * this.m_u2.y);
        bA.m_sweep.c.x += bA.m_invMass * p1X;
        bA.m_sweep.c.y += bA.m_invMass * p1Y;
        bA.m_sweep.a += bA.m_invI * (r1X * p1Y - r1Y * p1X);
        bB.m_sweep.c.x += bB.m_invMass * p2X;
        bB.m_sweep.c.y += bB.m_invMass * p2Y;
        bB.m_sweep.a += bB.m_invI * (r2X * p2Y - r2Y * p2X);
        bA.synchronizeTransform();
        bB.synchronizeTransform();
    }
    if (this.m_limitState1 == box2d.dynamics.joints.Joint.e_atUpperLimit) {
        tMat = bA.m_xf.R;
        r1X = this.m_localAnchor1.x - bA.m_sweep.localCenter.x;
        r1Y = this.m_localAnchor1.y - bA.m_sweep.localCenter.y;
        tX = (tMat.col1.x * r1X + tMat.col2.x * r1Y);
        r1Y = (tMat.col1.y * r1X + tMat.col2.y * r1Y);
        r1X = tX;
        p1X = bA.m_sweep.c.x + r1X;
        p1Y = bA.m_sweep.c.y + r1Y;
        this.m_u1.set(p1X - s1X, p1Y - s1Y);
        length1 = this.m_u1.length();
        if (length1 > box2d.common.Settings.linearSlop) {
            this.m_u1.x *= 1.0 / length1;
            this.m_u1.y *= 1.0 / length1;
        } else {
            this.m_u1.setZero();
        }
        C = this.m_maxlength1 - length1;
        linearError = Math.max(linearError, (-C));
        C = box2d.common.math.Math.clamp(C + box2d.common.Settings.linearSlop, (-box2d.common.Settings.bmaxLinearCorrection), 0.0);
        impulse = (-this.m_limitMass1 * C);
        p1X = (-impulse * this.m_u1.x);
        p1Y = (-impulse * this.m_u1.y);
        bA.m_sweep.c.x += bA.m_invMass * p1X;
        bA.m_sweep.c.y += bA.m_invMass * p1Y;
        bA.m_sweep.a += bA.m_invI * (r1X * p1Y - r1Y * p1X);
        bA.synchronizeTransform();
    }
    if (this.m_limitState2 == box2d.dynamics.joints.Joint.e_atUpperLimit) {
        tMat = bB.m_xf.R;
        r2X = this.m_localAnchor2.x - bB.m_sweep.localCenter.x;
        r2Y = this.m_localAnchor2.y - bB.m_sweep.localCenter.y;
        tX = (tMat.col1.x * r2X + tMat.col2.x * r2Y);
        r2Y = (tMat.col1.y * r2X + tMat.col2.y * r2Y);
        r2X = tX;
        p2X = bB.m_sweep.c.x + r2X;
        p2Y = bB.m_sweep.c.y + r2Y;
        this.m_u2.set(p2X - s2X, p2Y - s2Y);
        length2 = this.m_u2.length();
        if (length2 > box2d.common.Settings.linearSlop) {
            this.m_u2.x *= 1.0 / length2;
            this.m_u2.y *= 1.0 / length2;
        }
        else {
            this.m_u2.setZero();
        }
        C = this.m_maxlength2 - length2;
        linearError = Math.max(linearError, (-C));
        C = box2d.common.math.Math.clamp(C + box2d.common.Settings.linearSlop, (-box2d.common.Settings.bmaxLinearCorrection), 0.0);
        impulse = (-this.m_limitMass2 * C);
        p2X = (-impulse * this.m_u2.x);
        p2Y = (-impulse * this.m_u2.y);
        bB.m_sweep.c.x += bB.m_invMass * p2X;
        bB.m_sweep.c.y += bB.m_invMass * p2Y;
        bB.m_sweep.a += bB.m_invI * (r2X * p2Y - r2Y * p2X);
        bB.synchronizeTransform();
    }
    return linearError < box2d.common.Settings.linearSlop;
};

box2d.dynamics.joints.PulleyJoint.minPulleylength = 1.0;
