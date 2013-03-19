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

goog.provide('box2d.dynamics.contacts.Contactsolver');

goog.require('box2d.collision.WorldManifold');
goog.require('box2d.common.Settings');
goog.require('box2d.common.math.Math');
goog.require('box2d.dynamics.TimeStep');
goog.require('box2d.dynamics.contacts.ContactConstraint');
goog.require('box2d.dynamics.contacts.PositionsolverManifold');

/**
 * @constructor
 */
box2d.dynamics.contacts.Contactsolver = function() {

    /**
     * @type {Array.<!box2d.dynamics.contacts.ContactConstraint>}
     */
    this.m_constraints = [];
};

/**
 * @param {!box2d.dynamics.TimeStep} step
 * @param {Array.<!box2d.dynamics.contacts.Contact>} contacts
 * @param {number} contactCount
 */
box2d.dynamics.contacts.Contactsolver.prototype.initialize = function(step, contacts, contactCount) {
    this.m_constraintCount = contactCount;
    while (this.m_constraints.length < this.m_constraintCount) {
        this.m_constraints[this.m_constraints.length] = new box2d.dynamics.contacts.ContactConstraint();
    }
    for (var i = 0; i < contactCount; i++) {
        var contact = contacts[i];
        var fixtureA = contact.m_fixtureA;
        var fixtureB = contact.m_fixtureB;
        var shapeA = fixtureA.m_shape;
        var shapeB = fixtureB.m_shape;
        var radiusA = shapeA.m_radius;
        var radiusB = shapeB.m_radius;
        var bodyA = fixtureA.getBody();
        var bodyB = fixtureB.getBody();
        var manifold = contact.getManifold();
        var friction = box2d.common.Settings.mixFriction(fixtureA.getFriction(), fixtureB.getFriction());
        var restitution = box2d.common.Settings.mixRestitution(fixtureA.getRestitution(), fixtureB.getRestitution());
        var vAX = bodyA.m_linearVelocity.x;
        var vAY = bodyA.m_linearVelocity.y;
        var vBX = bodyB.m_linearVelocity.x;
        var vBY = bodyB.m_linearVelocity.y;
        var wA = bodyA.m_angularVelocity;
        var wB = bodyB.m_angularVelocity;
        box2d.common.Settings.assert(manifold.m_pointCount > 0);
        box2d.dynamics.contacts.Contactsolver.s_worldManifold.initialize(manifold, bodyA.m_xf, radiusA, bodyB.m_xf, radiusB);
        var normalX = box2d.dynamics.contacts.Contactsolver.s_worldManifold.m_normal.x;
        var normalY = box2d.dynamics.contacts.Contactsolver.s_worldManifold.m_normal.y;
        var cc = this.m_constraints[i];
        cc.bodyA = bodyA;
        cc.bodyB = bodyB;
        cc.manifold = manifold;
        cc.normal.x = normalX;
        cc.normal.y = normalY;
        cc.pointCount = manifold.m_pointCount;
        cc.friction = friction;
        cc.restitution = restitution;
        cc.localPlaneNormal.x = manifold.m_localPlaneNormal.x;
        cc.localPlaneNormal.y = manifold.m_localPlaneNormal.y;
        cc.localPoint.x = manifold.m_localPoint.x;
        cc.localPoint.y = manifold.m_localPoint.y;
        cc.radius = radiusA + radiusB;
        cc.type = manifold.m_type;
        for (var k = 0; k < cc.pointCount; ++k) {
            var cp = manifold.m_points[k];
            var ccp = cc.points[k];
            ccp.normalImpulse = cp.m_normalImpulse;
            ccp.tangentImpulse = cp.m_tangentImpulse;
            ccp.localPoint.setV(cp.m_localPoint);
            var rAX = ccp.rA.x = box2d.dynamics.contacts.Contactsolver.s_worldManifold.m_points[k].x - bodyA.m_sweep.c.x;
            var rAY = ccp.rA.y = box2d.dynamics.contacts.Contactsolver.s_worldManifold.m_points[k].y - bodyA.m_sweep.c.y;
            var rBX = ccp.rB.x = box2d.dynamics.contacts.Contactsolver.s_worldManifold.m_points[k].x - bodyB.m_sweep.c.x;
            var rBY = ccp.rB.y = box2d.dynamics.contacts.Contactsolver.s_worldManifold.m_points[k].y - bodyB.m_sweep.c.y;
            var rnA = rAX * normalY - rAY * normalX;
            var rnB = rBX * normalY - rBY * normalX;
            rnA *= rnA;
            rnB *= rnB;
            var kNormal = bodyA.m_invMass + bodyB.m_invMass + bodyA.m_invI * rnA + bodyB.m_invI * rnB;
            ccp.normalMass = 1.0 / kNormal;
            var kEqualized = bodyA.m_mass * bodyA.m_invMass + bodyB.m_mass * bodyB.m_invMass;
            kEqualized += bodyA.m_mass * bodyA.m_invI * rnA + bodyB.m_mass * bodyB.m_invI * rnB;
            ccp.equalizedMass = 1.0 / kEqualized;
            var tangentX = normalY;
            var tangentY = (-normalX);
            var rtA = rAX * tangentY - rAY * tangentX;
            var rtB = rBX * tangentY - rBY * tangentX;
            rtA *= rtA;
            rtB *= rtB;
            var kTangent = bodyA.m_invMass + bodyB.m_invMass + bodyA.m_invI * rtA + bodyB.m_invI * rtB;
            ccp.tangentMass = 1.0 / kTangent;
            ccp.velocityBias = 0.0;
            var tX = vBX + ((-wB * rBY)) - vAX - ((-wA * rAY));
            var tY = vBY + (wB * rBX) - vAY - (wA * rAX);
            var vRel = cc.normal.x * tX + cc.normal.y * tY;
            if (vRel < (-box2d.common.Settings.velocityThreshold)) {
                ccp.velocityBias += (-cc.restitution * vRel);
            }
        }
        if (cc.pointCount == 2) {
            var ccp1 = cc.points[0];
            var ccp2 = cc.points[1];
            var invMassA = bodyA.m_invMass;
            var invIA = bodyA.m_invI;
            var invMassB = bodyB.m_invMass;
            var invIB = bodyB.m_invI;
            var rn1A = ccp1.rA.x * normalY - ccp1.rA.y * normalX;
            var rn1B = ccp1.rB.x * normalY - ccp1.rB.y * normalX;
            var rn2A = ccp2.rA.x * normalY - ccp2.rA.y * normalX;
            var rn2B = ccp2.rB.x * normalY - ccp2.rB.y * normalX;
            var k11 = invMassA + invMassB + invIA * rn1A * rn1A + invIB * rn1B * rn1B;
            var k22 = invMassA + invMassB + invIA * rn2A * rn2A + invIB * rn2B * rn2B;
            var k12 = invMassA + invMassB + invIA * rn1A * rn2A + invIB * rn1B * rn2B;
            var k_maxConditionNumber = 100.0;
            if (k11 * k11 < k_maxConditionNumber * (k11 * k22 - k12 * k12)) {
                cc.K.col1.set(k11, k12);
                cc.K.col2.set(k12, k22);
                cc.K.getInverse(cc.normalMass);
            } else {
                cc.pointCount = 1;
            }
        }
    }
};

/**
 * @param {!box2d.dynamics.TimeStep} step
 */
box2d.dynamics.contacts.Contactsolver.prototype.initVelocityConstraints = function(step) {
    for (var i = 0; i < this.m_constraintCount; ++i) {
        var c = this.m_constraints[i];
        var bodyA = c.bodyA;
        var bodyB = c.bodyB;
        var invMassA = bodyA.m_invMass;
        var invIA = bodyA.m_invI;
        var invMassB = bodyB.m_invMass;
        var invIB = bodyB.m_invI;
        var normalX = c.normal.x;
        var normalY = c.normal.y;
        var tangentX = normalY;
        var tangentY = (-normalX);
        var tX = 0;
        var j = 0;
        var tCount = 0;
        if (step.warmStarting) {
            tCount = c.pointCount;
            for (j = 0; j < tCount; ++j) {
                var ccp = c.points[j];
                ccp.normalImpulse *= step.dtRatio;
                ccp.tangentImpulse *= step.dtRatio;
                var PX = ccp.normalImpulse * normalX + ccp.tangentImpulse * tangentX;
                var PY = ccp.normalImpulse * normalY + ccp.tangentImpulse * tangentY;
                bodyA.m_angularVelocity -= invIA * (ccp.rA.x * PY - ccp.rA.y * PX);
                bodyA.m_linearVelocity.x -= invMassA * PX;
                bodyA.m_linearVelocity.y -= invMassA * PY;
                bodyB.m_angularVelocity += invIB * (ccp.rB.x * PY - ccp.rB.y * PX);
                bodyB.m_linearVelocity.x += invMassB * PX;
                bodyB.m_linearVelocity.y += invMassB * PY;
            }
        } else {
            tCount = c.pointCount;
            for (j = 0; j < tCount; ++j) {
                var ccp2 = c.points[j];
                ccp2.normalImpulse = 0.0;
                ccp2.tangentImpulse = 0.0;
            }
        }
    }
};

box2d.dynamics.contacts.Contactsolver.prototype.solveVelocityConstraints = function() {
    for (var i = 0; i < this.m_constraintCount; i++) {
        this.solveVelocityConstraints_Constraint(this.m_constraints[i]);
    }
};

/**
 * @param {!box2d.dynamics.contacts.ContactConstraint} c
 */
box2d.dynamics.contacts.Contactsolver.prototype.solveVelocityConstraints_Constraint = function(c) {
    var normalX = c.normal.x;
    var normalY = c.normal.y;

    for (var j = 0; j < c.pointCount; j++) {
        box2d.dynamics.contacts.Contactsolver.prototype.solveVelocityConstraints_ConstraintPoint(c, c.points[j]);
    }
    if (c.pointCount == 1) {
        var ccp = c.points[0];
        var dvX = c.bodyB.m_linearVelocity.x - (c.bodyB.m_angularVelocity * ccp.rB.y) - c.bodyA.m_linearVelocity.x + (c.bodyA.m_angularVelocity * ccp.rA.y);
        var dvY = c.bodyB.m_linearVelocity.y + (c.bodyB.m_angularVelocity * ccp.rB.x) - c.bodyA.m_linearVelocity.y - (c.bodyA.m_angularVelocity * ccp.rA.x);
        var vn = dvX * normalX + dvY * normalY;
        var newImpulse = ccp.normalImpulse - (ccp.normalMass * (vn - ccp.velocityBias));
        newImpulse = newImpulse > 0 ? newImpulse : 0.0;
        var impulseLambda = newImpulse - ccp.normalImpulse;
        var PX = impulseLambda * normalX;
        var PY = impulseLambda * normalY;
        c.bodyA.m_linearVelocity.x -= c.bodyA.m_invMass * PX;
        c.bodyA.m_linearVelocity.y -= c.bodyA.m_invMass * PY;
        c.bodyA.m_angularVelocity -= c.bodyA.m_invI * (ccp.rA.x * PY - ccp.rA.y * PX);
        c.bodyB.m_linearVelocity.x += c.bodyB.m_invMass * PX;
        c.bodyB.m_linearVelocity.y += c.bodyB.m_invMass * PY;
        c.bodyB.m_angularVelocity += c.bodyB.m_invI * (ccp.rB.x * PY - ccp.rB.y * PX);
        ccp.normalImpulse = newImpulse;
    } else {
        var cp1 = c.points[0];
        var cp2 = c.points[1];
        var aX = cp1.normalImpulse;
        var aY = cp2.normalImpulse;
        var dv1X = c.bodyB.m_linearVelocity.x - c.bodyB.m_angularVelocity * cp1.rB.y - c.bodyA.m_linearVelocity.x + c.bodyA.m_angularVelocity * cp1.rA.y;
        var dv1Y = c.bodyB.m_linearVelocity.y + c.bodyB.m_angularVelocity * cp1.rB.x - c.bodyA.m_linearVelocity.y - c.bodyA.m_angularVelocity * cp1.rA.x;
        var dv2X = c.bodyB.m_linearVelocity.x - c.bodyB.m_angularVelocity * cp2.rB.y - c.bodyA.m_linearVelocity.x + c.bodyA.m_angularVelocity * cp2.rA.y;
        var dv2Y = c.bodyB.m_linearVelocity.y + c.bodyB.m_angularVelocity * cp2.rB.x - c.bodyA.m_linearVelocity.y - c.bodyA.m_angularVelocity * cp2.rA.x;
        var bX = (dv1X * normalX + dv1Y * normalY) - cp1.velocityBias;
        var bY = (dv2X * normalX + dv2Y * normalY) - cp2.velocityBias;
        bX -= c.K.col1.x * aX + c.K.col2.x * aY;
        bY -= c.K.col1.y * aX + c.K.col2.y * aY;
        for (;;) {
            var firstX = (-(c.normalMass.col1.x * bX + c.normalMass.col2.x * bY));
            if (firstX >= 0) {
                var firstY = (-(c.normalMass.col1.y * bX + c.normalMass.col2.y * bY));
                if (firstY >= 0) {
                    var dX = firstX - aX;
                    var dY = firstY - aY;
                    this.solveVelocityConstraints_ConstraintPointUpdate(c, cp1, cp2, firstX - aX, firstY - aY);
                    cp1.normalImpulse = firstX;
                    cp2.normalImpulse = firstY;
                    break;
                }
            }
            var secondX = (-cp1.normalMass * bX);
            if (secondX >= 0) {
                if ((c.K.col1.y * secondX + bY) >= 0) {
                    var dX = secondX - aX;
                    var dY = -aY;
                    this.solveVelocityConstraints_ConstraintPointUpdate(c, cp1, cp2, secondX - aX, -aY);
                    cp1.normalImpulse = secondX;
                    cp2.normalImpulse = 0;
                    break;
                }
            }
            var secondY = (-cp2.normalMass * bY);
            if (secondY >= 0) {
                if ((c.K.col2.x * secondY + bX) >= 0) {
                    this.solveVelocityConstraints_ConstraintPointUpdate(c, cp1, cp2, -aX, secondY - aY);
                    cp1.normalImpulse = 0;
                    cp2.normalImpulse = secondY;
                    break;
                }
            }
            if (bX >= 0 && bY >= 0) {
                this.solveVelocityConstraints_ConstraintPointUpdate(c, cp1, cp2, -aX, -aY);
                cp1.normalImpulse = 0;
                cp2.normalImpulse = 0;
                break;
            }
            break;
        }
    }
};

/**
 * @param {!box2d.dynamics.contacts.ContactConstraint} c
 * @param {!box2d.dynamics.contacts.ContactConstraintPoint} ccp
 */
box2d.dynamics.contacts.Contactsolver.prototype.solveVelocityConstraints_ConstraintPoint = function(c, ccp) {
    var tangentX = c.normal.y;
    var tangentY = -c.normal.x;
    var dvX = c.bodyB.m_linearVelocity.x - c.bodyB.m_angularVelocity * ccp.rB.y - c.bodyA.m_linearVelocity.x + c.bodyA.m_angularVelocity * ccp.rA.y;
    var dvY = c.bodyB.m_linearVelocity.y + c.bodyB.m_angularVelocity * ccp.rB.x - c.bodyA.m_linearVelocity.y - c.bodyA.m_angularVelocity * ccp.rA.x;
    var vt = dvX * tangentX + dvY * tangentY;
    var maxFriction = c.friction * ccp.normalImpulse;
    var newImpulse = box2d.common.math.Math.clamp(ccp.tangentImpulse - ccp.tangentMass * vt, -maxFriction, maxFriction);
    var impulseLambda = newImpulse - ccp.tangentImpulse;
    var PX = impulseLambda * tangentX;
    var PY = impulseLambda * tangentY;
    c.bodyA.m_linearVelocity.x -= c.bodyA.m_invMass * PX;
    c.bodyA.m_linearVelocity.y -= c.bodyA.m_invMass * PY;
    c.bodyA.m_angularVelocity -= c.bodyA.m_invI * (ccp.rA.x * PY - ccp.rA.y * PX);
    c.bodyB.m_linearVelocity.x += c.bodyB.m_invMass * PX;
    c.bodyB.m_linearVelocity.y += c.bodyB.m_invMass * PY;
    c.bodyB.m_angularVelocity += c.bodyB.m_invI * (ccp.rB.x * PY - ccp.rB.y * PX);
    ccp.tangentImpulse = newImpulse;
};

/**
 * @param {!box2d.dynamics.contacts.ContactConstraint} c
 * @param {!box2d.dynamics.contacts.ContactConstraintPoint} cp1
 * @param {!box2d.dynamics.contacts.ContactConstraintPoint} cp2
 * @param {number} dX
 * @param {number} dY
 */
box2d.dynamics.contacts.Contactsolver.prototype.solveVelocityConstraints_ConstraintPointUpdate = function(c, cp1, cp2, dX, dY) {
    var P1X = dX * c.normal.x;
    var P1Y = dX * c.normal.y;
    var P2X = dY * c.normal.x;
    var P2Y = dY * c.normal.y;
    c.bodyA.m_linearVelocity.x -= c.bodyA.m_invMass * (P1X + P2X);
    c.bodyA.m_linearVelocity.y -= c.bodyA.m_invMass * (P1Y + P2Y);
    c.bodyA.m_angularVelocity -= c.bodyA.m_invI * (cp1.rA.x * P1Y - cp1.rA.y * P1X + cp2.rA.x * P2Y - cp2.rA.y * P2X);
    c.bodyB.m_linearVelocity.x += c.bodyB.m_invMass * (P1X + P2X);
    c.bodyB.m_linearVelocity.y += c.bodyB.m_invMass * (P1Y + P2Y);
    c.bodyB.m_angularVelocity += c.bodyB.m_invI * (cp1.rB.x * P1Y - cp1.rB.y * P1X + cp2.rB.x * P2Y - cp2.rB.y * P2X);
    cp1.normalImpulse = 0;
    cp2.normalImpulse = 0;
};

box2d.dynamics.contacts.Contactsolver.prototype.finalizeVelocityConstraints = function() {
    for (var i = 0; i < this.m_constraintCount; ++i) {
        var c = this.m_constraints[i];
        var m = c.manifold;
        for (var j = 0; j < c.pointCount; ++j) {
            var point1 = m.m_points[j];
            var point2 = c.points[j];
            point1.m_normalImpulse = point2.normalImpulse;
            point1.m_tangentImpulse = point2.tangentImpulse;
        }
    }
};

box2d.dynamics.contacts.Contactsolver.prototype.solvePositionConstraints = function(baumgarte) {
    if (baumgarte === undefined) baumgarte = 0;
    var minSeparation = 0.0;
    for (var i = 0; i < this.m_constraintCount; i++) {
        var c = this.m_constraints[i];
        var bodyA = c.bodyA;
        var bodyB = c.bodyB;
        var invMassA = bodyA.m_mass * bodyA.m_invMass;
        var invIA = bodyA.m_mass * bodyA.m_invI;
        var invMassB = bodyB.m_mass * bodyB.m_invMass;
        var invIB = bodyB.m_mass * bodyB.m_invI;
        box2d.dynamics.contacts.Contactsolver.s_psm.initialize(c);
        var normal = box2d.dynamics.contacts.Contactsolver.s_psm.m_normal;
        for (var j = 0; j < c.pointCount; j++) {
            var ccp = c.points[j];
            var point = box2d.dynamics.contacts.Contactsolver.s_psm.m_points[j];
            var separation = box2d.dynamics.contacts.Contactsolver.s_psm.m_separations[j];
            var rAX = point.x - bodyA.m_sweep.c.x;
            var rAY = point.y - bodyA.m_sweep.c.y;
            var rBX = point.x - bodyB.m_sweep.c.x;
            var rBY = point.y - bodyB.m_sweep.c.y;
            minSeparation = minSeparation < separation ? minSeparation : separation;
            var C = box2d.common.math.Math.clamp(baumgarte * (separation + box2d.common.Settings.linearSlop), (-box2d.common.Settings.bmaxLinearCorrection), 0.0);
            var impulse = (-ccp.equalizedMass * C);
            var PX = impulse * normal.x;
            var PY = impulse * normal.y;
            bodyA.m_sweep.c.x -= invMassA * PX;
            bodyA.m_sweep.c.y -= invMassA * PY;
            bodyA.m_sweep.a -= invIA * (rAX * PY - rAY * PX);
            bodyA.synchronizeTransform();
            bodyB.m_sweep.c.x += invMassB * PX;
            bodyB.m_sweep.c.y += invMassB * PY;
            bodyB.m_sweep.a += invIB * (rBX * PY - rBY * PX);
            bodyB.synchronizeTransform();
        }
    }
    return minSeparation > (-1.5 * box2d.common.Settings.linearSlop);
};

box2d.dynamics.contacts.Contactsolver.prototype.solvePositionConstraints_NEW = function(baumgarte) {
    if (baumgarte === undefined) baumgarte = 0;
    var minSeparation = 0.0;
    for (var i = 0; i < this.m_constraintCount; i++) {
        var c = this.m_constraints[i];
        var bodyA = c.bodyA;
        var bodyB = c.bodyB;
        var invMassA = bodyA.m_mass * bodyA.m_invMass;
        var invIA = bodyA.m_mass * bodyA.m_invI;
        var invMassB = bodyB.m_mass * bodyB.m_invMass;
        var invIB = bodyB.m_mass * bodyB.m_invI;
        box2d.dynamics.contacts.Contactsolver.s_psm.initialize(c);
        var normal = box2d.dynamics.contacts.Contactsolver.s_psm.m_normal;
        for (var j = 0; j < c.pointCount; j++) {
            var ccp = c.points[j];
            var point = box2d.dynamics.contacts.Contactsolver.s_psm.m_points[j];
            var separation = box2d.dynamics.contacts.Contactsolver.s_psm.m_separations[j];
            var rAX = point.x - bodyA.m_sweep.c.x;
            var rAY = point.y - bodyA.m_sweep.c.y;
            var rBX = point.x - bodyB.m_sweep.c.x;
            var rBY = point.y - bodyB.m_sweep.c.y;
            if (separation < minSeparation) {
                minSeparation = separation;
            }
            var C = 0;
            if (baumgarte != 0) {
                box2d.common.math.Math.clamp(baumgarte * (separation + box2d.common.Settings.linearSlop), (-box2d.common.Settings.bmaxLinearCorrection), 0.0);
            }
            var impulse = (-ccp.equalizedMass * C);
            var PX = impulse * normal.x;
            var PY = impulse * normal.y;
            bodyA.m_sweep.c.x -= invMassA * PX;
            bodyA.m_sweep.c.y -= invMassA * PY;
            bodyA.m_sweep.a -= invIA * (rAX * PY - rAY * PX);
            bodyA.synchronizeTransform();
            bodyB.m_sweep.c.x += invMassB * PX;
            bodyB.m_sweep.c.y += invMassB * PY;
            bodyB.m_sweep.a += invIB * (rBX * PY - rBY * PX);
            bodyB.synchronizeTransform();
        }
    }
    return minSeparation > (-1.5 * box2d.common.Settings.linearSlop);
};

box2d.dynamics.contacts.Contactsolver.s_worldManifold = new box2d.collision.WorldManifold();
box2d.dynamics.contacts.Contactsolver.s_psm = new box2d.dynamics.contacts.PositionsolverManifold();
