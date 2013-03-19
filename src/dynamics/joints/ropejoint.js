goog.provide('box2d.dynamics.joints.RopeJoint');

goog.require('box2d.dynamics.joints.Joint');

/**
 * @constructor
 * @extends {box2d.dynamics.joints.Joint}
 * @param {!box2d.dynamics.joints.RopeJointDef} def
 */
box2d.dynamics.joints.RopeJoint = function(def) {
    goog.base(this, def);

    this.m_localAnchor1 = box2d.common.math.Vec2.get(0, 0);
    this.m_localAnchor2 = box2d.common.math.Vec2.get(0, 0);
    this.m_u = box2d.common.math.Vec2.get(0, 0);
    this.m_impulse = 0.0;
    this.m_mass = 0;
    this.m_length = 0;
    this.m_maxlength = def.maxlength;

    this.m_localAnchor1.setV(def.localAnchorA);
    this.m_localAnchor2.setV(def.localAnchorB);

    this.m_state = box2d.dynamics.joints.Joint.e_inactiveLimit;
};
goog.inherits(box2d.dynamics.joints.RopeJoint, box2d.dynamics.joints.Joint);

/**
 * @inheritDoc
 */
box2d.dynamics.joints.RopeJoint.prototype.initVelocityConstraints = function(step) {
    var tMat;
    var tX;
    var bA = this.m_bodyA;
    var bB = this.m_bodyB;

    tMat = bA.getTransform().R;
    var r1X = this.m_localAnchor1.x - bA.getSweep().localCenter.x;
    var r1Y = this.m_localAnchor1.y - bA.getSweep().localCenter.y;
    tX = (tMat.col1.x * r1X + tMat.col2.x * r1Y);
    r1Y = (tMat.col1.y * r1X + tMat.col2.y * r1Y);
    r1X = tX;
    tMat = bB.getTransform().R;
    var r2X = this.m_localAnchor2.x - bB.getSweep().localCenter.x;
    var r2Y = this.m_localAnchor2.y - bB.getSweep().localCenter.y;
    tX = (tMat.col1.x * r2X + tMat.col2.x * r2Y);
    r2Y = (tMat.col1.y * r2X + tMat.col2.y * r2Y);
    r2X = tX;

    this.m_u.x = bB.getSweep().c.x + r2X - bA.getSweep().c.x - r1X;
    this.m_u.y = bB.getSweep().c.y + r2Y - bA.getSweep().c.y - r1Y;

    this.m_length = Math.sqrt(this.m_u.x * this.m_u.x + this.m_u.y * this.m_u.y);

    var c = this.m_length - this.m_maxlength;

    if (c > 0) {
        this.m_state = box2d.dynamics.joints.Joint.e_atUpperLimit;
    }
    else {
        this.m_state = box2d.dynamics.joints.Joint.e_inactiveLimit;
    }

    if (this.m_length > box2d.common.Settings.linearSlop) {
        this.m_u.multiply(1.0 / this.m_length);
    }
    else {
        this.m_u.setZero();
        this.m_mass = 0.0;
        this.m_impulse = 0.0;
        return;
    }

    var crA = (r1X * this.m_u.y - r1Y * this.m_u.x);
    var crB = (r2X * this.m_u.y - r2Y * this.m_u.x);
    var invMass = bA.getInvertedMass() + bA.getInvertedInertia() * crA * crA + bB.getInvertedMass() + bB.getInvertedInertia() * crB * crB;
    this.m_mass = invMass != 0.0 ? 1.0 / invMass : 0.0;

    if (step.warmStarting) {
        this.m_impulse *= step.dtRatio;

        var PX = this.m_impulse * this.m_u.x;
        var PY = this.m_impulse * this.m_u.y;
        bA.getLinearVelocity().x -= bA.getInvertedMass() * PX;
        bA.getLinearVelocity().y -= bA.getInvertedMass() * PY;
        bA.m_angularVelocity -= bA.getInvertedInertia() * (r1X * PY - r1Y * PX);
        bB.getLinearVelocity().x += bB.getInvertedMass() * PX;
        bB.getLinearVelocity().y += bB.getInvertedMass() * PY;
        bB.m_angularVelocity += bB.getInvertedInertia() * (r2X * PY - r2Y * PX);
    }
};

/**
 * @inheritDoc
 */
box2d.dynamics.joints.RopeJoint.prototype.solveVelocityConstraints = function(step) {
    var r1X, r1Y, r2X, r2Y, tX, v1X, v1Y, v2X, v2Y, impulse, C, Cdot, oldImpulse, PX, PY, tMat,
        bA = this.m_bodyA,
        bB = this.m_bodyB;

    tMat = bA.getTransform().R;
    r1X = this.m_localAnchor1.x - bA.getLocalCenter().x;
    r1Y = this.m_localAnchor1.y - bA.getLocalCenter().y;
    tX = (tMat.col1.x * r1X + tMat.col2.x * r1Y);
    r1Y = (tMat.col1.y * r1X + tMat.col2.y * r1Y);
    r1X = tX;

    tMat = bB.getTransform().R;
    r2X = this.m_localAnchor2.x - bB.getLocalCenter().x;
    r2Y = this.m_localAnchor2.y - bB.getLocalCenter().x;
    tX = (tMat.col1.x * r2X + tMat.col2.x * r2Y);
    r2Y = (tMat.col1.y * r2X + tMat.col2.y * r2Y);
    r2X = tX;

    v1X = bA.getLinearVelocity().x + (-bA.getAngularVelocity() * r1Y);
    v1Y = bA.getLinearVelocity().y + (bA.getAngularVelocity() * r1X);

    v2X = bB.getLinearVelocity().x + (-bB.getAngularVelocity() * r2Y);
    v2Y = bB.getLinearVelocity().y + (bB.getAngularVelocity() * r2X);

    C = this.m_length - this.m_maxlength;
    Cdot = (this.m_u.x * (v2X - v1X) + this.m_u.y * (v2Y - v1Y));
    if (C < 0) {
        Cdot += step.inv_dt * C;
    }

    impulse = -this.m_mass * Cdot;
    oldImpulse = this.m_impulse;
    this.m_impulse = Math.min(0, this.m_impulse + impulse);
    impulse = this.m_impulse - oldImpulse;

    PX = impulse * this.m_u.x;
    PY = impulse * this.m_u.y;

    bA.getLinearVelocity().x -= bA.getInvertedMass() * PX;
    bA.getLinearVelocity().y -= bA.getInvertedMass() * PY;

    bA.m_angularVelocity -= bA.getInvertedInertia() * (r1X * PY - r1Y * PX);

    bB.getLinearVelocity().x += bB.getInvertedMass() * PX;
    bB.getLinearVelocity().y += bB.getInvertedMass() * PY;

    bB.m_angularVelocity += bB.getInvertedInertia() * (r2X * PY - r2Y * PX);
};

/**
 * @inheritDoc
 */
box2d.dynamics.joints.RopeJoint.prototype.solvePositionConstraints = function(baumgarte) {
    var tMat, r1X, r1Y, tX, r2X, r2Y, dX, dY, length, C, impulse, PX, PY,
        bA = this.m_bodyA,
        bB = this.m_bodyB;

    tMat = bA.getTransform().R;
    r1X = this.m_localAnchor1.x - bA.getLocalCenter().x;
    r1Y = this.m_localAnchor1.y - bA.getLocalCenter().y;
    tX = (tMat.col1.x * r1X + tMat.col2.x * r1Y);
    r1Y = (tMat.col1.y * r1X + tMat.col2.y * r1Y);
    r1X = tX;

    tMat = bB.getTransform().R;
    r2X = this.m_localAnchor2.x - bB.getLocalCenter().x;
    r2Y = this.m_localAnchor2.y - bB.getLocalCenter().y;
    tX = (tMat.col1.x * r2X + tMat.col2.x * r2Y);
    r2Y = (tMat.col1.y * r2X + tMat.col2.y * r2Y);
    r2X = tX;

    dX = bB.getWorldCenter().x + r2X - bA.getWorldCenter().x - r1X;
    dY = bB.getWorldCenter().y + r2Y - bA.getWorldCenter().y - r1Y;

    length = Math.sqrt(dX * dX + dY * dY);
    if (length == 0) {
        length = 1;
    }
    dX /= length;
    dY /= length;

    C = length - this.m_maxlength;
    C = box2d.common.math.Math.clamp(C, 0, box2d.common.Settings.bmaxLinearCorrection);

    impulse = -this.m_mass * C;
    this.m_u.set(dX, dY);
    PX = impulse * this.m_u.x;
    PY = impulse * this.m_u.y;

    bA.getWorldCenter().x -= bA.getInvertedMass() * PX;
    bA.getWorldCenter().y -= bA.getInvertedMass() * PY;

    bA.getSweep().a -= bA.getInvertedInertia() * (r1X * PY - r1Y * PX);

    bB.getWorldCenter().x += bB.getInvertedMass() * PX;
    bB.getWorldCenter().y += bB.getInvertedMass() * PY;
    bB.getSweep().a += bB.getInvertedInertia() * (r2X * PY - r2Y * PX);

    bA.synchronizeTransform();
    bB.synchronizeTransform();

    return length - this.m_maxlength < box2d.common.Settings.linearSlop;
};

/**
 * @inheritDoc
 */
box2d.dynamics.joints.RopeJoint.prototype.getAnchorA = function() {
    return this.m_bodyA.getWorldPoint(this.m_localAnchor1);
};

/**
 * @inheritDoc
 */
box2d.dynamics.joints.RopeJoint.prototype.getAnchorB = function() {
    return this.m_bodyB.getWorldPoint(this.m_localAnchor2);
};

/**
 *  @inheritDoc
 */
box2d.dynamics.joints.RopeJoint.prototype.getReactionForce = function(inv_dt) {
    return box2d.common.math.Vec2.get(inv_dt * this.m_impulse * this.m_u.x, inv_dt * this.m_impulse * this.m_u.y);
};

/**
 * @inheritDoc
 */
box2d.dynamics.joints.RopeJoint.prototype.getReactionTorque = function(inv_dt) {
    return 0.0;
};

/**
 * @return {number}
 */
box2d.dynamics.joints.RopeJoint.prototype.getMaxLength = function() {
    return this.m_maxlength;
};

/**
 * @return {number}
 */
box2d.dynamics.joints.RopeJoint.prototype.getLimitState = function() {
    return this.m_state;
};
