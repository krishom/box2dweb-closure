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

goog.provide('box2d.dynamics.Island');

goog.require('UsageTracker');
goog.require('box2d.common.Settings');
goog.require('box2d.common.math.Math');
goog.require('box2d.dynamics.BodyDef');
goog.require('box2d.dynamics.ContactImpulse');

/**
 * @param {!box2d.dynamics.IContactListener} listener
 * @param {!box2d.dynamics.contacts.Contactsolver} contactsolver
 * @constructor
 */
box2d.dynamics.Island = function(listener, contactsolver) {
    UsageTracker.get('box2d.dynamics.Island').trackCreate();

    /**
     * @type {!box2d.dynamics.IContactListener}
     */
    this.m_listener = listener;

    /**
     * @type {!box2d.dynamics.contacts.Contactsolver}
     */
    this.m_contactsolver = contactsolver;

    /**
     * @type {Array.<!box2d.dynamics.Body>}
     */
    this.m_bodies = [];

    /**
     * @type {Array.<!box2d.dynamics.Body>}
     */
    this.m_dynamicBodies = [];

    /**
     * @type {Array.<!box2d.dynamics.Body>}
     */
    this.m_nonStaticBodies = [];

    /**
     * @type {Array.<!box2d.dynamics.contacts.Contact>}
     */
    this.m_contacts = [];

    /**
     * @type {Array.<!box2d.dynamics.joints.Joint>}
     */
    this.m_joints = [];

    /**
     * @type {!box2d.dynamics.ContactImpulse}
     * @const
     */
    this.impulse = new box2d.dynamics.ContactImpulse();
};

/**
 * @param {!box2d.dynamics.IContactListener} listener
 * @param {!box2d.dynamics.contacts.Contactsolver} contactsolver
 */
box2d.dynamics.Island.prototype.reset = function(listener, contactsolver) {
    this.m_listener = listener;
    this.m_contactsolver = contactsolver;
};

box2d.dynamics.Island.prototype.clear = function() {
    this.m_bodies = [];
    this.m_dynamicBodies = [];
    this.m_nonStaticBodies = [];
    this.m_contacts = [];
    this.m_joints = [];
};

/**
 * @param {!box2d.dynamics.TimeStep} step
 * @param {!box2d.common.math.Vec2} gravity
 * @param {boolean} allowSleep
 */
box2d.dynamics.Island.prototype.solve = function(step, gravity, allowSleep) {
    this._initializeVelocities(step, gravity);
    this.m_contactsolver.initialize(step, this.m_contacts, this.m_contacts.length);
    this._solveVelocityConstraints(step);
    this._solveBodies(step);
    this._solvePositionConstraints(step);
    this.report(this.m_contactsolver.m_constraints);
    if (allowSleep) {
        this._sleepIfTired(step);
    }
};

/**
 * @param {!box2d.dynamics.TimeStep} step
 * @param {!box2d.common.math.Vec2} gravity
 * @private
 */
box2d.dynamics.Island.prototype._initializeVelocities = function(step, gravity) {
    for (var i = 0; i < this.m_dynamicBodies.length; i++) {
        var b = this.m_dynamicBodies[i];
        b.m_linearVelocity.x += step.dt * (gravity.x + b.m_invMass * b.m_force.x);
        b.m_linearVelocity.y += step.dt * (gravity.y + b.m_invMass * b.m_force.y);
        b.m_angularVelocity += step.dt * b.m_invI * b.m_torque;
        b.m_linearVelocity.multiply(box2d.common.math.Math.clamp(1.0 - step.dt * b.m_linearDamping, 0.0, 1.0));
        b.m_angularVelocity *= box2d.common.math.Math.clamp(1.0 - step.dt * b.m_angularDamping, 0.0, 1.0);
    }
};

/**
 * @param {!box2d.dynamics.TimeStep} step
 * @private
 */
box2d.dynamics.Island.prototype._solveVelocityConstraints = function(step) {
    this.m_contactsolver.initVelocityConstraints(step);
    for (var jointinitIdx = 0; jointinitIdx < this.m_joints.length; jointinitIdx++) {
        this.m_joints[jointinitIdx].initVelocityConstraints(step);
    }
    for (var velocityIterationCnt = 0; velocityIterationCnt < step.velocityIterations; velocityIterationCnt++) {
        for (var jointsolveIdx = 0; jointsolveIdx < this.m_joints.length; jointsolveIdx++) {
            this.m_joints[jointsolveIdx].solveVelocityConstraints(step);
        }
        this.m_contactsolver.solveVelocityConstraints();
    }
    for (var jointfinalizeIdx = 0; jointfinalizeIdx < this.m_joints.length; jointfinalizeIdx++) {
        this.m_joints[jointfinalizeIdx].finalizeVelocityConstraints();
    }
    this.m_contactsolver.finalizeVelocityConstraints();
};

/**
 * @param {!box2d.dynamics.TimeStep} step
 * @private
 */
box2d.dynamics.Island.prototype._solveBodies = function(step) {
    for (var i = 0; i < this.m_nonStaticBodies.length; ++i) {
        var b = this.m_nonStaticBodies[i];
        var translationX = step.dt * b.m_linearVelocity.x;
        var translationY = step.dt * b.m_linearVelocity.y;
        if ((translationX * translationX + translationY * translationY) > box2d.common.Settings.maxTranslationSquared) {
            b.m_linearVelocity.normalize();
            b.m_linearVelocity.x *= box2d.common.Settings.maxTranslation * step.inv_dt;
            b.m_linearVelocity.y *= box2d.common.Settings.maxTranslation * step.inv_dt;
        }
        var rotation = step.dt * b.m_angularVelocity;
        if (rotation * rotation > box2d.common.Settings.maxRotationSquared) {
            if (b.m_angularVelocity < 0.0) {
                b.m_angularVelocity = -box2d.common.Settings.maxRotation * step.inv_dt;
            } else {
                b.m_angularVelocity = box2d.common.Settings.maxRotation * step.inv_dt;
            }
        }
        b.m_sweep.c0.setV(b.m_sweep.c);
        b.m_sweep.a0 = b.m_sweep.a;
        b.m_sweep.c.x += step.dt * b.m_linearVelocity.x;
        b.m_sweep.c.y += step.dt * b.m_linearVelocity.y;
        b.m_sweep.a += step.dt * b.m_angularVelocity;
        b.synchronizeTransform();
    }
};

/**
 * @param {!box2d.dynamics.TimeStep} step
 * @private
 */
box2d.dynamics.Island.prototype._solvePositionConstraints = function(step) {
    for (var i = 0; i < step.positionIterations; i++) {
        var contactsOkay = this.m_contactsolver.solvePositionConstraints(box2d.common.Settings.contactBaumgarte);
        var jointsOkay = true;
        for (var j = 0; j < this.m_joints.length; j++) {
            var jointOkay = this.m_joints[j].solvePositionConstraints(box2d.common.Settings.contactBaumgarte);
            jointsOkay = jointsOkay && jointOkay;
        }
        if (contactsOkay && jointsOkay) {
            break;
        }
    }
};

/**
 * @param {!box2d.dynamics.TimeStep} step
 * @private
 */
box2d.dynamics.Island.prototype._sleepIfTired = function(step) {
    var minSleepTime = Number.MAX_VALUE;
    for (var nonstaticBodyIdx = 0; nonstaticBodyIdx < this.m_nonStaticBodies.length; nonstaticBodyIdx++) {
        var b = this.m_nonStaticBodies[nonstaticBodyIdx];
        if (!b.m_allowSleep || Math.abs(b.m_angularVelocity) > box2d.common.Settings.angularSleepTolerance || box2d.common.math.Math.Dot(b.m_linearVelocity, b.m_linearVelocity) > box2d.common.Settings.linearSleepToleranceSquared) {
            b.m_sleepTime = 0.0;
            minSleepTime = 0.0;
        } else {
            b.m_sleepTime += step.dt;
            minSleepTime = Math.min(minSleepTime, b.m_sleepTime);
        }
    }
    if (minSleepTime >= box2d.common.Settings.timeToSleep) {
        for (var bodyIdx = 0; bodyIdx < this.m_bodies.length; bodyIdx++) {
            this.m_bodies[bodyIdx].setAwake(false);
        }
    }
};

/**
 * @param {!box2d.dynamics.TimeStep} substep
 */
box2d.dynamics.Island.prototype.solveTOI = function(substep) {
    var i = 0;
    var j = 0;
    this.m_contactsolver.initialize(substep, this.m_contacts, this.m_contacts.length);
    var contactsolver = this.m_contactsolver;
    for (i = 0; i < this.m_joints.length; ++i) {
        this.m_joints[i].initVelocityConstraints(substep);
    }
    for (i = 0; i < substep.velocityIterations; ++i) {
        contactsolver.solveVelocityConstraints();
        for (j = 0; j < this.m_joints.length; ++j) {
            this.m_joints[j].solveVelocityConstraints(substep);
        }
    }
    for (i = 0; i < this.m_nonStaticBodies.length; ++i) {
        var b = this.m_nonStaticBodies[i];
        var translationX = substep.dt * b.m_linearVelocity.x;
        var translationY = substep.dt * b.m_linearVelocity.y;
        if ((translationX * translationX + translationY * translationY) > box2d.common.Settings.maxTranslationSquared) {
            b.m_linearVelocity.normalize();
            b.m_linearVelocity.x *= box2d.common.Settings.maxTranslation * substep.inv_dt;
            b.m_linearVelocity.y *= box2d.common.Settings.maxTranslation * substep.inv_dt;
        }
        var rotation = substep.dt * b.m_angularVelocity;
        if (rotation * rotation > box2d.common.Settings.maxRotationSquared) {
            if (b.m_angularVelocity < 0.0) {
                b.m_angularVelocity = (-box2d.common.Settings.maxRotation * substep.inv_dt);
            } else {
                b.m_angularVelocity = box2d.common.Settings.maxRotation * substep.inv_dt;
            }
        }
        b.m_sweep.c0.setV(b.m_sweep.c);
        b.m_sweep.a0 = b.m_sweep.a;
        b.m_sweep.c.x += substep.dt * b.m_linearVelocity.x;
        b.m_sweep.c.y += substep.dt * b.m_linearVelocity.y;
        b.m_sweep.a += substep.dt * b.m_angularVelocity;
        b.synchronizeTransform();
    }
    var k_toiBaumgarte = 0.75;
    for (i = 0; i < substep.positionIterations; ++i) {
        var contactsOkay = contactsolver.solvePositionConstraints(k_toiBaumgarte);
        var jointsOkay = true;
        for (j = 0; j < this.m_joints.length; ++j) {
            var jointOkay = this.m_joints[j].solvePositionConstraints(box2d.common.Settings.contactBaumgarte);
            jointsOkay = jointsOkay && jointOkay;
        }
        if (contactsOkay && jointsOkay) {
            break;
        }
    }
    this.report(contactsolver.m_constraints);
};

/**
 * @param {Array.<!box2d.dynamics.contacts.ContactConstraint>} constraints
 */
box2d.dynamics.Island.prototype.report = function(constraints) {
    if (this.m_listener == null) {
        return;
    }
    for (var i = 0; i < this.m_contacts.length; ++i) {
        var c = this.m_contacts[i];
        var cc = constraints[i];
        this.impulse.reset();
        for (var j = 0; j < cc.pointCount; ++j) {
            this.impulse.normalImpulses[j] = cc.points[j].normalImpulse;
            this.impulse.tangentImpulses[j] = cc.points[j].tangentImpulse;
        }
        this.m_listener.postSolve(c, this.impulse);
    }
};

/**
 * @param {!box2d.dynamics.Body} body
 */
box2d.dynamics.Island.prototype.addBody = function(body) {
    this.m_bodies.push(body);
    if (body.getType() != box2d.dynamics.BodyDef.staticBody) {
        this.m_nonStaticBodies.push(body);
        if (body.getType() == box2d.dynamics.BodyDef.dynamicBody) {
            this.m_dynamicBodies.push(body);
        }
    }
};

/**
 * @param {!box2d.dynamics.contacts.Contact} contact
 */
box2d.dynamics.Island.prototype.addContact = function(contact) {
    this.m_contacts.push(contact);
};

/**
 * @param {!box2d.dynamics.joints.Joint} joint
 */
box2d.dynamics.Island.prototype.addJoint = function(joint) {
    this.m_joints.push(joint);
};
