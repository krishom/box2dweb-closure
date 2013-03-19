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

goog.provide('box2d.dynamics.Body');

goog.require('UsageTracker');
goog.require('box2d.common.Settings');
goog.require('box2d.common.math.Math');
goog.require('box2d.common.math.Sweep');
goog.require('box2d.common.math.Transform');
goog.require('box2d.common.math.Vec2');
goog.require('box2d.dynamics.BodyDef');
goog.require('box2d.dynamics.Fixture');
goog.require('box2d.dynamics.FixtureDef');
goog.require('box2d.dynamics.FixtureList');
goog.require('box2d.dynamics.contacts.ContactList');
goog.require('box2d.dynamics.controllers.ControllerList');

/**
 * @param {!box2d.dynamics.BodyDef} bd
 * @param {!box2d.dynamics.World} world
 * @constructor
 */
box2d.dynamics.Body = function(bd, world) {
    UsageTracker.get('box2d.dynamics.Body').trackCreate();

    /**
     * @const
     * @type {string}
     */
    this.ID = 'Body' + box2d.dynamics.Body.NEXT_ID++;

    /**
     * @type {!box2d.common.math.Transform}
     */
    this.m_xf = new box2d.common.math.Transform();
    this.m_xf.position.setV(bd.position);
    this.m_xf.R.set(bd.angle);

    /**
     * @type {!box2d.common.math.Sweep}
     */
    this.m_sweep = new box2d.common.math.Sweep();
    this.m_sweep.localCenter.setZero();
    this.m_sweep.t0 = 1.0;
    this.m_sweep.a0 = this.m_sweep.a = bd.angle;
    this.m_sweep.c.x = (this.m_xf.R.col1.x * this.m_sweep.localCenter.x + this.m_xf.R.col2.x * this.m_sweep.localCenter.y);
    this.m_sweep.c.y = (this.m_xf.R.col1.y * this.m_sweep.localCenter.x + this.m_xf.R.col2.y * this.m_sweep.localCenter.y);
    this.m_sweep.c.x += this.m_xf.position.x;
    this.m_sweep.c.y += this.m_xf.position.y;
    this.m_sweep.c0.setV(this.m_sweep.c);

    /**
     * @type {!box2d.common.math.Vec2}
     */
    this.m_linearVelocity = bd.linearVelocity.copy();

    /**
     * @type {!box2d.common.math.Vec2}
     */
    this.m_force = box2d.common.math.Vec2.get(0, 0);

    /**
     * @type {boolean}
     */
    this.m_bullet = bd.bullet;

    /**
     * @type {boolean}
     */
    this.m_fixedRotation = bd.fixedRotation;

    /**
     * @type {boolean}
     */
    this.m_allowSleep = bd.allowSleep;

    /**
     * @type {boolean}
     */
    this.m_awake = bd.awake;

    /**
     * @type {boolean}
     */
    this.m_active = bd.active;

    /**
     * @type {!box2d.dynamics.World}
     */
    this.m_world = world;

    /**
     * @type {box2d.dynamics.joints.Joint}
     */
    this.m_jointList = null;

    /**
     * @type {!box2d.dynamics.contacts.ContactList}
     */
    this.contactList = new box2d.dynamics.contacts.ContactList();

    /**
     * @type {!box2d.dynamics.controllers.ControllerList}
     */
    this.controllerList = new box2d.dynamics.controllers.ControllerList();

    /**
     * @type {number}
     */
    this.m_controllerCount = 0;

    /**
     * @type {number}
     */
    this.m_angularVelocity = bd.angularVelocity;

    /**
     * @type {number}
     */
    this.m_linearDamping = bd.linearDamping;

    /**
     * @type {number}
     */
    this.m_angularDamping = bd.angularDamping;

    /**
     * @type {number}
     */
    this.m_torque = 0;

    /**
     * @type {number}
     */
    this.m_sleepTime = 0;

    /**
     * @type {number}
     */
    this.m_type = bd.type;

    /**
     * @type {number}
     */
    this.m_mass = this.m_type == box2d.dynamics.BodyDef.dynamicBody ? 1 : 0;

    /**
     * @type {number}
     */
    this.m_invMass = this.m_type == box2d.dynamics.BodyDef.dynamicBody ? 1 : 0;

    /**
     * @type {number}
     */
    this.m_I = 0;

    /**
     * @type {number}
     */
    this.m_invI = 0;

    /**
     * @type {number}
     */
    this.m_inertiaScale = bd.inertiaScale;

    /**
     * @type {!box2d.dynamics.FixtureList}
     */
    this.fixtureList = new box2d.dynamics.FixtureList();

    /**
     * @type {Array.<!box2d.dynamics.BodyList>}
     */
    this.m_lists = [];
};

/**
 * @param {!box2d.dynamics.FixtureDef} def
 * @return {!box2d.dynamics.Fixture}
 */
box2d.dynamics.Body.prototype.createFixture = function(def) {
    box2d.common.Settings.assert(!this.m_world.isLocked());
    var fixture = new box2d.dynamics.Fixture(this, this.m_xf, def);
    if (this.m_active) {
        var broadPhase = this.m_world.m_contactManager.m_broadPhase;
        fixture.createProxy(broadPhase, this.m_xf);
    }
    this.fixtureList.addFixture(fixture);
    fixture.m_body = this;
    if (fixture.m_density > 0.0) {
        this.resetMassData();
    }
    this.m_world.m_newFixture = true;
    return fixture;
};

/**
 * @param {!box2d.collision.shapes.Shape} shape
 * @param {number} density
 * @return {!box2d.dynamics.Fixture}
 */
box2d.dynamics.Body.prototype.createFixture2 = function(shape, density) {
    var def = new box2d.dynamics.FixtureDef();
    def.shape = shape;
    def.density = density;
    return this.createFixture(def);
};

box2d.dynamics.Body.prototype.destroy = function() {
    // These should also be freed
    //this.m_xf = new box2d.common.math.Transform();
    //this.m_sweep = new box2d.common.math.Sweep();
    box2d.common.math.Vec2.free(this.m_linearVelocity);
    box2d.common.math.Vec2.free(this.m_force);
};

/**
 * @param {!box2d.dynamics.Fixture} fixture
 */
box2d.dynamics.Body.prototype.destroyFixture = function(fixture) {
    box2d.common.Settings.assert(!this.m_world.isLocked());
    this.fixtureList.removeFixture(fixture);
    for (var contactNode = this.contactList.getFirstNode(box2d.dynamics.contacts.ContactList.TYPES.allContacts); contactNode; contactNode = contactNode.getNextNode()) {
        if (fixture == contactNode.contact.getFixtureA() || fixture == contactNode.contact.getFixtureB()) {
            this.m_world.m_contactManager.destroy(contactNode.contact);
        }
    }
    if (this.m_active) {
        var broadPhase = this.m_world.m_contactManager.m_broadPhase;
        fixture.destroyProxy(broadPhase);
    }
    fixture.destroy();
    this.resetMassData();
};

/**
 * @param {!box2d.common.math.Vec2} position
 * @param {number} angle
 */
box2d.dynamics.Body.prototype.setPositionAndAngle = function(position, angle) {
    box2d.common.Settings.assert(!this.m_world.isLocked());
    this.m_xf.R.set(angle);
    this.m_xf.position.setV(position);
    var tMat = this.m_xf.R;
    var tVec = this.m_sweep.localCenter;
    this.m_sweep.c.x = (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
    this.m_sweep.c.y = (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
    this.m_sweep.c.x += this.m_xf.position.x;
    this.m_sweep.c.y += this.m_xf.position.y;
    this.m_sweep.c0.setV(this.m_sweep.c);
    this.m_sweep.a0 = this.m_sweep.a = angle;
    var broadPhase = this.m_world.m_contactManager.m_broadPhase;

    for (var node = this.fixtureList.getFirstNode(); node; node = node.getNextNode()) {
        node.fixture.synchronize(broadPhase, this.m_xf, this.m_xf);
    }
    this.m_world.m_contactManager.findNewContacts();
};

/**
 * @param {!box2d.common.math.Transform} xf
 */
box2d.dynamics.Body.prototype.setTransform = function(xf) {
    this.setPositionAndAngle(xf.position, xf.getAngle());
};

/**
 * @return {!box2d.common.math.Transform}
 */
box2d.dynamics.Body.prototype.getTransform = function() {
    return this.m_xf;
};

/**
 * @return {!box2d.common.math.Vec2}
 */
box2d.dynamics.Body.prototype.getPosition = function() {
    return this.m_xf.position;
};

/**
 * @param {!box2d.common.math.Vec2} position
 */
box2d.dynamics.Body.prototype.setPosition = function(position) {
    this.setPositionAndAngle(position, this.getAngle());
};

/**
 * @return {number}
 */
box2d.dynamics.Body.prototype.getAngle = function() {
    return this.m_sweep.a;
};

/**
 * @param {number} angle
 */
box2d.dynamics.Body.prototype.setAngle = function(angle) {
    this.setPositionAndAngle(this.getPosition(), angle);
};

/**
 * @return {!box2d.common.math.Vec2}
 */
box2d.dynamics.Body.prototype.getWorldCenter = function() {
    return this.m_sweep.c;
};

/**
 * @return {!box2d.common.math.Sweep}
 */
box2d.dynamics.Body.prototype.getSweep = function() {
    return this.m_sweep;
};

/**
 * @return {!box2d.common.math.Vec2}
 */
box2d.dynamics.Body.prototype.getLocalCenter = function() {
    return this.m_sweep.localCenter;
};

/**
 * @param {!box2d.common.math.Vec2} v
 */
box2d.dynamics.Body.prototype.setLinearVelocity = function(v) {
    if (this.m_type == box2d.dynamics.BodyDef.staticBody) {
        return;
    }
    this.m_linearVelocity.setV(v);
};

/**
 * @return {!box2d.common.math.Vec2}
 */
box2d.dynamics.Body.prototype.getLinearVelocity = function() {
    return this.m_linearVelocity;
};

/**
 * @param {number} omega
 */
box2d.dynamics.Body.prototype.setAngularVelocity = function(omega) {
    if (this.m_type == box2d.dynamics.BodyDef.staticBody) {
        return;
    }
    this.m_angularVelocity = omega;
};

/**
 * @return {number}
 */
box2d.dynamics.Body.prototype.getAngularVelocity = function() {
    return this.m_angularVelocity;
};

/**
 * @return {!box2d.dynamics.BodyDef}
 */
box2d.dynamics.Body.prototype.getDefinition = function() {
    var bd = new box2d.dynamics.BodyDef();
    bd.type = this.getType();
    bd.allowSleep = this.m_allowSleep;
    bd.angle = this.getAngle();
    bd.angularDamping = this.m_angularDamping;
    bd.angularVelocity = this.m_angularVelocity;
    bd.fixedRotation = this.m_fixedRotation;
    bd.bullet = this.m_bullet;
    bd.active = this.m_active;
    bd.awake = this.m_awake;
    bd.linearDamping = this.m_linearDamping;
    bd.linearVelocity.setV(this.getLinearVelocity());
    bd.position.setV(this.getPosition());
    return bd;
};

/**
 * @param {!box2d.common.math.Vec2} force
 * @param {!box2d.common.math.Vec2} point
 */
box2d.dynamics.Body.prototype.applyForce = function(force, point) {
    if (this.m_type != box2d.dynamics.BodyDef.dynamicBody) {
        return;
    }
    this.setAwake(true);

    this.m_force.x += force.x;
    this.m_force.y += force.y;
    this.m_torque += ((point.x - this.m_sweep.c.x) * force.y - (point.y - this.m_sweep.c.y) * force.x);
};

/**
 * @param {number} torque
 */
box2d.dynamics.Body.prototype.applyTorque = function(torque) {
    if (this.m_type != box2d.dynamics.BodyDef.dynamicBody) {
        return;
    }
    this.setAwake(true);
    this.m_torque += torque;
};

/**
 * @param {!box2d.common.math.Vec2} impulse
 * @param {!box2d.common.math.Vec2} point
 */
box2d.dynamics.Body.prototype.applyImpulse = function(impulse, point) {
    if (this.m_type != box2d.dynamics.BodyDef.dynamicBody) {
        return;
    }
    this.setAwake(true);

    this.m_linearVelocity.x += this.m_invMass * impulse.x;
    this.m_linearVelocity.y += this.m_invMass * impulse.y;
    this.m_angularVelocity += this.m_invI * ((point.x - this.m_sweep.c.x) * impulse.y - (point.y - this.m_sweep.c.y) * impulse.x);
};

/**
 * @param {function(!box2d.dynamics.Fixture):boolean} callback
 * @return {!box2d.dynamics.Body}
 */
box2d.dynamics.Body.prototype.split = function(callback) {
    var linearVelocity = this.getLinearVelocity().copy();
    var angularVelocity = this.getAngularVelocity();
    var center = this.getWorldCenter();
    var body1 = this;
    var body2 = this.m_world.createBody(this.getDefinition());
    for (var node = body1.fixtureList.getFirstNode(); node; node = node.getNextNode()) {
        var f = node.fixture;
        if (callback(f)) {
            body1.fixtureList.removeFixture(f);
            body2.fixtureList.addFixture(f);
        }
    }
    body1.resetMassData();
    body2.resetMassData();
    var center1 = body1.getWorldCenter();
    var center2 = body2.getWorldCenter();
    var center1Diff = box2d.common.math.Math.subtractVV(center1, center);
    var center1Cross = box2d.common.math.Math.crossFV(angularVelocity, center1Diff);
    box2d.common.math.Vec2.free(center1Diff);
    var velocity1 = box2d.common.math.Math.addVV(linearVelocity, center1Cross);
    box2d.common.math.Vec2.free(center1Cross);
    body1.setLinearVelocity(velocity1);
    box2d.common.math.Vec2.free(velocity1);

    var center2Diff = box2d.common.math.Math.subtractVV(center2, center);
    var center2Cross = box2d.common.math.Math.crossFV(angularVelocity, center2Diff);
    box2d.common.math.Vec2.free(center2Diff);
    var velocity2 = box2d.common.math.Math.addVV(linearVelocity, center2Cross);
    box2d.common.math.Vec2.free(center2Cross);
    body2.setLinearVelocity(velocity2);
    box2d.common.math.Vec2.free(velocity2);
    box2d.common.math.Vec2.free(linearVelocity);

    body1.setAngularVelocity(angularVelocity);
    body2.setAngularVelocity(angularVelocity);
    body1.synchronizeFixtures();
    body2.synchronizeFixtures();
    return body2;
};

/**
 * @param {!box2d.dynamics.Body} other
 */
box2d.dynamics.Body.prototype.merge = function(other) {
    for (var node = other.fixtureList.getFirstNode(); node; node = node.getNextNode()) {
        this.fixtureList.addFixture(node.fixture);
        other.fixtureList.removeFixture(node.fixture);
    }
    other.resetMassData();
    this.resetMassData();
    this.synchronizeFixtures();
};

/**
 * @return {number}
 */
box2d.dynamics.Body.prototype.getMass = function() {
    return this.m_mass;
};

/**
 * @return {number}
 */
box2d.dynamics.Body.prototype.getInvertedMass = function() {
    return this.m_invMass;
};

/**
 * @return {number}
 */
box2d.dynamics.Body.prototype.getInertia = function() {
    return this.m_I;
};

/**
 * @return {number}
 */
box2d.dynamics.Body.prototype.getInvertedInertia = function() {
    return this.m_invI;
};

/**
 * @param {box2d.collision.shapes.MassData=} massData
 * @return {!box2d.collision.shapes.MassData}
 */
box2d.dynamics.Body.prototype.getMassData = function(massData) {
    if (!massData) {
        massData = box2d.collision.shapes.MassData.get();
    }
    massData.mass = this.m_mass;
    massData.I = this.m_I;
    massData.center.setV(this.m_sweep.localCenter);
    return massData;
};

/**
 * @param {!box2d.collision.shapes.MassData} massData
 */
box2d.dynamics.Body.prototype.setMassData = function(massData) {
    box2d.common.Settings.assert(!this.m_world.isLocked());
    if (this.m_type != box2d.dynamics.BodyDef.dynamicBody) {
        return;
    }
    this.m_invMass = 0.0;
    this.m_I = 0.0;
    this.m_invI = 0.0;
    this.m_mass = massData.mass;
    if (this.m_mass <= 0.0) {
        this.m_mass = 1.0;
    }
    this.m_invMass = 1.0 / this.m_mass;
    if (massData.I > 0.0 && !this.m_fixedRotation) {
        this.m_I = massData.I - this.m_mass * (massData.center.x * massData.center.x + massData.center.y * massData.center.y);
        this.m_invI = 1.0 / this.m_I;
    }
    var oldCenter = this.m_sweep.c.copy();
    this.m_sweep.localCenter.setV(massData.center);
    this.m_sweep.c0.setV(box2d.common.math.Math.mulX(this.m_xf, this.m_sweep.localCenter));
    this.m_sweep.c.setV(this.m_sweep.c0);
    this.m_linearVelocity.x += this.m_angularVelocity * (-(this.m_sweep.c.y - oldCenter.y));
    this.m_linearVelocity.y += this.m_angularVelocity * (+(this.m_sweep.c.x - oldCenter.x));
    box2d.common.math.Vec2.free(oldCenter);
};

box2d.dynamics.Body.prototype.resetMassData = function() {
    this.m_mass = 0.0;
    this.m_invMass = 0.0;
    this.m_I = 0.0;
    this.m_invI = 0.0;
    this.m_sweep.localCenter.setZero();
    if (this.m_type == box2d.dynamics.BodyDef.staticBody || this.m_type == box2d.dynamics.BodyDef.kinematicBody) {
        return;
    }
    var center = box2d.common.math.Vec2.get(0, 0);
    for (var node = this.fixtureList.getFirstNode(); node; node = node.getNextNode()) {
        var f = node.fixture;
        if (f.m_density == 0.0) {
            continue;
        }
        var massData = f.getMassData();
        this.m_mass += massData.mass;
        center.x += massData.center.x * massData.mass;
        center.y += massData.center.y * massData.mass;
        this.m_I += massData.I;
    }
    if (this.m_mass > 0.0) {
        this.m_invMass = 1.0 / this.m_mass;
        center.x *= this.m_invMass;
        center.y *= this.m_invMass;
    } else {
        this.m_mass = 1.0;
        this.m_invMass = 1.0;
    }
    if (this.m_I > 0.0 && !this.m_fixedRotation) {
        this.m_I -= this.m_mass * (center.x * center.x + center.y * center.y);
        this.m_I *= this.m_inertiaScale;
        box2d.common.Settings.assert(this.m_I > 0);
        this.m_invI = 1.0 / this.m_I;
    } else {
        this.m_I = 0.0;
        this.m_invI = 0.0;
    }
    var oldCenter = this.m_sweep.c.copy();
    this.m_sweep.localCenter.setV(center);
    this.m_sweep.c0.setV(box2d.common.math.Math.mulX(this.m_xf, this.m_sweep.localCenter));
    this.m_sweep.c.setV(this.m_sweep.c0);
    this.m_linearVelocity.x += this.m_angularVelocity * (-(this.m_sweep.c.y - oldCenter.y));
    this.m_linearVelocity.y += this.m_angularVelocity * (+(this.m_sweep.c.x - oldCenter.x));
    box2d.common.math.Vec2.free(center);
    box2d.common.math.Vec2.free(oldCenter);
};

/**
 * @param {!box2d.common.math.Vec2} localPoint
 * @return {!box2d.common.math.Vec2}
 */
box2d.dynamics.Body.prototype.getWorldPoint = function(localPoint) {
    var A = this.m_xf.R;
    var u = box2d.common.math.Vec2.get(A.col1.x * localPoint.x + A.col2.x * localPoint.y, A.col1.y * localPoint.x + A.col2.y * localPoint.y);
    u.x += this.m_xf.position.x;
    u.y += this.m_xf.position.y;
    return u;
};

/**
 * @param {!box2d.common.math.Vec2} localVector
 * @return {!box2d.common.math.Vec2}
 */
box2d.dynamics.Body.prototype.getWorldVector = function(localVector) {
    return box2d.common.math.Math.mulMV(this.m_xf.R, localVector);
};

/**
 * @param {!box2d.common.math.Vec2} worldPoint
 * @return {!box2d.common.math.Vec2}
 */
box2d.dynamics.Body.prototype.getLocalPoint = function(worldPoint) {
    return box2d.common.math.Math.mulXT(this.m_xf, worldPoint);
};

/**
 * @param {!box2d.common.math.Vec2} worldVector
 * @return {!box2d.common.math.Vec2}
 */
box2d.dynamics.Body.prototype.getLocalVector = function(worldVector) {
    return box2d.common.math.Math.mulTMV(this.m_xf.R, worldVector);
};

/**
 * @param {!box2d.common.math.Vec2} worldPoint
 * @return {!box2d.common.math.Vec2}
 */
box2d.dynamics.Body.prototype.getLinearVelocityFromWorldPoint = function(worldPoint) {
    return box2d.common.math.Vec2.get(this.m_linearVelocity.x - this.m_angularVelocity * (worldPoint.y - this.m_sweep.c.y), this.m_linearVelocity.y + this.m_angularVelocity * (worldPoint.x - this.m_sweep.c.x));
};

/**
 * @param {!box2d.common.math.Vec2} localPoint
 * @return {!box2d.common.math.Vec2}
 */
box2d.dynamics.Body.prototype.getLinearVelocityFromLocalPoint = function(localPoint) {
    var A = this.m_xf.R;
    var worldPoint = box2d.common.math.Vec2.get(A.col1.x * localPoint.x + A.col2.x * localPoint.y, A.col1.y * localPoint.x + A.col2.y * localPoint.y);
    worldPoint.x += this.m_xf.position.x;
    worldPoint.y += this.m_xf.position.y;
    var velocity = box2d.common.math.Vec2.get(this.m_linearVelocity.x - this.m_angularVelocity * (worldPoint.y - this.m_sweep.c.y), this.m_linearVelocity.y + this.m_angularVelocity * (worldPoint.x - this.m_sweep.c.x));
    box2d.common.math.Vec2.free(worldPoint);
    return velocity;
};

/**
 * @return {number}
 */
box2d.dynamics.Body.prototype.getLinearDamping = function() {
    return this.m_linearDamping;
};

/**
 * @param {number} linearDamping
 */
box2d.dynamics.Body.prototype.setLinearDamping = function(linearDamping) {
    this.m_linearDamping = linearDamping;
};

/**
 * @return {number}
 */
box2d.dynamics.Body.prototype.getAngularDamping = function() {
    return this.m_angularDamping;
};

/**
 * @param {number} angularDamping
 */
box2d.dynamics.Body.prototype.setAngularDamping = function(angularDamping) {
    this.m_angularDamping = angularDamping;
};

/**
 * @param {number} type
 */
box2d.dynamics.Body.prototype.setType = function(type) {
    if (this.m_type == type) {
        return;
    }
    this.m_type = type;
    this.resetMassData();
    if (this.m_type == box2d.dynamics.BodyDef.staticBody) {
        this.m_linearVelocity.setZero();
        this.m_angularVelocity = 0.0;
    }
    this.setAwake(true);
    this.m_force.setZero();
    this.m_torque = 0.0;
    for (var contactNode = this.contactList.getFirstNode(box2d.dynamics.contacts.ContactList.TYPES.allContacts); contactNode; contactNode = contactNode.getNextNode()) {
        contactNode.contact.flagForFiltering();
    }
    for (var i = 0; i < this.m_lists.length; i++) {
        this.m_lists[i].updateBody(this);
    }
};

/**
 * @return {number}
 */
box2d.dynamics.Body.prototype.getType = function() {
    return this.m_type;
};

/**
 * @param {boolean} flag
 */
box2d.dynamics.Body.prototype.setBullet = function(flag) {
    this.m_bullet = flag;
};

/**
 * @return {boolean}
 */
box2d.dynamics.Body.prototype.isBullet = function() {
    return this.m_bullet;
};

/**
 * @param {boolean} flag
 */
box2d.dynamics.Body.prototype.setSleepingAllowed = function(flag) {
    this.m_allowSleep = flag;
    if (!flag) {
        this.setAwake(true);
    }
};

/**
 * @param {boolean} flag
 */
box2d.dynamics.Body.prototype.setAwake = function(flag) {
    if (this.m_awake != flag) {
        this.m_awake = flag;
        this.m_sleepTime = 0;
        if (!flag) {
            this.m_linearVelocity.setZero();
            this.m_angularVelocity = 0.0;
            this.m_force.setZero();
            this.m_torque = 0.0;
        }
        for (var i = 0; i < this.m_lists.length; i++) {
            this.m_lists[i].updateBody(this);
        }
    }
};

/**
 * @return {boolean}
 */
box2d.dynamics.Body.prototype.isAwake = function() {
    return this.m_awake;
};

/**
 * @param {boolean} fixed
 */
box2d.dynamics.Body.prototype.setFixedRotation = function(fixed) {
    this.m_fixedRotation = fixed;
    this.resetMassData();
};

/**
 * @return {boolean}
 */
box2d.dynamics.Body.prototype.isFixedRotation = function() {
    return this.m_fixedRotation;
};

/**
 * @param {boolean} flag
 */
box2d.dynamics.Body.prototype.setActive = function(flag) {
    var broadPhase,
        node;

    if (flag == this.m_active) {
        return;
    }
    if (flag) {
        this.m_active = true;
        broadPhase = this.m_world.m_contactManager.m_broadPhase;
        for (node = this.fixtureList.getFirstNode(); node; node = node.getNextNode()) {
            node.fixture.createProxy(broadPhase, this.m_xf);
        }
    } else {
        this.m_active = false;
        broadPhase = this.m_world.m_contactManager.m_broadPhase;
        for (node = this.fixtureList.getFirstNode(); node; node = node.getNextNode()) {
            node.fixture.destroyProxy(broadPhase);
        }
        for (var contactNode = this.contactList.getFirstNode(box2d.dynamics.contacts.ContactList.TYPES.allContacts); contactNode; contactNode = contactNode.getNextNode()) {
            this.m_world.m_contactManager.destroy(contactNode.contact);
        }
    }
    for (var i = 0; i < this.m_lists.length; i++) {
        this.m_lists[i].updateBody(this);
    }
};

/**
 * @return {boolean}
 */
box2d.dynamics.Body.prototype.isActive = function() {
    return this.m_active;
};

/**
 * @return {boolean}
 */
box2d.dynamics.Body.prototype.isSleepingAllowed = function() {
    return this.m_allowSleep;
};

/**
 * @return {!box2d.dynamics.FixtureList}
 */
box2d.dynamics.Body.prototype.getFixtureList = function() {
    return this.fixtureList;
};

/**
 * @return {box2d.dynamics.joints.Joint}
 */
box2d.dynamics.Body.prototype.getJointList = function() {
    return this.m_jointList;
};

/**
 * @return {!box2d.dynamics.controllers.ControllerList}
 */
box2d.dynamics.Body.prototype.getControllerList = function() {
    return this.controllerList;
};

/**
 * @param {!box2d.dynamics.controllers.Controller} controller
 */
box2d.dynamics.Body.prototype.addController = function(controller) {
    this.controllerList.addController(controller);
};

/**
 * @param {!box2d.dynamics.controllers.Controller} controller
 */
box2d.dynamics.Body.prototype.removeController = function(controller) {
    this.controllerList.removeController(controller);
};

/**
 * @return {!box2d.dynamics.contacts.ContactList}
 */
box2d.dynamics.Body.prototype.getContactList = function() {
    return this.contactList;
};

/**
 * @return {!box2d.dynamics.World}
 */
box2d.dynamics.Body.prototype.getWorld = function() {
    return this.m_world;
};

box2d.dynamics.Body.prototype.synchronizeFixtures = function() {
    var xf1 = box2d.dynamics.Body.s_xf1;
    xf1.R.set(this.m_sweep.a0);
    var tMat = xf1.R;
    var tVec = this.m_sweep.localCenter;
    xf1.position.x = this.m_sweep.c0.x - (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
    xf1.position.y = this.m_sweep.c0.y - (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
    var broadPhase = this.m_world.m_contactManager.m_broadPhase;
    for (var node = this.fixtureList.getFirstNode(); node; node = node.getNextNode()) {
        node.fixture.synchronize(broadPhase, xf1, this.m_xf);
    }
};

box2d.dynamics.Body.prototype.synchronizeTransform = function() {
    this.m_xf.R.set(this.m_sweep.a);
    var tMat = this.m_xf.R;
    var tVec = this.m_sweep.localCenter;
    this.m_xf.position.x = this.m_sweep.c.x - (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
    this.m_xf.position.y = this.m_sweep.c.y - (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
};

/**
 * @param {!box2d.dynamics.Body} other
 * @return {boolean}
 */
box2d.dynamics.Body.prototype.shouldCollide = function(other) {
    if (this.m_type != box2d.dynamics.BodyDef.dynamicBody && other.m_type != box2d.dynamics.BodyDef.dynamicBody) {
        return false;
    }
    for (var jn = this.m_jointList; jn; jn = jn.next) {
        if (jn.other == other) if (jn.joint.m_collideConnected == false) {
            return false;
        }
    }
    return true;
};

/**
 * @param {number} t
 */
box2d.dynamics.Body.prototype.advance = function(t) {
    this.m_sweep.advance(t);
    this.m_sweep.c.setV(this.m_sweep.c0);
    this.m_sweep.a = this.m_sweep.a0;
    this.synchronizeTransform();
};

/**
 * @type {number}
 * @private
 */
box2d.dynamics.Body.NEXT_ID = 0;

/**
 * @type {!box2d.common.math.Transform}
 */
box2d.dynamics.Body.s_xf1 = new box2d.common.math.Transform();
