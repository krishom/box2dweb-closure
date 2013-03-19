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

goog.provide('box2d.dynamics.contacts.Contact');

goog.require('UsageTracker');
goog.require('box2d.collision.Manifold');
goog.require('box2d.collision.TOIInput');
goog.require('box2d.collision.TimeOfImpact');
goog.require('box2d.collision.shapes.Shape');
goog.require('box2d.common.Settings');
goog.require('box2d.dynamics.BodyDef');

/**
 * @param {!box2d.dynamics.Fixture} fixtureA
 * @param {!box2d.dynamics.Fixture} fixtureB
 * @constructor
 */
box2d.dynamics.contacts.Contact = function(fixtureA, fixtureB) {
    UsageTracker.get('box2d.dynamics.contacts.Contact').trackCreate();

    /**
     * @const
     * @type {string}
     */
    this.ID = 'Contact' + box2d.dynamics.contacts.Contact.NEXT_ID++;

    /**
     * @type {!box2d.collision.Manifold}
     */
    this.m_manifold = new box2d.collision.Manifold();

    /**
     * @type {!box2d.collision.Manifold}
     */
    this.m_oldManifold = new box2d.collision.Manifold();

    /**
     * @type {boolean}
     */
    this.touching = false;

    var bodyA = fixtureA.getBody();
    var bodyB = fixtureB.getBody();

    /**
     * @type {boolean}
     */
    this.continuous = (bodyA.getType() != box2d.dynamics.BodyDef.dynamicBody) ||
        bodyA.isBullet() ||
        (bodyB.getType() != box2d.dynamics.BodyDef.dynamicBody) ||
        bodyB.isBullet();

    /**
     * @type {boolean}
     */
    this.sensor = fixtureA.isSensor() || fixtureB.isSensor();

    /**
     * @type {boolean}
     */
    this.filtering = false;

    /**
     * @type {!box2d.dynamics.Fixture}
     */
    this.m_fixtureA = fixtureA;

    /**
     * @type {!box2d.dynamics.Fixture}
     */
    this.m_fixtureB = fixtureB;

    /**
     * @type {boolean}
     */
    this.enabled = true;

    /**
     * @type {!box2d.dynamics.contacts.ContactList}
     */
    this.bodyAList = bodyA.getContactList();

    /**
     * @type {!box2d.dynamics.contacts.ContactList}
     */
    this.bodyBList = bodyB.getContactList();

    /**
     * @type {!box2d.dynamics.contacts.ContactList}
     */
    this.worldList = bodyB.getWorld().getContactList();

    this.addToLists();
};

/**
 * @param {!box2d.dynamics.Fixture} fixtureA
 * @param {!box2d.dynamics.Fixture} fixtureB
 */
box2d.dynamics.contacts.Contact.prototype.reset = function(fixtureA, fixtureB) {
    this.m_manifold.reset();
    this.m_oldManifold.reset();
    this.touching = false;
    var bodyA = fixtureA.getBody();
    var bodyB = fixtureB.getBody();
    this.continuous = (bodyA.getType() != box2d.dynamics.BodyDef.dynamicBody) ||
        bodyA.isBullet() ||
        (bodyB.getType() != box2d.dynamics.BodyDef.dynamicBody) ||
        bodyB.isBullet();
    this.sensor = fixtureA.isSensor() || fixtureB.isSensor();
    this.filtering = false;
    this.m_fixtureA = fixtureA;
    this.m_fixtureB = fixtureB;
    this.enabled = true;
    this.bodyAList = bodyA.getContactList();
    this.bodyBList = bodyB.getContactList();
    this.worldList = bodyB.getWorld().getContactList();
    this.addToLists();
};

box2d.dynamics.contacts.Contact.prototype.addToLists = function() {
    this.bodyAList.addContact(this);
    this.bodyBList.addContact(this);
    this.worldList.addContact(this);
    this.updateLists();
};

box2d.dynamics.contacts.Contact.prototype.updateLists = function() {
    var nonSensorEnabledTouching = false;
    var nonSensorEnabledContinuous = false;
    if (!this.isSensor() && this.isEnabled()) {
        if (this.isTouching()) {
            nonSensorEnabledTouching = true;
        }
        if (this.isContinuous()) {
            nonSensorEnabledContinuous = true;
        }
    }
    this.bodyAList.updateContact(this, nonSensorEnabledTouching, nonSensorEnabledContinuous);
    this.bodyBList.updateContact(this, nonSensorEnabledTouching, nonSensorEnabledContinuous);
    this.worldList.updateContact(this, nonSensorEnabledTouching, nonSensorEnabledContinuous);
};

box2d.dynamics.contacts.Contact.prototype.removeFromLists = function() {
    this.bodyAList.removeContact(this);
    this.bodyBList.removeContact(this);
    this.worldList.removeContact(this);
};

/**
 * @return {!box2d.collision.Manifold}
 */
box2d.dynamics.contacts.Contact.prototype.getManifold = function() {
    return this.m_manifold;
};

/**
 * @param {!box2d.collision.WorldManifold} worldManifold
 */
box2d.dynamics.contacts.Contact.prototype.getWorldManifold = function(worldManifold) {
    var bodyA = this.m_fixtureA.getBody();
    var bodyB = this.m_fixtureB.getBody();
    var shapeA = this.m_fixtureA.getShape();
    var shapeB = this.m_fixtureB.getShape();
    worldManifold.initialize(this.m_manifold, bodyA.getTransform(), shapeA.m_radius, bodyB.getTransform(), shapeB.m_radius);
};

/**
 * @return {boolean}
 */
box2d.dynamics.contacts.Contact.prototype.isTouching = function() {
    return this.touching;
};

/**
 * @return {boolean}
 */
box2d.dynamics.contacts.Contact.prototype.isContinuous = function() {
    return this.continuous;
};

/**
 * @param {boolean} sensor
 */
box2d.dynamics.contacts.Contact.prototype.setSensor = function(sensor) {
    this.sensor = sensor;
    this.updateLists();
};

/**
 * @return {boolean}
 */
box2d.dynamics.contacts.Contact.prototype.isSensor = function() {
    return this.sensor;
};

/**
 * @param {boolean} flag
 */
box2d.dynamics.contacts.Contact.prototype.setEnabled = function(flag) {
    this.enabled = flag;
    this.updateLists();
};

/**
 * @return {boolean}
 */
box2d.dynamics.contacts.Contact.prototype.isEnabled = function() {
    return this.enabled;
};

/**
 * @return {box2d.dynamics.contacts.Contact}
 */
box2d.dynamics.contacts.Contact.prototype.getNext = function() {
    return this.m_next;
};

/**
 * @return {!box2d.dynamics.Fixture}
 */
box2d.dynamics.contacts.Contact.prototype.getFixtureA = function() {
    return this.m_fixtureA;
};

/**
 * @return {!box2d.dynamics.Fixture}
 */
box2d.dynamics.contacts.Contact.prototype.getFixtureB = function() {
    return this.m_fixtureB;
};

/**
 * @param {!box2d.dynamics.Body} body
 * @return {!box2d.dynamics.Body}
 */
box2d.dynamics.contacts.Contact.prototype.getOther = function(body) {
    var bodyA = this.m_fixtureA.getBody();
    if (bodyA != body) {
        return bodyA;
    } else {
        return this.m_fixtureB.getBody();
    }
};

box2d.dynamics.contacts.Contact.prototype.flagForFiltering = function() {
    this.filtering = true;
};

box2d.dynamics.contacts.Contact.prototype.clearFiltering = function() {
    this.filtering = false;
};

/**
 * @return {boolean}
 */
box2d.dynamics.contacts.Contact.prototype.isFiltering = function() {
    return this.filtering;
};

box2d.dynamics.contacts.Contact.prototype.update = function(listener) {
    var tManifold = this.m_oldManifold;
    this.m_oldManifold = this.m_manifold;
    this.m_manifold = tManifold;
    this.enabled = true;
    var touching = false;
    var wasTouching = this.isTouching();
    var bodyA = this.m_fixtureA.getBody();
    var bodyB = this.m_fixtureB.getBody();
    var aabbOverlap = this.m_fixtureA.m_aabb.testOverlap(this.m_fixtureB.m_aabb);
    if (this.sensor) {
        if (aabbOverlap) {
            touching = box2d.collision.shapes.Shape.testOverlap(this.m_fixtureA.getShape(), bodyA.getTransform(), this.m_fixtureB.getShape(), bodyB.getTransform());
        }
        this.m_manifold.m_pointCount = 0;
    } else {
        this.continuous = bodyA.getType() != box2d.dynamics.BodyDef.dynamicBody || bodyA.isBullet() || bodyB.getType() != box2d.dynamics.BodyDef.dynamicBody || bodyB.isBullet();
        if (aabbOverlap) {
            this.evaluate();
            touching = this.m_manifold.m_pointCount > 0;
            for (var i = 0; i < this.m_manifold.m_pointCount; i++) {
                var mp2 = this.m_manifold.m_points[i];
                mp2.m_normalImpulse = 0.0;
                mp2.m_tangentImpulse = 0.0;
                for (var j = 0; j < this.m_oldManifold.m_pointCount; j++) {
                    var mp1 = this.m_oldManifold.m_points[j];
                    if (mp1.m_id.getKey() == mp2.m_id.getKey()) {
                        mp2.m_normalImpulse = mp1.m_normalImpulse;
                        mp2.m_tangentImpulse = mp1.m_tangentImpulse;
                        break;
                    }
                }
            }
        } else {
            this.m_manifold.m_pointCount = 0;
        }
        if (touching != wasTouching) {
            bodyA.setAwake(true);
            bodyB.setAwake(true);
        }
    }
    this.touching = touching;
    if (touching != wasTouching) {
        this.updateLists();
    }

    if (!wasTouching && touching) {
        listener.beginContact(this);
    }
    if (wasTouching && !touching) {
        listener.endContact(this);
    }
    if (!this.sensor) {
        listener.preSolve(this, this.m_oldManifold);
    }
};

box2d.dynamics.contacts.Contact.prototype.evaluate = function() {
};

/**
 * @param {!box2d.common.math.Sweep} sweepA
 * @param {!box2d.common.math.Sweep} sweepB
 * @return {number}
 */
box2d.dynamics.contacts.Contact.prototype.computeTOI = function(sweepA, sweepB) {
    box2d.dynamics.contacts.Contact.s_input.proxyA.set(this.m_fixtureA.getShape());
    box2d.dynamics.contacts.Contact.s_input.proxyB.set(this.m_fixtureB.getShape());
    box2d.dynamics.contacts.Contact.s_input.sweepA = sweepA;
    box2d.dynamics.contacts.Contact.s_input.sweepB = sweepB;
    box2d.dynamics.contacts.Contact.s_input.tolerance = box2d.common.Settings.linearSlop;
    return box2d.collision.TimeOfImpact.TimeOfImpact(box2d.dynamics.contacts.Contact.s_input);
};

/**
 * @private
 * @const
 * @type {!box2d.collision.TOIInput}
 */
box2d.dynamics.contacts.Contact.s_input = new box2d.collision.TOIInput();

/**
 * @type {number}
 * @private
 */
box2d.dynamics.contacts.Contact.NEXT_ID = 0;
