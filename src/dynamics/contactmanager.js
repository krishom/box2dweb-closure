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

goog.provide('box2d.dynamics.ContactManager');

goog.require('UsageTracker');
goog.require('box2d.collision.ContactPoint');
goog.require('box2d.collision.DynamicTreeBroadPhase');
goog.require('box2d.dynamics.ContactFilter');
goog.require('box2d.dynamics.ContactListener');
goog.require('box2d.dynamics.contacts.ContactFactory');

/**
 * @param {!box2d.dynamics.World} world
 * @constructor
 */
box2d.dynamics.ContactManager = function(world) {
    UsageTracker.get('box2d.dynamics.ContactManager').trackCreate();

    /**
     * @const
     * @type {!box2d.dynamics.World}
     */
    this.m_world = world;

    /**
     * @type {!box2d.dynamics.IContactFilter}
     */
    this.m_contactFilter = new box2d.dynamics.ContactFilter();

    /**
     * @type {!box2d.dynamics.IContactListener}
     */
    this.m_contactListener = new box2d.dynamics.ContactListener();

    /**
     * @const
     * @type {!box2d.dynamics.contacts.ContactFactory}
     */
    this.m_contactFactory = new box2d.dynamics.contacts.ContactFactory();

    /**
     * @type {!box2d.collision.DynamicTreeBroadPhase}
     */
    this.m_broadPhase = new box2d.collision.DynamicTreeBroadPhase();
};

/**
 * @param {!box2d.dynamics.Fixture} fixtureA
 * @param {!box2d.dynamics.Fixture} fixtureB
 */
box2d.dynamics.ContactManager.prototype.addPair = function(fixtureA, fixtureB) {
    var bodyA = fixtureA.getBody();
    var bodyB = fixtureB.getBody();
    if (bodyA == bodyB) {
        return;
    }
    if (!bodyB.shouldCollide(bodyA)) {
        return;
    }
    if (!this.m_contactFilter.shouldCollide(fixtureA, fixtureB)) {
        return;
    }
    for (var contactNode = bodyB.contactList.getFirstNode(box2d.dynamics.contacts.ContactList.TYPES.allContacts); contactNode; contactNode = contactNode.getNextNode()) {
        var fA = contactNode.contact.getFixtureA();
        if (fA == fixtureA) {
            var fB = contactNode.contact.getFixtureB();
            if (fB == fixtureB) {
                return;
            }
        } else if (fA == fixtureB) {
            var fB = contactNode.contact.getFixtureB();
            if (fB == fixtureA) {
                return;
            }
        }
    }
    var c = this.m_contactFactory.create(fixtureA, fixtureB);
};

box2d.dynamics.ContactManager.prototype.findNewContacts = function() {
    var self = this;
    /** @type {function(!box2d.dynamics.Fixture, !box2d.dynamics.Fixture)} */
    var addPairCallback = function(fixtureA, fixtureB) {
        self.addPair(fixtureA, fixtureB);
    };
    this.m_broadPhase.updatePairs(addPairCallback);
};

/**
 * @param {!box2d.dynamics.contacts.Contact} c
 */
box2d.dynamics.ContactManager.prototype.destroy = function(c) {
    var fixtureA = c.getFixtureA();
    var fixtureB = c.getFixtureB();
    var bodyA = fixtureA.getBody();
    var bodyB = fixtureB.getBody();
    if (c.isTouching()) {
        this.m_contactListener.endContact(c);
    }
    if (c.m_manifold.m_pointCount > 0) {
        bodyA.setAwake(true);
        bodyB.setAwake(true);
    }
    c.removeFromLists();
    this.m_contactFactory.destroy(c);
};

box2d.dynamics.ContactManager.prototype.collide = function() {
    for (var contactNode = this.m_world.contactList.getFirstNode(box2d.dynamics.contacts.ContactList.TYPES.allContacts); contactNode; contactNode = contactNode.getNextNode()) {
        var c = contactNode.contact;
        var fixtureA = c.getFixtureA();
        var fixtureB = c.getFixtureB();
        var bodyA = fixtureA.getBody();
        var bodyB = fixtureB.getBody();
        if (!bodyA.isAwake() && !bodyB.isAwake()) {
            continue;
        }
        if (c.isFiltering()) {
            if (!bodyB.shouldCollide(bodyA)) {
                this.destroy(c);
                continue;
            }
            if (!this.m_contactFilter.shouldCollide(fixtureA, fixtureB)) {
                this.destroy(c);
                continue;
            }
            c.clearFiltering();
        }
        var proxyA = fixtureA.m_proxy;
        var proxyB = fixtureB.m_proxy;
        var overlap = this.m_broadPhase.testOverlap(proxyA, proxyB);
        if (!overlap) {
            this.destroy(c);
            continue;
        }
        c.update(this.m_contactListener);
    }
};

/**
 * @private
 * @const
 * @type {!box2d.collision.ContactPoint}
 */
box2d.dynamics.ContactManager.s_evalCP = new box2d.collision.ContactPoint();
