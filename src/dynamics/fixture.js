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

goog.provide('box2d.dynamics.Fixture');

goog.require('UsageTracker');
goog.require('box2d.collision.AABB');
goog.require('box2d.collision.shapes.MassData');
goog.require('box2d.common.math.Math');
goog.require('box2d.dynamics.FilterData');

/**
 * @param {!box2d.dynamics.Body} body
 * @param {!box2d.common.math.Transform} xf
 * @param {!box2d.dynamics.FixtureDef} def
 * @constructor
 */
box2d.dynamics.Fixture = function(body, xf, def) {
    UsageTracker.get('box2d.dynamics.Fixture').trackCreate();

    /**
     * @const
     * @type {string}
     */
    this.ID = 'Fixture' + box2d.dynamics.Fixture.NEXT_ID++;

    /**
     * @type {!box2d.dynamics.FilterData}
     */
    this.m_filter = def.filter.copy();

    /**
     * @type {!box2d.collision.AABB}
     */
    this.m_aabb = box2d.collision.AABB.get();

    /**
     * @type {!box2d.collision.AABB}
     */
    this.m_aabb_temp = box2d.collision.AABB.get();

    /**
     * @type {!box2d.dynamics.Body}
     */
    this.m_body = body;

    /**
     * @type {!box2d.collision.shapes.Shape}
     */
    this.m_shape = def.shape.copy();

    /**
     * @type {number}
     */
    this.m_density = def.density;

    /**
     * @type {number}
     */
    this.m_friction = def.friction;

    /**
     * @type {number}
     */
    this.m_restitution = def.restitution;

    /**
     * @type {boolean}
     */
    this.m_isSensor = def.isSensor;
};

/**
 * @return {!box2d.collision.shapes.Shape}
 */
box2d.dynamics.Fixture.prototype.getShape = function() {
    return this.m_shape;
};

/**
 * @param {boolean} sensor
 */
box2d.dynamics.Fixture.prototype.setSensor = function(sensor) {
    if (this.m_isSensor == sensor) {
        return;
    }
    this.m_isSensor = sensor;
    if (this.m_body == null) {
        return;
    }
    for (var contactNode = this.m_body.contactList.getFirstNode(box2d.dynamics.contacts.ContactList.TYPES.allContacts); contactNode; contactNode = contactNode.getNextNode()) {
        var fixtureA = contactNode.contact.getFixtureA();
        var fixtureB = contactNode.contact.getFixtureB();
        if (fixtureA == this || fixtureB == this) {
            contactNode.contact.setSensor(fixtureA.isSensor() || fixtureB.isSensor());
        }
    }
};

/**
 * @return {boolean}
 */
box2d.dynamics.Fixture.prototype.isSensor = function() {
    return this.m_isSensor;
};

/**
 * @param {!box2d.dynamics.FilterData} filter
 */
box2d.dynamics.Fixture.prototype.setFilterData = function(filter) {
    this.m_filter = filter.copy();
    if (this.m_body == null) {
        return;
    }
    for (var contactNode = this.m_body.contactList.getFirstNode(box2d.dynamics.contacts.ContactList.TYPES.allContacts); contactNode; contactNode = contactNode.getNextNode()) {
        if (contactNode.contact.getFixtureA() == this || contactNode.contact.getFixtureB() == this) {
            contactNode.contact.flagForFiltering();
        }
    }
};

/**
 * @return {!box2d.dynamics.FilterData}
 */
box2d.dynamics.Fixture.prototype.getFilterData = function() {
    return this.m_filter;
};

/**
 * @return {!box2d.dynamics.Body}
 */
box2d.dynamics.Fixture.prototype.getBody = function() {
    return this.m_body;
};

/**
 * @param {!box2d.common.math.Vec2} p
 * @return {boolean}
 */
box2d.dynamics.Fixture.prototype.testPoint = function(p) {
    return this.m_shape.testPoint(this.m_body.getTransform(), p);
};

/**
 * @param {!box2d.collision.RayCastOutput} output
 * @param {!box2d.collision.RayCastInput} input
 * @return {boolean}
 */
box2d.dynamics.Fixture.prototype.rayCast = function(output, input) {
    return this.m_shape.rayCast(output, input, this.m_body.getTransform());
};

/**
 * @param {box2d.collision.shapes.MassData=} massData
 * @return {!box2d.collision.shapes.MassData}
 */
box2d.dynamics.Fixture.prototype.getMassData = function(massData) {
    if (!massData) {
        massData = box2d.collision.shapes.MassData.get();
    }
    this.m_shape.computeMass(massData, this.m_density);
    return massData;
};

/**
 * @param {number} density
 */
box2d.dynamics.Fixture.prototype.setDensity = function(density) {
    this.m_density = density;
};

/**
 * @return {number}
 */
box2d.dynamics.Fixture.prototype.getDensity = function() {
    return this.m_density;
};

/**
 * @return {number}
 */
box2d.dynamics.Fixture.prototype.getFriction = function() {
    return this.m_friction;
};

/**
 * @param {number} friction
 */
box2d.dynamics.Fixture.prototype.setFriction = function(friction) {
    this.m_friction = friction;
};

/**
 * @return {number}
 */
box2d.dynamics.Fixture.prototype.getRestitution = function() {
    return this.m_restitution;
};

/**
 * @param {number} restitution
 */
box2d.dynamics.Fixture.prototype.setRestitution = function(restitution) {
    this.m_restitution = restitution;
};

/**
 * @return {!box2d.collision.AABB}
 */
box2d.dynamics.Fixture.prototype.getAABB = function() {
    return this.m_aabb;
};

box2d.dynamics.Fixture.prototype.destroy = function() {
    box2d.collision.AABB.free(this.m_aabb);
};

/**
 * @param {!box2d.collision.DynamicTreeBroadPhase} broadPhase
 * @param {!box2d.common.math.Transform} xf
 */
box2d.dynamics.Fixture.prototype.createProxy = function(broadPhase, xf) {
    this.m_shape.computeAABB(this.m_aabb, xf);
    this.m_proxy = broadPhase.createProxy(this.m_aabb, this);
};

/**
 * @param {!box2d.collision.DynamicTreeBroadPhase} broadPhase
 */
box2d.dynamics.Fixture.prototype.destroyProxy = function(broadPhase) {
    if (this.m_proxy == null) {
        return;
    }
    broadPhase.destroyProxy(this.m_proxy);
    this.m_proxy = null;
};

/**
 * @param {!box2d.collision.DynamicTreeBroadPhase} broadPhase
 * @param {!box2d.common.math.Transform} transform1
 * @param {!box2d.common.math.Transform} transform2
 */
box2d.dynamics.Fixture.prototype.synchronize = function(broadPhase, transform1, transform2) {
    if (!this.m_proxy) return;

    this.m_shape.computeAABB(this.m_aabb, transform1);
    this.m_shape.computeAABB(this.m_aabb_temp, transform2);
    this.m_aabb.combine(this.m_aabb, this.m_aabb_temp);

    var displacement = box2d.common.math.Math.subtractVV(transform2.position, transform1.position);
    broadPhase.moveProxy(this.m_proxy, this.m_aabb, displacement);
    box2d.common.math.Vec2.free(displacement);
};

/**
 * @type {number}
 * @private
 */
box2d.dynamics.Fixture.NEXT_ID = 0;
