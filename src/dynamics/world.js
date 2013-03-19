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

goog.provide('box2d.dynamics.World');

goog.require('box2d.collision.AABB');
goog.require('box2d.collision.RayCastInput');
goog.require('box2d.collision.RayCastOutput');
goog.require('box2d.collision.shapes.EdgeShape');
goog.require('box2d.collision.shapes.Shape');
goog.require('box2d.common.Color');
goog.require('box2d.common.math.Math');
goog.require('box2d.common.math.Sweep');
goog.require('box2d.common.math.Transform');
goog.require('box2d.common.math.Vec2');
goog.require('box2d.dynamics.Body');
goog.require('box2d.dynamics.BodyDef');
goog.require('box2d.dynamics.BodyList');
goog.require('box2d.dynamics.ContactManager');
goog.require('box2d.dynamics.DebugDraw');
goog.require('box2d.dynamics.DestructionListener');
goog.require('box2d.dynamics.Island');
goog.require('box2d.dynamics.TimeStep');
goog.require('box2d.dynamics.contacts.ContactList');
goog.require('box2d.dynamics.contacts.Contactsolver');
goog.require('box2d.dynamics.controllers.ControllerList');
goog.require('box2d.dynamics.joints.DistanceJoint');
goog.require('box2d.dynamics.joints.DistanceJointDef');
goog.require('box2d.dynamics.joints.Joint');
goog.require('box2d.dynamics.joints.MouseJoint');
goog.require('box2d.dynamics.joints.MouseJointDef');
goog.require('box2d.dynamics.joints.PulleyJoint');
goog.require('box2d.dynamics.joints.PulleyJointDef');
goog.require('goog.structs.Queue');

/**
 * @param {!box2d.common.math.Vec2} gravity
 * @param {boolean} doSleep
 * @constructor
 */
box2d.dynamics.World = function(gravity, doSleep) {

    /**
     * @type {!box2d.dynamics.ContactManager}
     */
    this.m_contactManager = new box2d.dynamics.ContactManager(this);

    /**
     * @type {!box2d.dynamics.contacts.Contactsolver}
     */
    this.m_contactsolver = new box2d.dynamics.contacts.Contactsolver();

    /**
     * @type {boolean}
     */
    this.m_isLocked = false;

    /**
     * @type {boolean}
     */
    this.m_newFixture = false;

    /**
     * @type {box2d.dynamics.DestructionListener}
     */
    this.m_destructionListener = null;

    /**
     * @type {box2d.dynamics.DebugDraw}
     */
    this.m_debugdraw = null;

    /**
     * @type {!box2d.dynamics.BodyList}
     */
    this.bodyList = new box2d.dynamics.BodyList();

    /**
     * @type {!box2d.dynamics.contacts.ContactList}
     */
    this.contactList = new box2d.dynamics.contacts.ContactList();

    /**
     * @type {box2d.dynamics.joints.Joint}
     */
    this.m_jointList = null;

    /**
     * @type {!box2d.dynamics.controllers.ControllerList}
     */
    this.controllerList = new box2d.dynamics.controllers.ControllerList();

    /**
     * @type {number}
     */
    this.m_jointCount = 0;

    /**
     * @type {boolean}
     */
    this.m_warmStarting = true;

    /**
     * @type {boolean}
     */
    this.m_continuousPhysics = true;

    /**
     * @type {boolean}
     */
    this.m_allowSleep = doSleep;

    /**
     * @type {!box2d.common.math.Vec2}
     */
    this.m_gravity = gravity;

    /**
     * @type {number}
     */
    this.m_inv_dt0 = 0.0;

    /**
     * @type {!box2d.dynamics.Body}
     */
    this.m_groundBody = this.createBody(new box2d.dynamics.BodyDef());

    /**
     * @type {!box2d.dynamics.TimeStep}
     * @const
     */
    this.mainTimestep = new box2d.dynamics.TimeStep(0, 0, 0, 0, this.m_warmStarting);

    /**
     * @type {!box2d.dynamics.TimeStep}
     * @const
     */
    this.islandTimestep = new box2d.dynamics.TimeStep(0, 0, 0, 0, this.m_warmStarting);

    /**
     * @type {!box2d.dynamics.Island}
     * @const
     */
    this.island = new box2d.dynamics.Island(this.m_contactManager.m_contactListener, this.m_contactsolver);
};

/**
 * @const
 * @type {number}
 */
box2d.dynamics.World.MAX_TOI = 1.0 - 100.0 * Number.MIN_VALUE;

/**
 * @param {!box2d.dynamics.DestructionListener} listener
 */
box2d.dynamics.World.prototype.setDestructionListener = function(listener) {
    this.m_destructionListener = listener;
};

/**
 * @param {!box2d.dynamics.IContactFilter} filter
 */
box2d.dynamics.World.prototype.setContactFilter = function(filter) {
    this.m_contactManager.m_contactFilter = filter;
};

/**
 * @param {!box2d.dynamics.IContactListener} listener
 */
box2d.dynamics.World.prototype.setContactListener = function(listener) {
    this.m_contactManager.m_contactListener = listener;
};

/**
 * @param {!box2d.dynamics.DebugDraw} debugdraw
 */
box2d.dynamics.World.prototype.setDebugDraw = function(debugdraw) {
    this.m_debugdraw = debugdraw;
};

/**
 * @param {!box2d.collision.DynamicTreeBroadPhase} broadPhase
 */
box2d.dynamics.World.prototype.setBroadPhase = function(broadPhase) {
    var oldBroadPhase = this.m_contactManager.m_broadPhase;
    this.m_contactManager.m_broadPhase = broadPhase;
    for (var node = this.bodyList.getFirstNode(box2d.dynamics.BodyList.TYPES.allBodies); node; node = node.getNextNode()) {
        for (var fixtureNode = node.body.getFixtureList().getFirstNode(); fixtureNode; fixtureNode = fixtureNode.getNextNode()) {
            var f = fixtureNode.fixture;
            f.m_proxy = broadPhase.createProxy(oldBroadPhase.getFatAABB(f.m_proxy), f);
        }
    }
};

/**
 * @param {!box2d.dynamics.BodyDef} def
 * @return {!box2d.dynamics.Body}
 */
box2d.dynamics.World.prototype.createBody = function(def) {
    box2d.common.Settings.assert(!this.isLocked());
    var b = new box2d.dynamics.Body(def, this);
    this.bodyList.addBody(b);
    return b;
};

/**
 * @param {!box2d.dynamics.Body} b
 */
box2d.dynamics.World.prototype.destroyBody = function(b) {
    box2d.common.Settings.assert(!this.isLocked());
    var jn = b.m_jointList;
    while (jn) {
        var jn0 = jn;
        jn = jn.next;
        if (this.m_destructionListener) {
            this.m_destructionListener.sayGoodbyeJoint(jn0.joint);
        }
        this.destroyJoint(jn0.joint);
    }
    for (var node = b.getControllerList().getFirstNode(); node; node = node.getNextNode()) {
        node.controller.removeBody(b);
    }
    for (var contactNode = b.contactList.getFirstNode(box2d.dynamics.contacts.ContactList.TYPES.allContacts); contactNode; contactNode = contactNode.getNextNode()) {
        this.m_contactManager.destroy(contactNode.contact);
    }
    for (var fixtureNode = b.getFixtureList().getFirstNode(); fixtureNode; fixtureNode = fixtureNode.getNextNode()) {
        // Why doesn't this happen in body.destroyFixture?
        if (this.m_destructionListener) {
            this.m_destructionListener.sayGoodbyeFixture(fixtureNode.fixture);
        }
        b.destroyFixture(fixtureNode.fixture);
    }
    b.destroy();
    this.bodyList.removeBody(b);
};

/**
 * @param {!box2d.dynamics.joints.JointDef} def
 * @return {!box2d.dynamics.joints.Joint}
 */
box2d.dynamics.World.prototype.createJoint = function(def) {
    var j = box2d.dynamics.joints.Joint.create(def);
    j.m_prev = null;
    j.m_next = this.m_jointList;
    if (this.m_jointList) {
        this.m_jointList.m_prev = j;
    }
    this.m_jointList = j;
    this.m_jointCount++;
    j.m_edgeA.joint = j;
    j.m_edgeA.other = j.m_bodyB;
    j.m_edgeA.prev = null;
    j.m_edgeA.next = j.m_bodyA.m_jointList;
    if (j.m_bodyA.m_jointList) {
        j.m_bodyA.m_jointList.prev = j.m_edgeA;
    }
    j.m_bodyA.m_jointList = j.m_edgeA;
    j.m_edgeB.joint = j;
    j.m_edgeB.other = j.m_bodyA;
    j.m_edgeB.prev = null;
    j.m_edgeB.next = j.m_bodyB.m_jointList;
    if (j.m_bodyB.m_jointList) {
        j.m_bodyB.m_jointList.prev = j.m_edgeB;
    }
    j.m_bodyB.m_jointList = j.m_edgeB;
    var bodyA = def.bodyA;
    var bodyB = def.bodyB;
    if (!def.collideConnected) {
        for (var contactNode = bodyB.contactList.getFirstNode(box2d.dynamics.contacts.ContactList.TYPES.allContacts); contactNode; contactNode = contactNode.getNextNode()) {
            if (contactNode.contact.getOther(bodyB) == bodyA) {
                contactNode.contact.flagForFiltering();
            }
        }
    }
    return j;
};

/**
 * @param {!box2d.dynamics.joints.Joint} j
 */
box2d.dynamics.World.prototype.destroyJoint = function(j) {
    var collideConnected = j.m_collideConnected;
    if (j.m_prev) {
        j.m_prev.m_next = j.m_next;
    }
    if (j.m_next) {
        j.m_next.m_prev = j.m_prev;
    }
    if (j == this.m_jointList) {
        this.m_jointList = j.m_next;
    }
    var bodyA = j.m_bodyA;
    var bodyB = j.m_bodyB;
    bodyA.setAwake(true);
    bodyB.setAwake(true);
    if (j.m_edgeA.prev) {
        j.m_edgeA.prev.next = j.m_edgeA.next;
    }
    if (j.m_edgeA.next) {
        j.m_edgeA.next.prev = j.m_edgeA.prev;
    }
    if (j.m_edgeA == bodyA.m_jointList) {
        bodyA.m_jointList = j.m_edgeA.next;
    }
    j.m_edgeA.prev = null;
    j.m_edgeA.next = null;
    if (j.m_edgeB.prev) {
        j.m_edgeB.prev.next = j.m_edgeB.next;
    }
    if (j.m_edgeB.next) {
        j.m_edgeB.next.prev = j.m_edgeB.prev;
    }
    if (j.m_edgeB == bodyB.m_jointList) {
        bodyB.m_jointList = j.m_edgeB.next;
    }
    j.m_edgeB.prev = null;
    j.m_edgeB.next = null;
    this.m_jointCount--;
    if (!collideConnected) {
        for (var contactNode = bodyB.contactList.getFirstNode(box2d.dynamics.contacts.ContactList.TYPES.allContacts); contactNode; contactNode = contactNode.getNextNode()) {
            if (contactNode.contact.getOther(bodyB) == bodyA) {
                contactNode.contact.flagForFiltering();
            }
        }
    }
};

/**
 * @return {!box2d.dynamics.controllers.ControllerList}
 */
box2d.dynamics.World.prototype.getControllerList = function() {
    return this.controllerList;
};

/**
 * @param {!box2d.dynamics.controllers.Controller} c
 * @return {!box2d.dynamics.controllers.Controller}
 */
box2d.dynamics.World.prototype.addController = function(c) {
    if (c.m_world !== null && c.m_world != this) {
        throw new Error('Controller can only be a member of one world');
    }
    this.controllerList.addController(c);
    c.m_world = this;
    return c;
};

/**
 * @param {!box2d.dynamics.controllers.Controller} c
 */
box2d.dynamics.World.prototype.removeController = function(c) {
    this.controllerList.removeController(c);
    c.m_world = null;
    c.clear();
};

/**
 * @param {!box2d.dynamics.controllers.Controller} controller
 * @return {!box2d.dynamics.controllers.Controller}
 */
box2d.dynamics.World.prototype.createController = function(controller) {
    return this.addController(controller);
};

/**
 * @param {!box2d.dynamics.controllers.Controller} controller
 */
box2d.dynamics.World.prototype.destroyController = function(controller) {
    this.removeController(controller);
};

/**
 * @param {boolean} flag
 */
box2d.dynamics.World.prototype.setWarmStarting = function(flag) {
    this.m_warmStarting = flag;
};

/**
 * @param {boolean} flag
 */
box2d.dynamics.World.prototype.setContinuousPhysics = function(flag) {
    this.m_continuousPhysics = flag;
};

/**
 * @return {number}
 */
box2d.dynamics.World.prototype.getBodyCount = function() {
    return this.bodyList.getBodyCount();
};

/**
 * @return {number}
 */
box2d.dynamics.World.prototype.getJointCount = function() {
    return this.m_jointCount;
};

/**
 * @return {number}
 */
box2d.dynamics.World.prototype.getContactCount = function() {
    return this.contactList.getContactCount();
};

/**
 * @param {!box2d.common.math.Vec2} gravity
 */
box2d.dynamics.World.prototype.setGravity = function(gravity) {
    this.m_gravity = gravity;
};

/**
 * @return {!box2d.common.math.Vec2}
 */
box2d.dynamics.World.prototype.getGravity = function() {
    return this.m_gravity;
};

/**
 * @return {!box2d.dynamics.Body}
 */
box2d.dynamics.World.prototype.getGroundBody = function() {
    return this.m_groundBody;
};

/**
 * @param {number} dt
 * @param {number} velocityIterations
 * @param {number} positionIterations
 */
box2d.dynamics.World.prototype.step = function(dt, velocityIterations, positionIterations) {
    if (this.m_newFixture) {
        this.m_contactManager.findNewContacts();
        this.m_newFixture = false;
    }
    this.m_isLocked = true;
    this.mainTimestep.reset(dt, this.m_inv_dt0 * dt /* dtRatio */, velocityIterations, positionIterations, this.m_warmStarting);
    this.m_contactManager.collide();
    if (this.mainTimestep.dt > 0.0) {
        this.solve(this.mainTimestep);
        if (this.m_continuousPhysics) {
            this.solveTOI(this.mainTimestep);
        }
        this.m_inv_dt0 = this.mainTimestep.inv_dt;
    }
    this.m_isLocked = false;
};

box2d.dynamics.World.prototype.clearForces = function() {
    for (var node = this.bodyList.getFirstNode(box2d.dynamics.BodyList.TYPES.dynamicBodies); node; node = node.getNextNode()) {
        node.body.m_force.setZero();
        node.body.m_torque = 0.0;
    }
};

box2d.dynamics.World.prototype.drawDebugData = function() {
    if (this.m_debugdraw === null) {
        return;
    }
    this.m_debugdraw.clear();
    var flags = this.m_debugdraw.getFlags();
    if (flags & box2d.dynamics.DebugDraw.e_shapeBit) {
        var color_inactive = box2d.dynamics.World.s_color_inactive;
        var color_static = box2d.dynamics.World.s_color_static;
        var color_kinematic = box2d.dynamics.World.s_color_kinematic;
        var color_dynamic_sleeping = box2d.dynamics.World.s_color_dynamic_sleeping;
        var color_dynamic_awake = box2d.dynamics.World.s_color_dynamic_awake;
        for (var bodyNode = this.bodyList.getFirstNode(box2d.dynamics.BodyList.TYPES.allBodies); bodyNode; bodyNode = bodyNode.getNextNode()) {
            var b = bodyNode.body;
            for (var fixtureNode = b.getFixtureList().getFirstNode(); fixtureNode; fixtureNode = fixtureNode.getNextNode()) {
                var f = fixtureNode.fixture;
                var s = f.getShape();
                if (!b.isActive()) {
                    this.drawShape(s, b.m_xf, color_inactive);
                } else if (b.getType() == box2d.dynamics.BodyDef.staticBody) {
                    this.drawShape(s, b.m_xf, color_static);
                } else if (b.getType() == box2d.dynamics.BodyDef.kinematicBody) {
                    this.drawShape(s, b.m_xf, color_kinematic);
                } else if (!b.isAwake()) {
                    this.drawShape(s, b.m_xf, color_dynamic_sleeping);
                } else {
                    this.drawShape(s, b.m_xf, color_dynamic_awake);
                }
            }
        }
    }
    if (flags & box2d.dynamics.DebugDraw.e_jointBit) {
        for (var j = this.m_jointList; j; j = j.m_next) {
            this.drawJoint(j);
        }
    }
    if (flags & box2d.dynamics.DebugDraw.e_controllerBit) {
        for (var controllerNode = this.controllerList.getFirstNode(); controllerNode; controllerNode = controllerNode.getNextNode()) {
            controllerNode.controller.draw(this.m_debugdraw);
        }
    }
    if (flags & box2d.dynamics.DebugDraw.e_pairBit) {
        var pairColor = box2d.dynamics.World.s_pairColor;
        for (var contactNode = this.contactList.getFirstNode(box2d.dynamics.contacts.ContactList.TYPES.allContacts); contactNode; contactNode = contactNode.getNextNode()) {
            var fixtureA = contactNode.contact.getFixtureA();
            var fixtureB = contactNode.contact.getFixtureB();
            var cA = fixtureA.getAABB().getCenter();
            var cB = fixtureB.getAABB().getCenter();
            this.m_debugdraw.drawSegment(cA, cB, pairColor);
            box2d.common.math.Vec2.free(cA);
            box2d.common.math.Vec2.free(cB);
        }
    }
    if (flags & box2d.dynamics.DebugDraw.e_aabbBit) {
        var aabbColor = box2d.dynamics.World.s_aabbColor;
        for (var bodyNode = this.bodyList.getFirstNode(box2d.dynamics.BodyList.TYPES.activeBodies); bodyNode; bodyNode = bodyNode.getNextNode()) {
            var b = bodyNode.body;
            for (var fixtureNode = b.getFixtureList().getFirstNode(); fixtureNode; fixtureNode = fixtureNode.getNextNode()) {
                var f = fixtureNode.fixture;
                var aabb = this.m_contactManager.m_broadPhase.getFatAABB(f.m_proxy);
                var vs = [box2d.common.math.Vec2.get(aabb.lowerBound.x, aabb.lowerBound.y),
                    box2d.common.math.Vec2.get(aabb.upperBound.x, aabb.lowerBound.y),
                    box2d.common.math.Vec2.get(aabb.upperBound.x, aabb.upperBound.y),
                    box2d.common.math.Vec2.get(aabb.lowerBound.x, aabb.upperBound.y)];
                this.m_debugdraw.drawPolygon(vs, 4, aabbColor);
                box2d.common.math.Vec2.free(vs[0]);
                box2d.common.math.Vec2.free(vs[1]);
                box2d.common.math.Vec2.free(vs[2]);
                box2d.common.math.Vec2.free(vs[3]);
            }
        }
    }
    if (flags & box2d.dynamics.DebugDraw.e_centerOfMassBit) {
        for (var bodyNode = this.bodyList.getFirstNode(box2d.dynamics.BodyList.TYPES.allBodies); bodyNode; bodyNode = bodyNode.getNextNode()) {
            var b = bodyNode.body;
            box2d.dynamics.World.s_xf.R = b.m_xf.R;
            box2d.dynamics.World.s_xf.position = b.getWorldCenter();
            this.m_debugdraw.drawTransform(box2d.dynamics.World.s_xf);
        }
    }
};

/**
 * @param {function(!box2d.dynamics.Fixture):boolean} callback
 * @param {!box2d.collision.AABB} aabb
 */
box2d.dynamics.World.prototype.queryAABB = function(callback, aabb) {
    this.m_contactManager.m_broadPhase.query(callback, aabb);
};

/**
 * @param {function(!box2d.dynamics.Fixture): boolean} callback
 * @param {!box2d.common.math.Vec2} p
 */
box2d.dynamics.World.prototype.queryPoint = function(callback, p) {
    /** @type {function(!box2d.dynamics.Fixture): boolean} */
    var WorldqueryWrapper = function(fixture) {
        if (fixture.testPoint(p)) {
            return callback(fixture);
        } else {
            return true;
        }
    };
    var aabb = box2d.collision.AABB.get();
    aabb.lowerBound.set(p.x - box2d.common.Settings.linearSlop, p.y - box2d.common.Settings.linearSlop);
    aabb.upperBound.set(p.x + box2d.common.Settings.linearSlop, p.y + box2d.common.Settings.linearSlop);
    this.m_contactManager.m_broadPhase.query(WorldqueryWrapper, aabb);
    box2d.collision.AABB.free(aabb);
};

/**
 * @param {function(!box2d.dynamics.Fixture, !box2d.common.math.Vec2, !box2d.common.math.Vec2, number): number} callback
 * @param {!box2d.common.math.Vec2} point1
 * @param {!box2d.common.math.Vec2} point2
 */
box2d.dynamics.World.prototype.rayCast = function(callback, point1, point2) {
    var broadPhase = this.m_contactManager.m_broadPhase;
    var output = new box2d.collision.RayCastOutput();

    /**
     * @param {!box2d.collision.RayCastInput} input
     * @param {!box2d.dynamics.Fixture} fixture
     */
    var rayCastWrapper = function(input, fixture) {
        var hit = fixture.rayCast(output, input);
        if (hit) {
            var flipFrac = 1 - output.fraction;
            var point = box2d.common.math.Vec2.get(flipFrac * point1.x + output.fraction * point2.x, flipFrac * point1.y + output.fraction * point2.y);
            var retVal = callback(fixture, point, output.normal, output.fraction);
            box2d.common.math.Vec2.free(point);
            return retVal;
        } else {
            return input.maxFraction;
        }
    };
    var input = new box2d.collision.RayCastInput(point1, point2, 1 /* maxFraction */);
    broadPhase.rayCast(rayCastWrapper, input);
};

/**
 * @param {!box2d.common.math.Vec2} point1
 * @param {!box2d.common.math.Vec2} point2
 * @return {box2d.dynamics.Fixture}
 */
box2d.dynamics.World.prototype.rayCastOne = function(point1, point2) {
    var result = null;
    /**
     * @param {!box2d.dynamics.Fixture} fixture
     * @param {!box2d.common.math.Vec2} point
     * @param {!box2d.common.math.Vec2} normal
     * @param {number} fraction
     * @return {number}
     */
    var rayCastOneWrapper = function(fixture, point, normal, fraction) {
        result = fixture;
        return fraction;
    };
    this.rayCast(rayCastOneWrapper, point1, point2);
    return result;
};

/**
 * @param {!box2d.common.math.Vec2} point1
 * @param {!box2d.common.math.Vec2} point2
 * @return {Array.<box2d.dynamics.Fixture>}
 */
box2d.dynamics.World.prototype.rayCastAll = function(point1, point2) {
    var result = [];

    /**
     * @param {!box2d.dynamics.Fixture} fixture
     * @param {!box2d.common.math.Vec2} point
     * @param {!box2d.common.math.Vec2} normal
     * @param {number} fraction
     * @return {number}
     */
    var rayCastAllWrapper = function(fixture, point, normal, fraction) {
        result.push(fixture);
        return 1;
    };
    this.rayCast(rayCastAllWrapper, point1, point2);
    return result;
};

/**
 * @return {!box2d.dynamics.BodyList}
 */
box2d.dynamics.World.prototype.getBodyList = function() {
    return this.bodyList;
};

/**
 * @return {box2d.dynamics.joints.Joint}
 */
box2d.dynamics.World.prototype.getJointList = function() {
    return this.m_jointList;
};

/**
 * @return {!box2d.dynamics.contacts.ContactList}
 */
box2d.dynamics.World.prototype.getContactList = function() {
    return this.contactList;
};

/**
 * @return {boolean}
 */
box2d.dynamics.World.prototype.isLocked = function() {
    return this.m_isLocked;
};

/**
 * @param {!box2d.dynamics.TimeStep} step
 */
box2d.dynamics.World.prototype.solve = function(step) {
    for (var controllerNode = this.controllerList.getFirstNode(); controllerNode; controllerNode = controllerNode.getNextNode()) {
        controllerNode.controller.step(step);
    }

    var m_island = this.island;
    m_island.reset(this.m_contactManager.m_contactListener, this.m_contactsolver);

    for (var bodyNode = this.bodyList.getFirstNode(box2d.dynamics.BodyList.TYPES.allBodies); bodyNode; bodyNode = bodyNode.getNextNode()) {
        bodyNode.body.m_islandFlag = false;
    }
    for (var contactNode = this.contactList.getFirstNode(box2d.dynamics.contacts.ContactList.TYPES.allContacts); contactNode; contactNode = contactNode.getNextNode()) {
        contactNode.contact.m_islandFlag = false;
    }
    for (var j = this.m_jointList; j; j = j.m_next) {
        j.m_islandFlag = false;
    }

    for (var bodyNode = this.bodyList.getFirstNode(box2d.dynamics.BodyList.TYPES.nonStaticActiveAwakeBodies); bodyNode; bodyNode = bodyNode.getNextNode()) {
        var seed = bodyNode.body;
        if (seed.m_islandFlag) {
            continue;
        }
        m_island.clear();
        var stack = [];
        stack.push(seed);
        seed.m_islandFlag = true;
        while (stack.length > 0) {
            var b = stack.pop();
            m_island.addBody(b);
            if (!b.isAwake()) {
                b.setAwake(true);
            }
            if (b.getType() == box2d.dynamics.BodyDef.staticBody) {
                continue;
            }
            for (var contactNode = b.contactList.getFirstNode(box2d.dynamics.contacts.ContactList.TYPES.nonSensorEnabledTouchingContacts); contactNode; contactNode = contactNode.getNextNode()) {
                var contact = contactNode.contact;
                if (contact.m_islandFlag) {
                    continue;
                }
                m_island.addContact(contact);
                contact.m_islandFlag = true;
                var other = contact.getOther(b);
                if (other.m_islandFlag) {
                    continue;
                }
                stack.push(other);
                other.m_islandFlag = true;
            }
            for (var jn = b.m_jointList; jn; jn = jn.next) {
                if (jn.joint.m_islandFlag || !jn.other.isActive()) {
                    continue;
                }
                m_island.addJoint(jn.joint);
                jn.joint.m_islandFlag = true;
                if (jn.other.m_islandFlag) {
                    continue;
                }
                stack.push(jn.other);
                jn.other.m_islandFlag = true;
            }
        }
        m_island.solve(step, this.m_gravity, this.m_allowSleep);
    }
    for (var bodyNode = this.bodyList.getFirstNode(box2d.dynamics.BodyList.TYPES.nonStaticActiveAwakeBodies); bodyNode; bodyNode = bodyNode.getNextNode()) {
        bodyNode.body.synchronizeFixtures();
    }
    this.m_contactManager.findNewContacts();
};

/**
 * @param {!box2d.dynamics.TimeStep} step
 */
box2d.dynamics.World.prototype.solveTOI = function(step) {
    var m_island = this.island;
    m_island.reset(this.m_contactManager.m_contactListener, this.m_contactsolver);

    for (var bodyNode = this.bodyList.getFirstNode(box2d.dynamics.BodyList.TYPES.allBodies); bodyNode; bodyNode = bodyNode.getNextNode()) {
        var b = bodyNode.body;
        b.m_islandFlag = false;
        b.m_sweep.t0 = 0.0;
    }
    for (var contactNode = this.contactList.getFirstNode(box2d.dynamics.contacts.ContactList.TYPES.allContacts); contactNode; contactNode = contactNode.getNextNode()) {
        contactNode.contact.m_islandFlag = false;
        contactNode.contact.m_toi = null;
    }
    for (var j = this.m_jointList; j; j = j.m_next) {
        j.m_islandFlag = false;
    }
    while (true) {
        var toi2 = this._solveTOI2(step);
        var minContact = toi2.minContact;
        var minTOI = toi2.minTOI;
        if (minContact === null || box2d.dynamics.World.MAX_TOI < minTOI) {
            break;
        }
        var fixtureABody = minContact.m_fixtureA.getBody();
        var fixtureBBody = minContact.m_fixtureB.getBody();
        box2d.dynamics.World.s_backupA.set(fixtureABody.m_sweep);
        box2d.dynamics.World.s_backupB.set(fixtureBBody.m_sweep);
        fixtureABody.advance(minTOI);
        fixtureBBody.advance(minTOI);
        minContact.update(this.m_contactManager.m_contactListener);
        minContact.m_toi = null;
        if (minContact.isSensor() || !minContact.isEnabled()) {
            fixtureABody.m_sweep.set(box2d.dynamics.World.s_backupA);
            fixtureBBody.m_sweep.set(box2d.dynamics.World.s_backupB);
            fixtureABody.synchronizeTransform();
            fixtureBBody.synchronizeTransform();
            continue;
        }
        if (!minContact.isTouching()) {
            continue;
        }
        var seed = fixtureABody;
        if (seed.getType() != box2d.dynamics.BodyDef.dynamicBody) {
            seed = fixtureBBody;
        }
        m_island.clear();
        var queue = new goog.structs.Queue();
        queue.enqueue(seed);
        seed.m_islandFlag = true;
        while (queue.getCount() > 0) {

            var b = /** @type {!box2d.dynamics.Body} */ (queue.dequeue());
            m_island.addBody(b);
            if (!b.isAwake()) {
                b.setAwake(true);
            }
            if (b.getType() != box2d.dynamics.BodyDef.dynamicBody) {
                continue;
            }
            for (var contactNode = b.contactList.getFirstNode(box2d.dynamics.contacts.ContactList.TYPES.nonSensorEnabledTouchingContacts); contactNode; contactNode = contactNode.getNextNode()) {
                if (m_island.m_contacts.length == box2d.common.Settings.maxTOIContactsPerIsland) {
                    break;
                }
                var contact = contactNode.contact;
                if (contact.m_islandFlag) {
                    continue;
                }
                m_island.addContact(contact);
                contact.m_islandFlag = true;

                var other = contact.getOther(b);
                if (other.m_islandFlag) {
                    continue;
                }
                if (other.getType() != box2d.dynamics.BodyDef.staticBody) {
                    other.advance(minTOI);
                    other.setAwake(true);
                    queue.enqueue(other);
                }
                other.m_islandFlag = true;
            }
            for (var jEdge = b.m_jointList; jEdge; jEdge = jEdge.next) {
                if (m_island.m_joints.length == box2d.common.Settings.maxTOIJointsPerIsland) {
                    continue;
                }
                if (jEdge.joint.m_islandFlag || !jEdge.other.isActive()) {
                    continue;
                }
                m_island.addJoint(jEdge.joint);
                jEdge.joint.m_islandFlag = true;
                if (jEdge.other.m_islandFlag) {
                    continue;
                }
                if (jEdge.other.getType() != box2d.dynamics.BodyDef.staticBody) {
                    jEdge.other.advance(minTOI);
                    jEdge.other.setAwake(true);
                    queue.enqueue(jEdge.other);
                }
                jEdge.other.m_islandFlag = true;
            }
        }
        this.islandTimestep.reset((1.0 - minTOI) * step.dt /* dt */, 0 /* dtRatio */, step.velocityIterations, step.positionIterations, false /* warmStarting */);
        m_island.solveTOI(this.islandTimestep);

        for (var i = 0; i < m_island.m_bodies.length; i++) {
            m_island.m_bodies[i].m_islandFlag = false;
            if (!m_island.m_bodies[i].isAwake() || m_island.m_bodies[i].getType() != box2d.dynamics.BodyDef.dynamicBody) {
                continue;
            }
            m_island.m_bodies[i].synchronizeFixtures();
            for (var contactNode = m_island.m_bodies[i].contactList.getFirstNode(box2d.dynamics.contacts.ContactList.TYPES.allContacts); contactNode; contactNode = contactNode.getNextNode()) {
                contactNode.contact.m_toi = null;
            }
        }
        for (var i = 0; i < m_island.m_contacts.length; i++) {
            m_island.m_contacts[i].m_islandFlag = false;
            m_island.m_contacts[i].m_toi = null;
        }
        for (var i = 0; i < m_island.m_joints.length; i++) {
            m_island.m_joints[i].m_islandFlag = false;
        }
        this.m_contactManager.findNewContacts();
    }
};

/**
 * @param {!box2d.dynamics.TimeStep} step
 * @return {{minContact: box2d.dynamics.contacts.Contact, minTOI: number}}
 */
box2d.dynamics.World.prototype._solveTOI2 = function(step) {
    var minContact = null;
    var minTOI = 1.0;
    for (var contactNode = this.contactList.getFirstNode(box2d.dynamics.contacts.ContactList.TYPES.nonSensorEnabledContinuousContacts); contactNode; contactNode = contactNode.getNextNode()) {
        var c = contactNode.contact;
        if (this._solveTOI2SkipContact(step, c)) {
            continue;
        }
        var toi = 1.0;
        if (c.m_toi != null) {
            toi = c.m_toi;
        } else if (c.isTouching()) {
            toi = 1;
            c.m_toi = toi;
        } else {
            var fixtureABody = c.m_fixtureA.getBody();
            var fixtureBBody = c.m_fixtureB.getBody();
            var t0 = fixtureABody.m_sweep.t0;
            if (fixtureABody.m_sweep.t0 < fixtureBBody.m_sweep.t0) {
                t0 = fixtureBBody.m_sweep.t0;
                fixtureABody.m_sweep.advance(t0);
            } else if (fixtureBBody.m_sweep.t0 < fixtureABody.m_sweep.t0) {
                t0 = fixtureABody.m_sweep.t0;
                fixtureBBody.m_sweep.advance(t0);
            }
            toi = c.computeTOI(fixtureABody.m_sweep, fixtureBBody.m_sweep);
            box2d.common.Settings.assert(0.0 <= toi && toi <= 1.0);
            if (toi > 0.0 && toi < 1.0) {
                toi = (1.0 - toi) * t0 + toi;
            }
            c.m_toi = toi;
        }
        if (Number.MIN_VALUE < toi && toi < minTOI) {
            minContact = c;
            minTOI = toi;
        }
    }
    return {
        minContact: minContact,
        minTOI: minTOI
    };
};

/**
 * @param {!box2d.dynamics.TimeStep} step
 * @param {!box2d.dynamics.contacts.Contact} c
 * @return {boolean}
 */
box2d.dynamics.World.prototype._solveTOI2SkipContact = function(step, c) {
    var fixtureABody = c.m_fixtureA.getBody();
    var fixtureBBody = c.m_fixtureB.getBody();
    if ((fixtureABody.getType() != box2d.dynamics.BodyDef.dynamicBody || !fixtureABody.isAwake()) && (fixtureBBody.getType() != box2d.dynamics.BodyDef.dynamicBody || !fixtureBBody.isAwake())) {
        return true;
    }
    return false;
};

/**
 * @param {!box2d.dynamics.joints.Joint} joint
 */
box2d.dynamics.World.prototype.drawJoint = function(joint) {
    if (joint instanceof box2d.dynamics.joints.DistanceJoint || joint instanceof box2d.dynamics.joints.MouseJoint) {
        var anchorA = joint.getAnchorA();
        var anchorB = joint.getAnchorB();
        this.m_debugdraw.drawSegment(anchorA, anchorB, box2d.dynamics.World.s_jointColor);
        box2d.common.math.Vec2.free(anchorA);
        box2d.common.math.Vec2.free(anchorB);
    } else if (joint instanceof box2d.dynamics.joints.PulleyJoint) {
        var anchorA = joint.getAnchorA();
        var anchorB = joint.getAnchorB();
        var groundA = joint.getGroundAnchorA();
        var groundB = joint.getGroundAnchorB();
        this.m_debugdraw.drawSegment(groundA, anchorA, box2d.dynamics.World.s_jointColor);
        this.m_debugdraw.drawSegment(groundB, anchorB, box2d.dynamics.World.s_jointColor);
        this.m_debugdraw.drawSegment(groundA, groundB, box2d.dynamics.World.s_jointColor);
        box2d.common.math.Vec2.free(anchorA);
        box2d.common.math.Vec2.free(anchorB);
        box2d.common.math.Vec2.free(groundA);
        box2d.common.math.Vec2.free(groundB);
    } else {
        var anchorA = joint.getAnchorA();
        var anchorB = joint.getAnchorB();
        if (joint.getBodyA() != this.m_groundBody) {
            this.m_debugdraw.drawSegment(joint.getBodyA().m_xf.position, anchorA, box2d.dynamics.World.s_jointColor);
        }
        this.m_debugdraw.drawSegment(anchorA, anchorB, box2d.dynamics.World.s_jointColor);
        if (joint.getBodyB() != this.m_groundBody) {
            this.m_debugdraw.drawSegment(joint.getBodyB().m_xf.position, anchorB, box2d.dynamics.World.s_jointColor);
        }
        box2d.common.math.Vec2.free(anchorA);
        box2d.common.math.Vec2.free(anchorB);
    }
};

/**
 * @param {!box2d.collision.shapes.Shape} shape
 * @param {!box2d.common.math.Transform} xf
 * @param {!box2d.common.Color} color
 */
box2d.dynamics.World.prototype.drawShape = function(shape, xf, color) {
    if (shape instanceof box2d.collision.shapes.CircleShape) {
        var circle = shape;
        var center = box2d.common.math.Math.mulX(xf, circle.m_p);
        var radius = circle.m_radius;
        var axis = xf.R.col1;
        this.m_debugdraw.drawSolidCircle(center, radius, axis, color);
        box2d.common.math.Vec2.free(center);
    } else if (shape instanceof box2d.collision.shapes.PolygonShape) {
        var i = 0;
        var poly = shape;
        var vertexCount = poly.getVertexCount();
        var localVertices = poly.getVertices();
        var vertices = [];
        for (i = 0; i < vertexCount; i++) {
            vertices[i] = box2d.common.math.Math.mulX(xf, localVertices[i]);
        }
        this.m_debugdraw.drawSolidPolygon(vertices, vertexCount, color);
        for (i = 0; i < vertexCount; i++) {
            box2d.common.math.Vec2.free(vertices[i]);
        }
    } else if (shape instanceof box2d.collision.shapes.EdgeShape) {
        var edge = shape;
        var v1 = box2d.common.math.Math.mulX(xf, edge.getVertex1());
        var v2 = box2d.common.math.Math.mulX(xf, edge.getVertex2());
        this.m_debugdraw.drawSegment(v1, v2, color);
        box2d.common.math.Vec2.free(v1);
        box2d.common.math.Vec2.free(v2);
    }
};

/** @type {!box2d.common.math.Transform} */
box2d.dynamics.World.s_xf = new box2d.common.math.Transform();

/** @type {!box2d.common.math.Sweep} */
box2d.dynamics.World.s_backupA = new box2d.common.math.Sweep();

/** @type {!box2d.common.math.Sweep} */
box2d.dynamics.World.s_backupB = new box2d.common.math.Sweep();

/**
 * @type {!box2d.common.Color}
 * @const
 */
box2d.dynamics.World.s_jointColor = new box2d.common.Color(0.5, 0.8, 0.8);

/**
 * @type {!box2d.common.Color}
 * @const
 */
box2d.dynamics.World.s_color_inactive = new box2d.common.Color(0.5, 0.5, 0.3);

/**
 * @type {!box2d.common.Color}
 * @const
 */
box2d.dynamics.World.s_color_static = new box2d.common.Color(0.5, 0.9, 0.5);

/**
 * @type {!box2d.common.Color}
 * @const
 */
box2d.dynamics.World.s_color_kinematic = new box2d.common.Color(0.5, 0.5, 0.9);

/**
 * @type {!box2d.common.Color}
 * @const
 */
box2d.dynamics.World.s_color_dynamic_sleeping = new box2d.common.Color(0.6, 0.6, 0.6);

/**
 * @type {!box2d.common.Color}
 * @const
 */
box2d.dynamics.World.s_color_dynamic_awake = new box2d.common.Color(0.9, 0.7, 0.7);

/**
 * @type {!box2d.common.Color}
 * @const
 */
box2d.dynamics.World.s_pairColor = new box2d.common.Color(0.3, 0.9, 0.9);

/**
 * @type {!box2d.common.Color}
 * @const
 */
box2d.dynamics.World.s_aabbColor = new box2d.common.Color(0.0, 0.0, 0.8);
