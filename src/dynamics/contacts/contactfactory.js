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

goog.provide('box2d.dynamics.contacts.ContactFactory');

goog.require('UsageTracker');
goog.require('box2d.collision.shapes.CircleShape');
goog.require('box2d.collision.shapes.EdgeShape');
goog.require('box2d.collision.shapes.PolygonShape');
goog.require('box2d.dynamics.contacts.CircleContact');
goog.require('box2d.dynamics.contacts.ContactRegister');
goog.require('box2d.dynamics.contacts.EdgeAndCircleContact');
goog.require('box2d.dynamics.contacts.PolyAndCircleContact');
goog.require('box2d.dynamics.contacts.PolyAndEdgeContact');
goog.require('box2d.dynamics.contacts.PolygonContact');

/**
 * @constructor
 */
box2d.dynamics.contacts.ContactFactory = function() {

    /**
     * @private
     */
    this.m_registers = {};

    /**
     * @private
     * @type {Object.<Object.<Array.<!box2d.dynamics.contacts.Contact>>>}
     */
    this.m_freeContacts = {};

    this.addType(box2d.dynamics.contacts.CircleContact, box2d.collision.shapes.CircleShape.NAME, box2d.collision.shapes.CircleShape.NAME);
    this.addType(box2d.dynamics.contacts.PolyAndCircleContact, box2d.collision.shapes.PolygonShape.NAME, box2d.collision.shapes.CircleShape.NAME);
    this.addType(box2d.dynamics.contacts.PolygonContact, box2d.collision.shapes.PolygonShape.NAME, box2d.collision.shapes.PolygonShape.NAME);
    this.addType(box2d.dynamics.contacts.EdgeAndCircleContact, box2d.collision.shapes.EdgeShape.NAME, box2d.collision.shapes.CircleShape.NAME);
    this.addType(box2d.dynamics.contacts.PolyAndEdgeContact, box2d.collision.shapes.PolygonShape.NAME, box2d.collision.shapes.EdgeShape.NAME);
};

box2d.dynamics.contacts.ContactFactory.prototype.addType = function(ctor, type1, type2) {
    this.m_freeContacts[type1] = this.m_freeContacts[type1] || {};
    this.m_freeContacts[type1][type2] = this.m_freeContacts[type1][type2] || [];

    this.m_registers[type1] = this.m_registers[type1] || {};
    this.m_registers[type1][type2] = new box2d.dynamics.contacts.ContactRegister();
    this.m_registers[type1][type2].ctor = ctor;
    this.m_registers[type1][type2].primary = true;
    if (type1 != type2) {
        this.m_registers[type2] = this.m_registers[type2] || {};
        this.m_registers[type2][type1] = new box2d.dynamics.contacts.ContactRegister();
        this.m_registers[type2][type1].ctor = ctor;
        this.m_registers[type2][type1].primary = false;
    }
};

box2d.dynamics.contacts.ContactFactory.prototype.create = function(fixtureA, fixtureB) {
    UsageTracker.get('box2d.dynamics.contacts.Contact').trackGet();
    var type1 = fixtureA.getShape().getTypeName();
    var type2 = fixtureB.getShape().getTypeName();
    var c;
    var reg = this.m_registers[type1][type2];
    var ctor = reg.ctor;
    if (ctor != null) {
        if (reg.primary) {
            if (this.m_freeContacts[type1][type2].length > 0) {
                c = this.m_freeContacts[type1][type2].pop();
                c.reset(fixtureA, fixtureB);
                return c;
            }
            return new ctor(fixtureA, fixtureB);
        } else {
            if (this.m_freeContacts[type2][type1].length > 0) {
                c = this.m_freeContacts[type2][type1].pop();
                c.reset(fixtureB, fixtureA);
                return c;
            }
            return new ctor(fixtureB, fixtureA);
        }
    } else {
        return null;
    }
};

box2d.dynamics.contacts.ContactFactory.prototype.destroy = function(contact) {
    UsageTracker.get('box2d.dynamics.contacts.Contact').trackFree();
    var type1 = contact.getFixtureA().getShape().getTypeName();
    var type2 = contact.getFixtureB().getShape().getTypeName();
    this.m_freeContacts[type1][type2].push(contact);
};
