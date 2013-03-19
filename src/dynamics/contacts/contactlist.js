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

goog.provide('box2d.dynamics.contacts.ContactList');

goog.require('box2d.dynamics.contacts.ContactListNode');

goog.require('goog.array');

/**
 * @constructor
 */
box2d.dynamics.contacts.ContactList = function() {

    /**
     * @private
     * @type {Array.<box2d.dynamics.contacts.ContactListNode>}
     */
    this.contactFirstNodes = [];
    for (var i = 0; i <= box2d.dynamics.contacts.ContactList.TYPES.allContacts; i++) {
        this.contactFirstNodes[i] = null;
    }

    /**
     * @private
     * @type {Array.<box2d.dynamics.contacts.ContactListNode>}
     */
    this.contactLastNodes = [];
    for (var i = 0; i <= box2d.dynamics.contacts.ContactList.TYPES.allContacts; i++) {
        this.contactLastNodes[i] = null;
    }

    /**
     * @private
     * @type {Object.<Array.<box2d.dynamics.contacts.ContactListNode>>}
     */
    this.contactNodeLookup = {};

    /**
     * @private
     * @type {number}
     */
    this.contactCount = 0;
};

/**
 * @param {number} type
 * @return {box2d.dynamics.contacts.ContactListNode}
 */
box2d.dynamics.contacts.ContactList.prototype.getFirstNode = function(type) {
    return this.contactFirstNodes[type];
};

/**
 * @param {!box2d.dynamics.contacts.Contact} contact
 */
box2d.dynamics.contacts.ContactList.prototype.addContact = function(contact) {
    var contactID = contact.ID;
    if (this.contactNodeLookup[contactID] == null) {
        this.contactNodeLookup[contactID] = [];
        for (var i = 0; i <= box2d.dynamics.contacts.ContactList.TYPES.allContacts; i++) {
            this.contactNodeLookup[contactID][i] = null;
        }
        this.createNode(contact, contactID, box2d.dynamics.contacts.ContactList.TYPES.allContacts);
        this.contactCount++;
    }
};

/**
 * @param {!box2d.dynamics.contacts.Contact} contact
 * @param {boolean} nonSensorEnabledTouching
 * @param {boolean} nonSensorEnabledContinuous
 */
box2d.dynamics.contacts.ContactList.prototype.updateContact = function(contact, nonSensorEnabledTouching, nonSensorEnabledContinuous) {
    if (nonSensorEnabledTouching) {
        this.createNode(contact, contact.ID, box2d.dynamics.contacts.ContactList.TYPES.nonSensorEnabledTouchingContacts);
    } else {
        this.removeNode(contact.ID, box2d.dynamics.contacts.ContactList.TYPES.nonSensorEnabledTouchingContacts);
    }
    if (nonSensorEnabledContinuous) {
        this.createNode(contact, contact.ID, box2d.dynamics.contacts.ContactList.TYPES.nonSensorEnabledContinuousContacts);
    } else {
        this.removeNode(contact.ID, box2d.dynamics.contacts.ContactList.TYPES.nonSensorEnabledContinuousContacts);
    }
};

/**
 * @param {!box2d.dynamics.contacts.Contact} contact
 */
box2d.dynamics.contacts.ContactList.prototype.removeContact = function(contact) {
    var contactID = contact.ID;
    if (this.contactNodeLookup[contactID] != null) {
        for (var i = 0; i <= box2d.dynamics.contacts.ContactList.TYPES.allContacts; i++) {
            this.removeNode(contactID, i);
        }
        delete this.contactNodeLookup[contactID];
        this.contactCount--;
    }
};

/**
 * @param {string} contactID
 * @param {number} type
 */
box2d.dynamics.contacts.ContactList.prototype.removeNode = function(contactID, type) {
    var nodeList = this.contactNodeLookup[contactID];
    if (nodeList == null) {
        return;
    }
    var node = nodeList[type];
    if (node == null) {
        return;
    }
    nodeList[type] = null;
    var prevNode = node.getPreviousNode();
    var nextNode = node.getNextNode();
    if (prevNode == null) {
        this.contactFirstNodes[type] = nextNode;
    } else {
        prevNode.setNextNode(nextNode);
    }
    if (nextNode == null) {
        this.contactLastNodes[type] = prevNode;
    } else {
        nextNode.setPreviousNode(prevNode);
    }
    box2d.dynamics.contacts.ContactListNode.freeNode(node);
};

/**
 * @param {!box2d.dynamics.contacts.Contact} contact
 * @param {string} contactID
 * @param {number} type
 */
box2d.dynamics.contacts.ContactList.prototype.createNode = function(contact, contactID, type) {
    var nodeList = this.contactNodeLookup[contactID];
    if (nodeList[type] == null) {
        nodeList[type] = box2d.dynamics.contacts.ContactListNode.getNode(contact);
        var prevNode = this.contactLastNodes[type];
        if (prevNode != null) {
            prevNode.setNextNode(nodeList[type]);
            nodeList[type].setPreviousNode(prevNode);
        } else {
            this.contactFirstNodes[type] = nodeList[type];
        }
        this.contactLastNodes[type] = nodeList[type];
    }
};

/**
 * @return {number}
 */
box2d.dynamics.contacts.ContactList.prototype.getContactCount = function() {
    return this.contactCount;
};

/**
 * @enum {number}
 */
box2d.dynamics.contacts.ContactList.TYPES = {
    nonSensorEnabledTouchingContacts: 0,
    nonSensorEnabledContinuousContacts: 1,
    allContacts: 2 // Assumed to be last by above code
};
