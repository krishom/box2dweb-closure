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

goog.provide('box2d.dynamics.contacts.ContactListNode');

/**
 * @param {!box2d.dynamics.contacts.Contact} contact
 * @constructor
 */
box2d.dynamics.contacts.ContactListNode = function(contact) {

    /**
     * @type {!box2d.dynamics.contacts.Contact}
     */
    this.contact = contact;

    /**
     * @private
     * @type {box2d.dynamics.contacts.ContactListNode}
     */
    this.next = null;

    /**
     * @private
     * @type {box2d.dynamics.contacts.ContactListNode}
     */
    this.previous = null;
};

/**
 * @private
 * @type {Array.<!box2d.dynamics.contacts.ContactListNode>}
 */
box2d.dynamics.contacts.ContactListNode.freeNodes = [];

/**
 * @param {!box2d.dynamics.contacts.Contact} contact
 * @return {!box2d.dynamics.contacts.ContactListNode}
 */
box2d.dynamics.contacts.ContactListNode.getNode = function(contact) {
    if (box2d.dynamics.contacts.ContactListNode.freeNodes.length > 0) {
        var node = box2d.dynamics.contacts.ContactListNode.freeNodes.pop();
        node.next = null;
        node.previous = null;
        node.contact = contact;
        return node;
    } else {
        return new box2d.dynamics.contacts.ContactListNode(contact);
    }
};

/**
 * @param {!box2d.dynamics.contacts.ContactListNode} node
 */
box2d.dynamics.contacts.ContactListNode.freeNode = function(node) {
    box2d.dynamics.contacts.ContactListNode.freeNodes.push(node);
};

/**
 * @param {box2d.dynamics.contacts.ContactListNode} node
 */
box2d.dynamics.contacts.ContactListNode.prototype.setNextNode = function(node) {
    this.next = node;
};

/**
 * @param {box2d.dynamics.contacts.ContactListNode} node
 */
box2d.dynamics.contacts.ContactListNode.prototype.setPreviousNode = function(node) {
    this.previous = node;
};

/**
 * @return {!box2d.dynamics.contacts.Contact}
 */
box2d.dynamics.contacts.ContactListNode.prototype.getContact = function() {
    return this.contact;
};

/**
 * @return {box2d.dynamics.contacts.ContactListNode}
 */
box2d.dynamics.contacts.ContactListNode.prototype.getNextNode = function() {
    return this.next;
};

/**
 * @return {box2d.dynamics.contacts.ContactListNode}
 */
box2d.dynamics.contacts.ContactListNode.prototype.getPreviousNode = function() {
    return this.previous;
};
