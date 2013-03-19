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

goog.provide('box2d.dynamics.BodyList');

goog.require('UsageTracker');
goog.require('box2d.dynamics.BodyListNode');
goog.require('goog.array');

/**
 * @constructor
 */
box2d.dynamics.BodyList = function() {
    UsageTracker.get('box2d.dynamics.BodyList').trackCreate();

    /**
     * @private
     * @type {!Array.<box2d.dynamics.BodyListNode>}
     */
    this.bodyFirstNodes = [];
    for (var i = 0; i <= box2d.dynamics.BodyList.TYPES.allBodies; i++) {
        this.bodyFirstNodes[i] = null;
    }

    /**
     * @private
     * @type {!Array.<box2d.dynamics.BodyListNode>}
     */
    this.bodyLastNodes = [];
    for (var i = 0; i <= box2d.dynamics.BodyList.TYPES.allBodies; i++) {
        this.bodyLastNodes[i] = null;
    }

    /**
     * @private
     * @type {Object.<!Array.<box2d.dynamics.BodyListNode>>}
     */
    this.bodyNodeLookup = {};

    /**
     * @private
     * @type {number}
     */
    this.bodyCount = 0;
};

/**
 * @param {number} type
 * @return {box2d.dynamics.BodyListNode}
 */
box2d.dynamics.BodyList.prototype.getFirstNode = function(type) {
    return this.bodyFirstNodes[type];
};

/**
 * @param {!box2d.dynamics.Body} body
 */
box2d.dynamics.BodyList.prototype.addBody = function(body) {
    var bodyID = body.ID;
    if (this.bodyNodeLookup[bodyID] == null) {
        this.createNode(body, bodyID, box2d.dynamics.BodyList.TYPES.allBodies);
        this.updateBody(body);
        body.m_lists.push(this);
        this.bodyCount++;
    }
};

/**
 * @param {!box2d.dynamics.Body} body
 */
box2d.dynamics.BodyList.prototype.updateBody = function(body) {
    var type = body.getType();
    var bodyID = body.ID;
    var awake = body.isAwake();
    var active = body.isActive();
    if (type == box2d.dynamics.BodyDef.dynamicBody) {
        this.createNode(body, bodyID, box2d.dynamics.BodyList.TYPES.dynamicBodies);
    } else {
        this.removeNode(bodyID, box2d.dynamics.BodyList.TYPES.dynamicBodies);
    }
    if (type != box2d.dynamics.BodyDef.staticBody) {
        this.createNode(body, bodyID, box2d.dynamics.BodyList.TYPES.nonStaticBodies);
    } else {
        this.removeNode(bodyID, box2d.dynamics.BodyList.TYPES.nonStaticBodies);
    }
    if (type != box2d.dynamics.BodyDef.staticBody && active && awake) {
        this.createNode(body, bodyID, box2d.dynamics.BodyList.TYPES.nonStaticActiveAwakeBodies);
    } else {
        this.removeNode(bodyID, box2d.dynamics.BodyList.TYPES.nonStaticActiveAwakeBodies);
    }
    if (awake) {
        this.createNode(body, bodyID, box2d.dynamics.BodyList.TYPES.awakeBodies);
    } else {
        this.removeNode(bodyID, box2d.dynamics.BodyList.TYPES.awakeBodies);
    }
    if (active) {
        this.createNode(body, bodyID, box2d.dynamics.BodyList.TYPES.activeBodies);
    } else {
        this.removeNode(bodyID, box2d.dynamics.BodyList.TYPES.activeBodies);
    }
};

/**
 * @param {!box2d.dynamics.Body} body
 */
box2d.dynamics.BodyList.prototype.removeBody = function(body) {
    var bodyID = body.ID;
    if (this.bodyNodeLookup[bodyID] != null) {
        goog.array.remove(body.m_lists, this);
        for (var i = 0; i <= box2d.dynamics.BodyList.TYPES.allBodies; i++) {
            this.removeNode(bodyID, i);
        }
        delete this.bodyNodeLookup[bodyID];
        this.bodyCount--;
    }
};

/**
 * @param {string} bodyID
 * @param {number} type
 */
box2d.dynamics.BodyList.prototype.removeNode = function(bodyID, type) {
    var nodeList = this.bodyNodeLookup[bodyID];
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
        this.bodyFirstNodes[type] = nextNode;
    } else {
        prevNode.setNextNode(nextNode);
    }
    if (nextNode == null) {
        this.bodyLastNodes[type] = prevNode;
    } else {
        nextNode.setPreviousNode(prevNode);
    }
};

/**
 * @param {!box2d.dynamics.Body} body
 * @param {string} bodyID
 * @param {number} type
 */
box2d.dynamics.BodyList.prototype.createNode = function(body, bodyID, type) {
    var nodeList = this.bodyNodeLookup[bodyID];
    if (nodeList == null) {
        nodeList = [];
        for (var i = 0; i <= box2d.dynamics.BodyList.TYPES.allBodies; i++) {
            nodeList[i] = null;
        }
        this.bodyNodeLookup[bodyID] = nodeList;
    }
    if (nodeList[type] == null) {
        nodeList[type] = new box2d.dynamics.BodyListNode(body);
        var prevNode = this.bodyLastNodes[type];
        if (prevNode != null) {
            prevNode.setNextNode(nodeList[type]);
        } else {
            this.bodyFirstNodes[type] = nodeList[type];
        }
        nodeList[type].setPreviousNode(prevNode);
        this.bodyLastNodes[type] = nodeList[type];
    }
};

/**
 * @return {number}
 */
box2d.dynamics.BodyList.prototype.getBodyCount = function() {
    return this.bodyCount;
};

/**
 * @enum {number}
 */
box2d.dynamics.BodyList.TYPES = {
    dynamicBodies: 0,
    nonStaticBodies: 1,
    activeBodies: 2,
    nonStaticActiveAwakeBodies: 3,
    awakeBodies: 4,
    allBodies: 5 // Assumed to be last by above code
};
