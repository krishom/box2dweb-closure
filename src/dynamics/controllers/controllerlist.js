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

goog.provide('box2d.dynamics.controllers.ControllerList');

goog.require('box2d.dynamics.controllers.ControllerListNode');

/**
 * @constructor
 */
box2d.dynamics.controllers.ControllerList = function() {

    /**
     * @private
     * @type {box2d.dynamics.controllers.ControllerListNode}
     */
    this.controllerFirstNode = null;

    /**
     * @private
     * @type {box2d.dynamics.controllers.ControllerListNode}
     */
    this.controllerLastNode = null;

    /**
     * @private
     * @type {Object.<box2d.dynamics.controllers.ControllerListNode>}
     */
    this.controllerNodeLookup = {};

    /**
     * @private
     * @type {number}
     */
    this.controllerCount = 0;
};

/**
 * @return {box2d.dynamics.controllers.ControllerListNode}
 */
box2d.dynamics.controllers.ControllerList.prototype.getFirstNode = function() {
    return this.controllerFirstNode;
};

/**
 * @param {!box2d.dynamics.controllers.Controller} controller
 */
box2d.dynamics.controllers.ControllerList.prototype.addController = function(controller) {
    var controllerID = controller.ID;
    if (this.controllerNodeLookup[controllerID] == null) {
        var node = new box2d.dynamics.controllers.ControllerListNode(controller);
        var prevNode = this.controllerLastNode;
        if (prevNode != null) {
            prevNode.setNextNode(node);
        } else {
            this.controllerFirstNode = node;
        }
        node.setPreviousNode(prevNode);
        this.controllerLastNode = node;
        this.controllerNodeLookup[controllerID] = node;
        this.controllerCount++;
    }
};

/**
 * @param {!box2d.dynamics.controllers.Controller} controller
 */
box2d.dynamics.controllers.ControllerList.prototype.removeController = function(controller) {
    var controllerID = controller.ID;
    var node = this.controllerNodeLookup[controllerID];
    if (node == null) {
        return;
    }
    var prevNode = node.getPreviousNode();
    var nextNode = node.getNextNode();
    if (prevNode == null) {
        this.controllerFirstNode = nextNode;
    } else {
        prevNode.setNextNode(nextNode);
    }
    if (nextNode == null) {
        this.controllerLastNode = prevNode;
    } else {
        nextNode.setPreviousNode(prevNode);
    }
    delete this.controllerNodeLookup[controllerID];
    this.controllerCount--;
};

/**
 * @return {number}
 */
box2d.dynamics.controllers.ControllerList.prototype.getControllerCount = function() {
    return this.controllerCount;
};
