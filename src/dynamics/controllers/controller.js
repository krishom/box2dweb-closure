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

goog.provide('box2d.dynamics.controllers.Controller');

goog.require('box2d.dynamics.BodyList');

/**
 * @constructor
 */
box2d.dynamics.controllers.Controller = function() {

    /**
     * @const
     * @type {string}
     */
    this.ID = 'Controller' + box2d.dynamics.controllers.Controller.NEXT_ID++;

    /**
     * @type {box2d.dynamics.World}
     */
    this.m_world = null;

    /**
     * @type {!box2d.dynamics.BodyList}
     */
    this.bodyList = new box2d.dynamics.BodyList();
};

box2d.dynamics.controllers.Controller.prototype.step = function(step) {
};

box2d.dynamics.controllers.Controller.prototype.draw = function(debugdraw) {
};

/**
 * @param {!box2d.dynamics.Body} body
 */
box2d.dynamics.controllers.Controller.prototype.addBody = function(body) {
    this.bodyList.addBody(body);
    body.addController(this);
};

/**
 * @param {!box2d.dynamics.Body} body
 */
box2d.dynamics.controllers.Controller.prototype.removeBody = function(body) {
    this.bodyList.removeBody(body);
    body.removeController(this);
};

box2d.dynamics.controllers.Controller.prototype.clear = function() {
    for (var node = this.bodyList.getFirstNode(box2d.dynamics.BodyList.TYPES.allBodies); node; node = node.getNextNode()) {
        this.removeBody(node.body);
    }
};

/**
 * @return {!box2d.dynamics.BodyList}
 */
box2d.dynamics.controllers.Controller.prototype.getBodyList = function() {
    return this.bodyList;
};

/**
 * @type {number}
 * @private
 */
box2d.dynamics.controllers.Controller.NEXT_ID = 0;
