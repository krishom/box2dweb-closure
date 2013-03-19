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

goog.provide('box2d.collision.DynamicTreeNode');

goog.require('UsageTracker');
goog.require('box2d.collision.AABB');

/**
 * @private
 * @param {box2d.dynamics.Fixture} fixture
 * @constructor
 */
box2d.collision.DynamicTreeNode = function(fixture) {
    UsageTracker.get('box2d.collision.DynamicTreeNode').trackCreate();

    /**
     * @type {!box2d.collision.AABB}
     */
    this.aabb = box2d.collision.AABB.get();

    /**
     * @type {box2d.collision.DynamicTreeNode}
     */
    this.child1 = null;

    /**
     * @type {box2d.collision.DynamicTreeNode}
     */
    this.child2 = null;

    /**
     * @type {box2d.collision.DynamicTreeNode}
     */
    this.parent = null;

    /**
     * @type {box2d.dynamics.Fixture}
     */
    this.fixture = fixture;
};

/**
 * @private
 * @type {Array.<!box2d.collision.DynamicTreeNode>}
 */
box2d.collision.DynamicTreeNode.freeCache_ = [];

/**
 * @param {box2d.dynamics.Fixture=} fixture
 * @return {!box2d.collision.DynamicTreeNode}
 */
box2d.collision.DynamicTreeNode.get = function(fixture) {
    if (typeof(fixture) == 'undefined') {
        fixture = null;
    }
    if (box2d.collision.DynamicTreeNode.freeCache_.length > 0) {
        var node = box2d.collision.DynamicTreeNode.freeCache_.pop();
        node.fixture = fixture;
        node.aabb.setZero();
        return node;
    }
    return new box2d.collision.DynamicTreeNode(fixture);
};

box2d.collision.DynamicTreeNode.prototype.destroy = function() {
    this.child1 = null;
    this.child2 = null;
    this.parent = null;
    this.fixture = null;
    box2d.collision.DynamicTreeNode.freeCache_.push(this);
};

/**
 * @return boolean.
 */
box2d.collision.DynamicTreeNode.prototype.isLeaf = function() {
    return this.child1 === null;
};
