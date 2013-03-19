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

goog.provide('box2d.collision.DynamicTreeBroadPhase');

goog.require('UsageTracker');
goog.require('box2d.collision.DynamicTree');
goog.require('box2d.collision.DynamicTreePair');
goog.require('goog.array');

/**
 * @constructor
 */
box2d.collision.DynamicTreeBroadPhase = function() {
    UsageTracker.get('box2d.collision.DynamicTreeBroadPhase').trackCreate();

    /**
     * @type {!box2d.collision.DynamicTree}
     */
    this.m_tree = new box2d.collision.DynamicTree();

    /**
     * @type {Array.<!box2d.collision.DynamicTreeNode>}
     */
    this.m_moveBuffer = [];

    /**
     * @type {box2d.dynamics.Fixture}
     */
    this.lastqueryFixtureA = null;

    /**
     * @type {box2d.dynamics.Fixture}
     */
    this.lastqueryFixtureB = null;

    /**
     * @type {?function(!box2d.dynamics.Fixture, !box2d.dynamics.Fixture): boolean}
     */
    this.updatePairsCallback = null;

    /**
     * @type {box2d.collision.DynamicTreeNode}
     */
    this.queryProxy = null;

};

/**
 * @param {!box2d.collision.AABB} aabb
 * @param {box2d.dynamics.Fixture} fixture
 * @return {!box2d.collision.DynamicTreeNode}
 */
box2d.collision.DynamicTreeBroadPhase.prototype.createProxy = function(aabb, fixture) {
    var proxy = this.m_tree.createProxy(aabb, fixture);
    this.bufferMove(proxy);
    return proxy;
};

/**
 * @param {!box2d.collision.DynamicTreeNode} proxy
 */
box2d.collision.DynamicTreeBroadPhase.prototype.destroyProxy = function(proxy) {
    this.unBufferMove(proxy);
    this.m_tree.destroyProxy(proxy);
};

/**
 * @param {!box2d.collision.DynamicTreeNode} proxy
 * @param {!box2d.collision.AABB} aabb
 * @param {!box2d.common.math.Vec2} displacement
 */
box2d.collision.DynamicTreeBroadPhase.prototype.moveProxy = function(proxy, aabb, displacement) {
    var buffer = this.m_tree.moveProxy(proxy, aabb, displacement);
    if (buffer) {
        this.bufferMove(proxy);
    }
};

/**
 * @param {!box2d.collision.DynamicTreeNode} proxyA
 * @param {!box2d.collision.DynamicTreeNode} proxyB
 * @return {boolean}
 */
box2d.collision.DynamicTreeBroadPhase.prototype.testOverlap = function(proxyA, proxyB) {
    var aabbA = this.m_tree.getFatAABB(proxyA);
    var aabbB = this.m_tree.getFatAABB(proxyB);
    return aabbA.testOverlap(aabbB);
};

/**
 * @param {!box2d.collision.DynamicTreeNode} proxy
 * @return {!box2d.collision.AABB}
 */
box2d.collision.DynamicTreeBroadPhase.prototype.getFatAABB = function(proxy) {
    return this.m_tree.getFatAABB(proxy);
};

/**
 * @param {function(!box2d.dynamics.Fixture, !box2d.dynamics.Fixture)} callback
 */
box2d.collision.DynamicTreeBroadPhase.prototype.updatePairs = function(callback) {
    this.lastqueryFixtureA = null;
    this.lastqueryFixtureB = null;
    this.updatePairsCallback = callback;
    while (this.m_moveBuffer.length > 0) {
        this.queryProxy = this.m_moveBuffer.pop();
        this.m_tree.query(this.queryCallback_, this.m_tree.getFatAABB(this.queryProxy), this);
    }
    this.lastqueryFixtureA = null;
    this.lastqueryFixtureB = null;
    this.updatePairsCallback = null;
    this.queryProxy = null;
};

/**
 * @param {box2d.dynamics.Fixture} fixture
 * @return {boolean}
 * @private
 */
box2d.collision.DynamicTreeBroadPhase.prototype.queryCallback_ = function(fixture) {
    if (fixture != this.queryProxy.fixture) {
        if (!(this.queryProxy.fixture == this.lastqueryFixtureA && fixture == this.lastqueryFixtureB)
            && !(this.queryProxy.fixture == this.lastqueryFixtureB && fixture == this.lastqueryFixtureA)) {
            this.updatePairsCallback(/** @type {!box2d.dynamics.Fixture} */ (this.queryProxy.fixture), /** @type {!box2d.dynamics.Fixture} */ (fixture));
            this.lastqueryFixtureA = this.queryProxy.fixture;
            this.lastqueryFixtureB = fixture;
        }
    }
    return true;
};

/**
 * @param {function(!box2d.dynamics.Fixture): boolean} callback
 * @param {!box2d.collision.AABB} aabb
 */
box2d.collision.DynamicTreeBroadPhase.prototype.query = function(callback, aabb) {
    this.m_tree.query(callback, aabb);
};

/**
 * @param {function(!box2d.collision.RayCastInput, !box2d.dynamics.Fixture): number} callback
 * @param {!box2d.collision.RayCastInput} input
 */
box2d.collision.DynamicTreeBroadPhase.prototype.rayCast = function(callback, input) {
    this.m_tree.rayCast(callback, input);
};

/**
 * @param {number} iterations
 */
box2d.collision.DynamicTreeBroadPhase.prototype.rebalance = function(iterations) {
    this.m_tree.rebalance(iterations);
};

box2d.collision.DynamicTreeBroadPhase.prototype.bufferMove = function(proxy) {
    this.m_moveBuffer.push(proxy);
};

box2d.collision.DynamicTreeBroadPhase.prototype.unBufferMove = function(proxy) {
    goog.array.remove(this.m_moveBuffer, proxy);
};
