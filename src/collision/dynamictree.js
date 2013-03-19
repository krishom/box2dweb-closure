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

goog.provide('box2d.collision.DynamicTree');

goog.require('UsageTracker');
goog.require('box2d.collision.AABB');
goog.require('box2d.collision.DynamicTreeNode');
goog.require('box2d.collision.RayCastInput');
goog.require('box2d.common.Settings');
goog.require('box2d.common.math.Math');

/**
 * @constructor
 */
box2d.collision.DynamicTree = function() {
    UsageTracker.get('box2d.collision.DynamicTree').trackCreate();

    /**
     * @type {box2d.collision.DynamicTreeNode}
     */
    this.m_root = null;

    /**
     * @type {number}
     */
    this.m_path = 0;

    /**
     * @type {number}
     */
    this.m_insertionCount = 0;
};

/**
 * @param {!box2d.collision.AABB} aabb
 * @param {box2d.dynamics.Fixture} fixture
 * @return {!box2d.collision.DynamicTreeNode}
 */
box2d.collision.DynamicTree.prototype.createProxy = function(aabb, fixture) {
    var node = box2d.collision.DynamicTreeNode.get(fixture);
    var extendX = box2d.common.Settings.aabbExtension;
    var extendY = box2d.common.Settings.aabbExtension;
    node.aabb.lowerBound.x = aabb.lowerBound.x - extendX;
    node.aabb.lowerBound.y = aabb.lowerBound.y - extendY;
    node.aabb.upperBound.x = aabb.upperBound.x + extendX;
    node.aabb.upperBound.y = aabb.upperBound.y + extendY;
    this.insertLeaf(node);
    return node;
};

/**
 * @param {!box2d.collision.DynamicTreeNode} proxy
 */
box2d.collision.DynamicTree.prototype.destroyProxy = function(proxy) {
    this.removeLeaf(proxy);
    proxy.destroy();
};

/**
 * @param {!box2d.collision.DynamicTreeNode} proxy
 * @param {!box2d.collision.AABB} aabb
 * @param {!box2d.common.math.Vec2} displacement
 * @return {boolean}
 */
box2d.collision.DynamicTree.prototype.moveProxy = function(proxy, aabb, displacement) {
    box2d.common.Settings.assert(proxy.isLeaf());
    if (proxy.aabb.contains(aabb)) {
        return false;
    }
    this.removeLeaf(proxy);
    var extendX = box2d.common.Settings.aabbExtension + box2d.common.Settings.aabbmultiplier * Math.abs(displacement.x);
    var extendY = box2d.common.Settings.aabbExtension + box2d.common.Settings.aabbmultiplier * Math.abs(displacement.y);
    proxy.aabb.lowerBound.x = aabb.lowerBound.x - extendX;
    proxy.aabb.lowerBound.y = aabb.lowerBound.y - extendY;
    proxy.aabb.upperBound.x = aabb.upperBound.x + extendX;
    proxy.aabb.upperBound.y = aabb.upperBound.y + extendY;
    this.insertLeaf(proxy);
    return true;
};

/**
 * @param {number} iterations
 */
box2d.collision.DynamicTree.prototype.rebalance = function(iterations) {
    if (this.m_root !== null) {
        for (var i = 0; i < iterations; i++) {
            var node = this.m_root;
            var bit = 0;
            while (!node.isLeaf()) {
                node = (this.m_path >> bit) & 1 ? node.child2 : node.child1;
                bit = (bit + 1) & 31;
            }
            this.m_path++;
            this.removeLeaf(node);
            this.insertLeaf(node);
        }
    }
};

/**
 * @param {!box2d.collision.DynamicTreeNode} proxy
 * @return {!box2d.collision.AABB}
 */
box2d.collision.DynamicTree.prototype.getFatAABB = function(proxy) {
    return proxy.aabb;
};

/**
 * @param {function(!box2d.dynamics.Fixture): boolean} callback
 * @param {!box2d.collision.AABB} aabb
 * @param {!Object=} callbackObject
 */
box2d.collision.DynamicTree.prototype.query = function(callback, aabb, callbackObject) {
    if (this.m_root !== null) {
        var stack = [];
        stack.push(this.m_root);
        while (stack.length > 0) {
            var node = stack.pop();
            if (node.aabb.testOverlap(aabb)) {
                if (node.isLeaf()) {
                    if (!callback.call(callbackObject, node.fixture)) {
                        return;
                    }
                } else {
                    stack.push(node.child1);
                    stack.push(node.child2);
                }
            }
        }
    }
};

/**
 * @param {function(!box2d.collision.RayCastInput, !box2d.dynamics.Fixture): number} callback
 * @param {!box2d.collision.RayCastInput} input
 */
box2d.collision.DynamicTree.prototype.rayCast = function(callback, input) {
    if (this.m_root === null) {
        return;
    }

    var r = box2d.common.math.Math.subtractVV(input.p1, input.p2);
    r.normalize();
    var v = box2d.common.math.Math.crossFV(1.0, r);
    box2d.common.math.Vec2.free(r);
    var abs_v = box2d.common.math.Math.absV(v);
    var maxFraction = input.maxFraction;
    var tX = input.p1.x + maxFraction * (input.p2.x - input.p1.x);
    var tY = input.p1.y + maxFraction * (input.p2.y - input.p1.y);

    var segmentAABB = box2d.collision.AABB.get();
    segmentAABB.lowerBound.x = Math.min(input.p1.x, tX);
    segmentAABB.lowerBound.y = Math.min(input.p1.y, tY);
    segmentAABB.upperBound.x = Math.max(input.p1.x, tX);
    segmentAABB.upperBound.y = Math.max(input.p1.y, tY);

    var stack = [];
    stack.push(this.m_root);
    while (stack.length > 0) {
        var node = stack.pop();
        if (!node.aabb.testOverlap(segmentAABB)) {
            continue;
        }
        var c = node.aabb.getCenter();
        var h = node.aabb.getExtents();
        var separation = Math.abs(v.x * (input.p1.x - c.x) + v.y * (input.p1.y - c.y)) - abs_v.x * h.x - abs_v.y * h.y;
        box2d.common.math.Vec2.free(c);
        box2d.common.math.Vec2.free(h);
        if (separation > 0.0) {
            continue;
        }
        if (node.isLeaf()) {
            var subInput = new box2d.collision.RayCastInput(input.p1, input.p2, input.maxFraction);
            maxFraction = callback(input, node.fixture);
            if (maxFraction == 0.0) {
                break;
            }
            if (maxFraction > 0.0) {
                tX = input.p1.x + maxFraction * (input.p2.x - input.p1.x);
                tY = input.p1.y + maxFraction * (input.p2.y - input.p1.y);
                segmentAABB.lowerBound.x = Math.min(input.p1.x, tX);
                segmentAABB.lowerBound.y = Math.min(input.p1.y, tY);
                segmentAABB.upperBound.x = Math.max(input.p1.x, tX);
                segmentAABB.upperBound.y = Math.max(input.p1.y, tY);
            }
        } else {
            stack.push(node.child1);
            stack.push(node.child2);
        }
    }
    box2d.common.math.Vec2.free(v);
    box2d.common.math.Vec2.free(abs_v);
    box2d.collision.AABB.free(segmentAABB);
};

/**
 * @param {!box2d.collision.DynamicTreeNode} leaf
 */
box2d.collision.DynamicTree.prototype.insertLeaf = function(leaf) {
    this.m_insertionCount++;
    if (this.m_root === null) {
        this.m_root = leaf;
        this.m_root.parent = null;
        return;
    }
    var sibling = this.getBestSibling(leaf);

    var parent = sibling.parent;
    var node2 = box2d.collision.DynamicTreeNode.get();
    node2.parent = parent;
    node2.aabb.combine(leaf.aabb, sibling.aabb);
    if (parent) {
        if (sibling.parent.child1 == sibling) {
            parent.child1 = node2;
        } else {
            parent.child2 = node2;
        }
        node2.child1 = sibling;
        node2.child2 = leaf;
        sibling.parent = node2;
        leaf.parent = node2;
        while (parent) {
            if (parent.aabb.contains(node2.aabb)) {
                break;
            }
            parent.aabb.combine(parent.child1.aabb, parent.child2.aabb);
            node2 = parent;
            parent = parent.parent;
        }
    } else {
        node2.child1 = sibling;
        node2.child2 = leaf;
        sibling.parent = node2;
        leaf.parent = node2;
        this.m_root = node2;
    }
};

/**
 * @param {!box2d.collision.DynamicTreeNode} leaf
 * @return {!box2d.collision.DynamicTreeNode}
 */
box2d.collision.DynamicTree.prototype.getBestSibling = function(leaf) {
    var center = leaf.aabb.getCenter();
    var sibling = this.m_root;
    while (!sibling.isLeaf()) {
        var child1 = sibling.child1;
        var child2 = sibling.child2;
        var norm1 = Math.abs((child1.aabb.lowerBound.x + child1.aabb.upperBound.x) / 2 - center.x) + Math.abs((child1.aabb.lowerBound.y + child1.aabb.upperBound.y) / 2 - center.y);
        var norm2 = Math.abs((child2.aabb.lowerBound.x + child2.aabb.upperBound.x) / 2 - center.x) + Math.abs((child2.aabb.lowerBound.y + child2.aabb.upperBound.y) / 2 - center.y);
        if (norm1 < norm2) {
            sibling = child1;
        } else {
            sibling = child2;
        }
    }
    box2d.common.math.Vec2.free(center);
    return sibling;
};

/**
 * @param {!box2d.collision.DynamicTreeNode} leaf
 */
box2d.collision.DynamicTree.prototype.removeLeaf = function(leaf) {
    if (leaf == this.m_root) {
        this.m_root = null;
        return;
    }
    var node2 = leaf.parent;
    var node1 = node2.parent;
    var sibling;
    if (node2.child1 == leaf) {
        sibling = node2.child2;
    } else {
        sibling = node2.child1;
    }
    if (node1) {
        if (node1.child1 == node2) {
            node1.child1 = sibling;
        } else {
            node1.child2 = sibling;
        }
        sibling.parent = node1;
        while (node1) {
            var oldAABB = node1.aabb;
            node1.aabb.combine(node1.child1.aabb, node1.child2.aabb);
            if (oldAABB.contains(node1.aabb)) {
                break;
            }
            node1 = node1.parent;
        }
    } else {
        this.m_root = sibling;
        sibling.parent = null;
    }
    node2.destroy();
};
