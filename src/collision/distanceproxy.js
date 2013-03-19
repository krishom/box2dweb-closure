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

goog.provide('box2d.collision.DistanceProxy');

goog.require('UsageTracker');
goog.require('box2d.common.Settings');

/**
 * @constructor
 */
box2d.collision.DistanceProxy = function() {
    UsageTracker.get('box2d.collision.DistanceProxy').trackCreate();

    /**
     * @type {number}
     */
    this.m_count = 0;

    /**
     * @type {number}
     */
    this.m_radius = 0;

    /**
     * @type {Array.<!box2d.common.math.Vec2>}
     */
    this.m_vertices = [];
};

/**
 * @param {number} count
 * @param {number} radius
 * @param {!Array.<!box2d.common.math.Vec2>} vertices
 */
box2d.collision.DistanceProxy.prototype.setValues = function(count, radius, vertices) {
    this.m_count = count;
    this.m_radius = radius;
    this.m_vertices = vertices;
};

/**
 * @param {!box2d.collision.shapes.Shape} shape
 */
box2d.collision.DistanceProxy.prototype.set = function(shape) {
    shape.setDistanceProxy(this);
};

box2d.collision.DistanceProxy.prototype.getSupport = function(d) {
    var bestIndex = 0;
    var bestValue = this.m_vertices[0].x * d.x + this.m_vertices[0].y * d.y;
    for (var i = 1; i < this.m_count; i++) {
        var value = this.m_vertices[i].x * d.x + this.m_vertices[i].y * d.y;
        if (value > bestValue) {
            bestIndex = i;
            bestValue = value;
        }
    }
    return bestIndex;
};

box2d.collision.DistanceProxy.prototype.getSupportVertex = function(d) {
    return this.m_vertices[this.getSupport(d)];
};

box2d.collision.DistanceProxy.prototype.getVertexCount = function() {
    return this.m_count;
};

box2d.collision.DistanceProxy.prototype.getVertex = function(index) {
    if (index === undefined) index = 0;
    box2d.common.Settings.assert(0 <= index && index < this.m_count);
    return this.m_vertices[index];
};
