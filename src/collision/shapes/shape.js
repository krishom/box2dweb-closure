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

goog.provide('box2d.collision.shapes.Shape');

goog.require('UsageTracker');
goog.require('box2d.collision.Distance');
goog.require('box2d.collision.DistanceInput');
goog.require('box2d.collision.DistanceOutput');
goog.require('box2d.collision.DistanceProxy');
goog.require('box2d.collision.SimplexCache');

/**
 * @constructor
 */
box2d.collision.shapes.Shape = function() {
    UsageTracker.get('box2d.collision.shapes.Shape').trackCreate();

    /**
     * @type {number}
     */
    this.m_radius = box2d.common.Settings.linearSlop;
};

/**
 * @return {string}
 */
box2d.collision.shapes.Shape.prototype.getTypeName = function() {
    return goog.abstractMethod();
};

/**
 * @return {!box2d.collision.shapes.Shape}
 */
box2d.collision.shapes.Shape.prototype.copy = function() {
    return goog.abstractMethod();
};

/**
 * @param {!box2d.collision.shapes.Shape} other
 */
box2d.collision.shapes.Shape.prototype.set = function(other) {
    this.m_radius = other.m_radius;
};

/**
 * @param {!box2d.common.math.Transform} xf
 * @param {!box2d.common.math.Vec2} p
 * @return {boolean}
 */
box2d.collision.shapes.Shape.prototype.testPoint = function(xf, p) {
    return goog.abstractMethod();
};

/**
 * @param {!box2d.collision.RayCastOutput} output
 * @param {!box2d.collision.RayCastInput} input
 * @param {!box2d.common.math.Transform} transform
 * @return {boolean}
 */
box2d.collision.shapes.Shape.prototype.rayCast = function(output, input, transform) {
    return goog.abstractMethod();
};

/**
 * @param {!box2d.collision.AABB} aabb
 * @param {!box2d.common.math.Transform} transform
 */
box2d.collision.shapes.Shape.prototype.computeAABB = function(aabb, transform) {
    goog.abstractMethod();
};

/**
 * @param {!box2d.collision.shapes.MassData} massData
 * @param {number} density
 */
box2d.collision.shapes.Shape.prototype.computeMass = function(massData, density) {
    goog.abstractMethod();
};

/**
 * @param {!box2d.common.math.Vec2} normal
 * @param {number} offset
 * @param {!box2d.common.math.Transform} xf
 * @param {!box2d.common.math.Vec2} c
 * @return {number}
 */
box2d.collision.shapes.Shape.prototype.computeSubmergedArea = function(normal, offset, xf, c) {
    return goog.abstractMethod();
};

/**
 * @param {!box2d.collision.DistanceProxy} proxy
 */
box2d.collision.shapes.Shape.prototype.setDistanceProxy = function(proxy) {
    goog.abstractMethod();
};

/**
 * @param {!box2d.collision.shapes.Shape} shape1
 * @param {!box2d.common.math.Transform} transform1
 * @param {!box2d.collision.shapes.Shape} shape2
 * @param {!box2d.common.math.Transform} transform2
 * @return {boolean}
 */
box2d.collision.shapes.Shape.testOverlap = function(shape1, transform1, shape2, transform2) {
    var input = new box2d.collision.DistanceInput();
    input.proxyA = new box2d.collision.DistanceProxy();
    input.proxyA.set(shape1);
    input.proxyB = new box2d.collision.DistanceProxy();
    input.proxyB.set(shape2);
    input.transformA = transform1;
    input.transformB = transform2;
    input.useRadii = true;
    var simplexCache = new box2d.collision.SimplexCache();
    simplexCache.count = 0;
    var output = new box2d.collision.DistanceOutput();
    box2d.collision.Distance.distance(output, simplexCache, input);
    box2d.common.math.Vec2.free(output.pointA);
    box2d.common.math.Vec2.free(output.pointB);
    return output.distance < 10.0 * Number.MIN_VALUE;
};

/**
 * @const
 * @type {number}
 */
box2d.collision.shapes.Shape.e_startsInsidecollide = -1;

/**
 * @const
 * @type {number}
 */
box2d.collision.shapes.Shape.e_misscollide = 0;

/**
 * @const
 * @type {number}
 */
box2d.collision.shapes.Shape.e_hitcollide = 1;
