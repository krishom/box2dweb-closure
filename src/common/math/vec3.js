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

goog.provide('box2d.common.math.Vec3');

goog.require('UsageTracker');

/**
 * @param {number} x
 * @param {number} y
 * @param {number} z
 * @constructor
 * @private
 */
box2d.common.math.Vec3 = function(x, y, z) {
    UsageTracker.get('box2d.common.math.Vec3').trackCreate();

    this.x = x;
    this.y = y;
    this.z = z;
};

/**
 * @private
 * @type {Array.<!box2d.common.math.Vec3>}
 */
box2d.common.math.Vec3.freeCache_ = [];

/**
 * @param {number} x
 * @param {number} y
 * @param {number} z
 * @return {!box2d.common.math.Vec3}
 */
box2d.common.math.Vec3.get = function(x, y, z) {
    UsageTracker.get('box2d.common.math.Vec3').trackGet();
    if (box2d.common.math.Vec3.freeCache_.length > 0) {
        var vec = box2d.common.math.Vec3.freeCache_.pop();
        vec.set(x, y, z);
        return vec;
    }
    return new box2d.common.math.Vec3(x, y, z);
};

/**
 * @param {!box2d.common.math.Vec3} vec
 */
box2d.common.math.Vec3.free = function(vec) {
    if (vec != null) {
        UsageTracker.get('box2d.common.math.Vec3').trackFree();
        box2d.common.math.Vec3.freeCache_.push(vec);
    }
};

box2d.common.math.Vec3.prototype.setZero = function() {
    this.x = 0;
    this.y = 0;
    this.z = 0;
};

/**
 * @param {number} x
 * @param {number} y
 * @param {number} z
 */
box2d.common.math.Vec3.prototype.set = function(x, y, z) {
    this.x = x;
    this.y = y;
    this.z = z;
};

/**
 * @param {!box2d.common.math.Vec3} v
 */
box2d.common.math.Vec3.prototype.setV = function(v) {
    this.x = v.x;
    this.y = v.y;
    this.z = v.z;
};

/**
 * @return {!box2d.common.math.Vec3}
 */
box2d.common.math.Vec3.prototype.getNegative = function() {
    return box2d.common.math.Vec3.get((-this.x), (-this.y), (-this.z));
};

box2d.common.math.Vec3.prototype.negativeSelf = function() {
    this.x = (-this.x);
    this.y = (-this.y);
    this.z = (-this.z);
};

/**
 * @return {!box2d.common.math.Vec3}
 */
box2d.common.math.Vec3.prototype.copy = function() {
    return box2d.common.math.Vec3.get(this.x, this.y, this.z);
};

/**
 * @param {!box2d.common.math.Vec3} v
 */
box2d.common.math.Vec3.prototype.add = function(v) {
    this.x += v.x;
    this.y += v.y;
    this.z += v.z;
};

/**
 * @param {!box2d.common.math.Vec3} v
 */
box2d.common.math.Vec3.prototype.subtract = function(v) {
    this.x -= v.x;
    this.y -= v.y;
    this.z -= v.z;
};

/**
 * @param {number} a
 */
box2d.common.math.Vec3.prototype.multiply = function(a) {
    this.x *= a;
    this.y *= a;
    this.z *= a;
};
