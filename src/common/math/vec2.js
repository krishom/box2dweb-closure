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

goog.provide('box2d.common.math.Vec2');

goog.require('UsageTracker');

/**
 * @private
 * @param {number} x
 * @param {number} y
 * @constructor
 */
box2d.common.math.Vec2 = function(x, y) {
    UsageTracker.get('box2d.common.math.Vec2').trackCreate();
    this.x = x;
    this.y = y;
};

/**
 * @private
 * @type {Array.<!box2d.common.math.Vec2>}
 */
box2d.common.math.Vec2.freeCache_ = [];

/**
 * @param {number} x
 * @param {number} y
 * @return {!box2d.common.math.Vec2}
 */
box2d.common.math.Vec2.get = function(x, y) {
    UsageTracker.get('box2d.common.math.Vec2').trackGet();
    if (box2d.common.math.Vec2.freeCache_.length > 0) {
        var vec = box2d.common.math.Vec2.freeCache_.pop();
        vec.set(x, y);
        return vec;
    }
    return new box2d.common.math.Vec2(x, y);
};

/**
 * @param {!box2d.common.math.Vec2} vec
 */
box2d.common.math.Vec2.free = function(vec) {
    if (vec != null) {
        UsageTracker.get('box2d.common.math.Vec2').trackFree();
        box2d.common.math.Vec2.freeCache_.push(vec);
    }
};

box2d.common.math.Vec2.prototype.setZero = function() {
    this.x = 0.0;
    this.y = 0.0;
};

/**
 * @param {number} x
 * @param {number} y
 */
box2d.common.math.Vec2.prototype.set = function(x, y) {
    this.x = x;
    this.y = y;
};

/**
 * @param {!box2d.common.math.Vec2} v
 */
box2d.common.math.Vec2.prototype.setV = function(v) {
    this.x = v.x;
    this.y = v.y;
};

/**
 * @return {!box2d.common.math.Vec2}
 */
box2d.common.math.Vec2.prototype.getNegative = function() {
    return box2d.common.math.Vec2.get((-this.x), (-this.y));
};

box2d.common.math.Vec2.prototype.negativeSelf = function() {
    this.x = (-this.x);
    this.y = (-this.y);
};

/**
 * @return {!box2d.common.math.Vec2}
 */
box2d.common.math.Vec2.prototype.copy = function() {
    return box2d.common.math.Vec2.get(this.x, this.y);
};

/**
 * @param {!box2d.common.math.Vec2} v
 */
box2d.common.math.Vec2.prototype.add = function(v) {
    this.x += v.x;
    this.y += v.y;
};

/**
 * @param {!box2d.common.math.Vec2} v
 */
box2d.common.math.Vec2.prototype.subtract = function(v) {
    this.x -= v.x;
    this.y -= v.y;
};

/**
 * @param {number} a
 */
box2d.common.math.Vec2.prototype.multiply = function(a) {
    this.x *= a;
    this.y *= a;
};

/**
 * @param {box2d.common.math.Mat22} A
 */
box2d.common.math.Vec2.prototype.mulM = function(A) {
    var tX = this.x;
    this.x = A.col1.x * tX + A.col2.x * this.y;
    this.y = A.col1.y * tX + A.col2.y * this.y;
};

/**
 * @param {box2d.common.math.Mat22} A
 */
box2d.common.math.Vec2.prototype.mulTM = function(A) {
    var tX = this.x * A.col1.x + this.y * A.col1.y;
    this.y = this.x * A.col2.x + this.y * A.col2.y;
    this.x = tX;
};

/**
 * @param {number} s
 */
box2d.common.math.Vec2.prototype.crossVF = function(s) {
    var tX = this.x;
    this.x = s * this.y;
    this.y = (-s * tX);
};

/**
 * @param {number} s
 */
box2d.common.math.Vec2.prototype.crossFV = function(s) {
    var tX = this.x;
    this.x = (-s * this.y);
    this.y = s * tX;
};

/**
 * @param {!box2d.common.math.Vec2} b
 */
box2d.common.math.Vec2.prototype.minV = function(b) {
    this.x = Math.min(this.x, b.x);
    this.y = Math.min(this.y, b.y);
};

/**
 * @param {!box2d.common.math.Vec2} b
 */
box2d.common.math.Vec2.prototype.maxV = function(b) {
    this.x = Math.max(this.x, b.x);
    this.y = Math.max(this.y, b.y);
};

box2d.common.math.Vec2.prototype.abs = function() {
    this.x = Math.abs(this.x);
    this.y = Math.abs(this.y);
};

/**
 * @return {number}
 */
box2d.common.math.Vec2.prototype.length = function() {
    return Math.sqrt(this.lengthSquared());
};

/**
 * @return {number}
 */
box2d.common.math.Vec2.prototype.lengthSquared = function() {
    return (this.x * this.x + this.y * this.y);
};

/**
 * @return {number}
 */
box2d.common.math.Vec2.prototype.normalize = function() {
    var length = this.length();
    if (length < Number.MIN_VALUE) {
        return 0.0;
    }
    var invlength = 1.0 / length;
    this.x *= invlength;
    this.y *= invlength;
    return length;
};

/**
 * @return {boolean}
 */
box2d.common.math.Vec2.prototype.isValid = function() {
    return isFinite(this.x) && isFinite(this.y);
};
