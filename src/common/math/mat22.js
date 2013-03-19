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

goog.provide('box2d.common.math.Mat22');

goog.require('UsageTracker');
goog.require('box2d.common.math.Vec2');

/**
 * @constructor
 * @private
 */
box2d.common.math.Mat22 = function() {
    UsageTracker.get('box2d.common.math.Mat22').trackCreate();

    this.col1 = box2d.common.math.Vec2.get(0, 0);
    this.col2 = box2d.common.math.Vec2.get(0, 0);
    this.setIdentity();
};

/**
 * @private
 * @type {Array.<!box2d.common.math.Mat22>}
 */
box2d.common.math.Mat22.freeCache_ = [];

/**
 * @return {!box2d.common.math.Mat22}
 */
box2d.common.math.Mat22.get = function() {
    UsageTracker.get('box2d.common.math.Mat22').trackGet();
    if (box2d.common.math.Mat22.freeCache_.length > 0) {
        var mat = box2d.common.math.Mat22.freeCache_.pop();
        mat.setZero();
        return mat;
    }
    return new box2d.common.math.Mat22();
};

/**
 * @param {!box2d.common.math.Mat22} mat
 */
box2d.common.math.Mat22.free = function(mat) {
    if (mat != null) {
        UsageTracker.get('box2d.common.math.Mat22').trackFree();
        box2d.common.math.Mat22.freeCache_.push(mat);
    }
};

/**
 * @param {number} angle
 * @return {!box2d.common.math.Mat22}
 */
box2d.common.math.Mat22.fromAngle = function(angle) {
    var mat = box2d.common.math.Mat22.get();
    mat.set(angle);
    return mat;
};

/**
 * @param {!box2d.common.math.Vec2} c1
 * @param {!box2d.common.math.Vec2} c2
 * @return {!box2d.common.math.Mat22}
 */
box2d.common.math.Mat22.fromVV = function(c1, c2) {
    var mat = box2d.common.math.Mat22.get();
    mat.setVV(c1, c2);
    return mat;
};

/**
 * @param {number} angle
 */
box2d.common.math.Mat22.prototype.set = function(angle) {
    var c = Math.cos(angle);
    var s = Math.sin(angle);
    this.col1.set(c, s);
    this.col2.set(-s, c);
};

/**
 * @param {!box2d.common.math.Vec2} c1
 * @param {!box2d.common.math.Vec2} c2
 */
box2d.common.math.Mat22.prototype.setVV = function(c1, c2) {
    this.col1.setV(c1);
    this.col2.setV(c2);
};

/**
 * @return {!box2d.common.math.Mat22}
 */
box2d.common.math.Mat22.prototype.copy = function() {
    var mat = box2d.common.math.Mat22.get();
    mat.setM(this);
    return mat;
};

/**
 * @param {!box2d.common.math.Mat22} m
 */
box2d.common.math.Mat22.prototype.setM = function(m) {
    this.col1.setV(m.col1);
    this.col2.setV(m.col2);
};

/**
 * @param {!box2d.common.math.Mat22} m
 */
box2d.common.math.Mat22.prototype.addM = function(m) {
    this.col1.add(m.col1);
    this.col2.add(m.col2);
};

box2d.common.math.Mat22.prototype.setIdentity = function() {
    this.col1.set(1, 0);
    this.col2.set(0, 1);
};

box2d.common.math.Mat22.prototype.setZero = function() {
    this.col1.set(0, 0);
    this.col2.set(0, 0);
};

/**
 * @return {number}
 */
box2d.common.math.Mat22.prototype.getAngle = function() {
    return Math.atan2(this.col1.y, this.col1.x);
};

/**
 * @param {!box2d.common.math.Mat22} out
 * @return {!box2d.common.math.Mat22}
 */
box2d.common.math.Mat22.prototype.getInverse = function(out) {
    var det = this.col1.x * this.col2.y - this.col2.x * this.col1.y;
    if (det !== 0) {
        det = 1 / det;
    }
    out.col1.x = det * this.col2.y;
    out.col2.x = -det * this.col2.x;
    out.col1.y = -det * this.col1.y;
    out.col2.y = det * this.col1.x;
    return out;
};

/**
 * @param {!box2d.common.math.Vec2} out
 * @param {number} bX
 * @param {number} bY
 * @return {!box2d.common.math.Vec2}
 */
box2d.common.math.Mat22.prototype.solve = function(out, bX, bY) {
    var det = this.col1.x * this.col2.y - this.col2.x * this.col1.y;
    if (det !== 0) {
        det = 1 / det;
    }
    out.x = det * (this.col2.y * bX - this.col2.x * bY);
    out.y = det * (this.col1.x * bY - this.col1.y * bX);
    return out;
};

box2d.common.math.Mat22.prototype.abs = function() {
    this.col1.abs();
    this.col2.abs();
};
