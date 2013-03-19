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

goog.provide('box2d.common.math.Math');

goog.require('box2d.common.math.Mat22');
goog.require('box2d.common.math.Vec2');

box2d.common.math.Math = {};

/**
 * @param {!box2d.common.math.Vec2} a
 * @param {!box2d.common.math.Vec2} b
 * @return {number}
 */
box2d.common.math.Math.Dot = function(a, b) {
    return a.x * b.x + a.y * b.y;
};

/**
 * @param {!box2d.common.math.Vec2} a
 * @param {!box2d.common.math.Vec2} b
 * @return {number}
 */
box2d.common.math.Math.crossVV = function(a, b) {
    return a.x * b.y - a.y * b.x;
};

/**
 * @param {!box2d.common.math.Vec2} a
 * @param {number} s
 * @return {!box2d.common.math.Vec2}
 */
box2d.common.math.Math.crossVF = function(a, s) {
    return box2d.common.math.Vec2.get(s * a.y, (-s * a.x));
};

/**
 * @param {number} s
 * @param {!box2d.common.math.Vec2} a
 * @return {!box2d.common.math.Vec2}
 */
box2d.common.math.Math.crossFV = function(s, a) {
    return box2d.common.math.Vec2.get((-s * a.y), s * a.x);
};

/**
 * @param {!box2d.common.math.Mat22} A
 * @param {!box2d.common.math.Vec2} v
 * @return {!box2d.common.math.Vec2}
 */
box2d.common.math.Math.mulMV = function(A, v) {
    return box2d.common.math.Vec2.get(A.col1.x * v.x + A.col2.x * v.y, A.col1.y * v.x + A.col2.y * v.y);
};

/**
 * @param {!box2d.common.math.Mat22} A
 * @param {!box2d.common.math.Vec2} v
 * @return {!box2d.common.math.Vec2}
 */
box2d.common.math.Math.mulTMV = function(A, v) {
    return box2d.common.math.Vec2.get(box2d.common.math.Math.Dot(v, A.col1), box2d.common.math.Math.Dot(v, A.col2));
};

/**
 * @param {!box2d.common.math.Transform} T
 * @param {!box2d.common.math.Vec2} v
 * @return {!box2d.common.math.Vec2}
 */
box2d.common.math.Math.mulX = function(T, v) {
    var a = box2d.common.math.Math.mulMV(T.R, v);
    a.x += T.position.x;
    a.y += T.position.y;
    return a;
};

/**
 * @param {!box2d.common.math.Transform} T
 * @param {!box2d.common.math.Vec2} v
 * @return {!box2d.common.math.Vec2}
 */
box2d.common.math.Math.mulXT = function(T, v) {
    var a = box2d.common.math.Math.subtractVV(v, T.position);
    var tX = (a.x * T.R.col1.x + a.y * T.R.col1.y);
    a.y = (a.x * T.R.col2.x + a.y * T.R.col2.y);
    a.x = tX;
    return a;
};

/**
 * @param {!box2d.common.math.Vec2} a
 * @param {!box2d.common.math.Vec2} b
 * @return {!box2d.common.math.Vec2}
 */
box2d.common.math.Math.addVV = function(a, b) {
    return box2d.common.math.Vec2.get(a.x + b.x, a.y + b.y);
};

/**
 * @param {!box2d.common.math.Vec2} a
 * @param {!box2d.common.math.Vec2} b
 * @return {!box2d.common.math.Vec2}
 */
box2d.common.math.Math.subtractVV = function(a, b) {
    return box2d.common.math.Vec2.get(a.x - b.x, a.y - b.y);
};

/**
 * @param {!box2d.common.math.Vec2} a
 * @param {!box2d.common.math.Vec2} b
 * @return {number}
 */
box2d.common.math.Math.distance = function(a, b) {
    return Math.sqrt(box2d.common.math.Math.distanceSquared(a, b));
};

/**
 * @param {!box2d.common.math.Vec2} a
 * @param {!box2d.common.math.Vec2} b
 * @return {number}
 */
box2d.common.math.Math.distanceSquared = function(a, b) {
    var cX = a.x - b.x;
    var cY = a.y - b.y;
    return (cX * cX + cY * cY);
};

/**
 * @param {number} s
 * @param {!box2d.common.math.Vec2} a
 * @return {!box2d.common.math.Vec2}
 */
box2d.common.math.Math.mulFV = function(s, a) {
    return box2d.common.math.Vec2.get(s * a.x, s * a.y);
};

/**
 * @param {!box2d.common.math.Mat22} A
 * @param {!box2d.common.math.Mat22} B
 * @return {!box2d.common.math.Mat22}
 */
box2d.common.math.Math.addMM = function(A, B) {
    var v1 = box2d.common.math.Math.addVV(A.col1, B.col1);
    var v2 = box2d.common.math.Math.addVV(A.col2, B.col2);
    var m = box2d.common.math.Mat22.fromVV(v1, v2);
    box2d.common.math.Vec2.free(v1);
    box2d.common.math.Vec2.free(v2);
    return m;
};

/**
 * @param {!box2d.common.math.Mat22} A
 * @param {!box2d.common.math.Mat22} B
 * @return {!box2d.common.math.Mat22}
 */
box2d.common.math.Math.mulMM = function(A, B) {
    var v1 = box2d.common.math.Math.mulMV(A, B.col1);
    var v2 = box2d.common.math.Math.mulMV(A, B.col2);
    var m = box2d.common.math.Mat22.fromVV(v1, v2);
    box2d.common.math.Vec2.free(v1);
    box2d.common.math.Vec2.free(v2);
    return m;
};

/**
 * @param {!box2d.common.math.Mat22} A
 * @param {!box2d.common.math.Mat22} B
 * @return {!box2d.common.math.Mat22}
 */
box2d.common.math.Math.mulTMM = function(A, B) {
    var c1 = box2d.common.math.Vec2.get(box2d.common.math.Math.Dot(A.col1, B.col1), box2d.common.math.Math.Dot(A.col2, B.col1));
    var c2 = box2d.common.math.Vec2.get(box2d.common.math.Math.Dot(A.col1, B.col2), box2d.common.math.Math.Dot(A.col2, B.col2));
    var m = box2d.common.math.Mat22.fromVV(c1, c2);
    box2d.common.math.Vec2.free(c1);
    box2d.common.math.Vec2.free(c2);
    return m;
};

/**
 * @param {!box2d.common.math.Vec2} a
 * @return {!box2d.common.math.Vec2}
 */
box2d.common.math.Math.absV = function(a) {
    return box2d.common.math.Vec2.get(Math.abs(a.x), Math.abs(a.y));
};

/**
 * @param {!box2d.common.math.Mat22} A
 * @return {!box2d.common.math.Mat22}
 */
box2d.common.math.Math.absM = function(A) {
    var v1 = box2d.common.math.Math.absV(A.col1);
    var v2 = box2d.common.math.Math.absV(A.col2);
    var m = box2d.common.math.Mat22.fromVV(v1, v2);
    box2d.common.math.Vec2.free(v1);
    box2d.common.math.Vec2.free(v2);
    return m;
};

/**
 * @param {number} a
 * @param {number} low
 * @param {number} high
 * @return {number}
 */
box2d.common.math.Math.clamp = function(a, low, high) {
    return a < low ? low : a > high ? high : a;
};

/**
 * @param {!box2d.common.math.Vec2} a
 * @param {!box2d.common.math.Vec2} low
 * @param {!box2d.common.math.Vec2} high
 * @return {!box2d.common.math.Vec2}
 */
box2d.common.math.Math.clampV = function(a, low, high) {
    var x = box2d.common.math.Math.clamp(a.x, low.x, high.x);
    var y = box2d.common.math.Math.clamp(a.y, low.y, high.y);
    return box2d.common.math.Vec2.get(x, y);
};
