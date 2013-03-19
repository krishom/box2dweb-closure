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

goog.provide('box2d.collision.AABB');

goog.require('box2d.common.math.Vec2');

/**
 * @private
 * @constructor
 */
box2d.collision.AABB = function() {
    UsageTracker.get('box2d.collision.AABB').trackCreate();

    /**
     * @type {!box2d.common.math.Vec2}
     */
    this.lowerBound = box2d.common.math.Vec2.get(0, 0);

    /**
     * @type {!box2d.common.math.Vec2}
     */
    this.upperBound = box2d.common.math.Vec2.get(0, 0);
};

/**
 * @private
 * @type {Array.<!box2d.collision.AABB>}
 */
box2d.collision.AABB.freeCache_ = [];

/**
 * @return {!box2d.collision.AABB}
 */
box2d.collision.AABB.get = function() {
    UsageTracker.get('box2d.collision.AABB').trackGet();
    if (box2d.collision.AABB.freeCache_.length > 0) {
        var aabb = box2d.collision.AABB.freeCache_.pop();
        aabb.setZero();
        return aabb;
    }
    return new box2d.collision.AABB();
};

/**
 * @param {!box2d.collision.AABB} aabb
 */
box2d.collision.AABB.free = function(aabb) {
    if (aabb != null) {
        UsageTracker.get('box2d.collision.AABB').trackFree();
        box2d.collision.AABB.freeCache_.push(aabb);
    }
};

box2d.collision.AABB.prototype.setZero = function() {
    this.lowerBound.set(0, 0);
    this.upperBound.set(0, 0);
};

/**
 * @return {boolean}
 */
box2d.collision.AABB.prototype.isValid = function() {
    var dX = this.upperBound.x - this.lowerBound.x;
    if (dX < 0) {
        return false;
    }
    var dY = this.upperBound.y - this.lowerBound.y;
    if (dY < 0) {
        return false;
    }
    return this.lowerBound.isValid() && this.upperBound.isValid();
};

/**
 * @return {!box2d.common.math.Vec2}
 */
box2d.collision.AABB.prototype.getCenter = function() {
    return box2d.common.math.Vec2.get((this.lowerBound.x + this.upperBound.x) / 2, (this.lowerBound.y + this.upperBound.y) / 2);
};

/**
 * @param {!box2d.common.math.Vec2} newCenter
 */
box2d.collision.AABB.prototype.setCenter = function(newCenter) {
    var oldCenter = this.getCenter();
    this.lowerBound.subtract(oldCenter);
    this.upperBound.subtract(oldCenter);
    this.lowerBound.add(newCenter);
    this.upperBound.add(newCenter);
    box2d.common.math.Vec2.free(oldCenter);
};

/**
 * @return {!box2d.common.math.Vec2}
 */
box2d.collision.AABB.prototype.getExtents = function() {
    return box2d.common.math.Vec2.get((this.upperBound.x - this.lowerBound.x) / 2, (this.upperBound.y - this.lowerBound.y) / 2);
};

/**
 * @param {!box2d.collision.AABB} aabb
 * @return {boolean}
 */
box2d.collision.AABB.prototype.contains = function(aabb) {
    var result = true;
    result = result && this.lowerBound.x <= aabb.lowerBound.x;
    result = result && this.lowerBound.y <= aabb.lowerBound.y;
    result = result && aabb.upperBound.x <= this.upperBound.x;
    result = result && aabb.upperBound.y <= this.upperBound.y;
    return result;
};

/**
 * @param {!box2d.collision.RayCastOutput} output
 * @param {!box2d.collision.RayCastInput} input
 * @return {boolean}
 */
box2d.collision.AABB.prototype.rayCast = function(output, input) {
    var tmin = (-Number.MAX_VALUE);
    var tmax = Number.MAX_VALUE;

    var dX = input.p2.x - input.p1.x;
    var absDX = Math.abs(dX);
    if (absDX < Number.MIN_VALUE) {
        if (input.p1.x < this.lowerBound.x || this.upperBound.x < input.p1.x) {
            return false;
        }
    } else {
        var inv_d = 1.0 / dX;
        var t1 = (this.lowerBound.x - input.p1.x) * inv_d;
        var t2 = (this.upperBound.x - input.p1.x) * inv_d;
        var s = (-1.0);
        if (t1 > t2) {
            var t3 = t1;
            t1 = t2;
            t2 = t3;
            s = 1.0;
        }
        if (t1 > tmin) {
            output.normal.x = s;
            output.normal.y = 0;
            tmin = t1;
        }
        tmax = Math.min(tmax, t2);
        if (tmin > tmax) return false;
    }

    var dY = input.p2.y - input.p1.y;
    var absDY = Math.abs(dY);
    if (absDY < Number.MIN_VALUE) {
        if (input.p1.y < this.lowerBound.y || this.upperBound.y < input.p1.y) {
            return false;
        }
    } else {
        var inv_d = 1.0 / dY;
        var t1 = (this.lowerBound.y - input.p1.y) * inv_d;
        var t2 = (this.upperBound.y - input.p1.y) * inv_d;
        var s = (-1.0);
        if (t1 > t2) {
            var t3 = t1;
            t1 = t2;
            t2 = t3;
            s = 1.0;
        }
        if (t1 > tmin) {
            output.normal.y = s;
            output.normal.x = 0;
            tmin = t1;
        }
        tmax = Math.min(tmax, t2);
        if (tmin > tmax) {
            return false;
        }
    }
    output.fraction = tmin;
    return true;
};

/**
 * @param {!box2d.collision.AABB} other
 * @return {boolean}
 */
box2d.collision.AABB.prototype.testOverlap = function(other) {
    if (other.lowerBound.x - this.upperBound.x > 0) {
        return false;
    }
    if (other.lowerBound.y - this.upperBound.y > 0) {
        return false;
    }
    if (this.lowerBound.x - other.upperBound.x > 0) {
        return false;
    }
    return this.lowerBound.y - other.upperBound.y <= 0;
};

/**
 * @param {!box2d.collision.AABB} aabb1
 * @param {!box2d.collision.AABB} aabb2
 * @return {!box2d.collision.AABB}
 */
box2d.collision.AABB.combine = function(aabb1, aabb2) {
    var aabb = box2d.collision.AABB.get();
    aabb.combine(aabb1, aabb2);
    return aabb;
};

/**
 * @param {!box2d.collision.AABB} aabb1
 * @param {!box2d.collision.AABB} aabb2
 */
box2d.collision.AABB.prototype.combine = function(aabb1, aabb2) {
    this.lowerBound.x = Math.min(aabb1.lowerBound.x, aabb2.lowerBound.x);
    this.lowerBound.y = Math.min(aabb1.lowerBound.y, aabb2.lowerBound.y);
    this.upperBound.x = Math.max(aabb1.upperBound.x, aabb2.upperBound.x);
    this.upperBound.y = Math.max(aabb1.upperBound.y, aabb2.upperBound.y);
};
