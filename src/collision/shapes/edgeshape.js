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

goog.provide('box2d.collision.shapes.EdgeShape');

goog.require('UsageTracker');
goog.require('box2d.collision.shapes.Shape');
goog.require('box2d.common.Settings');
goog.require('box2d.common.math.Math');
goog.require('box2d.common.math.Vec2');

/**
 * @param {!box2d.common.math.Vec2} v1
 * @param {!box2d.common.math.Vec2} v2
 * @constructor
 * @extends {box2d.collision.shapes.Shape}
 */
box2d.collision.shapes.EdgeShape = function(v1, v2) {
    UsageTracker.get('box2d.collision.shapes.EdgeShape').trackCreate();
    goog.base(this);

    /**
     * @type {box2d.collision.shapes.EdgeShape}
     */
    this.m_prevEdge = null;

    /**
     * @type {box2d.collision.shapes.EdgeShape}
     */
    this.m_nextEdge = null;

    /**
     * @type {!box2d.common.math.Vec2}
     */
    this.m_v1 = v1;

    /**
     * @type {!box2d.common.math.Vec2}
     */
    this.m_v2 = v2;

    /**
     * @type {!box2d.common.math.Vec2}
     */
    this.m_direction = box2d.common.math.Vec2.get(this.m_v2.x - this.m_v1.x, this.m_v2.y - this.m_v1.y);

    /**
     * @type {number}
     */
    this.m_length = this.m_direction.normalize();

    /**
     * @type {!box2d.common.math.Vec2}
     */
    this.m_normal = box2d.common.math.Vec2.get(this.m_direction.y, -this.m_direction.x);

    /**
     * @type {!box2d.common.math.Vec2}
     */
    this.m_coreV1 = box2d.common.math.Vec2.get((-box2d.common.Settings.toiSlop * (this.m_normal.x - this.m_direction.x)) + this.m_v1.x, (-box2d.common.Settings.toiSlop * (this.m_normal.y - this.m_direction.y)) + this.m_v1.y);

    /**
     * @type {!box2d.common.math.Vec2}
     */
    this.m_coreV2 = box2d.common.math.Vec2.get((-box2d.common.Settings.toiSlop * (this.m_normal.x + this.m_direction.x)) + this.m_v2.x, (-box2d.common.Settings.toiSlop * (this.m_normal.y + this.m_direction.y)) + this.m_v2.y);

    /**
     * @type {!box2d.common.math.Vec2}
     */
    this.m_cornerDir1 = this.m_normal;

    /**
     * @type {!box2d.common.math.Vec2}
     */
    this.m_cornerDir2 = box2d.common.math.Vec2.get(-this.m_normal.x, -this.m_normal.y);

    /**
     * @type {boolean}
     */
    this.m_cornerConvex1 = false;

    /**
     * @type {boolean}
     */
    this.m_cornerConvex2 = false;
};
goog.inherits(box2d.collision.shapes.EdgeShape, box2d.collision.shapes.Shape);

/**
 * @return {string}
 */
box2d.collision.shapes.EdgeShape.prototype.getTypeName = function() {
    return box2d.collision.shapes.EdgeShape.NAME;
};

/**
 * @param {!box2d.common.math.Transform} transform
 * @param {!box2d.common.math.Vec2} p
 * @return {boolean}
 */
box2d.collision.shapes.EdgeShape.prototype.testPoint = function(transform, p) {
    return false;
};

/**
 * @param {!box2d.collision.RayCastOutput} output
 * @param {!box2d.collision.RayCastInput} input
 * @param {!box2d.common.math.Transform} transform
 * @return {boolean}
 */
box2d.collision.shapes.EdgeShape.prototype.rayCast = function(output, input, transform) {
    var rX = input.p2.x - input.p1.x;
    var rY = input.p2.y - input.p1.y;
    var tMat = transform.R;
    var v1X = transform.position.x + (tMat.col1.x * this.m_v1.x + tMat.col2.x * this.m_v1.y);
    var v1Y = transform.position.y + (tMat.col1.y * this.m_v1.x + tMat.col2.y * this.m_v1.y);
    var nX = transform.position.y + (tMat.col1.y * this.m_v2.x + tMat.col2.y * this.m_v2.y) - v1Y;
    var nY = (-(transform.position.x + (tMat.col1.x * this.m_v2.x + tMat.col2.x * this.m_v2.y) - v1X));
    var k_slop = 100.0 * Number.MIN_VALUE;
    var denom = (-(rX * nX + rY * nY));
    if (denom > k_slop) {
        var bX = input.p1.x - v1X;
        var bY = input.p1.y - v1Y;
        var a = (bX * nX + bY * nY);
        if (0.0 <= a && a <= input.maxFraction * denom) {
            var mu2 = (-rX * bY) + rY * bX;
            if ((-k_slop * denom) <= mu2 && mu2 <= denom * (1.0 + k_slop)) {
                a /= denom;
                output.fraction = a;
                var nLen = Math.sqrt(nX * nX + nY * nY);
                output.normal.x = nX / nLen;
                output.normal.y = nY / nLen;
                return true;
            }
        }
    }
    return false;
};

/**
 * @param {!box2d.collision.AABB} aabb
 * @param {!box2d.common.math.Transform} transform
 */
box2d.collision.shapes.EdgeShape.prototype.computeAABB = function(aabb, transform) {
    var tMat = transform.R;
    var v1X = transform.position.x + (tMat.col1.x * this.m_v1.x + tMat.col2.x * this.m_v1.y);
    var v1Y = transform.position.y + (tMat.col1.y * this.m_v1.x + tMat.col2.y * this.m_v1.y);
    var v2X = transform.position.x + (tMat.col1.x * this.m_v2.x + tMat.col2.x * this.m_v2.y);
    var v2Y = transform.position.y + (tMat.col1.y * this.m_v2.x + tMat.col2.y * this.m_v2.y);
    if (v1X < v2X) {
        aabb.lowerBound.x = v1X;
        aabb.upperBound.x = v2X;
    } else {
        aabb.lowerBound.x = v2X;
        aabb.upperBound.x = v1X;
    }
    if (v1Y < v2Y) {
        aabb.lowerBound.y = v1Y;
        aabb.upperBound.y = v2Y;
    } else {
        aabb.lowerBound.y = v2Y;
        aabb.upperBound.y = v1Y;
    }
};

/**
 * @param {!box2d.collision.shapes.MassData} massData
 * @param {number} density
 */
box2d.collision.shapes.EdgeShape.prototype.computeMass = function(massData, density) {
    massData.mass = 0;
    massData.center.setV(this.m_v1);
    massData.I = 0;
};

/**
 * @param {!box2d.common.math.Vec2} normal
 * @param {number} offset
 * @param {!box2d.common.math.Transform} xf
 * @param {!box2d.common.math.Vec2} c
 * @return {number}
 */
box2d.collision.shapes.EdgeShape.prototype.computeSubmergedArea = function(normal, offset, xf, c) {
    if (offset === undefined) offset = 0;
    var v0 = box2d.common.math.Vec2.get(normal.x * offset, normal.y * offset);
    var v1 = box2d.common.math.Math.mulX(xf, this.m_v1);
    var v2 = box2d.common.math.Math.mulX(xf, this.m_v2);
    var d1 = box2d.common.math.Math.Dot(normal, v1) - offset;
    var d2 = box2d.common.math.Math.Dot(normal, v2) - offset;
    if (d1 > 0) {
        if (d2 > 0) {
            return 0;
        } else {
            v1.x = (-d2 / (d1 - d2) * v1.x) + d1 / (d1 - d2) * v2.x;
            v1.y = (-d2 / (d1 - d2) * v1.y) + d1 / (d1 - d2) * v2.y;
        }
    } else {
        if (d2 > 0) {
            v2.x = (-d2 / (d1 - d2) * v1.x) + d1 / (d1 - d2) * v2.x;
            v2.y = (-d2 / (d1 - d2) * v1.y) + d1 / (d1 - d2) * v2.y;
        }
    }
    c.x = (v0.x + v1.x + v2.x) / 3;
    c.y = (v0.y + v1.y + v2.y) / 3;
    return 0.5 * ((v1.x - v0.x) * (v2.y - v0.y) - (v1.y - v0.y) * (v2.x - v0.x));
};

/**
 * @return {number}
 */
box2d.collision.shapes.EdgeShape.prototype.getLength = function() {
    return this.m_length;
};

/**
 * @return {!box2d.common.math.Vec2}
 */
box2d.collision.shapes.EdgeShape.prototype.getVertex1 = function() {
    return this.m_v1;
};

/**
 * @return {!box2d.common.math.Vec2}
 */
box2d.collision.shapes.EdgeShape.prototype.getVertex2 = function() {
    return this.m_v2;
};

/**
 * @return {!box2d.common.math.Vec2}
 */
box2d.collision.shapes.EdgeShape.prototype.getCoreVertex1 = function() {
    return this.m_coreV1;
};

/**
 * @return {!box2d.common.math.Vec2}
 */
box2d.collision.shapes.EdgeShape.prototype.getCoreVertex2 = function() {
    return this.m_coreV2;
};

/**
 * @return {!box2d.common.math.Vec2}
 */
box2d.collision.shapes.EdgeShape.prototype.getNormalVector = function() {
    return this.m_normal;
};

/**
 * @return {!box2d.common.math.Vec2}
 */
box2d.collision.shapes.EdgeShape.prototype.getDirectionVector = function() {
    return this.m_direction;
};

/**
 * @return {!box2d.common.math.Vec2}
 */
box2d.collision.shapes.EdgeShape.prototype.getCorner1Vector = function() {
    return this.m_cornerDir1;
};

/**
 * @return {!box2d.common.math.Vec2}
 */
box2d.collision.shapes.EdgeShape.prototype.getCorner2Vector = function() {
    return this.m_cornerDir2;
};

/**
 * @return {boolean}
 */
box2d.collision.shapes.EdgeShape.prototype.Corner1IsConvex = function() {
    return this.m_cornerConvex1;
};

/**
 * @return {boolean}
 */
box2d.collision.shapes.EdgeShape.prototype.Corner2IsConvex = function() {
    return this.m_cornerConvex2;
};

/**
 * @param {!box2d.common.math.Transform} xf
 * @return {!box2d.common.math.Vec2}
 */
box2d.collision.shapes.EdgeShape.prototype.getFirstVertex = function(xf) {
    var tMat = xf.R;
    return box2d.common.math.Vec2.get(xf.position.x + (tMat.col1.x * this.m_coreV1.x + tMat.col2.x * this.m_coreV1.y), xf.position.y + (tMat.col1.y * this.m_coreV1.x + tMat.col2.y * this.m_coreV1.y));
};

/**
 * @return {box2d.collision.shapes.EdgeShape}
 */
box2d.collision.shapes.EdgeShape.prototype.getNextEdge = function() {
    return this.m_nextEdge;
};

/**
 * @return {box2d.collision.shapes.EdgeShape}
 */
box2d.collision.shapes.EdgeShape.prototype.getPrevEdge = function() {
    return this.m_prevEdge;
};

/**
 * @param {!box2d.common.math.Transform} xf
 * @param {number} dX
 * @param {number} dY
 * @return {!box2d.common.math.Vec2}
 */
box2d.collision.shapes.EdgeShape.prototype.Support = function(xf, dX, dY) {
    var tMat = xf.R;
    var v1X = xf.position.x + (tMat.col1.x * this.m_coreV1.x + tMat.col2.x * this.m_coreV1.y);
    var v1Y = xf.position.y + (tMat.col1.y * this.m_coreV1.x + tMat.col2.y * this.m_coreV1.y);
    var v2X = xf.position.x + (tMat.col1.x * this.m_coreV2.x + tMat.col2.x * this.m_coreV2.y);
    var v2Y = xf.position.y + (tMat.col1.y * this.m_coreV2.x + tMat.col2.y * this.m_coreV2.y);
    if ((v1X * dX + v1Y * dY) > (v2X * dX + v2Y * dY)) {
        return box2d.common.math.Vec2.get(v1X, v1Y);
    } else {
        return box2d.common.math.Vec2.get(v2X, v2Y);
    }
};

/**
 * @param {box2d.collision.shapes.EdgeShape} edge
 * @param {!box2d.common.math.Vec2} core
 * @param {!box2d.common.math.Vec2} cornerDir
 * @param {boolean} convex
 */
box2d.collision.shapes.EdgeShape.prototype.setPrevEdge = function(edge, core, cornerDir, convex) {
    this.m_prevEdge = edge;
    this.m_coreV1 = core;
    this.m_cornerDir1 = cornerDir;
    this.m_cornerConvex1 = convex;
};

/**
 * @param {box2d.collision.shapes.EdgeShape} edge
 * @param {!box2d.common.math.Vec2} core
 * @param {!box2d.common.math.Vec2} cornerDir
 * @param {boolean} convex
 */
box2d.collision.shapes.EdgeShape.prototype.setNextEdge = function(edge, core, cornerDir, convex) {
    this.m_nextEdge = edge;
    this.m_coreV2 = core;
    this.m_cornerDir2 = cornerDir;
    this.m_cornerConvex2 = convex;
};

/**
 * @const
 * @type {string}
 */
box2d.collision.shapes.EdgeShape.NAME = 'EdgeShape';
