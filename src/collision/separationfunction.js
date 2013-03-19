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

goog.provide('box2d.collision.SeparationFunction');

goog.require('UsageTracker');
goog.require('box2d.common.Settings');
goog.require('box2d.common.math.Math');
goog.require('box2d.common.math.Vec2');

/**
 * @constructor
 */
box2d.collision.SeparationFunction = function() {
    UsageTracker.get('box2d.collision.SeparationFunction').trackCreate();

    /**
     * @type {!box2d.common.math.Vec2}
     */
    this.m_localPoint = box2d.common.math.Vec2.get(0, 0);

    /**
     * @type {!box2d.common.math.Vec2}
     */
    this.m_axis = box2d.common.math.Vec2.get(0, 0);

    /**
     * @type {box2d.collision.DistanceProxy}
     */
    this.m_proxyA = null;

    /**
     * @type {box2d.collision.DistanceProxy}
     */
    this.m_proxyB = null;
};

/**
 * @param {!box2d.collision.SimplexCache} cache
 * @param {!box2d.collision.DistanceProxy} proxyA
 * @param {!box2d.common.math.Transform} transformA
 * @param {!box2d.collision.DistanceProxy} proxyB
 * @param {!box2d.common.math.Transform} transformB
 */
box2d.collision.SeparationFunction.prototype.initialize = function(cache, proxyA, transformA, proxyB, transformB) {
    this.m_proxyA = proxyA;
    this.m_proxyB = proxyB;
    var count = cache.count;
    box2d.common.Settings.assert(0 < count && count < 3);
    var localPointA;
    var localPointA1;
    var localPointA2;
    var localPointB;
    var localPointB1;
    var localPointB2;
    var pointAX = 0;
    var pointAY = 0;
    var pointBX = 0;
    var pointBY = 0;
    var normalX = 0;
    var normalY = 0;
    var tMat;
    var tVec;
    var s = 0;
    var sgn = 0;
    if (count == 1) {
        this.m_type = box2d.collision.SeparationFunction.e_points;
        localPointA = this.m_proxyA.getVertex(cache.indexA[0]);
        localPointB = this.m_proxyB.getVertex(cache.indexB[0]);
        tVec = localPointA;
        tMat = transformA.R;
        pointAX = transformA.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
        pointAY = transformA.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
        tVec = localPointB;
        tMat = transformB.R;
        pointBX = transformB.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
        pointBY = transformB.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
        this.m_axis.x = pointBX - pointAX;
        this.m_axis.y = pointBY - pointAY;
        this.m_axis.normalize();
    } else if (cache.indexB[0] == cache.indexB[1]) {
        this.m_type = box2d.collision.SeparationFunction.e_faceA;
        localPointA1 = this.m_proxyA.getVertex(cache.indexA[0]);
        localPointA2 = this.m_proxyA.getVertex(cache.indexA[1]);
        localPointB = this.m_proxyB.getVertex(cache.indexB[0]);
        this.m_localPoint.x = 0.5 * (localPointA1.x + localPointA2.x);
        this.m_localPoint.y = 0.5 * (localPointA1.y + localPointA2.y);
        var tempVec = box2d.common.math.Math.subtractVV(localPointA2, localPointA1);
        box2d.common.math.Vec2.free(this.m_axis);
        this.m_axis = box2d.common.math.Math.crossVF(tempVec, 1.0);
        box2d.common.math.Vec2.free(tempVec);
        this.m_axis.normalize();
        tVec = this.m_axis;
        tMat = transformA.R;
        normalX = tMat.col1.x * tVec.x + tMat.col2.x * tVec.y;
        normalY = tMat.col1.y * tVec.x + tMat.col2.y * tVec.y;
        tVec = this.m_localPoint;
        tMat = transformA.R;
        pointAX = transformA.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
        pointAY = transformA.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
        tVec = localPointB;
        tMat = transformB.R;
        pointBX = transformB.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
        pointBY = transformB.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
        s = (pointBX - pointAX) * normalX + (pointBY - pointAY) * normalY;
        if (s < 0.0) {
            this.m_axis.negativeSelf();
        }
    } else if (cache.indexA[0] == cache.indexA[0]) {
        this.m_type = box2d.collision.SeparationFunction.e_faceB;
        localPointB1 = this.m_proxyB.getVertex(cache.indexB[0]);
        localPointB2 = this.m_proxyB.getVertex(cache.indexB[1]);
        localPointA = this.m_proxyA.getVertex(cache.indexA[0]);
        this.m_localPoint.x = 0.5 * (localPointB1.x + localPointB2.x);
        this.m_localPoint.y = 0.5 * (localPointB1.y + localPointB2.y);
        var tempVec = box2d.common.math.Math.subtractVV(localPointB2, localPointB1);
        box2d.common.math.Vec2.free(this.m_axis);
        this.m_axis = box2d.common.math.Math.crossVF(tempVec, 1.0);
        box2d.common.math.Vec2.free(tempVec);
        this.m_axis.normalize();
        tVec = this.m_axis;
        tMat = transformB.R;
        normalX = tMat.col1.x * tVec.x + tMat.col2.x * tVec.y;
        normalY = tMat.col1.y * tVec.x + tMat.col2.y * tVec.y;
        tVec = this.m_localPoint;
        tMat = transformB.R;
        pointBX = transformB.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
        pointBY = transformB.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
        tVec = localPointA;
        tMat = transformA.R;
        pointAX = transformA.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
        pointAY = transformA.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
        s = (pointAX - pointBX) * normalX + (pointAY - pointBY) * normalY;
        if (s < 0.0) {
            this.m_axis.negativeSelf();
        }
    } else {
        localPointA1 = this.m_proxyA.getVertex(cache.indexA[0]);
        localPointA2 = this.m_proxyA.getVertex(cache.indexA[1]);
        localPointB1 = this.m_proxyB.getVertex(cache.indexB[0]);
        localPointB2 = this.m_proxyB.getVertex(cache.indexB[1]);
        var tempVec = box2d.common.math.Math.subtractVV(localPointA2, localPointA1);
        var dA = box2d.common.math.Math.mulMV(transformA.R, tempVec);
        box2d.common.math.Vec2.free(tempVec);
        tempVec = box2d.common.math.Math.subtractVV(localPointB2, localPointB1);
        var dB = box2d.common.math.Math.mulMV(transformB.R, tempVec);
        box2d.common.math.Vec2.free(tempVec);
        var a = dA.x * dA.x + dA.y * dA.y;
        var e = dB.x * dB.x + dB.y * dB.y;
        var r = box2d.common.math.Math.subtractVV(dB, dA);
        var c = dA.x * r.x + dA.y * r.y;
        var f = dB.x * r.x + dB.y * r.y;
        box2d.common.math.Vec2.free(r);
        var b = dA.x * dB.x + dA.y * dB.y;
        var denom = a * e - b * b;
        s = 0.0;
        if (denom != 0.0) {
            s = box2d.common.math.Math.clamp((b * f - c * e) / denom, 0.0, 1.0);
        }
        var t = (b * s + f) / e;
        if (t < 0.0) {
            t = 0.0;
            s = box2d.common.math.Math.clamp((b - c) / a, 0.0, 1.0);
        }
        localPointA = box2d.common.math.Vec2.get(0, 0);
        localPointA.x = localPointA1.x + s * (localPointA2.x - localPointA1.x);
        localPointA.y = localPointA1.y + s * (localPointA2.y - localPointA1.y);
        localPointB = box2d.common.math.Vec2.get(0, 0);
        localPointB.x = localPointB1.x + s * (localPointB2.x - localPointB1.x);
        localPointB.y = localPointB1.y + s * (localPointB2.y - localPointB1.y);
        if (s == 0.0 || s == 1.0) {
            this.m_type = box2d.collision.SeparationFunction.e_faceB;
            tempVec = box2d.common.math.Math.subtractVV(localPointB2, localPointB1);
            box2d.common.math.Vec2.free(this.m_axis);
            this.m_axis = box2d.common.math.Math.crossVF(tempVec, 1.0);
            box2d.common.math.Vec2.free(tempVec);
            this.m_axis.normalize();
            this.m_localPoint = localPointB;
            tVec = this.m_axis;
            tMat = transformB.R;
            normalX = tMat.col1.x * tVec.x + tMat.col2.x * tVec.y;
            normalY = tMat.col1.y * tVec.x + tMat.col2.y * tVec.y;
            tVec = this.m_localPoint;
            tMat = transformB.R;
            pointBX = transformB.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
            pointBY = transformB.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
            tVec = localPointA;
            tMat = transformA.R;
            pointAX = transformA.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
            pointAY = transformA.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
            sgn = (pointAX - pointBX) * normalX + (pointAY - pointBY) * normalY;
            if (s < 0.0) {
                this.m_axis.negativeSelf();
            }
        } else {
            this.m_type = box2d.collision.SeparationFunction.e_faceA;
            tempVec = box2d.common.math.Math.subtractVV(localPointA2, localPointA1);
            box2d.common.math.Vec2.free(this.m_axis);
            this.m_axis = box2d.common.math.Math.crossVF(tempVec, 1.0);
            box2d.common.math.Vec2.free(tempVec);
            this.m_localPoint = localPointA;
            tVec = this.m_axis;
            tMat = transformA.R;
            normalX = tMat.col1.x * tVec.x + tMat.col2.x * tVec.y;
            normalY = tMat.col1.y * tVec.x + tMat.col2.y * tVec.y;
            tVec = this.m_localPoint;
            tMat = transformA.R;
            pointAX = transformA.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
            pointAY = transformA.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
            tVec = localPointB;
            tMat = transformB.R;
            pointBX = transformB.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
            pointBY = transformB.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
            sgn = (pointBX - pointAX) * normalX + (pointBY - pointAY) * normalY;
            if (s < 0.0) {
                this.m_axis.negativeSelf();
            }
        }
        box2d.common.math.Vec2.free(localPointA);
        box2d.common.math.Vec2.free(localPointB);
    }
};

/**
 * @param {!box2d.common.math.Transform} transformA
 * @param {!box2d.common.math.Transform} transformB
 * @return {number}
 */
box2d.collision.SeparationFunction.prototype.evaluate = function(transformA, transformB) {
    var seperation = 0;
    switch (this.m_type) {
        case box2d.collision.SeparationFunction.e_points:
            var axisA = box2d.common.math.Math.mulTMV(transformA.R, this.m_axis);
            var negMAxis = this.m_axis.getNegative();
            var axisB = box2d.common.math.Math.mulTMV(transformB.R, negMAxis);
            box2d.common.math.Vec2.free(negMAxis);
            var localPointA = this.m_proxyA.getSupportVertex(axisA);
            box2d.common.math.Vec2.free(axisA);
            var localPointB = this.m_proxyB.getSupportVertex(axisB);
            box2d.common.math.Vec2.free(axisB);
            var pointA = box2d.common.math.Math.mulX(transformA, localPointA);
            var pointB = box2d.common.math.Math.mulX(transformB, localPointB);
            seperation = (pointB.x - pointA.x) * this.m_axis.x + (pointB.y - pointA.y) * this.m_axis.y;
            box2d.common.math.Vec2.free(pointA);
            box2d.common.math.Vec2.free(pointB);
            break;
        case box2d.collision.SeparationFunction.e_faceA:
            var normal = box2d.common.math.Math.mulMV(transformA.R, this.m_axis);
            var negNormal = normal.getNegative();
            var axisB = box2d.common.math.Math.mulTMV(transformB.R, negNormal);
            box2d.common.math.Vec2.free(negNormal);
            var localPointB = this.m_proxyB.getSupportVertex(axisB);
            box2d.common.math.Vec2.free(axisB);
            var pointA = box2d.common.math.Math.mulX(transformA, this.m_localPoint);
            var pointB = box2d.common.math.Math.mulX(transformB, localPointB);
            seperation = (pointB.x - pointA.x) * normal.x + (pointB.y - pointA.y) * normal.y;
            box2d.common.math.Vec2.free(normal);
            box2d.common.math.Vec2.free(pointA);
            box2d.common.math.Vec2.free(pointB);
            break;
        case box2d.collision.SeparationFunction.e_faceB:
            var normal = box2d.common.math.Math.mulMV(transformB.R, this.m_axis);
            var negNormal = normal.getNegative();
            var axisA = box2d.common.math.Math.mulTMV(transformA.R, negNormal);
            box2d.common.math.Vec2.free(negNormal);
            var localPointA = this.m_proxyA.getSupportVertex(axisA);
            box2d.common.math.Vec2.free(axisA);
            var pointA = box2d.common.math.Math.mulX(transformA, localPointA);
            var pointB = box2d.common.math.Math.mulX(transformB, this.m_localPoint);
            seperation = (pointA.x - pointB.x) * normal.x + (pointA.y - pointB.y) * normal.y;
            box2d.common.math.Vec2.free(normal);
            box2d.common.math.Vec2.free(pointA);
            box2d.common.math.Vec2.free(pointB);
            break;
        default:
            box2d.common.Settings.assert(false);
            break;
    }
    return seperation;
};

/**
 * @const
 * @type {number}
 */
box2d.collision.SeparationFunction.e_points = 0x01;

/**
 * @const
 * @type {number}
 */
box2d.collision.SeparationFunction.e_faceA = 0x02;

/**
 * @const
 * @type {number}
 */
box2d.collision.SeparationFunction.e_faceB = 0x04;
