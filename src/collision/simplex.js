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

goog.provide('box2d.collision.Simplex');

goog.require('UsageTracker');
goog.require('box2d.collision.SimplexVertex');
goog.require('box2d.common.Settings');
goog.require('box2d.common.math.Math');
goog.require('box2d.common.math.Vec2');

/**
 * @constructor
 * @private
 */
box2d.collision.Simplex = function() {
    UsageTracker.get('box2d.collision.Simplex').trackCreate();
    this.m_v1 = new box2d.collision.SimplexVertex();
    this.m_v2 = new box2d.collision.SimplexVertex();
    this.m_v3 = new box2d.collision.SimplexVertex();
    this.m_vertices = [this.m_v1, this.m_v2, this.m_v3];
};

/**
 * @private
 * @type {Array.<!box2d.collision.Simplex>}
 */
box2d.collision.Simplex.freeCache_ = [];

/**
 * @return {!box2d.collision.Simplex}
 */
box2d.collision.Simplex.get = function() {
    if (box2d.collision.Simplex.freeCache_.length > 0) {
        var simplex = box2d.collision.Simplex.freeCache_.pop();
        for (var i = 0; i < simplex.m_vertices.length; i++) {
            var v = simplex.m_vertices[i];
            if (v.wA != null) {
                v.wA.set(0, 0);
            }
            if (v.wB != null) {
                v.wB.set(0, 0);
            }
            if (v.w != null) {
                v.w.set(0, 0);
            }
            v.indexA = 0;
            v.indexB = 0;
            v.a = 0;
        }
        return simplex;
    }
    return new box2d.collision.Simplex();
};

/**
 * @param {!box2d.collision.Simplex} simplex
 */
box2d.collision.Simplex.free = function(simplex) {
    if (simplex != null) {
        box2d.collision.Simplex.freeCache_.push(simplex);
    }
};

box2d.collision.Simplex.prototype.readCache = function(cache, proxyA, transformA, proxyB, transformB) {
    box2d.common.Settings.assert(0 <= cache.count && cache.count <= 3);
    var wALocal;
    var wBLocal;
    this.m_count = cache.count;
    var vertices = this.m_vertices;
    for (var i = 0; i < this.m_count; i++) {
        var v = vertices[i];
        v.indexA = cache.indexA[i];
        v.indexB = cache.indexB[i];
        wALocal = proxyA.getVertex(v.indexA);
        wBLocal = proxyB.getVertex(v.indexB);
        box2d.common.math.Vec2.free(v.wA);
        box2d.common.math.Vec2.free(v.wB);
        box2d.common.math.Vec2.free(v.w);
        v.wA = box2d.common.math.Math.mulX(transformA, wALocal);
        v.wB = box2d.common.math.Math.mulX(transformB, wBLocal);
        v.w = box2d.common.math.Math.subtractVV(v.wB, v.wA);
        v.a = 0;
    }
    if (this.m_count > 1) {
        var metric1 = cache.metric;
        var metric2 = this.getMetric();
        if (metric2 < .5 * metric1 || 2.0 * metric1 < metric2 || metric2 < Number.MIN_VALUE) {
            this.m_count = 0;
        }
    }
    if (this.m_count == 0) {
        v = vertices[0];
        v.indexA = 0;
        v.indexB = 0;
        wALocal = proxyA.getVertex(0);
        wBLocal = proxyB.getVertex(0);
        box2d.common.math.Vec2.free(v.wA);
        box2d.common.math.Vec2.free(v.wB);
        box2d.common.math.Vec2.free(v.w);
        v.wA = box2d.common.math.Math.mulX(transformA, wALocal);
        v.wB = box2d.common.math.Math.mulX(transformB, wBLocal);
        v.w = box2d.common.math.Math.subtractVV(v.wB, v.wA);
        this.m_count = 1;
    }
};

box2d.collision.Simplex.prototype.writeCache = function(cache) {
    cache.metric = this.getMetric();
    cache.count = this.m_count;
    var vertices = this.m_vertices;
    for (var i = 0; i < this.m_count; i++) {
        cache.indexA[i] = vertices[i].indexA;
        cache.indexB[i] = vertices[i].indexB;
    }
};

box2d.collision.Simplex.prototype.getSearchDirection = function() {
    if (this.m_count == 1) {
        return this.m_v1.w.getNegative();
    } else if (this.m_count == 2) {
        var e12 = box2d.common.math.Math.subtractVV(this.m_v2.w, this.m_v1.w);
        var neg = this.m_v1.w.getNegative();
        var sgn = box2d.common.math.Math.crossVV(e12, neg);
        box2d.common.math.Vec2.free(neg);
        var ret = null;
        if (sgn > 0.0) {
            ret = box2d.common.math.Math.crossFV(1.0, e12);
        } else {
            ret = box2d.common.math.Math.crossVF(e12, 1.0);
        }
        box2d.common.math.Vec2.free(e12);
        return ret;
    } else {
        box2d.common.Settings.assert(false);
        return box2d.common.math.Vec2.get(0, 0);
    }
};

box2d.collision.Simplex.prototype.getClosestPoint = function() {
    if (this.m_count == 1) {
        return this.m_v1.w;
    } else if (this.m_count == 2) {
        return box2d.common.math.Vec2.get(this.m_v1.a * this.m_v1.w.x + this.m_v2.a * this.m_v2.w.x, this.m_v1.a * this.m_v1.w.y + this.m_v2.a * this.m_v2.w.y);
    } else {
        box2d.common.Settings.assert(false);
        return box2d.common.math.Vec2.get(0, 0);
    }
};

box2d.collision.Simplex.prototype.getWitnessPoints = function(pA, pB) {
    if (this.m_count == 1) {
        pA.setV(this.m_v1.wA);
        pB.setV(this.m_v1.wB);
    } else if (this.m_count == 2) {
        pA.x = this.m_v1.a * this.m_v1.wA.x + this.m_v2.a * this.m_v2.wA.x;
        pA.y = this.m_v1.a * this.m_v1.wA.y + this.m_v2.a * this.m_v2.wA.y;
        pB.x = this.m_v1.a * this.m_v1.wB.x + this.m_v2.a * this.m_v2.wB.x;
        pB.y = this.m_v1.a * this.m_v1.wB.y + this.m_v2.a * this.m_v2.wB.y;
    } else if (this.m_count == 3) {
        pB.x = pA.x = this.m_v1.a * this.m_v1.wA.x + this.m_v2.a * this.m_v2.wA.x + this.m_v3.a * this.m_v3.wA.x;
        pB.y = pA.y = this.m_v1.a * this.m_v1.wA.y + this.m_v2.a * this.m_v2.wA.y + this.m_v3.a * this.m_v3.wA.y;
    } else {
        box2d.common.Settings.assert(false);
    }
};

box2d.collision.Simplex.prototype.getMetric = function() {
    if (this.m_count == 1) {
        return 0.0;
    } else if (this.m_count == 2) {
        var v = box2d.common.math.Math.subtractVV(this.m_v1.w, this.m_v2.w);
        var metric = v.length();
        box2d.common.math.Vec2.free(v);
        return metric;
    } else if (this.m_count == 3) {
        var vA = box2d.common.math.Math.subtractVV(this.m_v2.w, this.m_v1.w);
        var vB = box2d.common.math.Math.subtractVV(this.m_v3.w, this.m_v1.w);
        var metric = box2d.common.math.Math.crossVV(vA, vB);
        box2d.common.math.Vec2.free(vA);
        box2d.common.math.Vec2.free(vB);
        return metric;
    } else {
        box2d.common.Settings.assert(false);
        return 0.0;
    }
};

box2d.collision.Simplex.prototype.solve2 = function() {
    var w1 = this.m_v1.w;
    var w2 = this.m_v2.w;
    var e12 = box2d.common.math.Math.subtractVV(w2, w1);
    var d12_2 = (-(w1.x * e12.x + w1.y * e12.y));
    if (d12_2 <= 0.0) {
        box2d.common.math.Vec2.free(e12);
        this.m_v1.a = 1.0;
        this.m_count = 1;
        return;
    }
    var d12_1 = (w2.x * e12.x + w2.y * e12.y);
    box2d.common.math.Vec2.free(e12);
    if (d12_1 <= 0.0) {
        this.m_v2.a = 1.0;
        this.m_count = 1;
        this.m_v1.set(this.m_v2);
        return;
    }
    var inv_d12 = 1.0 / (d12_1 + d12_2);
    this.m_v1.a = d12_1 * inv_d12;
    this.m_v2.a = d12_2 * inv_d12;
    this.m_count = 2;
};

box2d.collision.Simplex.prototype.solve3 = function() {
    var w1 = this.m_v1.w;
    var w2 = this.m_v2.w;
    var w3 = this.m_v3.w;

    var e12 = box2d.common.math.Math.subtractVV(w2, w1);
    var w1e12 = box2d.common.math.Math.Dot(w1, e12);
    var w2e12 = box2d.common.math.Math.Dot(w2, e12);
    var d12_1 = w2e12;
    var d12_2 = (-w1e12);

    var e13 = box2d.common.math.Math.subtractVV(w3, w1);
    var w1e13 = box2d.common.math.Math.Dot(w1, e13);
    var w3e13 = box2d.common.math.Math.Dot(w3, e13);

    var n123 = box2d.common.math.Math.crossVV(e12, e13);
    box2d.common.math.Vec2.free(e12);
    box2d.common.math.Vec2.free(e13);

    var d13_1 = w3e13;
    var d13_2 = (-w1e13);

    var e23 = box2d.common.math.Math.subtractVV(w3, w2);
    var w2e23 = box2d.common.math.Math.Dot(w2, e23);
    var w3e23 = box2d.common.math.Math.Dot(w3, e23);
    box2d.common.math.Vec2.free(e23);
    var d23_1 = w3e23;
    var d23_2 = (-w2e23);
    var d123_1 = n123 * box2d.common.math.Math.crossVV(w2, w3);
    var d123_2 = n123 * box2d.common.math.Math.crossVV(w3, w1);
    var d123_3 = n123 * box2d.common.math.Math.crossVV(w1, w2);
    if (d12_2 <= 0.0 && d13_2 <= 0.0) {
        this.m_v1.a = 1.0;
        this.m_count = 1;
        return;
    }
    if (d12_1 > 0.0 && d12_2 > 0.0 && d123_3 <= 0.0) {
        var inv_d12 = 1.0 / (d12_1 + d12_2);
        this.m_v1.a = d12_1 * inv_d12;
        this.m_v2.a = d12_2 * inv_d12;
        this.m_count = 2;
        return;
    }
    if (d13_1 > 0.0 && d13_2 > 0.0 && d123_2 <= 0.0) {
        var inv_d13 = 1.0 / (d13_1 + d13_2);
        this.m_v1.a = d13_1 * inv_d13;
        this.m_v3.a = d13_2 * inv_d13;
        this.m_count = 2;
        this.m_v2.set(this.m_v3);
        return;
    }
    if (d12_1 <= 0.0 && d23_2 <= 0.0) {
        this.m_v2.a = 1.0;
        this.m_count = 1;
        this.m_v1.set(this.m_v2);
        return;
    }
    if (d13_1 <= 0.0 && d23_1 <= 0.0) {
        this.m_v3.a = 1.0;
        this.m_count = 1;
        this.m_v1.set(this.m_v3);
        return;
    }
    if (d23_1 > 0.0 && d23_2 > 0.0 && d123_1 <= 0.0) {
        var inv_d23 = 1.0 / (d23_1 + d23_2);
        this.m_v2.a = d23_1 * inv_d23;
        this.m_v3.a = d23_2 * inv_d23;
        this.m_count = 2;
        this.m_v1.set(this.m_v3);
        return;
    }
    var inv_d123 = 1.0 / (d123_1 + d123_2 + d123_3);
    this.m_v1.a = d123_1 * inv_d123;
    this.m_v2.a = d123_2 * inv_d123;
    this.m_v3.a = d123_3 * inv_d123;
    this.m_count = 3;
};
