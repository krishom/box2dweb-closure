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

goog.provide('box2d.collision.shapes.PolygonShape');

goog.require('UsageTracker');
goog.require('box2d.collision.shapes.MassData');
goog.require('box2d.collision.shapes.Shape');
goog.require('box2d.common.Settings');
goog.require('box2d.common.math.Mat22');
goog.require('box2d.common.math.Math');
goog.require('box2d.common.math.Transform');
goog.require('box2d.common.math.Vec2');

/**
 * @constructor
 * @extends {box2d.collision.shapes.Shape}
 */
box2d.collision.shapes.PolygonShape = function() {
    UsageTracker.get('box2d.collision.shapes.PolygonShape').trackCreate();
    goog.base(this);

    /**
     * @type {!box2d.common.math.Vec2}
     */
    this.m_centroid = box2d.common.math.Vec2.get(0, 0);

    /**
     * @type {!Array.<!box2d.common.math.Vec2>}
     */
    this.m_vertices = [];

    /**
     * @type {!Array.<!box2d.common.math.Vec2>}
     */
    this.m_normals = [];
};
goog.inherits(box2d.collision.shapes.PolygonShape, box2d.collision.shapes.Shape);

/**
 * @return {string}
 */
box2d.collision.shapes.PolygonShape.prototype.getTypeName = function() {
    return box2d.collision.shapes.PolygonShape.NAME;
};

/**
 * @return {!box2d.collision.shapes.PolygonShape}
 */
box2d.collision.shapes.PolygonShape.prototype.copy = function() {
    var s = new box2d.collision.shapes.PolygonShape();
    s.set(this);
    return s;
};

/**
 * @param {!box2d.collision.shapes.Shape} other
 */
box2d.collision.shapes.PolygonShape.prototype.set = function(other) {
    goog.base(this, 'set', other);
    if (other instanceof box2d.collision.shapes.PolygonShape) {
        this.m_centroid.setV(other.m_centroid);
        this.m_vertexCount = other.m_vertexCount;
        this.Reserve(this.m_vertexCount);
        for (var i = 0; i < this.m_vertexCount; i++) {
            this.m_vertices[i].setV(other.m_vertices[i]);
            this.m_normals[i].setV(other.m_normals[i]);
        }
    }
};

/**
 * @param {Array.<box2d.common.math.Vec2>} vertices
 */
box2d.collision.shapes.PolygonShape.prototype.setAsArray = function(vertices) {
    this.setAsVector(vertices);
};

/**
 * @param {Array.<box2d.common.math.Vec2>} vertices
 * @return {!box2d.collision.shapes.PolygonShape}
 */
box2d.collision.shapes.PolygonShape.AsArray = function(vertices) {
    var polygonShape = new box2d.collision.shapes.PolygonShape();
    polygonShape.setAsArray(vertices);
    return polygonShape;
};

/**
 * @param {Array.<!box2d.common.math.Vec2>} vertices
 */
box2d.collision.shapes.PolygonShape.prototype.setAsVector = function(vertices) {
    var vertexCount = vertices.length;
    box2d.common.Settings.assert(2 <= vertexCount);
    this.m_vertexCount = vertexCount;
    this.Reserve(vertexCount);
    var i = 0;
    for (i = 0; i < this.m_vertexCount; i++) {
        this.m_vertices[i].setV(vertices[i]);
    }
    for (i = 0; i < this.m_vertexCount; ++i) {
        var i1 = i;
        var i2 = i + 1 < this.m_vertexCount ? i + 1 : 0;
        var edge = box2d.common.math.Math.subtractVV(this.m_vertices[i2], this.m_vertices[i1]);
        box2d.common.Settings.assert(edge.lengthSquared() > Number.MIN_VALUE);
        var edgeCross = box2d.common.math.Math.crossVF(edge, 1.0);
        box2d.common.math.Vec2.free(edge);
        this.m_normals[i].setV(edgeCross);
        box2d.common.math.Vec2.free(edgeCross);
        this.m_normals[i].normalize();
    }
    this.m_centroid = box2d.collision.shapes.PolygonShape.computeCentroid(this.m_vertices, this.m_vertexCount);
};

/**
 * @param {Array.<box2d.common.math.Vec2>} vertices
 * @return {!box2d.collision.shapes.PolygonShape}
 */
box2d.collision.shapes.PolygonShape.AsVector = function(vertices) {
    var polygonShape = new box2d.collision.shapes.PolygonShape();
    polygonShape.setAsVector(vertices);
    return polygonShape;
};

/**
 * @param {number} hx
 * @param {number} hy
 */
box2d.collision.shapes.PolygonShape.prototype.setAsBox = function(hx, hy) {
    this.m_vertexCount = 4;
    this.Reserve(4);
    this.m_vertices[0].set((-hx), (-hy));
    this.m_vertices[1].set(hx, (-hy));
    this.m_vertices[2].set(hx, hy);
    this.m_vertices[3].set((-hx), hy);
    this.m_normals[0].set(0.0, (-1.0));
    this.m_normals[1].set(1.0, 0.0);
    this.m_normals[2].set(0.0, 1.0);
    this.m_normals[3].set((-1.0), 0.0);
    this.m_centroid.setZero();
};

/**
 * @param {number} hx
 * @param {number} hy
 * @return {!box2d.collision.shapes.PolygonShape}
 */
box2d.collision.shapes.PolygonShape.AsBox = function(hx, hy) {
    var polygonShape = new box2d.collision.shapes.PolygonShape();
    polygonShape.setAsBox(hx, hy);
    return polygonShape;
};

/**
 * @param {number} hx
 * @param {number} hy
 * @param {!box2d.common.math.Vec2} center
 * @param {number} angle
 */
box2d.collision.shapes.PolygonShape.prototype.setAsOrientedBox = function(hx, hy, center, angle) {
    this.m_vertexCount = 4;
    this.Reserve(4);
    this.m_vertices[0].set((-hx), (-hy));
    this.m_vertices[1].set(hx, (-hy));
    this.m_vertices[2].set(hx, hy);
    this.m_vertices[3].set((-hx), hy);
    this.m_normals[0].set(0.0, (-1.0));
    this.m_normals[1].set(1.0, 0.0);
    this.m_normals[2].set(0.0, 1.0);
    this.m_normals[3].set((-1.0), 0.0);
    this.m_centroid = center;
    var mat = box2d.common.math.Mat22.get();
    mat.set(angle);
    var xf = new box2d.common.math.Transform(center, mat);
    for (var i = 0; i < this.m_vertexCount; ++i) {
        this.m_vertices[i] = box2d.common.math.Math.mulX(xf, this.m_vertices[i]);
        this.m_normals[i] = box2d.common.math.Math.mulMV(xf.R, this.m_normals[i]);
    }
};

/**
 * @param {number} hx
 * @param {number} hy
 * @param {!box2d.common.math.Vec2} center
 * @param {number} angle
 * @return {!box2d.collision.shapes.PolygonShape}
 */
box2d.collision.shapes.PolygonShape.AsOrientedBox = function(hx, hy, center, angle) {
    var polygonShape = new box2d.collision.shapes.PolygonShape();
    polygonShape.setAsOrientedBox(hx, hy, center, angle);
    return polygonShape;
};

/**
 * @param {!box2d.common.math.Vec2} v1
 * @param {!box2d.common.math.Vec2} v2
 */
box2d.collision.shapes.PolygonShape.prototype.setAsEdge = function(v1, v2) {
    this.m_vertexCount = 2;
    this.Reserve(2);
    this.m_vertices[0].setV(v1);
    this.m_vertices[1].setV(v2);
    this.m_centroid.x = 0.5 * (v1.x + v2.x);
    this.m_centroid.y = 0.5 * (v1.y + v2.y);
    var d = box2d.common.math.Math.subtractVV(v2, v1);
    var crossD = box2d.common.math.Math.crossVF(d, 1.0);
    box2d.common.math.Vec2.free(d);
    this.m_normals[0] = crossD;
    this.m_normals[0].normalize();
    this.m_normals[1].x = (-this.m_normals[0].x);
    this.m_normals[1].y = (-this.m_normals[0].y);
};

/**
 * @param {!box2d.common.math.Vec2} v1
 * @param {!box2d.common.math.Vec2} v2
 * @return {!box2d.collision.shapes.PolygonShape}
 */
box2d.collision.shapes.PolygonShape.AsEdge = function(v1, v2) {
    var polygonShape = new box2d.collision.shapes.PolygonShape();
    polygonShape.setAsEdge(v1, v2);
    return polygonShape;
};

/**
 * @param {!box2d.common.math.Transform} xf
 * @param {!box2d.common.math.Vec2} p
 * @return {boolean}
 */
box2d.collision.shapes.PolygonShape.prototype.testPoint = function(xf, p) {
    var tVec;
    var tMat = xf.R;
    var tX = p.x - xf.position.x;
    var tY = p.y - xf.position.y;
    var pLocalX = (tX * tMat.col1.x + tY * tMat.col1.y);
    var pLocalY = (tX * tMat.col2.x + tY * tMat.col2.y);
    for (var i = 0; i < this.m_vertexCount; ++i) {
        tVec = this.m_vertices[i];
        tX = pLocalX - tVec.x;
        tY = pLocalY - tVec.y;
        tVec = this.m_normals[i];
        var dot = (tVec.x * tX + tVec.y * tY);
        if (dot > 0.0) {
            return false;
        }
    }
    return true;
};

/**
 * @param {!box2d.collision.RayCastOutput} output
 * @param {!box2d.collision.RayCastInput} input
 * @param {!box2d.common.math.Transform} transform
 * @return {boolean}
 */
box2d.collision.shapes.PolygonShape.prototype.rayCast = function(output, input, transform) {
    var lower = 0.0;
    var upper = input.maxFraction;
    var tX = 0;
    var tY = 0;
    var tMat;
    var tVec;
    tX = input.p1.x - transform.position.x;
    tY = input.p1.y - transform.position.y;
    tMat = transform.R;
    var p1X = (tX * tMat.col1.x + tY * tMat.col1.y);
    var p1Y = (tX * tMat.col2.x + tY * tMat.col2.y);
    tX = input.p2.x - transform.position.x;
    tY = input.p2.y - transform.position.y;
    tMat = transform.R;
    var p2X = (tX * tMat.col1.x + tY * tMat.col1.y);
    var p2Y = (tX * tMat.col2.x + tY * tMat.col2.y);
    var dX = p2X - p1X;
    var dY = p2Y - p1Y;
    var index = -1;
    for (var i = 0; i < this.m_vertexCount; ++i) {
        tVec = this.m_vertices[i];
        tX = tVec.x - p1X;
        tY = tVec.y - p1Y;
        tVec = this.m_normals[i];
        var numerator = (tVec.x * tX + tVec.y * tY);
        var denominator = (tVec.x * dX + tVec.y * dY);
        if (denominator == 0.0) {
            if (numerator < 0.0) {
                return false;
            }
        } else {
            if (denominator < 0.0 && numerator < lower * denominator) {
                lower = numerator / denominator;
                index = i;
            } else if (denominator > 0.0 && numerator < upper * denominator) {
                upper = numerator / denominator;
            }
        }
        if (upper < lower - Number.MIN_VALUE) {
            return false;
        }
    }
    if (index >= 0) {
        output.fraction = lower;
        tMat = transform.R;
        tVec = this.m_normals[index];
        output.normal.x = (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
        output.normal.y = (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
        return true;
    }
    return false;
};

/**
 * @param {!box2d.collision.AABB} aabb
 * @param {!box2d.common.math.Transform} xf
 */
box2d.collision.shapes.PolygonShape.prototype.computeAABB = function(aabb, xf) {
    var tMat = xf.R;
    var tVec = this.m_vertices[0];
    var lowerX = xf.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
    var lowerY = xf.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
    var upperX = lowerX;
    var upperY = lowerY;
    for (var i = 1; i < this.m_vertexCount; ++i) {
        tVec = this.m_vertices[i];
        var vX = xf.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
        var vY = xf.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
        lowerX = lowerX < vX ? lowerX : vX;
        lowerY = lowerY < vY ? lowerY : vY;
        upperX = upperX > vX ? upperX : vX;
        upperY = upperY > vY ? upperY : vY;
    }
    aabb.lowerBound.x = lowerX - this.m_radius;
    aabb.lowerBound.y = lowerY - this.m_radius;
    aabb.upperBound.x = upperX + this.m_radius;
    aabb.upperBound.y = upperY + this.m_radius;
};

/**
 * @param {!box2d.collision.shapes.MassData} massData
 * @param {number} density
 */
box2d.collision.shapes.PolygonShape.prototype.computeMass = function(massData, density) {
    if (this.m_vertexCount == 2) {
        massData.center.x = 0.5 * (this.m_vertices[0].x + this.m_vertices[1].x);
        massData.center.y = 0.5 * (this.m_vertices[0].y + this.m_vertices[1].y);
        massData.mass = 0.0;
        massData.I = 0.0;
        return;
    }
    var centerX = 0.0;
    var centerY = 0.0;
    var area = 0.0;
    var I = 0.0;
    var p1X = 0.0;
    var p1Y = 0.0;
    var k_inv3 = 1.0 / 3.0;
    for (var i = 0; i < this.m_vertexCount; ++i) {
        var p2 = this.m_vertices[i];
        var p3 = i + 1 < this.m_vertexCount ? this.m_vertices[i + 1] : this.m_vertices[0];
        var e1X = p2.x - p1X;
        var e1Y = p2.y - p1Y;
        var e2X = p3.x - p1X;
        var e2Y = p3.y - p1Y;
        var D = e1X * e2Y - e1Y * e2X;
        var triangleArea = 0.5 * D;
        area += triangleArea;
        centerX += triangleArea * k_inv3 * (p1X + p2.x + p3.x);
        centerY += triangleArea * k_inv3 * (p1Y + p2.y + p3.y);
        var px = p1X;
        var py = p1Y;
        var ex1 = e1X;
        var ey1 = e1Y;
        var ex2 = e2X;
        var ey2 = e2Y;
        var intx2 = k_inv3 * (0.25 * (ex1 * ex1 + ex2 * ex1 + ex2 * ex2) + (px * ex1 + px * ex2)) + 0.5 * px * px;
        var inty2 = k_inv3 * (0.25 * (ey1 * ey1 + ey2 * ey1 + ey2 * ey2) + (py * ey1 + py * ey2)) + 0.5 * py * py;
        I += D * (intx2 + inty2);
    }
    centerX *= 1.0 / area;
    centerY *= 1.0 / area;
    massData.set(density * area, centerX, centerY, density * I);
};

/**
 * @param {!box2d.common.math.Vec2} normal
 * @param {number} offset
 * @param {!box2d.common.math.Transform} xf
 * @param {!box2d.common.math.Vec2} c
 * @return {number}
 */
box2d.collision.shapes.PolygonShape.prototype.computeSubmergedArea = function(normal, offset, xf, c) {
    var normalL = box2d.common.math.Math.mulTMV(xf.R, normal);
    var offsetL = offset - box2d.common.math.Math.Dot(normal, xf.position);
    var depths = [];
    var diveCount = 0;
    var intoIndex = -1;
    var outoIndex = -1;
    var lastSubmerged = false;
    var i = 0;
    for (i = 0; i < this.m_vertexCount; ++i) {
        depths[i] = box2d.common.math.Math.Dot(normalL, this.m_vertices[i]) - offsetL;
        var isSubmerged = depths[i] < (-Number.MIN_VALUE);
        if (i > 0) {
            if (isSubmerged) {
                if (!lastSubmerged) {
                    intoIndex = i - 1;
                    diveCount++;
                }
            } else {
                if (lastSubmerged) {
                    outoIndex = i - 1;
                    diveCount++;
                }
            }
        }
        lastSubmerged = isSubmerged;
    }
    box2d.common.math.Vec2.free(normalL);
    switch (diveCount) {
        case 0:
            if (lastSubmerged) {
                var md = box2d.collision.shapes.MassData.get();
                this.computeMass(md, 1);
                var newV = box2d.common.math.Math.mulX(xf, md.center);
                c.setV(newV);
                box2d.common.math.Vec2.free(newV);
                var mass = md.mass;
                box2d.collision.shapes.MassData.free(md);
                return mass;
            } else {
                return 0;
            }
            break;
        case 1:
            if (intoIndex == (-1)) {
                intoIndex = this.m_vertexCount - 1;
            } else {
                outoIndex = this.m_vertexCount - 1;
            }
            break;
    }
    var intoIndex2 = ((intoIndex + 1) % this.m_vertexCount);
    var outoIndex2 = ((outoIndex + 1) % this.m_vertexCount);
    var intoLamdda = (0 - depths[intoIndex]) / (depths[intoIndex2] - depths[intoIndex]);
    var outoLamdda = (0 - depths[outoIndex]) / (depths[outoIndex2] - depths[outoIndex]);
    var intoVec = box2d.common.math.Vec2.get(this.m_vertices[intoIndex].x * (1 - intoLamdda) + this.m_vertices[intoIndex2].x * intoLamdda,
        this.m_vertices[intoIndex].y * (1 - intoLamdda) + this.m_vertices[intoIndex2].y * intoLamdda);
    var outoVec = box2d.common.math.Vec2.get(this.m_vertices[outoIndex].x * (1 - outoLamdda) + this.m_vertices[outoIndex2].x * outoLamdda,
        this.m_vertices[outoIndex].y * (1 - outoLamdda) + this.m_vertices[outoIndex2].y * outoLamdda);
    var area = 0;
    var center = box2d.common.math.Vec2.get(0, 0);
    var p2 = this.m_vertices[intoIndex2];
    var p3;
    i = intoIndex2;
    while (i != outoIndex2) {
        i = (i + 1) % this.m_vertexCount;
        if (i == outoIndex2) p3 = outoVec;
        else p3 = this.m_vertices[i];
        var triangleArea = 0.5 * ((p2.x - intoVec.x) * (p3.y - intoVec.y) - (p2.y - intoVec.y) * (p3.x - intoVec.x));
        area += triangleArea;
        center.x += triangleArea * (intoVec.x + p2.x + p3.x) / 3;
        center.y += triangleArea * (intoVec.y + p2.y + p3.y) / 3;
        p2 = p3;
    }
    box2d.common.math.Vec2.free(intoVec);
    box2d.common.math.Vec2.free(outoVec);
    center.multiply(1 / area);
    var newV = box2d.common.math.Math.mulX(xf, center);
    box2d.common.math.Vec2.free(center);
    c.setV(newV);
    box2d.common.math.Vec2.free(newV);
    return area;
};

/**
 * @param {!box2d.collision.DistanceProxy} proxy
 */
box2d.collision.shapes.PolygonShape.prototype.setDistanceProxy = function(proxy) {
    proxy.setValues(this.m_vertexCount, this.m_radius, this.m_vertices);
};

/**
 * @return {number}
 */
box2d.collision.shapes.PolygonShape.prototype.getVertexCount = function() {
    return this.m_vertexCount;
};

/**
 * @return {Array.<!box2d.common.math.Vec2>}
 */
box2d.collision.shapes.PolygonShape.prototype.getVertices = function() {
    return this.m_vertices;
};

/**
 * @return {Array.<!box2d.common.math.Vec2>}
 */
box2d.collision.shapes.PolygonShape.prototype.getNormals = function() {
    return this.m_normals;
};

/**
 * @param {!box2d.common.math.Vec2} d
 * @return {number}.
 */
box2d.collision.shapes.PolygonShape.prototype.getSupport = function(d) {
    var bestIndex = 0;
    var bestValue = this.m_vertices[0].x * d.x + this.m_vertices[0].y * d.y;
    for (var i = 1; i < this.m_vertexCount; ++i) {
        var value = this.m_vertices[i].x * d.x + this.m_vertices[i].y * d.y;
        if (value > bestValue) {
            bestIndex = i;
            bestValue = value;
        }
    }
    return bestIndex;
};

/**
 * @param {!box2d.common.math.Vec2} d
 * @return {!box2d.common.math.Vec2}.
 */
box2d.collision.shapes.PolygonShape.prototype.getSupportVertex = function(d) {
    var bestIndex = 0;
    var bestValue = this.m_vertices[0].x * d.x + this.m_vertices[0].y * d.y;
    for (var i = 1; i < this.m_vertexCount; ++i) {
        var value = this.m_vertices[i].x * d.x + this.m_vertices[i].y * d.y;
        if (value > bestValue) {
            bestIndex = i;
            bestValue = value;
        }
    }
    return this.m_vertices[bestIndex];
};

/**
 * @param {number} count
 */
box2d.collision.shapes.PolygonShape.prototype.Reserve = function(count) {
    for (var i = 0; i < this.m_vertices.length; i++) {
        box2d.common.math.Vec2.free(this.m_vertices[i]);
        box2d.common.math.Vec2.free(this.m_normals[i]);
    }
    this.m_vertices = [];
    this.m_normals = [];
    for (var i = 0; i < count; i++) {
        this.m_vertices[i] = box2d.common.math.Vec2.get(0, 0);
        this.m_normals[i] = box2d.common.math.Vec2.get(0, 0);
    }
};

/**
 * @param {Array.<!box2d.common.math.Vec2>} vs
 * @param {number} count
 * @return {!box2d.common.math.Vec2}.
 */
box2d.collision.shapes.PolygonShape.computeCentroid = function(vs, count) {
    var c = box2d.common.math.Vec2.get(0, 0);
    var area = 0.0;
    var p1X = 0.0;
    var p1Y = 0.0;
    var inv3 = 1.0 / 3.0;
    for (var i = 0; i < count; ++i) {
        var p2 = vs[i];
        var p3 = i + 1 < count ? vs[i + 1] : vs[0];
        var e1X = p2.x - p1X;
        var e1Y = p2.y - p1Y;
        var e2X = p3.x - p1X;
        var e2Y = p3.y - p1Y;
        var D = (e1X * e2Y - e1Y * e2X);
        var triangleArea = 0.5 * D;
        area += triangleArea;
        c.x += triangleArea * inv3 * (p1X + p2.x + p3.x);
        c.y += triangleArea * inv3 * (p1Y + p2.y + p3.y);
    }
    c.x *= 1.0 / area;
    c.y *= 1.0 / area;
    return c;
};

/** @type {!box2d.common.math.Mat22} */
box2d.collision.shapes.PolygonShape.s_mat = box2d.common.math.Mat22.get();

/**
 * @const
 * @type {string}
 */
box2d.collision.shapes.PolygonShape.NAME = 'PolygonShape';
