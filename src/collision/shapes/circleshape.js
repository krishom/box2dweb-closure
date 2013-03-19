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

goog.provide('box2d.collision.shapes.CircleShape');

goog.require('UsageTracker');
goog.require('box2d.collision.shapes.Shape');
goog.require('box2d.common.math.Math');
goog.require('box2d.common.math.Vec2');

/**
 * @param {number} radius
 * @constructor
 * @extends {box2d.collision.shapes.Shape}
 */
box2d.collision.shapes.CircleShape = function(radius) {
    UsageTracker.get('box2d.collision.shapes.CircleShape').trackCreate();
    goog.base(this);

    /**
     * @type {number}
     */
    this.m_radius = radius;

    /**
     * @type {number}
     */
    this.m_radiusSquared = radius * radius;

    /**
     * @const
     * @type {!box2d.common.math.Vec2}
     */
    this.m_p = box2d.common.math.Vec2.get(0, 0);

    /**
     * Used to minimize the creation of arrays for setDistanceProxy only
     * @const
     * @type {!Array.<!box2d.common.math.Vec2>}
     */
    this.m_vertices = [this.m_p];
};
goog.inherits(box2d.collision.shapes.CircleShape, box2d.collision.shapes.Shape);

/**
 * @return {string}
 */
box2d.collision.shapes.CircleShape.prototype.getTypeName = function() {
    return box2d.collision.shapes.CircleShape.NAME;
};

/**
 * @return {!box2d.collision.shapes.CircleShape}
 */
box2d.collision.shapes.CircleShape.prototype.copy = function() {
    var s = new box2d.collision.shapes.CircleShape(this.m_radius);
    s.set(this);
    return s;
};

/**
 * @param {!box2d.collision.shapes.Shape} other
 */
box2d.collision.shapes.CircleShape.prototype.set = function(other) {
    goog.base(this, 'set', other);
    if (other instanceof box2d.collision.shapes.CircleShape) {
        this.m_p.setV(other.m_p);
    }
};

/**
 * @param {!box2d.common.math.Transform} transform
 * @param {!box2d.common.math.Vec2} p
 * @return {boolean}
 */
box2d.collision.shapes.CircleShape.prototype.testPoint = function(transform, p) {
    var dX = p.x - (transform.position.x + (transform.R.col1.x * this.m_p.x + transform.R.col2.x * this.m_p.y));
    var dY = p.y - (transform.position.y + (transform.R.col1.y * this.m_p.x + transform.R.col2.y * this.m_p.y));
    return (dX * dX + dY * dY) <= this.m_radiusSquared;
};

/**
 * @param {!box2d.collision.RayCastOutput} output
 * @param {!box2d.collision.RayCastInput} input
 * @param {!box2d.common.math.Transform} transform
 * @return {boolean}
 */
box2d.collision.shapes.CircleShape.prototype.rayCast = function(output, input, transform) {
    var tMat = transform.R;
    var positionX = transform.position.x + (tMat.col1.x * this.m_p.x + tMat.col2.x * this.m_p.y);
    var positionY = transform.position.y + (tMat.col1.y * this.m_p.x + tMat.col2.y * this.m_p.y);
    var sX = input.p1.x - positionX;
    var sY = input.p1.y - positionY;
    var b = (sX * sX + sY * sY) - this.m_radiusSquared;
    var rX = input.p2.x - input.p1.x;
    var rY = input.p2.y - input.p1.y;
    var c = (sX * rX + sY * rY);
    var rr = (rX * rX + rY * rY);
    var sigma = c * c - rr * b;
    if (sigma < 0.0 || rr < Number.MIN_VALUE) {
        return false;
    }
    var a = (-(c + Math.sqrt(sigma)));
    if (0.0 <= a && a <= input.maxFraction * rr) {
        a /= rr;
        output.fraction = a;
        output.normal.x = sX + a * rX;
        output.normal.y = sY + a * rY;
        output.normal.normalize();
        return true;
    }
    return false;
};

/**
 * @param {!box2d.collision.AABB} aabb
 * @param {!box2d.common.math.Transform} transform
 */
box2d.collision.shapes.CircleShape.prototype.computeAABB = function(aabb, transform) {
    var tMat = transform.R;
    var pX = transform.position.x + (tMat.col1.x * this.m_p.x + tMat.col2.x * this.m_p.y);
    var pY = transform.position.y + (tMat.col1.y * this.m_p.x + tMat.col2.y * this.m_p.y);
    aabb.lowerBound.set(pX - this.m_radius, pY - this.m_radius);
    aabb.upperBound.set(pX + this.m_radius, pY + this.m_radius);
};

/**
 * @param {!box2d.collision.shapes.MassData} massData
 * @param {number} density
 */
box2d.collision.shapes.CircleShape.prototype.computeMass = function(massData, density) {
    var mass = density * Math.PI * this.m_radiusSquared;
    massData.setV(mass, this.m_p, mass * (0.5 * this.m_radiusSquared + (this.m_p.x * this.m_p.x + this.m_p.y * this.m_p.y)));
};

/**
 * @param {!box2d.common.math.Vec2} normal
 * @param {number} offset
 * @param {!box2d.common.math.Transform} xf
 * @param {!box2d.common.math.Vec2} c
 * @return {number}
 */
box2d.collision.shapes.CircleShape.prototype.computeSubmergedArea = function(normal, offset, xf, c) {
    var p = box2d.common.math.Math.mulX(xf, this.m_p);
    var l = (-(box2d.common.math.Math.Dot(normal, p) - offset));
    if (l < (-this.m_radius) + Number.MIN_VALUE) {
        box2d.common.math.Vec2.free(p);
        return 0;
    }
    if (l > this.m_radius) {
        box2d.common.math.Vec2.free(p);
        c.setV(p);
        return Math.PI * this.m_radiusSquared;
    }
    var l2 = l * l;
    var area = this.m_radiusSquared * (Math.asin(l / this.m_radius) + Math.PI / 2) + l * Math.sqrt(this.m_radiusSquared - l2);
    var com = (-2 / 3 * Math.pow(this.m_radiusSquared - l2, 1.5) / area);
    c.x = p.x + normal.x * com;
    c.y = p.y + normal.y * com;
    box2d.common.math.Vec2.free(p);
    return area;
};

/**
 * @param {!box2d.collision.DistanceProxy} proxy
 */
box2d.collision.shapes.CircleShape.prototype.setDistanceProxy = function(proxy) {
    proxy.setValues(1, this.m_radius, this.m_vertices);
};

/**
 * @return {!box2d.common.math.Vec2}
 */
box2d.collision.shapes.CircleShape.prototype.getLocalPosition = function() {
    return this.m_p;
};

/**
 * @param {!box2d.common.math.Vec2} position
 */
box2d.collision.shapes.CircleShape.prototype.setLocalPosition = function(position) {
    this.m_p.setV(position);
};

/**
 * @return {number}
 */
box2d.collision.shapes.CircleShape.prototype.getRadius = function() {
    return this.m_radius;
};

/**
 * @param {number} radius
 */
box2d.collision.shapes.CircleShape.prototype.setRadius = function(radius) {
    this.m_radius = radius;
    this.m_radiusSquared = radius * radius;
};

/**
 * @const
 * @type {string}
 */
box2d.collision.shapes.CircleShape.NAME = 'CircleShape';
