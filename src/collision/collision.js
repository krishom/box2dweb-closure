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

goog.provide('box2d.collision.Collision');

goog.require('box2d.collision.ClipVertex');
goog.require('box2d.collision.Manifold');
goog.require('box2d.common.Settings');
goog.require('box2d.common.math.Vec2');

box2d.collision.Collision = {};

/**
 * @param {!Array.<!box2d.collision.ClipVertex>} vOut
 * @param {!Array.<!box2d.collision.ClipVertex>} vIn
 * @param {!box2d.common.math.Vec2} normal
 * @param {number} offset
 */
box2d.collision.Collision.clipSegmentToLine = function(vOut, vIn, normal, offset) {
    var numOut = 0;
    var vIn0 = vIn[0].v;
    var vIn1 = vIn[1].v;
    var distance0 = normal.x * vIn0.x + normal.y * vIn0.y - offset;
    var distance1 = normal.x * vIn1.x + normal.y * vIn1.y - offset;
    if (distance0 <= 0.0) {
        vOut[numOut++].set(vIn[0]);
    }
    if (distance1 <= 0.0) {
        vOut[numOut++].set(vIn[1]);
    }
    if (distance0 * distance1 < 0.0) {
        var interp = distance0 / (distance0 - distance1);
        var tVec = vOut[numOut].v;
        tVec.x = vIn0.x + interp * (vIn1.x - vIn0.x);
        tVec.y = vIn0.y + interp * (vIn1.y - vIn0.y);
        if (distance0 > 0.0) {
            vOut[numOut].id = vIn[0].id;
        } else {
            vOut[numOut].id = vIn[1].id;
        }
        numOut++;
    }
    return numOut;
};

/**
 * @param {!box2d.collision.shapes.PolygonShape} poly1
 * @param {!box2d.common.math.Transform} xf1
 * @param {number} edge1
 * @param {!box2d.collision.shapes.PolygonShape} poly2
 * @param {!box2d.common.math.Transform} xf2
 * @return {number}
 */
box2d.collision.Collision.edgeSeparation = function(poly1, xf1, edge1, poly2, xf2) {
    var normal1WorldX = (xf1.R.col1.x * poly1.m_normals[edge1].x + xf1.R.col2.x * poly1.m_normals[edge1].y);
    var normal1WorldY = (xf1.R.col1.y * poly1.m_normals[edge1].x + xf1.R.col2.y * poly1.m_normals[edge1].y);
    var normal1X = (xf2.R.col1.x * normal1WorldX + xf2.R.col1.y * normal1WorldY);
    var normal1Y = (xf2.R.col2.x * normal1WorldX + xf2.R.col2.y * normal1WorldY);
    var index = 0;
    var minDot = Number.MAX_VALUE;
    for (var i = 0; i < poly2.m_vertexCount; i++) {
        var dot = poly2.m_vertices[i].x * normal1X + poly2.m_vertices[i].y * normal1Y;
        if (dot < minDot) {
            minDot = dot;
            index = i;
        }
    }
    var v1X = xf1.position.x + (xf1.R.col1.x * poly1.m_vertices[edge1].x + xf1.R.col2.x * poly1.m_vertices[edge1].y);
    var v1Y = xf1.position.y + (xf1.R.col1.y * poly1.m_vertices[edge1].x + xf1.R.col2.y * poly1.m_vertices[edge1].y);
    var v2X = xf2.position.x + (xf2.R.col1.x * poly2.m_vertices[index].x + xf2.R.col2.x * poly2.m_vertices[index].y);
    var v2Y = xf2.position.y + (xf2.R.col1.y * poly2.m_vertices[index].x + xf2.R.col2.y * poly2.m_vertices[index].y);
    var separation = (v2X - v1X) * normal1WorldX + (v2Y - v1Y) * normal1WorldY;
    return separation;
};

/**
 * @param {!box2d.collision.shapes.PolygonShape} poly1
 * @param {!box2d.common.math.Transform} xf1
 * @param {!box2d.collision.shapes.PolygonShape} poly2
 * @param {!box2d.common.math.Transform} xf2
 * @return {{bestEdge: number, separation: number}}
 */
box2d.collision.Collision.findMaxSeparation = function(poly1, xf1, poly2, xf2) {
    var dX = xf2.position.x + (xf2.R.col1.x * poly2.m_centroid.x + xf2.R.col2.x * poly2.m_centroid.y);
    var dY = xf2.position.y + (xf2.R.col1.y * poly2.m_centroid.x + xf2.R.col2.y * poly2.m_centroid.y);
    dX -= xf1.position.x + (xf1.R.col1.x * poly1.m_centroid.x + xf1.R.col2.x * poly1.m_centroid.y);
    dY -= xf1.position.y + (xf1.R.col1.y * poly1.m_centroid.x + xf1.R.col2.y * poly1.m_centroid.y);
    var dLocal1X = (dX * xf1.R.col1.x + dY * xf1.R.col1.y);
    var dLocal1Y = (dX * xf1.R.col2.x + dY * xf1.R.col2.y);
    var edge = 0;
    var maxDot = (-Number.MAX_VALUE);
    for (var i = 0; i < poly1.m_vertexCount; ++i) {
        var dot = (poly1.m_normals[i].x * dLocal1X + poly1.m_normals[i].y * dLocal1Y);
        if (dot > maxDot) {
            maxDot = dot;
            edge = i;
        }
    }
    var s = box2d.collision.Collision.edgeSeparation(poly1, xf1, edge, poly2, xf2);
    var prevEdge = edge - 1;
    if (prevEdge < 0) {
        prevEdge = poly1.m_vertexCount - 1;
    }
    var sPrev = box2d.collision.Collision.edgeSeparation(poly1, xf1, prevEdge, poly2, xf2);
    var nextEdge = edge + 1;
    if (nextEdge >= poly1.m_vertexCount) {
        nextEdge = 0;
    }
    var sNext = box2d.collision.Collision.edgeSeparation(poly1, xf1, nextEdge, poly2, xf2);
    var bestEdge = 0;
    var bestSeparation = 0;
    if (sPrev > s && sPrev > sNext) {
        bestEdge = prevEdge;
        bestSeparation = sPrev;
        while (true) {
            edge = bestEdge - 1;
            if (edge < 0) {
                edge = poly1.m_vertexCount - 1;
            }
            s = box2d.collision.Collision.edgeSeparation(poly1, xf1, edge, poly2, xf2);
            if (s > bestSeparation) {
                bestEdge = edge;
                bestSeparation = s;
            } else {
                break;
            }
        }
    } else if (sNext > s) {
        bestEdge = nextEdge;
        bestSeparation = sNext;
        while (true) {
            edge = bestEdge + 1;
            if (edge >= poly1.m_vertexCount) {
                edge = 0;
            }
            s = box2d.collision.Collision.edgeSeparation(poly1, xf1, edge, poly2, xf2);
            if (s > bestSeparation) {
                bestEdge = edge;
                bestSeparation = s;
            } else {
                break;
            }
        }
    } else {
        bestEdge = edge;
        bestSeparation = s;
    }
    return {bestEdge: bestEdge, separation: bestSeparation};
};

/**
 * @param {!Array.<!box2d.collision.ClipVertex>} c
 * @param {!box2d.collision.shapes.PolygonShape} poly1
 * @param {!box2d.common.math.Transform} xf1
 * @param {number} edge1
 * @param {!box2d.collision.shapes.PolygonShape} poly2
 * @param {!box2d.common.math.Transform} xf2
 */
box2d.collision.Collision.findIncidentEdge = function(c, poly1, xf1, edge1, poly2, xf2) {
    var normal1X = (xf1.R.col1.x * poly1.m_normals[edge1].x + xf1.R.col2.x * poly1.m_normals[edge1].y);
    var normal1Y = (xf1.R.col1.y * poly1.m_normals[edge1].x + xf1.R.col2.y * poly1.m_normals[edge1].y);
    var tX = (xf2.R.col1.x * normal1X + xf2.R.col1.y * normal1Y);
    normal1Y = (xf2.R.col2.x * normal1X + xf2.R.col2.y * normal1Y);
    normal1X = tX;
    var i1 = 0;
    var minDot = Number.MAX_VALUE;
    for (var i = 0; i < poly2.m_vertexCount; i++) {
        var dot = (normal1X * poly2.m_normals[i].x + normal1Y * poly2.m_normals[i].y);
        if (dot < minDot) {
            minDot = dot;
            i1 = i;
        }
    }
    var i2 = i1 + 1;
    if (i2 >= poly2.m_vertexCount) {
        i2 = 0;
    }
    c[0].v.x = xf2.position.x + (xf2.R.col1.x * poly2.m_vertices[i1].x + xf2.R.col2.x * poly2.m_vertices[i1].y);
    c[0].v.y = xf2.position.y + (xf2.R.col1.y * poly2.m_vertices[i1].x + xf2.R.col2.y * poly2.m_vertices[i1].y);
    c[0].id.setReferenceEdge(edge1);
    c[0].id.setIncidentEdge(i1);
    c[0].id.setIncidentVertex(0);
    c[1].v.x = xf2.position.x + (xf2.R.col1.x * poly2.m_vertices[i2].x + xf2.R.col2.x * poly2.m_vertices[i2].y);
    c[1].v.y = xf2.position.y + (xf2.R.col1.y * poly2.m_vertices[i2].x + xf2.R.col2.y * poly2.m_vertices[i2].y);
    c[1].id.setReferenceEdge(edge1);
    c[1].id.setIncidentEdge(i2);
    c[1].id.setIncidentVertex(1);
};

/**
 * @return {!Array.<!box2d.collision.ClipVertex>}
 */
box2d.collision.Collision.makeClipPointVector = function() {
    return [new box2d.collision.ClipVertex(), new box2d.collision.ClipVertex()];
};

/**
 * @param {!box2d.collision.Manifold} manifold
 * @param {!box2d.collision.shapes.PolygonShape} polyA
 * @param {!box2d.common.math.Transform} xfA
 * @param {!box2d.collision.shapes.PolygonShape} polyB
 * @param {!box2d.common.math.Transform} xfB
 */
box2d.collision.Collision.collidePolygons = function(manifold, polyA, xfA, polyB, xfB) {
    manifold.m_pointCount = 0;
    var totalRadius = polyA.m_radius + polyB.m_radius;

    var separationEdgeA = box2d.collision.Collision.findMaxSeparation(polyA, xfA, polyB, xfB);
    if (separationEdgeA.separation > totalRadius) {
        return;
    }

    var separationEdgeB = box2d.collision.Collision.findMaxSeparation(polyB, xfB, polyA, xfA);
    if (separationEdgeB.separation > totalRadius) {
        return;
    }

    var poly1 = polyA;
    var poly2 = polyB;
    var xf1 = xfA;
    var xf2 = xfB;
    var flip = 0;
    var edge1 = separationEdgeA.bestEdge;

    manifold.m_type = box2d.collision.Manifold.e_faceA;
    if (separationEdgeB.separation > 0.98 /* k_relativeTol */ * separationEdgeA.separation + 0.001 /* k_absoluteTol */) {
        poly1 = polyB;
        poly2 = polyA;
        xf1 = xfB;
        xf2 = xfA;
        edge1 = separationEdgeB.bestEdge;
        manifold.m_type = box2d.collision.Manifold.e_faceB;
        flip = 1;
    }
    var incidentEdge = box2d.collision.Collision.s_incidentEdge;
    box2d.collision.Collision.findIncidentEdge(incidentEdge, poly1, xf1, edge1, poly2, xf2);
    var local_v11 = poly1.m_vertices[edge1];
    var local_v12;
    if (edge1 + 1 < poly1.m_vertexCount) {
        local_v12 = poly1.m_vertices[edge1 + 1];
    } else {
        local_v12 = poly1.m_vertices[0];
    }
    box2d.collision.Collision.s_localTangent.set(local_v12.x - local_v11.x, local_v12.y - local_v11.y);
    box2d.collision.Collision.s_localTangent.normalize();
    box2d.collision.Collision.s_localNormal.x = box2d.collision.Collision.s_localTangent.y;
    box2d.collision.Collision.s_localNormal.y = (-box2d.collision.Collision.s_localTangent.x);
    box2d.collision.Collision.s_planePoint.set(0.5 * (local_v11.x + local_v12.x), 0.5 * (local_v11.y + local_v12.y));
    box2d.collision.Collision.s_tangent.x = (xf1.R.col1.x * box2d.collision.Collision.s_localTangent.x + xf1.R.col2.x * box2d.collision.Collision.s_localTangent.y);
    box2d.collision.Collision.s_tangent.y = (xf1.R.col1.y * box2d.collision.Collision.s_localTangent.x + xf1.R.col2.y * box2d.collision.Collision.s_localTangent.y);
    box2d.collision.Collision.s_tangent2.x = (-box2d.collision.Collision.s_tangent.x);
    box2d.collision.Collision.s_tangent2.y = (-box2d.collision.Collision.s_tangent.y);
    box2d.collision.Collision.s_normal.x = box2d.collision.Collision.s_tangent.y;
    box2d.collision.Collision.s_normal.y = (-box2d.collision.Collision.s_tangent.x);
    box2d.collision.Collision.s_v11.x = xf1.position.x + (xf1.R.col1.x * local_v11.x + xf1.R.col2.x * local_v11.y);
    box2d.collision.Collision.s_v11.y = xf1.position.y + (xf1.R.col1.y * local_v11.x + xf1.R.col2.y * local_v11.y);
    box2d.collision.Collision.s_v12.x = xf1.position.x + (xf1.R.col1.x * local_v12.x + xf1.R.col2.x * local_v12.y);
    box2d.collision.Collision.s_v12.y = xf1.position.y + (xf1.R.col1.y * local_v12.x + xf1.R.col2.y * local_v12.y);
    var sideOffset1 = (-box2d.collision.Collision.s_tangent.x * box2d.collision.Collision.s_v11.x) - box2d.collision.Collision.s_tangent.y * box2d.collision.Collision.s_v11.y + totalRadius;
    if (box2d.collision.Collision.clipSegmentToLine(box2d.collision.Collision.s_clipPoints1, incidentEdge, box2d.collision.Collision.s_tangent2, sideOffset1) < 2) {
        return;
    }
    var sideOffset2 = box2d.collision.Collision.s_tangent.x * box2d.collision.Collision.s_v12.x + box2d.collision.Collision.s_tangent.y * box2d.collision.Collision.s_v12.y + totalRadius;
    if (box2d.collision.Collision.clipSegmentToLine(box2d.collision.Collision.s_clipPoints2, box2d.collision.Collision.s_clipPoints1, box2d.collision.Collision.s_tangent, sideOffset2) < 2) {
        return;
    }
    manifold.m_localPlaneNormal.setV(box2d.collision.Collision.s_localNormal);
    manifold.m_localPoint.setV(box2d.collision.Collision.s_planePoint);
    var frontOffset = box2d.collision.Collision.s_normal.x * box2d.collision.Collision.s_v11.x + box2d.collision.Collision.s_normal.y * box2d.collision.Collision.s_v11.y;
    var pointCount = 0;
    for (var i = 0; i < box2d.common.Settings.maxManifoldPoints; ++i) {
        var separation = box2d.collision.Collision.s_normal.x * box2d.collision.Collision.s_clipPoints2[i].v.x + box2d.collision.Collision.s_normal.y * box2d.collision.Collision.s_clipPoints2[i].v.y - frontOffset;
        if (separation <= totalRadius) {
            var tX = box2d.collision.Collision.s_clipPoints2[i].v.x - xf2.position.x;
            var tY = box2d.collision.Collision.s_clipPoints2[i].v.y - xf2.position.y;
            manifold.m_points[pointCount].m_localPoint.x = (tX * xf2.R.col1.x + tY * xf2.R.col1.y);
            manifold.m_points[pointCount].m_localPoint.y = (tX * xf2.R.col2.x + tY * xf2.R.col2.y);
            manifold.m_points[pointCount].m_id.set(box2d.collision.Collision.s_clipPoints2[i].id);
            manifold.m_points[pointCount].m_id.setFlip(flip);
            pointCount++;
        }
    }
    manifold.m_pointCount = pointCount;
};

/**
 * @param {!box2d.collision.Manifold} manifold
 * @param {!box2d.collision.shapes.CircleShape} circle1
 * @param {!box2d.common.math.Transform} xf1
 * @param {!box2d.collision.shapes.CircleShape} circle2
 * @param {!box2d.common.math.Transform} xf2
 */
box2d.collision.Collision.collideCircles = function(manifold, circle1, xf1, circle2, xf2) {
    manifold.m_pointCount = 0;
    var p1X = xf1.position.x + (xf1.R.col1.x * circle1.m_p.x + xf1.R.col2.x * circle1.m_p.y);
    var p1Y = xf1.position.y + (xf1.R.col1.y * circle1.m_p.x + xf1.R.col2.y * circle1.m_p.y);
    var p2X = xf2.position.x + (xf2.R.col1.x * circle2.m_p.x + xf2.R.col2.x * circle2.m_p.y);
    var p2Y = xf2.position.y + (xf2.R.col1.y * circle2.m_p.x + xf2.R.col2.y * circle2.m_p.y);
    var dX = p2X - p1X;
    var dY = p2Y - p1Y;
    var distSqr = dX * dX + dY * dY;
    var radius = circle1.m_radius + circle2.m_radius;
    if (distSqr > radius * radius) {
        return;
    }
    manifold.m_type = box2d.collision.Manifold.e_circles;
    manifold.m_localPoint.setV(circle1.m_p);
    manifold.m_localPlaneNormal.setZero();
    manifold.m_pointCount = 1;
    manifold.m_points[0].m_localPoint.setV(circle2.m_p);
    manifold.m_points[0].m_id.setKey(0);
};

/**
 * @param {!box2d.collision.Manifold} manifold
 * @param {!box2d.collision.shapes.PolygonShape} polygon
 * @param {!box2d.common.math.Transform} xf1
 * @param {!box2d.collision.shapes.CircleShape} circle
 * @param {!box2d.common.math.Transform} xf2
 */
box2d.collision.Collision.collidePolygonAndCircle = function(manifold, polygon, xf1, circle, xf2) {
    manifold.m_pointCount = 0;
    var dX = xf2.position.x + (xf2.R.col1.x * circle.m_p.x + xf2.R.col2.x * circle.m_p.y) - xf1.position.x;
    var dY = xf2.position.y + (xf2.R.col1.y * circle.m_p.x + xf2.R.col2.y * circle.m_p.y) - xf1.position.y;
    var cLocalX = (dX * xf1.R.col1.x + dY * xf1.R.col1.y);
    var cLocalY = (dX * xf1.R.col2.x + dY * xf1.R.col2.y);
    var normalIndex = 0;
    var separation = (-Number.MAX_VALUE);
    var radius = polygon.m_radius + circle.m_radius;
    for (var i = 0; i < polygon.m_vertexCount; ++i) {
        var s = polygon.m_normals[i].x * (cLocalX - polygon.m_vertices[i].x) + polygon.m_normals[i].y * (cLocalY - polygon.m_vertices[i].y);
        if (s > radius) {
            return;
        }
        if (s > separation) {
            separation = s;
            normalIndex = i;
        }
    }
    var vertIndex2 = normalIndex + 1;
    if (vertIndex2 >= polygon.m_vertexCount) {
        vertIndex2 = 0;
    }
    var v1 = polygon.m_vertices[normalIndex];
    var v2 = polygon.m_vertices[vertIndex2];
    if (separation < Number.MIN_VALUE) {
        manifold.m_pointCount = 1;
        manifold.m_type = box2d.collision.Manifold.e_faceA;
        manifold.m_localPlaneNormal.setV(polygon.m_normals[normalIndex]);
        manifold.m_localPoint.x = 0.5 * (v1.x + v2.x);
        manifold.m_localPoint.y = 0.5 * (v1.y + v2.y);
        manifold.m_points[0].m_localPoint.setV(circle.m_p);
        manifold.m_points[0].m_id.setKey(0);
    } else {
        var u1 = (cLocalX - v1.x) * (v2.x - v1.x) + (cLocalY - v1.y) * (v2.y - v1.y);
        if (u1 <= 0.0) {
            if ((cLocalX - v1.x) * (cLocalX - v1.x) + (cLocalY - v1.y) * (cLocalY - v1.y) > radius * radius) return;
            manifold.m_pointCount = 1;
            manifold.m_type = box2d.collision.Manifold.e_faceA;
            manifold.m_localPlaneNormal.x = cLocalX - v1.x;
            manifold.m_localPlaneNormal.y = cLocalY - v1.y;
            manifold.m_localPlaneNormal.normalize();
            manifold.m_localPoint.setV(v1);
            manifold.m_points[0].m_localPoint.setV(circle.m_p);
            manifold.m_points[0].m_id.setKey(0);
        } else {
            var u2 = (cLocalX - v2.x) * (v1.x - v2.x) + (cLocalY - v2.y) * (v1.y - v2.y);
            if (u2 <= 0) {
                if ((cLocalX - v2.x) * (cLocalX - v2.x) + (cLocalY - v2.y) * (cLocalY - v2.y) > radius * radius) return;
                manifold.m_pointCount = 1;
                manifold.m_type = box2d.collision.Manifold.e_faceA;
                manifold.m_localPlaneNormal.x = cLocalX - v2.x;
                manifold.m_localPlaneNormal.y = cLocalY - v2.y;
                manifold.m_localPlaneNormal.normalize();
                manifold.m_localPoint.setV(v2);
                manifold.m_points[0].m_localPoint.setV(circle.m_p);
                manifold.m_points[0].m_id.setKey(0);
            } else {
                var faceCenterX = 0.5 * (v1.x + v2.x);
                var faceCenterY = 0.5 * (v1.y + v2.y);
                separation = (cLocalX - faceCenterX) * polygon.m_normals[normalIndex].x + (cLocalY - faceCenterY) * polygon.m_normals[normalIndex].y;
                if (separation > radius) return;
                manifold.m_pointCount = 1;
                manifold.m_type = box2d.collision.Manifold.e_faceA;
                manifold.m_localPlaneNormal.x = polygon.m_normals[normalIndex].x;
                manifold.m_localPlaneNormal.y = polygon.m_normals[normalIndex].y;
                manifold.m_localPlaneNormal.normalize();
                manifold.m_localPoint.set(faceCenterX, faceCenterY);
                manifold.m_points[0].m_localPoint.setV(circle.m_p);
                manifold.m_points[0].m_id.setKey(0);
            }
        }
    }
};

/**
 * @private
 * @type {!Array.<!box2d.collision.ClipVertex>}
 */
box2d.collision.Collision.s_incidentEdge = box2d.collision.Collision.makeClipPointVector();

/**
 * @private
 * @type {!Array.<!box2d.collision.ClipVertex>}
 */
box2d.collision.Collision.s_clipPoints1 = box2d.collision.Collision.makeClipPointVector();

/**
 * @private
 * @type {!Array.<!box2d.collision.ClipVertex>}
 */
box2d.collision.Collision.s_clipPoints2 = box2d.collision.Collision.makeClipPointVector();

/**
 * @private
 * @type {!box2d.common.math.Vec2}
 */
box2d.collision.Collision.s_localTangent = box2d.common.math.Vec2.get(0, 0);

/**
 * @private
 * @type {!box2d.common.math.Vec2}
 */
box2d.collision.Collision.s_localNormal = box2d.common.math.Vec2.get(0, 0);

/**
 * @private
 * @type {!box2d.common.math.Vec2}
 */
box2d.collision.Collision.s_planePoint = box2d.common.math.Vec2.get(0, 0);

/**
 * @private
 * @type {!box2d.common.math.Vec2}
 */
box2d.collision.Collision.s_normal = box2d.common.math.Vec2.get(0, 0);

/**
 * @private
 * @type {!box2d.common.math.Vec2}
 */
box2d.collision.Collision.s_tangent = box2d.common.math.Vec2.get(0, 0);

/**
 * @private
 * @type {!box2d.common.math.Vec2}
 */
box2d.collision.Collision.s_tangent2 = box2d.common.math.Vec2.get(0, 0);

/**
 * @private
 * @type {!box2d.common.math.Vec2}
 */
box2d.collision.Collision.s_v11 = box2d.common.math.Vec2.get(0, 0);

/**
 * @private
 * @type {!box2d.common.math.Vec2}
 */
box2d.collision.Collision.s_v12 = box2d.common.math.Vec2.get(0, 0);
