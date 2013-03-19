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

goog.provide('box2d.collision.Distance');

goog.require('box2d.collision.Simplex');
goog.require('box2d.common.Settings');
goog.require('box2d.common.math.Math');
goog.require('box2d.common.math.Vec2');

box2d.collision.Distance = {};

/**
 * @param {!box2d.collision.DistanceOutput} output
 * @param {!box2d.collision.SimplexCache} cache
 * @param {!box2d.collision.DistanceInput} input
 */
box2d.collision.Distance.distance = function(output, cache, input) {
    var s_simplex = box2d.collision.Simplex.get();
    s_simplex.readCache(cache, input.proxyA, input.transformA, input.proxyB, input.transformB);
    if (s_simplex.m_count < 1 || s_simplex.m_count > 3) {
        box2d.common.Settings.assert(false);
    }
    var iter = 0;
    while (iter < 20) {
        var save = [];
        for (var i = 0; i < s_simplex.m_count; i++) {
            save[i] = {};
            save[i].indexA = s_simplex.m_vertices[i].indexA;
            save[i].indexB = s_simplex.m_vertices[i].indexB;
        }
        if (s_simplex.m_count == 2) {
            s_simplex.solve2();
        } else if (s_simplex.m_count == 3) {
            s_simplex.solve3();
        }
        if (s_simplex.m_count == 3) {
            // m_count can be changed by s_simplex.solve3/solve2
            break;
        }
        var d = s_simplex.getSearchDirection();
        if (d.lengthSquared() < box2d.common.Settings.MIN_VALUE_SQUARED) {
            box2d.common.math.Vec2.free(d);
            break;
        }
        box2d.common.math.Vec2.free(s_simplex.m_vertices[s_simplex.m_count].wA);
        box2d.common.math.Vec2.free(s_simplex.m_vertices[s_simplex.m_count].wB);
        box2d.common.math.Vec2.free(s_simplex.m_vertices[s_simplex.m_count].w);
        var negD = d.getNegative();
        var aNegD = box2d.common.math.Math.mulTMV(input.transformA.R, negD);
        box2d.common.math.Vec2.free(negD);
        s_simplex.m_vertices[s_simplex.m_count].indexA = input.proxyA.getSupport(aNegD);
        box2d.common.math.Vec2.free(aNegD);
        s_simplex.m_vertices[s_simplex.m_count].wA = box2d.common.math.Math.mulX(input.transformA, input.proxyA.getVertex(s_simplex.m_vertices[s_simplex.m_count].indexA));
        var bD = box2d.common.math.Math.mulTMV(input.transformB.R, d);
        box2d.common.math.Vec2.free(d);
        s_simplex.m_vertices[s_simplex.m_count].indexB = input.proxyB.getSupport(bD);
        box2d.common.math.Vec2.free(bD);
        s_simplex.m_vertices[s_simplex.m_count].wB = box2d.common.math.Math.mulX(input.transformB, input.proxyB.getVertex(s_simplex.m_vertices[s_simplex.m_count].indexB));
        s_simplex.m_vertices[s_simplex.m_count].w = box2d.common.math.Math.subtractVV(s_simplex.m_vertices[s_simplex.m_count].wB, s_simplex.m_vertices[s_simplex.m_count].wA);

        iter++;
        var duplicate = false;
        for (var i = 0; i < save.length; i++) {
            if (s_simplex.m_vertices[s_simplex.m_count].indexA == save[i].indexA && s_simplex.m_vertices[s_simplex.m_count].indexB == save[i].indexB) {
                duplicate = true;
                break;
            }
        }
        if (duplicate) {
            break;
        }
        s_simplex.m_count++;
    }
    s_simplex.getWitnessPoints(output.pointA, output.pointB);
    var distanceV = box2d.common.math.Math.subtractVV(output.pointA, output.pointB);
    output.distance = distanceV.length();
    box2d.common.math.Vec2.free(distanceV);
    s_simplex.writeCache(cache);
    box2d.collision.Simplex.free(s_simplex);
    if (input.useRadii) {
        var rA = input.proxyA.m_radius;
        var rB = input.proxyB.m_radius;
        if (output.distance > rA + rB && output.distance > Number.MIN_VALUE) {
            output.distance -= rA + rB;
            var normal = box2d.common.math.Math.subtractVV(output.pointB, output.pointA);
            normal.normalize();
            output.pointA.x += rA * normal.x;
            output.pointA.y += rA * normal.y;
            output.pointB.x -= rB * normal.x;
            output.pointB.y -= rB * normal.y;
            box2d.common.math.Vec2.free(normal);
        } else {
            var p = box2d.common.math.Vec2.get(0, 0);
            p.x = 0.5 * (output.pointA.x + output.pointB.x);
            p.y = 0.5 * (output.pointA.y + output.pointB.y);
            output.pointA.x = output.pointB.x = p.x;
            output.pointA.y = output.pointB.y = p.y;
            output.distance = 0.0;
            box2d.common.math.Vec2.free(p);
        }
    }
};
