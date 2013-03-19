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

goog.provide('box2d.collision.TimeOfImpact');

goog.require('box2d.collision.SimplexCache');
goog.require('box2d.collision.DistanceInput');
goog.require('box2d.collision.DistanceOutput');
goog.require('box2d.collision.SeparationFunction');
goog.require('box2d.common.math.Transform');
goog.require('box2d.common.Settings');
goog.require('box2d.collision.Distance');

box2d.collision.TimeOfImpact = {};

/**
 * @param {!box2d.collision.TOIInput} input
 * @return {number}
 */
box2d.collision.TimeOfImpact.TimeOfImpact = function(input) {
    box2d.collision.TimeOfImpact.toiCalls++;
    var proxyA = input.proxyA;
    var proxyB = input.proxyB;
    var sweepA = input.sweepA;
    var sweepB = input.sweepB;
    box2d.common.Settings.assert(sweepA.t0 == sweepB.t0);
    box2d.common.Settings.assert(1.0 - sweepA.t0 > Number.MIN_VALUE);
    var radius = proxyA.m_radius + proxyB.m_radius;
    var tolerance = input.tolerance;
    var alpha = 0.0;
    var k_maxIterations = 1000;
    var iter = 0;
    var target = 0.0;
    box2d.collision.TimeOfImpact.s_cache.count = 0;
    box2d.collision.TimeOfImpact.s_distanceInput.useRadii = false;
    for (;;) {
        sweepA.getTransform(box2d.collision.TimeOfImpact.s_xfA, alpha);
        sweepB.getTransform(box2d.collision.TimeOfImpact.s_xfB, alpha);
        box2d.collision.TimeOfImpact.s_distanceInput.proxyA = proxyA;
        box2d.collision.TimeOfImpact.s_distanceInput.proxyB = proxyB;
        box2d.collision.TimeOfImpact.s_distanceInput.transformA = box2d.collision.TimeOfImpact.s_xfA;
        box2d.collision.TimeOfImpact.s_distanceInput.transformB = box2d.collision.TimeOfImpact.s_xfB;
        box2d.collision.Distance.distance(box2d.collision.TimeOfImpact.s_distanceOutput, box2d.collision.TimeOfImpact.s_cache, box2d.collision.TimeOfImpact.s_distanceInput);
        if (box2d.collision.TimeOfImpact.s_distanceOutput.distance <= 0.0) {
            alpha = 1.0;
            break;
        }
        box2d.collision.TimeOfImpact.s_fcn.initialize(box2d.collision.TimeOfImpact.s_cache, proxyA, box2d.collision.TimeOfImpact.s_xfA, proxyB, box2d.collision.TimeOfImpact.s_xfB);
        var separation = box2d.collision.TimeOfImpact.s_fcn.evaluate(box2d.collision.TimeOfImpact.s_xfA, box2d.collision.TimeOfImpact.s_xfB);
        if (separation <= 0.0) {
            alpha = 1.0;
            break;
        }
        if (iter == 0) {
            if (separation > radius) {
                target = Math.max(radius - tolerance, 0.75 * radius);
            } else {
                target = Math.max(separation - tolerance, 0.02 * radius);
            }
        }
        if (separation - target < 0.5 * tolerance) {
            if (iter == 0) {
                alpha = 1.0;
                break;
            }
            break;
        }
        var newAlpha = alpha;
        {
            var x1 = alpha;
            var x2 = 1.0;
            var f1 = separation;
            sweepA.getTransform(box2d.collision.TimeOfImpact.s_xfA, x2);
            sweepB.getTransform(box2d.collision.TimeOfImpact.s_xfB, x2);
            var f2 = box2d.collision.TimeOfImpact.s_fcn.evaluate(box2d.collision.TimeOfImpact.s_xfA, box2d.collision.TimeOfImpact.s_xfB);
            if (f2 >= target) {
                alpha = 1.0;
                break;
            }
            var rootIterCount = 0;
            for (;;) {
                var x = 0;
                if (rootIterCount & 1) {
                    x = x1 + (target - f1) * (x2 - x1) / (f2 - f1);
                } else {
                    x = 0.5 * (x1 + x2);
                }
                sweepA.getTransform(box2d.collision.TimeOfImpact.s_xfA, x);
                sweepB.getTransform(box2d.collision.TimeOfImpact.s_xfB, x);
                var f = box2d.collision.TimeOfImpact.s_fcn.evaluate(box2d.collision.TimeOfImpact.s_xfA, box2d.collision.TimeOfImpact.s_xfB);
                if (Math.abs(f - target) < 0.025 * tolerance) {
                    newAlpha = x;
                    break;
                }
                if (f > target) {
                    x1 = x;
                    f1 = f;
                } else {
                    x2 = x;
                    f2 = f;
                }
                rootIterCount++;
                box2d.collision.TimeOfImpact.toiRootIters++;
                if (rootIterCount == 50) {
                    break;
                }
            }
            box2d.collision.TimeOfImpact.toiMaxRootIters = Math.max(box2d.collision.TimeOfImpact.toiMaxRootIters, rootIterCount);
        }
        if (newAlpha < (1.0 + 100.0 * Number.MIN_VALUE) * alpha) {
            break;
        }
        alpha = newAlpha;
        iter++;
        box2d.collision.TimeOfImpact.toiIters++;
        if (iter == k_maxIterations) {
            break;
        }
    }
    box2d.collision.TimeOfImpact.toiMaxIters = Math.max(box2d.collision.TimeOfImpact.toiMaxIters, iter);
    return alpha;
};

/**
 * @private
 * @type {number}
 */
box2d.collision.TimeOfImpact.toiCalls = 0;

/**
 * @private
 * @type {number}
 */
box2d.collision.TimeOfImpact.toiIters = 0;

/**
 * @private
 * @type {number}
 */
box2d.collision.TimeOfImpact.toiMaxIters = 0;

/**
 * @private
 * @type {number}
 */
box2d.collision.TimeOfImpact.toiRootIters = 0;

/**
 * @private
 * @type {number}
 */
box2d.collision.TimeOfImpact.toiMaxRootIters = 0;

/**
 * @private
 * @type {!box2d.collision.SimplexCache}
 */
box2d.collision.TimeOfImpact.s_cache = new box2d.collision.SimplexCache();

/**
 * @private
 * @type {!box2d.collision.DistanceInput}
 */
box2d.collision.TimeOfImpact.s_distanceInput = new box2d.collision.DistanceInput();

/**
 * @private
 * @type {!box2d.common.math.Transform}
 */
box2d.collision.TimeOfImpact.s_xfA = new box2d.common.math.Transform();

/**
 * @private
 * @type {!box2d.common.math.Transform}
 */
box2d.collision.TimeOfImpact.s_xfB = new box2d.common.math.Transform();

/**
 * @private
 * @type {!box2d.collision.SeparationFunction}
 */
box2d.collision.TimeOfImpact.s_fcn = new box2d.collision.SeparationFunction();

/**
 * @private
 * @type {!box2d.collision.DistanceOutput}
 */
box2d.collision.TimeOfImpact.s_distanceOutput = new box2d.collision.DistanceOutput();
