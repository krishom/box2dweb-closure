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

goog.provide('box2d.collision.RayCastInput');

goog.require('UsageTracker');
goog.require('box2d.common.math.Vec2');

/**
 * @constructor
 * @param {!box2d.common.math.Vec2} p1
 * @param {!box2d.common.math.Vec2} p2
 * @param {number} maxFraction
 */
box2d.collision.RayCastInput = function(p1, p2, maxFraction) {
    UsageTracker.get('box2d.collision.RayCastInput').trackCreate();

    /**
     * @type {!box2d.common.math.Vec2}
     */
    this.p1 = box2d.common.math.Vec2.get(p1.x, p1.y);

    /**
     * @type {!box2d.common.math.Vec2}
     */
    this.p2 = box2d.common.math.Vec2.get(p2.x, p2.y);

    /**
     * @type {number}
     */
    this.maxFraction = maxFraction;
};
