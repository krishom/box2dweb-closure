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

goog.provide('box2d.collision.DynamicTreePair');

goog.require('UsageTracker');

/**
 * @private
 * @param {!box2d.dynamics.Fixture} fixtureA
 * @param {!box2d.dynamics.Fixture} fixtureB
 * @constructor
 */
box2d.collision.DynamicTreePair = function(fixtureA, fixtureB) {
    UsageTracker.get('box2d.collision.DynamicTreePair').trackCreate();

    /**
     * @type {!box2d.dynamics.Fixture}
     */
    this.fixtureA = fixtureA;

    /**
     * @type {!box2d.dynamics.Fixture}
     */
    this.fixtureB = fixtureB;
};

/**
 * @private
 * @type {Array.<!box2d.collision.DynamicTreePair>}
 */
box2d.collision.DynamicTreePair.freeCache_ = [];

/**
 * @param {!box2d.dynamics.Fixture} fixtureA
 * @param {!box2d.dynamics.Fixture} fixtureB
 * @return {!box2d.collision.DynamicTreePair}
 */
box2d.collision.DynamicTreePair.get = function(fixtureA, fixtureB) {
    UsageTracker.get('box2d.collision.DynamicTreePair').trackGet();
    if (box2d.collision.DynamicTreePair.freeCache_.length > 0) {
        var pair = box2d.collision.DynamicTreePair.freeCache_.pop();
        pair.fixtureA = fixtureA;
        pair.fixtureB = fixtureB;
        return pair;
    }
    return new box2d.collision.DynamicTreePair(fixtureA, fixtureB);
};

/**
 * @param {!box2d.collision.DynamicTreePair} pair
 */
box2d.collision.DynamicTreePair.free = function(pair) {
    if (pair != null) {
        UsageTracker.get('box2d.collision.DynamicTreePair').trackFree();
        box2d.collision.DynamicTreePair.freeCache_.push(pair);
    }
};

