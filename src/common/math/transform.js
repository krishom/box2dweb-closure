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

goog.provide('box2d.common.math.Transform');

goog.require('UsageTracker');
goog.require('box2d.common.math.Mat22');
goog.require('box2d.common.math.Vec2');

/**
 * @param {!box2d.common.math.Vec2=} pos
 * @param {!box2d.common.math.Mat22=} r
 * @constructor
 */
box2d.common.math.Transform = function(pos, r) {
    UsageTracker.get('box2d.common.math.Transform').trackCreate();
    this.position = box2d.common.math.Vec2.get(0, 0);
    this.R = box2d.common.math.Mat22.get();
    if (pos) {
        this.position.setV(pos);
    }
    if (r) {
        this.R.setM(r);
    }
};

box2d.common.math.Transform.prototype.initialize = function(pos, r) {
    this.position.setV(pos);
    this.R.setM(r);
};

box2d.common.math.Transform.prototype.setIdentity = function() {
    this.position.setZero();
    this.R.setIdentity();
};

box2d.common.math.Transform.prototype.set = function(x) {
    this.position.setV(x.position);
    this.R.setM(x.R);
};

box2d.common.math.Transform.prototype.getAngle = function() {
    return Math.atan2(this.R.col1.y, this.R.col1.x);
};
