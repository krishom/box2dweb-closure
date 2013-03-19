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

goog.provide('box2d.dynamics.controllers.GravityController');

goog.require('box2d.common.math.Vec2');
goog.require('box2d.dynamics.controllers.Controller');

/**
 * @constructor
 * @extends {box2d.dynamics.controllers.Controller}
 */
box2d.dynamics.controllers.GravityController = function() {
    goog.base(this);
    this.G = 1;
    this.invSqr = true;
};
goog.inherits(box2d.dynamics.controllers.GravityController, box2d.dynamics.controllers.Controller);

box2d.dynamics.controllers.GravityController.prototype.step = function(step) {
    var i = null;
    var body1 = null;
    var p1 = null;
    var mass1 = 0;
    var j = null;
    var body2 = null;
    var p2 = null;
    var dx = 0;
    var dy = 0;
    var r2 = 0;
    var f = null;
    if (this.invSqr) {
        for (var body1Node = this.bodyList.getFirstNode(box2d.dynamics.BodyList.TYPES.allBodies); body1Node; body1Node = body1Node.getNextNode()) {
            var body1 = body1Node.body;
            var p1 = body1.getWorldCenter();
            var mass1 = body1.getMass();
            for (var body2Node = this.bodyList.getFirstNode(box2d.dynamics.BodyList.TYPES.allBodies); body2Node; body2Node = body2Node.getNextNode()) {
                var body2 = body2Node.body;
                if (!body1.isAwake() && !body2.isAwake()) {
                    continue;
                }
                var p2 = body2.getWorldCenter();
                var dx = p2.x - p1.x;
                var dy = p2.y - p1.y;
                var r2 = dx * dx + dy * dy;
                if (r2 < Number.MIN_VALUE) {
                    continue;
                }
                var f = box2d.common.math.Vec2.get(dx, dy);
                f.multiply(this.G / r2 / Math.sqrt(r2) * mass1 * body2.getMass());
                if (body1.isAwake()) {
                    body1.applyForce(f, p1);
                }
                f.multiply(-1);
                if (body2.isAwake()) {
                    body2.applyForce(f, p2);
                }
                box2d.common.math.Vec2.free(f);
            }
        }
    } else {
        for (var body1Node = this.bodyList.getFirstNode(box2d.dynamics.BodyList.TYPES.allBodies); body1Node; body1Node = body1Node.getNextNode()) {
            var body1 = bodyNode.body;
            var p1 = body1.getWorldCenter();
            var mass1 = body1.getMass();
            for (var body2Node = this.bodyList.getFirstNode(box2d.dynamics.BodyList.TYPES.allBodies); body2Node; body2Node = body2Node.getNextNode()) {
                var body2 = bodyNode.body;
                if (!body1.isAwake() && !body2.isAwake()) {
                    continue;
                }
                var p2 = body2.getWorldCenter();
                var dx = p2.x - p1.x;
                var dy = p2.y - p1.y;
                var r2 = dx * dx + dy * dy;
                if (r2 < Number.MIN_VALUE) {
                    continue;
                }
                var f = box2d.common.math.Vec2.get(dx, dy);
                f.multiply(this.G / r2 * mass1 * body2.getMass());
                if (body1.isAwake()) {
                    body1.applyForce(f, p1);
                }
                f.multiply(-1);
                if (body2.isAwake()) {
                    body2.applyForce(f, p2);
                }
                box2d.common.math.Vec2.free(f);
            }
        }
    }
};
