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

goog.provide('box2d.dynamics.controllers.TensorDampingController');

goog.require('box2d.common.math.Mat22');
goog.require('box2d.common.math.Math');
goog.require('box2d.common.math.Vec2');
goog.require('box2d.dynamics.controllers.Controller');

/**
 * @constructor
 * @extends {box2d.dynamics.controllers.Controller}
 */
box2d.dynamics.controllers.TensorDampingController = function() {
    goog.base(this);
    this.T = new box2d.common.math.Mat22();
    this.maxTimestep = 0;
};
goog.inherits(box2d.dynamics.controllers.TensorDampingController, box2d.dynamics.controllers.Controller);

/**
 * @param {number} xDamping
 * @param {number} yDamping
 */
box2d.dynamics.controllers.TensorDampingController.prototype.setAxisAligned = function(xDamping, yDamping) {
    this.T.col1.x = (-xDamping);
    this.T.col1.y = 0;
    this.T.col2.x = 0;
    this.T.col2.y = (-yDamping);
    if (xDamping > 0 || yDamping > 0) {
        this.maxTimestep = 1 / Math.max(xDamping, yDamping);
    } else {
        this.maxTimestep = 0;
    }
};

box2d.dynamics.controllers.TensorDampingController.prototype.step = function(step) {
    var timestep = step.dt;
    if (timestep <= Number.MIN_VALUE) return;
    if (timestep > this.maxTimestep && this.maxTimestep > 0) timestep = this.maxTimestep;
    for (var bodyNode = this.bodyList.getFirstNode(box2d.dynamics.BodyList.TYPES.awakeBodies); bodyNode; bodyNode = bodyNode.getNextNode()) {
        var body = bodyNode.body;
        var lV = body.getLocalVector(body.getLinearVelocity());
        var tV = box2d.common.math.Math.mulMV(this.T, lV);
        box2d.common.math.Vec2.free(lV);
        var damping = body.getWorldVector(tV);
        box2d.common.math.Vec2.free(tV);
        var newV = box2d.common.math.Vec2.get(body.getLinearVelocity().x + damping.x * timestep, body.getLinearVelocity().y + damping.y * timestep);
        box2d.common.math.Vec2.free(damping);
        body.setLinearVelocity(newV);
        box2d.common.math.Vec2.free(newV);
    }
};
