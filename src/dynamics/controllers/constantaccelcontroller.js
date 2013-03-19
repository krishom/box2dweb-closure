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

goog.provide('box2d.dynamics.controllers.ConstantAccelController');

goog.require('box2d.common.math.Vec2');
goog.require('box2d.dynamics.controllers.Controller');

/**
 * @constructor
 * @extends {box2d.dynamics.controllers.Controller}
 */
box2d.dynamics.controllers.ConstantAccelController = function() {
    goog.base(this);
    this.A = box2d.common.math.Vec2.get(0, 0);
};
goog.inherits(box2d.dynamics.controllers.ConstantAccelController, box2d.dynamics.controllers.Controller);

box2d.dynamics.controllers.ConstantAccelController.prototype.step = function(step) {
    var smallA = box2d.common.math.Vec2.get(this.A.x * step.dt, this.A.y * step.dt);
    for (var bodyNode = this.bodyList.getFirstNode(box2d.dynamics.BodyList.TYPES.awakeBodies); bodyNode; bodyNode = bodyNode.getNextNode()) {
        var body = bodyNode.body;
        var oldVelocity = body.getLinearVelocity();
        var newVelocity = box2d.common.math.Vec2.get(oldVelocity.x + smallA.x, oldVelocity.y + smallA.y);
        body.setLinearVelocity(newVelocity);
        box2d.common.math.Vec2.free(newVelocity);
    }
    box2d.common.math.Vec2.free(smallA);
};
