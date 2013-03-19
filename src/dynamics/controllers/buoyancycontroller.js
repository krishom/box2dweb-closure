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

goog.provide('box2d.dynamics.controllers.BuoyancyController');

goog.require('box2d.common.Color');
goog.require('box2d.common.math.Vec2');
goog.require('box2d.dynamics.controllers.Controller');

/**
 * @constructor
 * @extends {box2d.dynamics.controllers.Controller}
 */
box2d.dynamics.controllers.BuoyancyController = function() {
    goog.base(this);

    this.normal = box2d.common.math.Vec2.get(0, -1);
    this.offset = 0;
    this.density = 0;
    this.velocity = box2d.common.math.Vec2.get(0, 0);
    this.linearDrag = 2;
    this.angularDrag = 1;
    this.useDensity = false;
    this.useWorldGravity = true;
    this.gravity = null;
};
goog.inherits(box2d.dynamics.controllers.BuoyancyController, box2d.dynamics.controllers.Controller);

box2d.dynamics.controllers.BuoyancyController.prototype.step = function(step) {
    if (this.useWorldGravity) {
        this.gravity = this.m_world.getGravity();
    }
    var areac = box2d.common.math.Vec2.get(0, 0);
    var massc = box2d.common.math.Vec2.get(0, 0);
    var sc = box2d.common.math.Vec2.get(0, 0);
    for (var bodyNode = this.bodyList.getFirstNode(box2d.dynamics.BodyList.TYPES.awakeBodies); bodyNode; bodyNode = bodyNode.getNextNode()) {
        massc.set(0, 0);
        areac.set(0, 0);
        var body = bodyNode.body;
        var area = 0.0;
        var mass = 0.0;
        for (var fixtureNode = body.getFixtureList().getFirstNode(); fixtureNode; fixtureNode = fixtureNode.getNextNode()) {
            sc.set(0, 0);
            var sarea = fixtureNode.fixture.getShape().computeSubmergedArea(this.normal, this.offset, body.getTransform(), sc);
            area += sarea;
            areac.x += sarea * sc.x;
            areac.y += sarea * sc.y;
            var shapeDensity = 0;
            if (this.useDensity) {
                shapeDensity = 1;
            } else {
                shapeDensity = 1;
            }
            mass += sarea * shapeDensity;
            massc.x += sarea * sc.x * shapeDensity;
            massc.y += sarea * sc.y * shapeDensity;
        }
        if (area < Number.MIN_VALUE) {
            continue;
        }
        areac.x /= area;
        areac.y /= area;
        massc.x /= mass;
        massc.y /= mass;
        var buoyancyForce = this.gravity.getNegative();
        buoyancyForce.multiply(this.density * area);
        body.applyForce(buoyancyForce, massc);
        box2d.common.math.Vec2.free(buoyancyForce);
        var dragForce = body.getLinearVelocityFromWorldPoint(areac);
        dragForce.subtract(this.velocity);
        dragForce.multiply((-this.linearDrag * area));
        body.applyForce(dragForce, areac);
        box2d.common.math.Vec2.free(dragForce);
        body.applyTorque((-body.getInertia() / body.getMass() * area * body.getAngularVelocity() * this.angularDrag));
    }
    box2d.common.math.Vec2.free(sc);
    box2d.common.math.Vec2.free(massc);
    box2d.common.math.Vec2.free(areac);
};

box2d.dynamics.controllers.BuoyancyController.prototype.draw = function(debugdraw) {
    var r = 1000;
    var p1 = box2d.common.math.Vec2.get(this.normal.x * this.offset + this.normal.y * r, this.normal.y * this.offset - this.normal.x * r);
    var p2 = box2d.common.math.Vec2.get(this.normal.x * this.offset - this.normal.y * r, this.normal.y * this.offset + this.normal.x * r);
    debugdraw.drawSegment(p1, p2, box2d.dynamics.controllers.BuoyancyController.color);
    box2d.common.math.Vec2.free(p1);
    box2d.common.math.Vec2.free(p2);
};

/**
 * @type {!box2d.common.Color}
 * @const
 */
box2d.dynamics.controllers.BuoyancyController.color = new box2d.common.Color(0, 0, 1);
