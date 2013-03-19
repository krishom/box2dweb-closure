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

goog.provide('box2d.common.Settings');

box2d.common.Settings = {};

/**
 * @param {number} friction1
 * @param {number} friction2
 */
box2d.common.Settings.mixFriction = function(friction1, friction2) {
    return Math.sqrt(friction1 * friction2);
};

/**
 * @param {number} restitution1
 * @param {number} restitution2
 */
box2d.common.Settings.mixRestitution = function(restitution1, restitution2) {
    return restitution1 > restitution2 ? restitution1 : restitution2;
};

/**
 * @param {boolean} a
 */
box2d.common.Settings.assert = function(a) {
    if (!a) {
        throw 'Assertion Failed';
    }
};

/**
 * @const
 * @type {string}
 */
box2d.common.Settings.VERSION = '2.1alpha-illandril';

/**
 * @const
 * @type {number}
 */
box2d.common.Settings.USHRT_MAX = 0x0000ffff;

/**
 * @const
 * @type {number}
 */
box2d.common.Settings.maxManifoldPoints = 2;

/**
 * @const
 * @type {number}
 */
box2d.common.Settings.aabbExtension = 0.1;

/**
 * @const
 * @type {number}
 */
box2d.common.Settings.aabbmultiplier = 2;

/**
 * @const
 * @type {number}
 */
box2d.common.Settings.polygonRadius = 2 * box2d.common.Settings.linearSlop;

/**
 * @const
 * @type {number}
 */
box2d.common.Settings.linearSlop = 0.005;

/**
 * @const
 * @type {number}
 */
box2d.common.Settings.angularSlop = 2 / 180 * Math.PI;

/**
 * @const
 * @type {number}
 */
box2d.common.Settings.toiSlop = 8 * box2d.common.Settings.linearSlop;

/**
 * @const
 * @type {number}
 */
box2d.common.Settings.maxTOIContactsPerIsland = 32;

/**
 * @const
 * @type {number}
 */
box2d.common.Settings.maxTOIJointsPerIsland = 32;

/**
 * @const
 * @type {number}
 */
box2d.common.Settings.velocityThreshold = 1;

/**
 * @const
 * @type {number}
 */
box2d.common.Settings.bmaxLinearCorrection = 0.2;

/**
 * @const
 * @type {number}
 */
box2d.common.Settings.maxAngularCorrection = 8 / 180 * Math.PI;

/**
 * @const
 * @type {number}
 */
box2d.common.Settings.maxTranslation = 2;

/**
 * @const
 * @type {number}
 */
box2d.common.Settings.maxTranslationSquared = box2d.common.Settings.maxTranslation * box2d.common.Settings.maxTranslation;

/**
 * @const
 * @type {number}
 */
box2d.common.Settings.maxRotation = 0.5 * Math.PI;

/**
 * @const
 * @type {number}
 */
box2d.common.Settings.maxRotationSquared = box2d.common.Settings.maxRotation * box2d.common.Settings.maxRotation;

/**
 * @const
 * @type {number}
 */
box2d.common.Settings.contactBaumgarte = 0.2;

/**
 * @const
 * @type {number}
 */
box2d.common.Settings.timeToSleep = 0.5;

/**
 * @const
 * @type {number}
 */
box2d.common.Settings.linearSleepTolerance = 0.01;

/**
 * @const
 * @type {number}
 */
box2d.common.Settings.linearSleepToleranceSquared = box2d.common.Settings.linearSleepTolerance * box2d.common.Settings.linearSleepTolerance;

/**
 * @const
 * @type {number}
 */
box2d.common.Settings.angularSleepTolerance = 2 / 180 * Math.PI;

/**
 * @const
 * @type {number}
 */
box2d.common.Settings.angularSleepToleranceSquared = box2d.common.Settings.angularSleepTolerance * box2d.common.Settings.angularSleepTolerance;

/**
 * @const
 * @type {number}
 */
box2d.common.Settings.MIN_VALUE_SQUARED = Number.MIN_VALUE * Number.MIN_VALUE;
