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

goog.provide('box2d.collision.ContactID');

goog.require('UsageTracker');

/**
 * @constructor
 */
box2d.collision.ContactID = function() {
    UsageTracker.get('box2d.collision.ContactID').trackCreate();

    /**
     * @private
     * @type {number}
     */
    this.key_ = 0;

    /**
     * @private
     * @type {number}
     */
    this.referenceEdge_ = 0;

    /**
     * @private
     * @type {number}
     */
    this.incidentEdge_ = 0;

    /**
     * @private
     * @type {number}
     */
    this.incidentVertex_ = 0;
};

/**
 * @return {number}
 */
box2d.collision.ContactID.prototype.getKey = function() {
    return this.key_;
};

/**
 * @param {number} key
 */
box2d.collision.ContactID.prototype.setKey = function(key) {
    this.key_ = key;
    this.referenceEdge_ = this.key_ & 0x000000ff;
    this.incidentEdge_ = ((this.key_ & 0x0000ff00) >> 8) & 0x000000ff;
    this.incidentVertex_ = ((this.key_ & 0x00ff0000) >> 16) & 0x000000ff;
    this._flip = ((this.key_ & 0xff000000) >> 24) & 0x000000ff;
};

/**
 * @param {!box2d.collision.ContactID} id
 */
box2d.collision.ContactID.prototype.set = function(id) {
    this.setKey(id.key_);
};

/**
 * @param {number} edge
 */
box2d.collision.ContactID.prototype.setReferenceEdge = function(edge) {
    this.referenceEdge_ = edge;
    this.key_ = (this.key_ & 0xffffff00) | (this.referenceEdge_ & 0x000000ff);
};

/**
 * @param {number} edge
 */
box2d.collision.ContactID.prototype.setIncidentEdge = function(edge) {
    this.incidentEdge_ = edge;
    this.key_ = (this.key_ & 0xffff00ff) | ((this.incidentEdge_ << 8) & 0x0000ff00);
};

/**
 * @param {number} vertex
 */
box2d.collision.ContactID.prototype.setIncidentVertex = function(vertex) {
    this.incidentVertex_ = vertex;
    this.key_ = (this.key_ & 0xff00ffff) | ((this.incidentVertex_ << 16) & 0x00ff0000);
};

/**
 * @param {number} flip
 */
box2d.collision.ContactID.prototype.setFlip = function(flip) {
    this._flip = flip;
    this.key_ = (this.key_ & 0x00ffffff) | ((this._flip << 24) & 0xff000000);
};

/**
 * @return {!box2d.collision.ContactID}
 */
box2d.collision.ContactID.prototype.copy = function() {
    var id = new box2d.collision.ContactID();
    id.set(this);
    return id;
};
