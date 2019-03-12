/*!
 *  ======== CC26XX AESCCM ========
 *
 *  Device-specific AESCCM (Advanced Encryption Standard Module, CCM mode) configuration
 *  support.
 */
"use strict";
/* global exports, system */

/* $super is used to call generic module's methods */
let $super;

/* get Common /ti/drivers utility functions */
let Common = system.getScript("/ti/drivers/Common.js");

/*
 *  ======== devSpecific ========
 *  Device-specific extensions to be added to base AESCCM configuration
 */
let devSpecific = {
    templates : {
        boardc: "/ti/drivers/aesccm/AESCCMCC26XX.Board.c.xdt",
    }
}

/*!
 * The base object is passed to this function and allows the device
 * specific object to incorporate properties from the base object.
 *
 * @param base - the exports from the base object
 *
 * @returns - a composite object of the base + device specific modules
 */
function extend(base) {
    return Object.assign({}, base, devSpecific);
}

/* Export device-specific extensions to base exports */
exports = { extend : extend };
