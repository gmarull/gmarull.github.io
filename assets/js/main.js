/**
 * Utilities
 */
var utils = (function () {
    "use strict";

    var ut = {};

    /* Plotting utility */
    ut.Figure = function (options) {
        this.cvs = options.canvas;
        this.ctx = this.cvs.getContext("2d");

        /* Handle retina displays */
        this.dpr = window.devicePixelRatio || 1;

        var oldWidth = this.cvs.width;
        var oldHeight = this.cvs.height;

        this.cvs.width = oldWidth * this.dpr;
        this.cvs.height = oldHeight * this.dpr;

        this.cvs.style.width = oldWidth + 'px';
        this.cvs.style.height = oldHeight + 'px';

        this.ctx.scale(this.dpr, this.dpr);

        /* Setup limits and range */
        this.limits = options.limits;

        this.range = {
            x: this.limits.xmax - this.limits.xmin,
            y: this.limits.ymax - this.limits.ymin
        };

    };

    ut.Figure.prototype.transformContext = function () {
        this.ctx.translate((1.0 / this.dpr) * (this.cvs.width / 2),
                           (1.0 / this.dpr) * (this.cvs.height / 2));
        this.ctx.scale((1.0 / this.dpr) * this.cvs.width / this.range.x,
                       (1.0 / this.dpr) * this.cvs.height / this.range.y);
    };

    ut.Figure.prototype.parametricPlot = function (f, g, range, options) {
        var u;

        options = options || {};

        /* Clear */
        this.ctx.clearRect(0, 0, this.cvs.width, this.cvs.height);

        /* Plot */
        this.ctx.save();
        this.transformContext();

        this.ctx.beginPath();
        this.ctx.moveTo(f(range.min), g(range.min));

        for (u = range.min + range.step; u <= range.max; u += range.step) {
            this.ctx.lineTo(f(u), g(u));
        }

        this.ctx.closePath();
        this.ctx.restore();

        /* Line style, color, etc. */
        this.ctx.save();
        this.ctx.lineJoin = 'round';

        if (options.color) {
            this.ctx.strokeStyle = options.color;
        } else {
            this.ctx.strokeStyle = 'black';
        }

        if (options.lineWidth) {
            this.ctx.lineWidth = options.lineWidth;
        } else {
            this.ctx.lineWidth = 1;
        }

        this.ctx.stroke();
        this.ctx.restore();
    };

    return ut;
}());


/**
 * Plot Lissajous curve
 */
var fig = new utils.Figure({
    canvas: document.querySelector('.site-logo'),
    limits: {
        xmin: -1,
        xmax: 1,
        ymin: -1,
        ymax: 1
    }
});

var timeLast = (new Date()).getTime();
var theta = 0;

function drawLogo() {
    var time = (new Date()).getTime(),
        timeDiff = time - timeLast;

    timeLast = time;

    theta = (theta + 0.25 * (timeDiff / 1000) * (2 * Math.PI)) % (2 * Math.PI);

    fig.parametricPlot(function (u) {
        return 0.65*Math.sin(u + theta);
    }, function (u) {
        return 0.65*Math.sin(3*u + theta);
    }, {
        min: 0,
        max: 2 * Math.PI,
        step: 1 / (2 * Math.PI * 25)
    }, {
        color: 'white',
        lineWidth: 1
    });
}

var loop = setInterval(drawLogo, 1000 / 30);


/**
 * Fluidvids
 */
fluidvids.init({
  selector: ['iframe'],
  players: ['www.youtube.com', 'player.vimeo.com']
});

