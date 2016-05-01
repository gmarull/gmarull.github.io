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

        /* Setup limits, range, step */
        this.limits = options.limits;

        this.range = {
            x: this.limits.xmax - this.limits.xmin,
            y: this.limits.ymax - this.limits.ymin
        };

        this.step = this.range.x / options.samples;

    };

    ut.Figure.prototype.transformContext = function () {
        this.ctx.translate((1.0 / this.dpr) * (this.cvs.width / 2),
                           (1.0 / this.dpr) * (this.cvs.height / 2));
        this.ctx.scale((1.0 / this.dpr) * this.cvs.width / this.range.x,
                       (1.0 / this.dpr) * this.cvs.height / this.range.y);
    };

    ut.Figure.prototype.plot = function (f, options) {
        var x;

        options = options || {};

        /* Clear */
        this.ctx.clearRect(0, 0, this.cvs.width, this.cvs.height);

        /* Plot */
        this.ctx.save();
        this.transformContext();

        this.ctx.beginPath();
        this.ctx.moveTo(this.limits.xmin, f(this.limits.xmin));

        for (x = this.limits.xmin + this.step; x <= this.limits.xmax; x += this.step) {
            this.ctx.lineTo(x, f(x));
        }

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
 * Plot logo
 */
var logo = document.querySelector('.site-logo');
var timeLast = (new Date()).getTime();
var theta = 0;
var noise = 0;

var fig = new utils.Figure({
    canvas: logo,
    limits: {
        xmin: -1.25 * Math.PI,
        xmax:  1.25 * Math.PI,
        ymin: -1,
        ymax:  1
    },
    samples: 25
});

function drawLogo() {
    var time = (new Date()).getTime(),
        timeDiff = time - timeLast;

    timeLast = time;

    theta = (theta + 0.25 * (timeDiff / 1000) * (2 * Math.PI)) % (2 * Math.PI);

    fig.plot(function (u) {
        return 0.5 * Math.sin(u + theta) + noise * 0.25 * (Math.random() - 0.5);
    }, {
        color: '#fff',
        lineWidth: 2
    });
}

var loop = setInterval(drawLogo, 1000 / 30);

/* noise effect */
function noiseCallback() {
    if (noise) {
        noise = 0;
    } else {
        noise = 1;
    }

    setTimeout(noiseCallback, 5000 - noise * 4000 + 1000 * (Math.random() - 0.5));
}

setTimeout(noiseCallback, 5000);

/**
 * Fluidvids
 */
fluidvids.init({
  selector: ['iframe'],
  players: ['www.youtube.com', 'player.vimeo.com']
});

