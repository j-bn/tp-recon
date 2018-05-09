// Setup context
var canvas = document.getElementById('canvas');
var ctx = canvas.getContext('2d');

// Constants
w = canvas.width;
h = canvas.height;
s = 100;
hs = s / 2;
dt = 1/50;

// Variables
t = 0;

// from https://stackoverflow.com/questions/17125632/html5-canvas-rotate-object-without-moving-coordinates
function radians(degrees) {
	return degrees*Math.PI/180;
}
function drawRotatedRect(x,y,width,height,degrees){
    // first save the untranslated/unrotated context
    ctx.save();

    ctx.beginPath();
    // move the rotation point to the center of the rect
    ctx.translate( x+width/2, y+height/2 );
    // rotate the rect
    ctx.rotate(radians(degrees));

    // draw the rect on the transformed context
    // Note: after transforming [0,0] is visually [x,y]
    //       so the rect needs to be offset accordingly when drawn
    ctx.rect( -width/2, -height/2, width,height);

    ctx.fillStyle="gold";
    ctx.fill();

    // restore the context to its untranslated/unrotated state
    ctx.restore();
}
function drawCentredRect(x,y,w,h) {
	// Draw a rectangle of size w, h around centre x, y

	ctx.strokeRect(x - w/2, y - h/2, w, h);
}

function draw() {
	ctx.clearRect(0, 0, w, h);

	// Debug text
	ctx.fillText("t = " + t + "s", 5, 12);

	a = t * 30; // degrees
	drawRotatedRect(200, 200, s, s, a);
	ab = radians(a % 90);
	wb = (Math.sin(ab) + Math.cos(ab)) * s;
	drawCentredRect(200 + hs, 200 + hs, wb, wb);

	t += dt;
}

setInterval(draw, dt * 1000);