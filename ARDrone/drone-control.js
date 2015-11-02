/*
* Test Contol of AR-Drone using Node.js and node-ar-drone.
*
* Stephen Markham 04/03/15 
* works with nodejs -v 0.6.19
* modify commented code to work on v 0.12.0
*/

//Setup Drone
var arDrone = require('ar-drone');
var client  = arDrone.createClient();
var speed = 0.0;

//Storing previous key stroke for speed increase
var prevKey;

//Print out Controls
console.log("t = takeoff");
console.log("l = land");
console.log("'space' = hover");
console.log("w/s = forward/back");
console.log("a/d = left/right");
console.log("q/e = rotate left/right");
console.log("m/n = up/down");

//setting up input

/* ***** v 0.6.19 ****** */
//var stdin = process.openStdin();
//require('tty').setRawMode(true);
/* ********************* */

/* ***** v 0.12.0 ******  */
var stdin = process.stdin;
stdin.setRawMode( true );
/*  ********************** */

client.animateLeds('doubleMissile', 2, 2);
stdin.resume();
stdin.setEncoding( 'utf8' );
stdin.on( 'data', function( key ){
	
  //allows for speed increase if same key is pressed multiple times
  if (prevKey != key){
  	speed = 0.0;
  }
  
  // Ctrl C (to exit)
  if ( key === '\u0003' ) {
    process.exit();
  }

  //If key pressed, do action

  //Takeoff
  if ( key == 't'){
	console.log("Taking Off");
  	speed = 0.0;
  	client.takeoff();
  	client.after(7000, function(){
  		this.stop();
  		});
  }

  //Landing
  if (key == 'l'){
	 console.log("Landing");
  	speed = 0.0;
  	client.stop();
  	client.land();
  }

  //Hovers in Place
  if (key == ' '){
  	speed = 0.0;
  	client.stop();
  }

  //Goes Left
  if (key == 'a'){
  	speed+=0.05;
  	client.left(speed);
  }

  //Goes Right
  if (key == 'd'){
  	speed+=0.05;
  	client.right(speed);
  }

  //Goes Forward
  if (key == 'w'){
  	speed+=0.05;
  	client.front(speed);
  }

  //Goes Backwards
  if (key == 's'){
  	speed+=0.05;
  	client.back(speed);
  }

  //Rotates Left
  if (key == 'q'){
  	speed+=0.05;
  	client.counterClockwise(speed);
  }

  //Rotates Right
  if (key == 'e'){
  	speed+=0.05;
  	client.clockwise(speed);
  }

  //Increases Height
  if (key == 'm'){
  	speed+=0.05;
  	client.up(speed);
  }

  //Decreases Height
  if (key == 'n'){
  	speed+=0.05;
  	client.down(speed);
  }
  
  //Remembers key press so that it can be compared for speed increase.
  prevKey = key;

});



