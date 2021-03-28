/*
	Cat laser tower - original idea from http://www.lafabriquediy.com/tutoriel/tour-laser-pour-chat-259/
	Created by Anton Pedersen 2017-07-23
*/

#include <Servo.h>
#include <math.h>

// Instantiating two servos
Servo xServo;
Servo yServo;


// Variables
int startX = 90; // Start degree left to right, 90 degrees is center
int startY = 20; // Start degree near to far, 0 is nearest

int minX = 70;	 // Left bottom corner of triangle area
int maxX = 110;  // Right bottom corner of triangle area
int minY = 0; 	 // Base of triangle area
int maxY = 45; 	 // Top of triangle area

int movementTime = 1000; // Total movement time (ms)
int movementSteps = 10;	 // Number of steps to move during movement_time


void attachServos() {
	xServo.attach(6); // attaches the x servo on pin 6 to the servo object
	yServo.attach(9); // attaches the y servo on pin 9 to the servo object
}

void detachServos() {
	xServo.detach();
	yServo.detach();
}


// Helper function
float sign (Point p1, Point p2, Point p3) {
    return (p1.x - p3.x) * (p2.y - p3.y) - (p2.x - p3.x) * (p1.y - p3.y);
}

// Helper function
bool pointInTriangle (Point pt, Point v1, Point v2, Point v3) {
    float d1, d2, d3;
    bool has_neg, has_pos;

    d1 = sign(pt, v1, v2);
    d2 = sign(pt, v2, v3);
    d3 = sign(pt, v3, v1);

    has_neg = (d1 < 0) || (d2 < 0) || (d3 < 0);
    has_pos = (d1 > 0) || (d2 > 0) || (d3 > 0);

    return !(has_neg && has_pos);
}


// Point class
class Point {
	public:
		float x, y;
		float speedX, speedY;
		float targetDistance;
		int targetX, targetY;
		bool active = false;

		// Constructor
		Point(int initX, int initY) {
			x = initX;
			y = initY;
		}

	// Update a targetPosition to a new point inside the defined triangle area
	void updateValidTargetPosition() {
		Point targetPoint;
		bool isInTriangle = false;

		while ( !isInTriangle ) {
			targetPoint.x = random(minX, (maxX + 1));
			targetPoint.y = random(minY, (maxY + 1));
			isInTriangle = pointInTriangle(targetPoint, p1, p2, p3);
		}

		targetX = targetPoint.x;
		targetY = targetPoint.y;
	}

	// Update distance between current point and target point
	void updateCurrentToTargetDistance() {
    	targetDistance = sqrt(pow(x - targetX, 2) + pow(y - targetY, 2) * 1.0);
	}

	// Update distance per movement itteration
	void updateSpeed() {
		speedX = (targetX - x) / movementSteps;
		speedY = (targetY - y) / movementSteps;
	}

	// The further the distance between current and target point, increase delay exponentially
	float getIdleTimer() {
		float factor = 1 + (3 * pow((targetDistance/100), 5));
		float timer = 1000 * factor;
		if ( timer < 10000 ) {
			return timer;
		}
		return 10000;
	}

	void deactivate() {
		active = false;
		detachServos();
	}

	// Initialize continous movment
	void activate() {
		active = true;
		attachServos();
		updateValidTargetPosition();
		updateCurrentToTargetDistance();
		updateSpeed();
		move(1);
	}

	// Move towards target location
	void move(int i) {
		x = x + speedX;
		y = y + speedY;

		// Move servo
		xServo.write(x);
		yServo.write(y);

		if ( i < movementSteps ) {
			delay( movementTime/movementSteps );
			move(i + 1);

		} else {
			idle();
		}
	}

	// Set to idle
	void idle() {

		// Detach to remove buzzing sound from the servo
		detachServos();

		delay( getIdleTimer() );
		if ( active ) {
			activate();
		}
	}
};


// Create corners of triangle area
Point p1(minX, minY);
Point p2((minX+((maxX - minX)/2)), maxY);
Point p3(maxX, minY);

// Create point for laser pointer
Point laserPoint(startX, startY);


// Runs on start
void setup() {
	
	// Initialize serial communication at 9600 bits per second:
	Serial.begin(9600);

	attachServos();

	// Start laser
	pinMode (13, OUTPUT);
	digitalWrite (13, HIGH); // switch on the laser
	//digitalWrite (13, LOW); // switch off the laser

	// Set servo start-pos
	xServo.write(startX);
	yServo.write(startY);
}


// Runs continuously after setup() is done
void loop() {
	if ( !laserPoint.active ) {
		laserPoint.activate();
	}
}