/*
	Cat laser tower - original idea from http://www.lafabriquediy.com/tutoriel/tour-laser-pour-chat-259/
	Created by Anton Pedersen 2017-07-23
*/

#include <Servo.h>
#include <math.h>

struct Point {
	int x;
	int y;
};

bool debug = false;

// Variables
byte laserPin = 13;
byte xServoPin = 6;
byte yServoPin = 9;
int minX = 70;	 // Left bottom corner of triangle area
int maxX = 110;  // Right bottom corner of triangle area
int minY = 0; 	 // Base of triangle area
int maxY = 45; 	 // Top of triangle area

int startX = (minX+((maxX - minX)/2));
int startY = maxY/2;

int movementTime = 1000; // Total movement time (ms)
int movementSteps = 10;	 // Number of steps to move during movement_time

// Create corners of triangle area
Point pointA = { minX, minY };
Point pointB = { startX, maxY};
Point pointC = { maxX, minY };


// Laser helper class
class Servos {
	private: 
		Servo xServo;
		Servo yServo;
		byte lPin;
		byte xPin;
		byte yPin;

	public:
		Servos(byte p1, byte p2, byte p3 ) {
			lPin = p1;
			xPin = p2;
			yPin = p3;

			// Attach laser
			pinMode(lPin, OUTPUT);
		}

		void activateLaser() {
			digitalWrite(lPin, HIGH);
		}

		void deactivateLaser() {
			digitalWrite(lPin, LOW);
		}

		void attach() {
			xServo.attach(xPin);
			yServo.attach(yPin);
		}

		void detach() {
			xServo.detach();
			yServo.detach();
		}

		void pointAt(Point p) {
			xServo.write(p.x);
			yServo.write(p.y);
		}
};

// Declare laser
Point laserPosition = { startX, startY };
Servos Laser(laserPin, xServoPin, yServoPin);


// Helper function for pointInTriangle
float sign (Point p1, Point p2, Point p3) {
    return (p1.x - p3.x) * (p2.y - p3.y) - (p2.x - p3.x) * (p1.y - p3.y);
}

// Test if point is inside defined triangle area
bool pointInTriangle (Point point) {
    bool hasNeg, hasPos;
    float d1 = sign(point, pointA, pointB);
    float d2 = sign(point, pointB, pointC);
    float d3 = sign(point, pointC, pointA);

    hasNeg = (d1 < 0) || (d2 < 0) || (d3 < 0);
    hasPos = (d1 > 0) || (d2 > 0) || (d3 > 0);

    return !(hasNeg && hasPos);
}

// Generate points inside defined area until one is inside the allowed triangle
// The points pointA, pointB, pointC define the triangle
Point getValidTarget() {
	Point position;
	bool isInTriangle = false;

	while ( !isInTriangle ) {
		position.x = random(minX, (maxX + 1));
		position.y = random(minY, (maxY + 1));
		isInTriangle = pointInTriangle(position);
	}

	return position;
}


// Get distance between two points
int getDistance(Point p1, Point p2) {
	return sqrt(pow(p2.x - p1.x, 2) + pow(p2.y- p1.y, 2) * 1.0);
}

// Get the time to wait between loops
// The dealy will increase exponentially depending on point distances.
float getDelayTimer(int distance) {
	float factor = 1 + (3 * pow((distance/100), 5));
	float timer = 1000 * factor;
	if ( timer < 10000 ) {
		return timer;
	}
	return 10000;
}


// Runs on start
void setup() {

	// Initialize serial communication at 9600 bits per second:
	Serial.begin(9600);

	// Setup laser
	Laser.attach();
	Laser.activateLaser();
	Laser.pointAt(laserPosition);
}

// Runs continuously after setup() is done
void loop() {

	if ( debug ) {

		Serial.println("Debug");

	} else { 
		Laser.attach();

		// Get a new valid target for the laser
		Point newTarget = getValidTarget();
		Serial.print("Target acquired: x:");
		Serial.print(newTarget.x);
		Serial.print(", y: ");
		Serial.print(newTarget.y);
		Serial.println();

		// Get distance between new and current target
		int distance = getDistance(laserPosition, newTarget);

		// Move the laser a small step at a time
		int xStep = (newTarget.x - laserPosition.x) / movementSteps;
		int yStep = (newTarget.y - laserPosition.y) / movementSteps;
		Point laserTarget = { laserPosition.x, laserPosition.y };
		for (int i = 0; i < movementSteps; i++) {
			
			laserTarget.x = laserTarget.x + xStep;
			laserTarget.y = laserTarget.y + yStep;

			Laser.pointAt(laserTarget);

			delay( movementTime / movementSteps );
		}

		// Update the laser position for next loop
		laserPosition = newTarget;

		// Idle until next loop
		Laser.detach(); // Detach to prevent flickering
		float time = getDelayTimer(distance);
		Serial.print("Idle for: ");
		Serial.print(time);
		Serial.print("ms");
		Serial.println();
		delay(time);
	}
}