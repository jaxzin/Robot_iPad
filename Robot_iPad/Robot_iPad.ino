#include <Wire.h>
#include <AFMotor.h>
#include <Servo.h> 
#include <math.h>

#define LEFT 0
#define RIGHT 1

#define NECK_PIN 10
#define EYES_PIN A0

#define FULL_TURN_DELAY 725

#define HEADER_NECK 0
#define HEADER_WHEELS 1

#define LF 10

AF_DCMotor motorR(2);
AF_DCMotor motorL(1);
Servo neck;

boolean autonomousMode = false;

void setup() {
  
  // turn on motor
  motorR.setSpeed(200);
  motorL.setSpeed(200);
 
  motorR.run(RELEASE);
  motorL.run(RELEASE);
  
  neck.attach(NECK_PIN);
  neck.write(90);

  serial_init();
}

void loop() {
  int eyes = analogRead(EYES_PIN);
  Serial.write(byte(map(eyes,0,1024,-128,127)));
}

void loopOld() {

  if(!autonomousMode) {
    // In this mode, the joystick controls the wheels, tilt controls the 'neck' and the Z trigger switches to autonomous mode
  
    int jx, jy, ax, ay, az, bz, bc;
    if (serial_read(&jx, &jy, &ax, &ay, &az, &bz, &bc)) {
    
      // translate the joystick into a speed
      setSpeedByJoystick(jx, jy);
    
      // translate accelerometer into neck movement
      moveNeckByTilt(ax);

      // If the operator hits the 'Z' trigger, switch to autonomous mode
      if(bz == 1) {
        autonomousMode = true;
        
        // prepare robot for autonomous mode
        // stop the wheels
        stop();
        // face 'eyes' forward
        neck.write(90);
      }
    }
    delay(100);
  
  } else {
  
    // move forward until we are close to something
    Serial.println("Full speed ahead!");
    move(FORWARD, 255);
    while(analogRead(EYES_PIN) < 500) {
      ; // just keep moving
    }
    stop();
  
    // look around for clear path
    int path = searchForClearPath();
    Serial.print("Found clear path at ");
    Serial.println(path);
    
    // if a clear path is found
      //   then turn to the clear path
    if(path > -90 && path < 0) {
      Serial.print("Turning left by ");
      Serial.println(-path);
      turn(LEFT, -path);
    } else if (path > 0 && path < 90) {
      Serial.print("Turning right by ");
      Serial.println(path);
      turn(RIGHT, path);
    } else {
      //   else turn 180
      Serial.println("Turning around.");
      turn(LEFT, 180);
    }
    
    motorR.run(RELEASE);
    motorL.run(RELEASE);
    delay(1000);
  }
}

// Translates nunchuck tilt into neck/eye position
void moveNeckByTilt(int ax) {
    int neckPos = constrain(ax, -200, 200);
    // reverse sign of neckPos because accelerometer (left/right) is reversed in relation to servo
    neckPos = -neckPos;
    neckPos = map(neckPos, -200, 200, 0, 170);
    neck.write(neckPos);
}

// Translates joystick position into wheel movement
void setSpeedByJoystick(int jx, int jy) {
  // convert x/y to polar
  double r = polarR(jx, jy);
  double theta = polarTheta(jx, jy);
//  Serial.print("r: ");
//  Serial.print(r);
//  Serial.print("\ttheta: ");
//  Serial.print(theta);
  
  
  // convert polar into left/right wheel speeds
  int left = polarToLeftWheel(r, theta);
  int right = polarToRightWheel(r, theta);
//  Serial.print("\tleft: ");
//  Serial.print(left);
//  Serial.print("\tright: ");
//  Serial.println(right);
  
  motorL.run(left == 0 ? BRAKE : left > 0 ? FORWARD : BACKWARD);
  motorR.run(right == 0? BRAKE : right > 0? FORWARD : BACKWARD);
  motorL.setSpeed(abs(left));
  motorR.setSpeed(abs(right));
}

// Utility to convert x/y cartesian coordinates into polar r value
double polarR(int jx, int jy) {
  return sqrt(jx * jx + jy * jy);
}

// Utility to convert x/y cartesian coordinates into polar theta value (radians)
double polarTheta(double jx, double jy) {
  double polarTheta = atan(jy / jx);
  // correct atan for quadrant
  //Q2 (-x, + y)
  if(jx < 0 && jy >= 0) {
    return polarTheta + PI;
  } else 
  // Q3 (-x, -y)
  if(jx < 0 && jy < 0) {
    return polarTheta + PI;
  } else 
  // Q4 (+x, -y)
  if(jx >= 0 && jy < 0) {
    return polarTheta + 2 * PI;
  }
  // otherwise its Q1
  return polarTheta;
}

// Translates the polar coordinate of the joystick into the speed of the left wheel, negative values represent backward speed
int polarToLeftWheel(double r, double theta) {
  // Strip out values close to joystick center
  if(abs(r) < 5) {
    return 0;
  }
  
  // Normalized how far the joystick traveled from center to between 0.0 and 1.0
  r = min(r, 100.0);
  // normalize r
  r = mapd(r, 0.0, 100.0, 0.0, 1.0);

  // Now convert that speed based on the direction the joystick is pointing,
  // where 0 is right, PI/2 is up, PI is left, 3*PI/2 is down
  if(theta >= 0 && theta <= PI / 2.0) {
    // from full right to up, turn the left wheel forward
    return round(r * 255); 
  } else if(theta > PI/2.0 && theta < PI) {
    // from up to full left, linearly change from forward to backward
    int dir = mapd(theta, PI/2.0, PI, -255, 255);
    return round(r * -dir);
  } else if(theta >= PI && theta <= 3.0 * PI/2.0) {
    // from full left to down, turn the left wheel backward
    return round(r * -255);
  } else if(theta > 3.0 * PI/2.0 && theta < 2 * PI) {
    // from down to right, linearly change from backward to forward
    int dir = mapd(theta, 3.0 * PI/2.0, 2 * PI, -255, 255);
    return round(r * dir);
  }
}

// Translates the polar coordinate of the joystick into the speed of the right wheel, negative values represent backward speed
int polarToRightWheel(double r, double theta) {
  // Strip out values close to joystick center
  if(abs(r) < 5) {
    return 0;
  }

  // Normalized how far the joystick traveled from center to between 0.0 and 1.0
  r = min(r, 100.0);
  // normalize r
  r = mapd(r, 0.0, 100.0, 0.0, 1.0);

  // Now convert that speed based on the direction the joystick is pointing,
  // where 0 is right, PI/2 is up, PI is left, 3*PI/2 is down
  if(theta >= 0 && theta <= PI/2) {
    // from full right to up, linearly change from backward to forward
    int dir = mapd(theta, 0, PI/2, -255, 255);
    return round(r * dir); 
  } else if(theta > PI/2 && theta < PI) {
    // from up to full left, turn the left wheel forward
    return round(r * 255);
  } else if(theta >= PI && theta <= 3*PI/2) {
    // from full left to down, linearly change from forward to backward
    int dir = mapd(theta, PI, 3*PI/2, -255, 255);
    return round(r * -dir);
  } else if(theta > 3*PI/2 && theta < 2*PI) {
    // from down to right, turn the left wheel backward
    return round(r * -255);
  }
}

// Have the 'eyes' look around for an open path
int searchForClearPath() {
  int clearestAngle = 0;
  int clearestDist = 1000;
  int neckPos;
  for(neckPos = 0; neckPos < 170; neckPos++) {
    neck.write(neckPos);
    int currentDist = analogRead(EYES_PIN);
    if(currentDist < clearestDist) {
      clearestDist = currentDist;
      clearestAngle = neckPos;
    }
    delay(10);
  }
  
  neck.write(90);
  
  if(clearestDist < 500) {
    return map(clearestAngle, 0, 180, -90, 90);
  }
  
  // No clear path found
  return 0;
}

// Move in the given direction at the given speed
void move(uint8_t direction, uint8_t speed) {
  motorR.run(direction);
  motorL.run(direction);
  motorR.setSpeed(speed);  
  motorL.setSpeed(speed); 
} 

// Move in the given direction at the given speed, but first accelerate from zero
void moveWithAccel(uint8_t direction, uint8_t speed) {
  motorR.run(direction);
  motorL.run(direction);
  accel(0, speed);
} 

// Stop both wheels from moving
void stop() {
    motorR.setSpeed(0);  
    motorL.setSpeed(0);  
//    motorR.run(RELEASE);
//    motorL.run(RELEASE);
}  

// Accelerate both wheels from the start speed to the end speed
void accel(uint8_t startSpeed, uint8_t endSpeed) {
  uint8_t i;
  for (i=startSpeed; i<endSpeed; i++) {
    motorR.setSpeed(i);  
    motorL.setSpeed(i);  
    delay(10);
  }
}

// Decelerate both wheels from the start speed to the end speed
// TODO: combine accel() and decel()
void decel(uint8_t startSpeed, uint8_t endSpeed) {
  uint8_t i;
  for (i=startSpeed; i>endSpeed; i--) {
    motorR.setSpeed(i);  
    motorL.setSpeed(i);  
    delay(10);
 }
}

// Turn the robot in the given LEFT/RIGHT direction by the approximate amount of degrees. YMMV since this was calibrated to my motors
void turn(uint8_t direction, uint8_t degrees) {
  motorR.run(direction == LEFT ? BACKWARD : FORWARD);
  motorL.run(direction == LEFT ? FORWARD : BACKWARD);

  motorR.setSpeed(128);
  motorL.setSpeed(128);

  delay(degrees/180.0f * FULL_TURN_DELAY);  

  motorR.setSpeed(0);  
  motorL.setSpeed(0);
  
}

// Utility: Double-typed version of map function
double mapd(double x, double in_min, double in_max, double out_min, double out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// ============ Serial functions ==============
void serial_init() {
  Serial.begin(9600);
}

/*
 * Reads the current values from the Serial port as 6 bytes.
 *
 * jx/y:   Joystick, ranging from -128 to 127
 * ax/y/z: Acceleration, ranging from -512 to 511
 * bz/c:   Buttons; 1 means pressed, 0 means released
 *
 * Returns 1 on success, 0 on failure.
 */
int serial_read(int *jx, int *jy,
                 int *ax, int *ay, int *az,
                 int *bz, int *bc) {
  Serial.write(byte(0)); // Request data
  Serial.write(byte(10)); // End of request
//  Serial.flush();
//  delayMicroseconds(200); // Give nunchuk time to respond.
  byte buf[8];  // Allocate a few extra bytes just in case.
  int cnt = 0;
  while (Serial.available()) {
    buf[cnt++] = Serial.read();//(Serial.read() ^ 0x17) + 0x17;
  }
  if (cnt < 6) {
    return 0;
  }
  
  *jx = 0;//buf[0] - 128;
  *jy = 0;//buf[1] - 128;
  *ax = buf[0];
  *ay = 0;
  *az = 0;
  *bz = 0;
  *bc = 0;
  return 1;
  
  /*
  byte b = buf[5];
  *ax = ((buf[2] << 2) | ((b >> 2) & 3)) - 512;
  *ay = ((buf[3] << 2) | ((b >> 4) & 3)) - 512;
  *az = ((buf[4] << 2) | ((b >> 6) & 3)) - 512;
  *bz = (b & 1) ^ 1;
  *bc = ((b >> 1) & 1) ^ 1;
  return 1;
  */
}

  byte buf[8];
  int cnt = 0;

void serialEvent() {
  while(Serial.available()) {
    byte temp = Serial.read();
    buf[cnt++] = temp;
    if(temp == LF) {
      break;
    }
  }
  if(buf[cnt-1] == LF) { 
  
    byte header = buf[0];
    switch(header) {
      case HEADER_NECK:
        moveNeckByTilt(map(buf[1],0,127,-200,200));
        break;
      case HEADER_WHEELS:
        byte x = buf[1], y = buf[2];
        setSpeedByJoystick(map(x,0,255,-128,127),map(y,0,255,-128,127));
        break;
    }
    
    cnt = 0;
  }
}
