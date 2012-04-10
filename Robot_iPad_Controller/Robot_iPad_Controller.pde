import oscP5.*;        //  Load OSC P5 library
import netP5.*;        //  Load net P5 library
import processing.serial.*;    //  Load serial library

Serial arduinoPort;        //  Set arduinoPort as serial connection
OscP5 oscP5;            //  Set oscP5 as OSC connection
NetAddressList myNetAddressList = new NetAddressList();
int outgoingPort = 9000;
int incomingPort = 8000;

int redLED = 0;        //  redLED lets us know if the LED is on or off
int [] led = new int [2];    //  Array allows us to add more toggle buttons in TouchOSC
int currentPos = 0;
int desiredPos = 0;

int joystickX = 0;
int joystickY = 0;

byte lf = 10;      // ASCII linefeed 

byte HEADER_NECK = 0;
byte HEADER_WHEELS = 1;

void setup() {
  size(100,100);        // Processing screen size
  noStroke();            //  We don’t want an outline or Stroke on our graphics
    oscP5 = new OscP5(this,8000);  // Start oscP5, listening for incoming messages at port 8000
   arduinoPort = new Serial(this, Serial.list()[0], 9600);    // Set arduino to 9600 baud
//   arduinoPort.bufferUntil(); // Buffer serial data from arduino until a line-feed is read. 
}

void oscEvent(OscMessage theOscMessage) {   //  This runs whenever there is a new OSC message

    connect(theOscMessage.netAddress().address());

    String addr = theOscMessage.addrPattern();  //  Creates a string out of the OSC message
    if(addr.indexOf("/1/neck") !=-1){   // Filters out any toggle buttons
      //int i = int((addr.charAt(9) )) - 0x30;   // returns the ASCII number so convert into a real number by subtracting 0x30
      desiredPos  = int(theOscMessage.get(0).floatValue());     //  Puts button value into led[i]
    // Button values can be read by using led[0], led[1], led[2], etc.
      arduinoPort.write(HEADER_NECK);
      arduinoPort.write(byte(map(desiredPos,0,180,0,127)));
      arduinoPort.write(lf);
      println("Moving to " + desiredPos);
    }
    else if(addr.indexOf("/1/joystick") !=-1){   // Filters out any toggle buttons
      joystickX = int(theOscMessage.get(0).floatValue());
      joystickY = int(theOscMessage.get(1).floatValue());
      println("Joystick: "+joystickX +"," + joystickY);
      arduinoPort.write(HEADER_WHEELS);
      arduinoPort.write(byte(joystickX));
      arduinoPort.write(byte(joystickY));
      arduinoPort.write(lf);
    }
}

void draw() {
 background(50);        // Sets the background to a dark grey, can be 0-255

//  if(currentPos != desiredPos) {
//    String pos = str(desiredPos);
//    println("Moving to position " + pos);
//    arduinoPort.write(pos);
//    arduinoPort.write('.');
//    currentPos = desiredPos;
//  }

  redLED = int(map(desiredPos, 0, 180, 0, 255));
  fill(redLED,0,0);            // Fill rectangle with redLED amount
  ellipse(50, 50, 50, 50);    // Created an ellipse at 50 pixels from the left...
                // 50 pixels from the top and a width of 50 and height of 50 pixels
}

void serialEvent(Serial p) {
  int ir = p.read();
  oscP5.send(new OscMessage("/1/ir_value", new Object[]{"" + ir}), myNetAddressList);
}

void connect(String theIPaddress) {
  if (!myNetAddressList.contains(theIPaddress, outgoingPort)) {
       myNetAddressList.add(new NetAddress(theIPaddress, outgoingPort));
  }
}

