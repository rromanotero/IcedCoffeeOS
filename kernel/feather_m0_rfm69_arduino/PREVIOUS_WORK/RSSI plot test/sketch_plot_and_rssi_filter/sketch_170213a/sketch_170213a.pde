import processing.serial.*;

Serial myPort;        // The serial port
int xPos = 1;         // horizontal position of the graph
float inByte = 0;

void setup () {
  // set the window size:
  size(800, 600);

  // List all the available serial ports
  // if using Processing 2.1 or later, use Serial.printArray()
  println(Serial.list());

  // I know that the first port in the serial list on my mac
  // is always my  Arduino, so I open Serial.list()[0].
  // Open whatever port is the one you're using.
  myPort = new Serial(this, Serial.list()[0], 9600);

  // don't generate a serialEvent() unless you get a newline character:
  myPort.bufferUntil('\n');

  // set inital background:
  background(0);
}
void draw () {
  // draw the line:
  stroke(127, 34, 255);
  line(xPos, height, xPos, height - inByte);

  // at the edge of the screen, go back to the beginning:
  if (xPos >= width) {
    xPos = 0;
    background(0);
  } else {
    // increment the horizontal position:
    xPos++;
  }
}


void serialEvent (Serial myPort) {
  // get the ASCII string:
  String inString = myPort.readStringUntil('\n');

  if (inString != null) {
    // trim off any whitespace:
    inString = trim(inString);
    // convert to an int and map to the screen height:
    inByte = myfilter(int(inString))*10;
    
    println(inByte);
    inByte = map(inByte, 0, 1023, 0, height);
  }
}

static final int MAX_DISTANCE = 5; //5 cm

  static final int NOT_MOVING = 0;
  static final int DIR_FAR = 1;
  static final int DIR_CLOSE = 2;

int raw_prev=0;
int d_prev=0;
int d_curr=0;

int counter = -1;

int myfilter(int raw_curr){
  counter++;
  
  if( counter == 0){
      //base case
      d_prev = raw_curr;
      raw_prev = raw_curr;
      
      return raw_curr;
  }
  
  
  int rate;
  int direction = NOT_MOVING;
  
  //calculates slope i
  rate = abs(raw_curr - raw_prev);
  
  //removes pikes
  rate = min( MAX_DISTANCE, rate);
 
  if( raw_curr - d_prev > 0 ) direction = DIR_FAR;
  else if( raw_curr - d_prev < 0 ) direction = DIR_CLOSE;
  else direction = NOT_MOVING;
  
  //calculates new position i
  if( direction == DIR_FAR ) 
     d_curr = d_prev + rate; 
  else if( direction == DIR_CLOSE )
     d_curr = d_prev - rate; 
  else 
     d_curr = d_prev;  //no moving
     
   //update previous
   raw_prev = raw_curr;
   d_prev = d_curr;
   
   return d_curr;
}