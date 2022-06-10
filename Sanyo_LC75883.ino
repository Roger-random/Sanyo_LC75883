/*
 *   Interfacing with a Toyota 86120-08010 tape deck faceplate which primarily
 *   centers around the Sanyo LC75853N LCD driver.
 */
#include <Encoder.h>

#define MSG_OUT_COUNT 3
#define MSG_OUT_BYTES 9
#define SEG_PER_MSG 57

#define MSG_IN_BYTES 4

#define ENCODER_STEPS_PER_DETENT 2

#define ANIMATION_FRAMES 12

// For best performance Encoder prefers interrupt pins.
// https://www.pjrc.com/teensy/td_libs_Encoder.html
// On an Arduino Nano that means pins 2 and 3. Adjust as needed for other hardware.
// https://www.arduino.cc/reference/en/language/functions/external-interrupts/attachinterrupt/
int pinEncoderA = 2;
int pinEncoderB = 3;

// Arduino digital pins to use for Sanyo CCB communication
int pinDataIn = 4;
int pinDataOut = 5;
int pinClock = 6;
int pinEnable = 7;

// Button that grounds this pin when pushed
int pinPowerButton = 9;
long lastPowerPress;

// LED output pin
int pinLED = 13;

// Logic analyzer trace of communication indicates values are held
// for 2.7us. However, according to documentation Arduino can only
// guarantee down to 3us.
// https://www.arduino.cc/reference/en/language/functions/time/delaymicroseconds/
int usHold = 3;
void hold()
{
  delayMicroseconds(usHold);
}

Encoder audioModeEncoder(pinEncoderA, pinEncoderB);
long audioModePosition;
long audioModePositionOffset;
long lastModePress;

uint8_t msgIn[MSG_IN_BYTES];
uint8_t msgOut[MSG_OUT_COUNT][MSG_OUT_BYTES];

uint8_t msgAllOn[MSG_OUT_COUNT][MSG_OUT_BYTES];

// Bit patterns that toggle one of 16 groups of segments, arranged left to right.
// Used for implementing animation with the LCD
uint8_t animation[ANIMATION_FRAMES][MSG_OUT_COUNT][MSG_OUT_BYTES] = {
  {
    { 0x08, 0x11, 0x20, 0x08, 0x00, 0x04, 0x40, 0x00, 0x00},
    { 0x18, 0x02, 0x80, 0x00, 0x10, 0x00, 0x41, 0x00, 0x80},
    { 0x00, 0x10, 0xDA, 0x00, 0x01, 0x02, 0x00, 0x00, 0x40}
  },
  {
    { 0x20, 0xA0, 0xFF, 0x07, 0x02, 0x00, 0x01, 0x00, 0x00},
    { 0x59, 0x00, 0x10, 0x00, 0x80, 0x00, 0x02, 0x00, 0x80},
    { 0x40, 0x80, 0x00, 0x20, 0x40, 0x00, 0x00, 0x00, 0x40}
  },
  {
    { 0x10, 0x40, 0x80, 0xF8, 0x3F, 0x00, 0x04, 0x00, 0x00},
    { 0x1A, 0x00, 0x02, 0x08, 0x00, 0x01, 0x04, 0x00, 0x80},
    { 0x80, 0x00, 0x01, 0x10, 0x20, 0x00, 0x00, 0x00, 0x40}
  },
  {
    { 0x40, 0x00, 0x10, 0x00, 0xC8, 0xFF, 0x1F, 0x01, 0x00},
    { 0x18, 0x40, 0x00, 0x01, 0x08, 0x10, 0x00, 0x00, 0x80},
    { 0x20, 0x40, 0x00, 0x40, 0x80, 0x00, 0x00, 0x00, 0x40}
  },
  {
    { 0x82, 0x0F, 0x02, 0x20, 0x00, 0x02, 0xE0, 0x01, 0x00},
    { 0x1B, 0x08, 0x20, 0x00, 0x01, 0x08, 0x00, 0x01, 0x80},
    { 0x00, 0x08, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x40}
  },
  {
    { 0x77, 0x88, 0x00, 0x10, 0x00, 0x01, 0x08, 0x00, 0x00},
    { 0x1C, 0x01, 0x04, 0x40, 0x00, 0x04, 0x80, 0x00, 0x80},
    { 0x00, 0x04, 0x00, 0x00, 0x08, 0x03, 0x00, 0x00, 0x40}
  },
  {
    { 0x00, 0x01, 0x20, 0x08, 0x00, 0x04, 0x40, 0x00, 0x00},
    { 0xB8, 0x00, 0x08, 0x00, 0x10, 0x00, 0x41, 0x00, 0x80},
    { 0x00, 0x10, 0x00, 0xC0, 0xED, 0x02, 0x00, 0x00, 0x40}
  },
  {
    { 0x20, 0x00, 0x00, 0x02, 0x02, 0x00, 0x01, 0x00, 0x00},
    { 0x19, 0x04, 0x40, 0x00, 0x80, 0x00, 0x02, 0x00, 0x80},
    { 0x40, 0x80, 0x00, 0x30, 0x40, 0x00, 0x00, 0x00, 0x40}
  },
  {
    { 0x10, 0x00, 0x80, 0x00, 0x20, 0x00, 0x04, 0x00, 0x00},
    { 0x1A, 0x20, 0x00, 0xC2, 0xFF, 0x03, 0x04, 0x00, 0x80},
    { 0x80, 0x02, 0x01, 0x14, 0x20, 0x00, 0x00, 0x00, 0x40}
  },
  {
    { 0x40, 0x00, 0x10, 0x00, 0x08, 0x10, 0x00, 0x01, 0x00},
    { 0x18, 0x00, 0x01, 0x10, 0x08, 0xFC, 0x1F, 0x00, 0x80},
    { 0x20, 0x40, 0x00, 0x42, 0x80, 0x00, 0x00, 0x00, 0x40}
  },
  {
    { 0x02, 0x04, 0x02, 0x20, 0x00, 0x02, 0x20, 0x00, 0x00},
    { 0x18, 0x80, 0x00, 0x20, 0x01, 0x08, 0xE0, 0x01, 0x80},
    { 0xFF, 0x08, 0x04, 0x01, 0x04, 0x00, 0x00, 0x00, 0x40}
  },
  {
    { 0x04, 0x88, 0x00, 0x10, 0x00, 0x01, 0x08, 0x00, 0x00},
    { 0x18, 0x10, 0x00, 0x44, 0x00, 0x04, 0x80, 0x00, 0x80},
    { 0x00, 0xFD, 0x01, 0x00, 0x08, 0x00, 0x00, 0x00, 0x40}
  }
};

// Change the control message to all segments OFF.
void msgOutReset()
{
  for(int m = 0; m < MSG_OUT_COUNT; m++)
  {
    for (int i = 0; i < MSG_OUT_BYTES; i++)
    {
      msgOut[m][i] = 0x00;
    }
  }
  msgOut[1][MSG_OUT_BYTES-1] = 0x80;
  msgOut[2][MSG_OUT_BYTES-1] = 0x40;
}

// Print control message to serial terminal
void msgOutPrint()
{
  Serial.println("{");
  for(int m = 0; m < MSG_OUT_COUNT; m++)
  {
    Serial.print("  { ");
    for (int i = 0; i < MSG_OUT_BYTES; i++)
    {
      if(msgOut[m][i]<0x10)
      {
        Serial.print("0x0");
      }
      else
      {
        Serial.print("0x");
      }
      Serial.print(msgOut[m][i],HEX);
      if (i < MSG_OUT_BYTES-1)
      {
        Serial.print(", ");
      }
    }
    if (m < MSG_OUT_COUNT-1)
    {
      Serial.println("},");
    }
    else
    {
      Serial.println("}");
    }
  }
  Serial.println("};");
}

// Print keyscan data to serial terminal
void msgInPrint()
{
  Serial.print("{ ");
  for (int i = 0; i < MSG_IN_BYTES; i++)
  {
    if(msgIn[i]<0x10)
    {
      Serial.print("0x0");
    }
    else
    {
      Serial.print("0x");
    }
    Serial.print(msgIn[i],HEX);

    if (i < MSG_IN_BYTES-1)
    {
      Serial.print(", ");
    }
  }
  Serial.println("};");
}

// Turn the selected segment on (turnOn=true) or off (turnOn=false)
// Note segment is zero-based counting and datasheet diagram starts at 1.
void setSegment(uint8_t segment, bool turnOn)
{
  segmentAccess(segment, false, turnOn);
}

// Queries state of the specified segment.
// Note segment is zero-based counting and datasheet diagram starts at 1.
bool getSegment(uint8_t segment)
{
  return segmentAccess(segment, true, false);
}

// Common helper for getSegment and setSegment
bool segmentAccess(uint8_t segment, bool getting, bool turnOn)
{
  // Calculate the bit that corresponds to the indicated segment.
  uint8_t message = 0;
  uint8_t index = 0;
  uint8_t segmentBit = 0;
  uint8_t bitPattern = 0;

  if (segment > SEG_PER_MSG*MSG_OUT_COUNT)
  {
    Serial.print("Segment out of range ");
    Serial.println(segment);
    return false;
  }

  message = segment/SEG_PER_MSG;
  segmentBit = segment%SEG_PER_MSG;
  index = segmentBit/8;
  bitPattern = 0x01 << (segmentBit%8);

  if (getting)
  {
    return 0x00 != (msgOut[message][index] & bitPattern);
  }
  else
  {
    if(turnOn)
    {
      msgOut[message][index] |= bitPattern;
    }
    else
    {
      msgOut[message][index] &= ~bitPattern;
    }
  }
}

// Couldn't use Arduino SPI library because it controls enable pin with SPI semantics,
// which is different from CCB.
// Couldn't use Arduino shiftOut() because it uses clock signal differently from CCB.
void writeByte(uint8_t dataByte)
{
  uint8_t shift = dataByte;

  // Send 8 bits, least significant bit first.
  for(uint8_t i = 0; i < 8; i++)
  {
    digitalWrite(pinClock, LOW);
    hold();
    if( shift & 0x01 ){
      digitalWrite(pinDataOut, HIGH);
    } else {
      digitalWrite(pinDataOut, LOW);
    }
    shift = shift >> 1;
    hold();
    digitalWrite(pinClock, HIGH);
    hold();
  }
    
  return;
}

// Couldn't use Arduino SPI library because it controls enable pin with SPI semantics,
// which is different from CCB.
// Couldn't use Arduino shiftOut() because it uses clock signal differently from CCB.
uint8_t readByte()
{
  uint8_t returnValue = 0;

  // Read 8 bits, least significant bit first.
  for(uint8_t i = 0; i < 8; i++)
  {
    digitalWrite(pinClock, LOW);
    hold();
    if (HIGH == digitalRead(pinDataIn))
    {
      returnValue |= 0x80;
    }
    else
    {
      returnValue &= 0x7F;
    }
    digitalWrite(pinClock, HIGH);
    hold();

    if (i < 7)
    {
      returnValue = returnValue >> 1;
    }
  }

  return returnValue;
}


// Drawing mode
long cursorPosition;
bool cursorState;
bool cursorSegmentState;
unsigned long cursorNextToggle;
bool audioModePress;

// Animation mode
uint8_t animationState;
int8_t animationDirection;

// Mode switch button
bool powerPress;

uint8_t currentMode;

void setup() {
  pinMode(pinDataOut, OUTPUT);
  pinMode(pinDataIn, INPUT);
  pinMode(pinClock, OUTPUT);
  pinMode(pinEnable, OUTPUT);

  pinMode(pinLED, OUTPUT);

  pinMode(pinPowerButton, INPUT_PULLUP);

  // Clock is active low, so initialize to high.
  digitalWrite(pinClock, HIGH);

  // Enable is active high, so initialize to low.
  digitalWrite(pinEnable, LOW);

  // DataOut initial value shouldn't matter.
  // DataIn is input and not our business to initialize.

  Serial.begin(115200);
  Serial.println("Honda CD Player Faceplate Test");

  audioModePosition = audioModeEncoder.read();
  audioModePositionOffset = 0;

  // Rather than hard-coding a set of values to represent state with all segments
  // on, programmatically create it so it automatically reflects whatever structure
  // the rest of the code is using. (Created after having to update too many times
  // by hand...)
  msgOutReset();
  for(uint8_t i = 0; i < SEG_PER_MSG*MSG_OUT_COUNT; i++)
  {
    setSegment(i, true);
  }
  memcpy(msgAllOn, msgOut, MSG_OUT_COUNT*MSG_OUT_BYTES);

  // Reset the output message again after creating allOn
  msgOutReset();

  lastPowerPress = millis();
  cursorNextToggle = millis();
  lastModePress = millis();

  cursorState = true;
  audioModePress = false;

  animationState = 0;
  animationDirection = 1;

  powerPress = false;
  currentMode = 0;
}

void cursorSegmentUpdate()
{
  setSegment(cursorPosition, cursorState);
  if (cursorState == cursorSegmentState)
  {
    cursorNextToggle = millis() + 500;
  }
  else
  {
    cursorNextToggle = millis() + 200;
  }
}

void copyFrame(uint8_t frameIndex)
{
  for(int m = 0; m < MSG_OUT_COUNT; m++)
  {
    for (int i = 0; i < MSG_OUT_BYTES; i++)
    {
      msgOut[m][i] |= animation[frameIndex][m][i];
    }
  }
}

void loop() {
  long newPos = audioModeEncoder.read() + audioModePositionOffset;
  if (newPos < 0)
  {
    // Segment min reached but knob kept moving down. Adjust offset so
    // the first turn back will immediately start moving up.
    audioModePositionOffset = -audioModeEncoder.read();
    newPos = 0;
  }
  else if (newPos >= SEG_PER_MSG*MSG_OUT_COUNT*ENCODER_STEPS_PER_DETENT )
  {
    // Segment max reached but knob kept moving up. Adjust offset so
    // the first turn back will immediately start moving down.
    audioModePositionOffset = SEG_PER_MSG*MSG_OUT_COUNT*ENCODER_STEPS_PER_DETENT-1 - audioModeEncoder.read();
    newPos = SEG_PER_MSG*MSG_OUT_COUNT*ENCODER_STEPS_PER_DETENT-1;
  }

  if (currentMode == 0)
  {
    // Drawing mode: see if the audio mode knob has been turned.
    if (newPos != audioModePosition)
    {
      // Knob moved. Leave old segment in the correct state and
      // get the state of the new segment.
      setSegment(cursorPosition,cursorSegmentState);
      cursorPosition = newPos/ENCODER_STEPS_PER_DETENT;
      cursorSegmentState = getSegment(cursorPosition);

      cursorState = true;
      cursorSegmentUpdate();

      audioModePosition = newPos;
      Serial.print("New position: ");
      Serial.println(cursorPosition);
    }
    else if (cursorNextToggle < millis())
    {
      // Knob was not moved, see if we need to blink our cursor.
      cursorState = !cursorState;
      cursorSegmentUpdate();
    }
  }
  else if (currentMode == 1)
  {
    uint8_t animationNext = (newPos/ENCODER_STEPS_PER_DETENT) % ANIMATION_FRAMES;

    if (animationNext != animationState)
    {
      msgOutReset();
      copyFrame(animationNext);

      animationState = animationNext;
    }
  }
  else if (currentMode == 2)
  {
    // Copy the byte pattern created in setup()
    memcpy(msgOut, msgAllOn, MSG_OUT_COUNT*MSG_OUT_BYTES);
  }
  else
  {
    Serial.println("Current mode has fallen out of bounds. Resetting to zero");
    currentMode = 0;
  }

  if (LOW == digitalRead(pinPowerButton))
  {
    if (millis() > lastPowerPress + 500)
    {
      powerPress = true;
    }
    lastPowerPress = millis();
  }

  if (HIGH == digitalRead(pinDataIn))
  {
    if (audioModePress)
    {
      // Switch mode
      audioModePress = false;
      currentMode = currentMode+1;
      if (currentMode >= 3)
      {
        currentMode = 0;
      }
      msgOutReset();
      cursorSegmentState = getSegment(cursorPosition);
      Serial.print("Mode ");
      Serial.println(currentMode);

      if (currentMode == 1)
      {
        copyFrame(animationState);
      }
    }

    if (0 == currentMode)
    {
      // Drawing mode
      if (powerPress)
      {
        // Power button pressed, toggle current segment state.
        cursorSegmentState = !cursorSegmentState;
        powerPress = false;
        cursorState = cursorSegmentState;
        setSegment(cursorPosition,cursorSegmentState);
        msgOutPrint();
        cursorNextToggle = millis() + 1000;
      }
    }

    // Draw the current canvas.
    for(int m = 0; m < MSG_OUT_COUNT; m++)
    {
      hold();
      digitalWrite(pinEnable, LOW);
      writeByte(0x42); // Sanyo LC75853N CCB address to control LCD segments
      digitalWrite(pinEnable, HIGH);
      for (int i = 0; i < MSG_OUT_BYTES; i++)
      {
        writeByte(msgOut[m][i]);
      }
      digitalWrite(pinEnable, LOW);
      hold();
    }
  }
  else
  {
    // Data In pin pulled low means there is key press data for us to read.
    hold();
    digitalWrite(pinEnable, LOW);
    writeByte(0x43); // Sanyo LC75853N CCB address to report keyscan data
    digitalWrite(pinEnable, HIGH);
    for (int i = 0; i < MSG_IN_BYTES; i++)
    {
      msgIn[i] = readByte();
    }
    digitalWrite(pinEnable, LOW);
    hold();

    if (!audioModePress && (msgIn[3]&0x10))
    {
      if (millis() > lastModePress + 500)
      {
        audioModePress=true;
        Serial.print("Mode switch");
      }
      lastModePress = millis();
    }
    else
    {
      msgInPrint();
    }
  }
}
