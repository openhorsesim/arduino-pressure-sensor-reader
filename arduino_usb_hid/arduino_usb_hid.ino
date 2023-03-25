#include <Keyboard.h>
#include <Mouse.h>
#include <SoftwareSerial.h>

// Software 2nd serial port on pins 8 and 9
// RX pin has to be change interrupt capable. See:
// https://docs.arduino.cc/learn/built-in-libraries/software-serial
// (used to receive instructions for synthetic key/mouse events to generate)
SoftwareSerial mySerial = SoftwareSerial(8, 9, false);

void setup() {
  Serial.begin(9600);
  Keyboard.begin();
  Mouse.begin();
  mySerial.begin(9600);
  delay(200);

  mySerial.print("UART to USB Keyboard/Mouse Relay");
}

int hex = 0;
int digits = 0;

void loop() {
  while (mySerial.available() > 0) {
    unsigned char c;
    c = mySerial.read();
    if (c >= '0' && c <= '9' || c >= 'a' && c <= 'f') {
      // Hex digit
      hex = hex << 4;
      if (c > '9') {
        c -= 'a';
        c += 10;
      } else c -= '0';
      hex |= c;
      digits++;
    } else if (c >= 'A' && c <= 'Z') {
      // Special keys
      switch (c) {
        case 'A':  // LSHIFT
          mySerial.println("Special: LSHIFT");
          Keyboard.press(KEY_LEFT_SHIFT);
          Keyboard.flush();
          Keyboard.release(KEY_LEFT_SHIFT);
          Keyboard.flush();
          break;
        case 'Z':  // LCTRL
          mySerial.println("Special: LCTRL");
          Keyboard.press(KEY_LEFT_CTRL);
          Keyboard.flush();
          Keyboard.release(KEY_LEFT_CTRL);
          Keyboard.flush();
          break;
        case 'C': // Release all keys
          Keyboard.releaseAll();
          break;
      }
    } else if (c == '\n' || c == '\r') {
      if (digits == 3) {
        char msg[80];
        snprintf(msg, 80, "Hex = $%03x", hex);
        mySerial.println(msg);
        switch (hex >> 8) {
          case 1:  // Keyboard
            mySerial.println("Press normal key");
            {
              char s[2];
              s[0] = hex & 0xff;
              s[1] = 0;
              Keyboard.print(s);
            }
            break;
          case 2:  // Mouse X
            mySerial.println("Update mouse X");
            Mouse.move(hex & 0xff, 0, 0);
            break;
          case 3:  // Mouse Y
            mySerial.println("Update mouse Y");
            Mouse.move(0, hex & 0xff, 0);
            break;
          case 4:  // Mouse Wheel
            mySerial.println("Update mouse wheel");
            Mouse.move(0, 0, hex & 0xff);
            break;
          case 5:  // Direct scan code press
            Keyboard.press(hex & 0xff);
            break;
          case 6:  // Direct scan code release
            Keyboard.release(hex & 0xff);
            break;
        }
      }
      digits = 0;
      hex = 0;
    }
  }
}
