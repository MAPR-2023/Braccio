/********************************************************
 * @brief Activating the "MOVE" button by pressing
 * the joystick enables a waving motion of the arm.
 ********************************************************/

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

// for controlling the braccio arm
#include <Braccio++.h>

// for setting up the Access Point
#include <SPI.h>
#include <WiFiNINA.h>

/**************************************************************************************
 * GLOBAL CONSTANTS
 **************************************************************************************/

// the corresponding home positions for every joint
float const home_position[6] = { SmartServoClass::MAX_ANGLE / 2.0f,
                                 SmartServoClass::MAX_ANGLE / 2.0f,
                                 SmartServoClass::MAX_ANGLE / 2.0f,
                                 SmartServoClass::MAX_ANGLE / 2.0f,
                                 SmartServoClass::MAX_ANGLE / 2.0f,
                                 90.0f };
// the button map for the reset button
static const char* btnm_map[] = { "Reset", "\0" };

// the wifi configuration settings
char ssid[] = "Braccio Arm - DO NOT CONNECT";
char pass[] = "braccioArm1234";
int keyIndex = 0;

int status = WL_IDLE_STATUS;
WiFiServer server(80);



/**************************************************************************************
 * GLOBAL VARIABLES
 **************************************************************************************/

bool resetPressed = false;

/**************************************************************************************
 * FUNCTIONS
 **************************************************************************************/

// converts 0 - 360 to the corresponding braccio angles of the base
float convertToBraccioBase(float angle) {
  float min = -180;
  float max = 180;
  float braccioRange = 315;

  float fromMinusToZero = angle - min;
  float clamped = fromMinusToZero > braccioRange ? braccioRange : fromMinusToZero;
  float reversed = braccioRange - clamped;

  return reversed;
}

// converts -90 - 90 to the corresponding braccio angles of the shoulder
float convertToBraccioShoulder(float angle) {
  float center = home_position[4];
  float fromZeroToCenter = angle + center;
  float braccioMin = 75;  //from doc
  float braccioMax = 232.23;
  float clamped = fromZeroToCenter < braccioMin ? braccioMin : fromZeroToCenter;
  clamped = clamped > braccioMax ? braccioMax : clamped;

  return clamped;
}

// converts -90 - 90 to the corresponding braccio angles of the elbow
float convertToBraccioElbow(float angle) {
  float center = home_position[3];
  float fromZeroToCenter = angle + center;
  float braccioMin = 47.33;
  float braccioMax = 261.84;
  float clamped = fromZeroToCenter < braccioMin ? braccioMin : fromZeroToCenter;
  clamped = clamped > braccioMax ? braccioMax : clamped;
  return clamped;
}

// converts -90 - 90 to the corresponding braccio angles of the wrist pitch
float convertToBraccioWristPitch(float angle) {
  float center = home_position[2];
  float fromZeroToCenter = angle + center;
  float braccioMin = 38.35;
  float braccioMax = 271.77;
  float clamped = fromZeroToCenter < braccioMin ? braccioMin : fromZeroToCenter;
  clamped = clamped > braccioMax ? braccioMax : clamped;
  return clamped;
}

// converts -90 - 90 to the corresponding braccio angles of the wrist roll
float convertToBraccioWristRoll(float angle) {
  float center = home_position[1];
  float fromZeroToCenter = angle + center;
  float braccioMin = 0;    //from doc
  float braccioMax = 315;  //from doc
  float clamped = fromZeroToCenter < braccioMin ? braccioMin : fromZeroToCenter;
  clamped = clamped > braccioMax ? braccioMax : clamped;
  return clamped;
}

// converts -90 - 90 to the corresponding braccio angles of the gripper
float convertToBraccioGripper(float angle) {
  float center = home_position[0];
  float fromZeroToCenter = angle + center;
  float braccioMin = 128.05;
  float braccioMax = 212.50;
  float clamped = fromZeroToCenter < braccioMin ? braccioMin : fromZeroToCenter;
  clamped = clamped > braccioMax ? braccioMax : clamped;
  return clamped;
}

// the event handler for the reset button on the display
static void event_handler(lv_event_t* e) {
  Braccio.lvgl_lock();
  lv_event_code_t code = lv_event_get_code(e);
  lv_obj_t* obj = lv_event_get_target(e);
  if (code == LV_EVENT_CLICKED) {
    uint32_t id = lv_btnmatrix_get_selected_btn(obj);
    const char* txt = lv_btnmatrix_get_btn_text(obj, id);

    LV_LOG_USER("%s was pressed\n", txt);
    if (Serial) Serial.println(txt);

    if (strcmp(txt, "Reset") == 0)
      resetPressed = !resetPressed;
  }
  Braccio.lvgl_unlock();
}

// the custom menu which has a reset button controllable by buttons
void customMenu() {
  Braccio.lvgl_lock();
  lv_obj_t* btnm1 = lv_btnmatrix_create(lv_scr_act());
  lv_btnmatrix_set_map(btnm1, btnm_map);
  lv_btnmatrix_set_btn_ctrl(btnm1, 0, LV_BTNMATRIX_CTRL_CHECKABLE);
  lv_obj_align(btnm1, LV_ALIGN_CENTER, 0, 0);
  lv_obj_add_event_cb(btnm1, event_handler, LV_EVENT_ALL, NULL);
  Braccio.lvgl_unlock();

  Braccio.connectJoystickTo(btnm1);
}

// sets the arm back to home position (pointing straight up)
void reset() {
  Braccio.moveTo(home_position[0], home_position[1], home_position[2], home_position[3], home_position[4], home_position[5]);
  printAngles(home_position[5], home_position[4], home_position[3], home_position[2]);
}

// prints every joint angle (base, shoulder, elbow, wrist pitch)
void printAngles(float baseRotation, float shoulderRotation, float elbowRotation, float wristRotation) {
  Serial.print("Base: ");
  Serial.print(baseRotation);
  Serial.print(" Shoulder: ");
  Serial.print(shoulderRotation);
  Serial.print(" Elbow: ");
  Serial.print(elbowRotation);
  Serial.print(" Wrist: ");
  Serial.println(wristRotation);
}

// retrieves the degrees given with a joint letter, if no joint is found -1 is returned
int extractValue(char prefix, String data) {
  int valueIndex = data.indexOf(prefix) + 1;  // Skip the prefix character
  if (valueIndex > 0) {
    return data.substring(valueIndex).toInt();
  } else {
    return -1;  // Indicate error if prefix not found
  }
}

/**
 * Retrieves the command from the given angles data and does the corresponding command
 * The format is the letter of the joint you want to rotate with the corresponding degrees behind it (no seperation)
 * The degrees can range from -90 to 90 for every joint except the base which can rotate between -180 and 180
 * B = base, S = shoulder, E = elbow, W = wrist pitch
 * Ex: 'B-180 S10 E40' will rotate the base to 90 degrees, the shoulder to 10 degrees and the elbow to 40 degrees
 * not all joints need to be specified at once
 * R stands for reset which resets the position of every joint to the home position (pointing straight up)
 * A stands for the angular velocity which can be manipulated and goes into effect after a reset (degrees/sec) 
 * Ex: 'A30' = slow, 'A45' = medium, 'A90' = fast, 'A180' = very fast (!warning!)
 */
void getCommandsFrom(String anglesData) {
  // Extract individual values
  int baseRotation = extractValue('B', anglesData);
  int shoulderRotation = extractValue('S', anglesData);
  int elbowRotation = extractValue('E', anglesData);
  int wristRotation = extractValue('W', anglesData);

  int resetValue = extractValue('R', anglesData);
  int angularVelocity = extractValue('A', anglesData);

  if (angularVelocity != -1) {
    Serial.print("setting angular velocity to:");
    Serial.println(angularVelocity);
    Braccio.setAngularVelocity(angularVelocity);
  }

  if (resetValue == -1) {
    // Process the extracted values
    float b = baseRotation == -1 ? -1 : convertToBraccioBase(baseRotation);
    float s = shoulderRotation == -1 ? -1 : convertToBraccioShoulder(shoulderRotation);
    float e = elbowRotation == -1 ? -1 : convertToBraccioElbow(elbowRotation);
    float w = wristRotation == -1 ? -1 : convertToBraccioWristPitch(wristRotation);
    if (baseRotation != -1 && shoulderRotation != -1 && elbowRotation != -1 && wristRotation != -1) {
      Braccio.moveTo(-1, -1, w, e, s, b);
      Serial.println("rotate all");
    } else {
      Braccio.move(6).to(b);
      Braccio.move(5).to(s);
      Braccio.move(4).to(e);
      Braccio.move(3).to(w);
      Serial.println("rotate some");
    }
    //printAngles(b, s, e, w);
  } else {
    reset();
  }
}

/** 
 * Sets up the Access Point so the braccio arm is easily controllable from every device
 */
void setupAP() {

  // check for the WiFi module:
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("Communication with WiFi module failed!");
    // don't continue
    while (true)
      ;
  }

  String fv = WiFi.firmwareVersion();
  if (fv < WIFI_FIRMWARE_LATEST_VERSION) {
    Serial.println("Please upgrade the firmware");
  }

  // by default the local IP address will be 192.168.4.1
  // you can override it with the following:
  WiFi.config(IPAddress(10, 0, 0, 1));

  // print the network name (SSID);
  Serial.print("Creating access point named: ");
  Serial.println(ssid);

  // Create open network. Change this line if you want to create an WEP network:
  status = WiFi.beginAP(ssid, pass);
  if (status != WL_AP_LISTENING) {
    Serial.println("Creating access point failed");
    // don't continue
    while (true)
      ;
  }

  // wait 10 seconds for connection:
  delay(10000);

  // start the web server on port 80
  server.begin();

  // you're connected now, so print out the status
  printWiFiStatus();
}

void printWiFiStatus() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your WiFi shield's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print where to go in a browser:
  Serial.print("To see this page in action, open a browser to http://");
  Serial.println(ip);
}

String loopWiFi() {
  // compare the previous status to the current status
  if (status != WiFi.status()) {
    // it has changed update the variable
    status = WiFi.status();

    if (status == WL_AP_CONNECTED) {
      // a device has connected to the AP
      Serial.println("Device connected to AP");
    } else {
      // a device has disconnected from the AP, and we are back in listening mode
      Serial.println("Device disconnected from AP");
    }
  }

  WiFiClient client = server.available();  // listen for incoming clients
  String anglesData = "";
  bool isHTTPrequest = false;

  if (client) {                    // if you get a client,
    Serial.print("new conn ~ ");  // print a message out the serial port
    String currentLine = "";       // make a String to hold incoming data from the client
    while (client.connected()) {   // loop while the client's connected
      if (client.available()) {    // if there's bytes to read from the client,
        char c = client.read();    // read a byte, then
        //Serial.write(c);           // print it out the serial monitor

        if (c == '\n') {           // if the byte is a newline character
          if (isHTTPrequest) {
            if (currentLine.length() != 0) {
               // if you got a newline, check for commands & then clear currentLine
              String resultButtons = checkForHTMLButtons(currentLine);
              String resultURL = checkForURLCommand(currentLine);
              if (resultButtons.compareTo("") != 0) anglesData = resultButtons;
              if (resultURL.compareTo("") != 0) anglesData = resultURL;
              currentLine = "";
            } else { 
              // if the current line is blank, you got two newline characters in a row.
              // that's the end of the client HTTP request, so send a response:
              returnWebPageHTML(client);
              // break out of the while loop:
              break;
            }
          } else {
            // this means it isn't a HTTP request which means it's the unity application (that does not follow the HTTP rules for efficiency)
            anglesData = currentLine;
            Serial.println(anglesData);
            getCommandsFrom(anglesData);
            if (currentLine.indexOf("exit()") != -1) {
              client.stop();
              Serial.println("user ended connection ~ conn closed");
              return "";
            }
            currentLine = "";
          }
        } else if (c != '\r') {  // if you got anything else but a carriage return character,
          currentLine += c;      // add it to the end of the currentLine
        }
        if (currentLine.indexOf("HTTP") != -1) isHTTPrequest = true;
        if (currentLine.indexOf("ico") != -1) {
          client.stop();
          Serial.println("favicon ~ conn closed");
          return ""; //ignore request if for favicon
        }
      }
    }

    // close the connection:
    client.stop();
    Serial.print(anglesData); Serial.println(" ~ conn closed");
  }
  return anglesData;
}

/**
 * returns the webpage html to the client
 */
void returnWebPageHTML(WiFiClient client) {
  // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
  // and a content-type so the client knows what's coming, then a blank line:
  client.println("HTTP/1.1 200 OK");
  client.println("Content-type:text/html");
  client.println();

  // the content of the HTTP response follows the header:
  client.print("<style>");
  client.print(".container {margin: 0 auto; text-align: center; margin-top: 100px;}");
  client.print("button {color: white; width: 100px; height: 100px;");
  client.print("border-radius: 50%; margin: 20px; border: none; font-size: 20px; outline: none; transition: all 0.2s;}");
  client.print(".red{background-color: rgb(196, 39, 39);}");
  client.print(".green{background-color: rgb(39, 121, 39);}");
  client.print(".blue {background-color: rgb(5, 87, 180);}");
  client.print(".off{background-color: grey;}");
  client.print("button:hover{cursor: pointer; opacity: 0.7;}");
  client.print("</style>");
  client.print("<div class='container'>");
  client.print("<button class='red' type='submit' onmousedown='location.href=\"/R\"'>Reset</button>");
  client.print("<button class='off' type='submit' onmousedown='location.href=\"/B90\"'>B90</button><br>");
  client.print("<button class='green' type='submit' onmousedown='location.href=\"/B0\"'>B0</button>");
  client.print("<button class='off' type='submit' onmousedown='location.href=\"/Bmin180\"'>B-180</button><br>");
  client.print("<button class='blue' type='submit' onmousedown='location.href=\"/S10\"'>S10</button>");
  client.print("<button class='off' type='submit' onmousedown='location.href=\"/S90\"'>S90</button>");
  client.print("</div>");

  // The HTTP response ends with another blank line:
  client.println();
}

/**
 * checks if the get request is for a html button url
 */
String checkForHTMLButtons(String currentLine) {
  String anglesData = "";
  // Check to see if the client request was /X
  if (currentLine.indexOf("GET /R") == 0)       anglesData = "R";
  if (currentLine.indexOf("GET /B90") == 0)     anglesData = "B90";
  if (currentLine.indexOf("GET /B0") == 0)      anglesData = "B0";
  if (currentLine.indexOf("GET /Bmin180") == 0) anglesData = "B-180";
  if (currentLine.indexOf("GET /S10") == 0)     anglesData = "S10";
  if (currentLine.indexOf("GET /S90") == 0)     anglesData = "S90";
  return anglesData;
}

String checkForURLCommand(String currentLine) {
  String anglesData = "";
  if (currentLine.indexOf("/rotate?") > 0) {
    String queryParameters = currentLine.substring(currentLine.indexOf("?") + 1);
    queryParameters.replace("=", "");
    queryParameters.replace('&', ' ');
    anglesData = queryParameters;
  }
  return anglesData;
}

/**************************************************************************************
 * SETUP/LOOP
 **************************************************************************************/

void setup() {
  Serial.begin(115200);
  for (auto const start = millis(); !Serial && ((millis() - start) < 5000); delay(10)) {}

  setupAP();

  if (!Braccio.begin(customMenu)) {
    if (Serial) Serial.println("Braccio.begin() failed.");
    for (;;) {}
  }

  reset();
  Braccio.setAngularVelocity(135.0f); /* 45 deg/sec , now i put it to 90 deg/sec */
  delay(1000);
}

void loop() {
  String anglesData = "";
  anglesData = loopWiFi();
  if (Serial.available() > 0) {
    anglesData = Serial.readStringUntil('\n');  // Read until newline
  }
  if (anglesData != "") {
    getCommandsFrom(anglesData);
  }

  if (resetPressed) {
    reset();
    resetPressed = false;
    delay(2000);
  }
}
