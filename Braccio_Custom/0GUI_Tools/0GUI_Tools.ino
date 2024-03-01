/********************************************************
 * @brief Activating the "MOVE" button by pressing
 * the joystick enables a waving motion of the arm.
 ********************************************************/
// +
/********************************************************
* FABRIK2D 4DOF example
* Creating the FABRIK object and moving the end effector in a linear motion in x, y, z coordinates.
* You can use whichever unit you want for coordinates, lengths and tolerances as long as you are consistent.
* Default unit is millimeters.
********************************************************/

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <Braccio++.h>
#include <FABRIK2D.h>

/**************************************************************************************
 * GLOBAL CONSTANTS
 **************************************************************************************/

float const home_position[6] = { SmartServoClass::MAX_ANGLE / 2.0f,
                                 SmartServoClass::MAX_ANGLE / 2.0f,
                                 SmartServoClass::MAX_ANGLE / 2.0f,
                                 SmartServoClass::MAX_ANGLE / 2.0f,
                                 SmartServoClass::MAX_ANGLE / 2.0f,
                                 90.0f };
static const char* btnm_map[] = { "Move", "\0" };
bool isDone = false;
bool isPrinting = true;

/**************************************************************************************
 * GLOBAL VARIABLES
 **************************************************************************************/

bool move_joint = false;

int lengths[] = { 200, 200, 100 };  // 3DOF arm where shoulder to elbow is 225mm, elbow to wrist is 150mm and wrist to end effector is 100mm.
Fabrik2D fabrik2D(4, lengths);      // This arm has 4 joints; one in the origin, the elbow, the wrist and the end effector.

// list of locations with x,y,z
const int amountOfLocations = 4;
const int locations[amountOfLocations][3] = {{100, 100, 100}, {200, 200, 200}, {-100, 100, 100}, {-200, 200, 200}};
int i = 0;

/**************************************************************************************
 * FUNCTIONS
 **************************************************************************************/

// a generic conversion function that takes the fabrik angle and the min and max of the braccio range
float convertToBraccio(float fabrikAngle, float braccioMin, float braccioMax) {
  float braccioRange = braccioMax - braccioMin;
  float fabrikMin = -180;
  float fabrikRange = 360;

  float braccioAngle = (((fabrikAngle - fabrikMin) / fabrikRange) * braccioRange) + braccioMin;
  return braccioAngle;
}

float convertToBraccioBase(float fabrikAngle) {
  float braccioMin = 0;    //from doc
  float braccioMax = 315;  //from doc
  return convertToBraccio(fabrikAngle, braccioMin, braccioMax);
}

float convertToBraccioShoulder(float fabrikAngle) {
  float braccioMin = 75;  //from doc
  float braccioMax = 232.23;
  return convertToBraccio(fabrikAngle, braccioMin, braccioMax);
}

float convertToBraccioElbow(float fabrikAngle) {
  float braccioMin = 47.33;
  float braccioMax = 261.84;
}

float convertToBraccioWristPitch(float fabrikAngle) {
  float braccioMin = 38.35;
  float braccioMax = 271.77;
  return convertToBraccio(fabrikAngle, braccioMin, braccioMax);
}

float convertToBraccioWristRoll(float fabrikAngle) {
  float braccioMin = 0;    //from doc
  float braccioMax = 315;  //from doc
  return convertToBraccio(fabrikAngle, braccioMin, braccioMax);
}

float convertToBraccioGripper(float fabrikAngle) {
  float braccioMin = 128.05;
  float braccioMax = 212.50;
  return convertToBraccio(fabrikAngle, braccioMin, braccioMax);
}

static void event_handler(lv_event_t* e) {
  Braccio.lvgl_lock();
  lv_event_code_t code = lv_event_get_code(e);
  lv_obj_t* obj = lv_event_get_target(e);
  if (code == LV_EVENT_CLICKED) {
    uint32_t id = lv_btnmatrix_get_selected_btn(obj);
    const char* txt = lv_btnmatrix_get_btn_text(obj, id);

    LV_LOG_USER("%s was pressed\n", txt);
    if (Serial) Serial.println(txt);

    if (strcmp(txt, "Move") == 0)
      move_joint = !move_joint;
  }
  Braccio.lvgl_unlock();
}

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

void fabrik_setup() {
  if (isPrinting) {
    Serial.print("pos");
    Serial.print("\t\t\t");
    Serial.print("angB");
    Serial.print("\t");
    Serial.print("ang0");
    Serial.print("\t");
    Serial.print("ang1");
    Serial.print("\t");
    Serial.print("ang2");
    Serial.print("\t");
    Serial.print("x0");
    Serial.print("\t");
    Serial.print("y0");
    Serial.print("\t");
    Serial.print("x1");
    Serial.print("\t");
    Serial.print("y1");
    Serial.print("\t");
    Serial.print("x2");
    Serial.print("\t");
    Serial.print("y2");
    Serial.print("\t");
    Serial.print("x3");
    Serial.print("\t");
    Serial.println("y3");
  }


  // Set tolerance to 0.5mm. If reachable, the end effector will approach
  // the target with this tolerance
  fabrik2D.setTolerance(0.5);
}

void fabrik_loop() {
  if (isDone)
    return;
  int x = locations[i][0];
  int y = locations[i][1];
  int z = locations[i][2];

  // Move from 0 to 50 in the z axis
  // if (z < 0) {
  //   toggle_z = 0;
  //   z = 0;
  // } else if (z > 50) {
  //   isDone = true;
  //   toggle_z = 1;
  //   z = 50;
  // }

  // // Move from 50 to 150 in the x axis
  // if (x < 50) {
  //   toggle_x = 0;
  //   x = 50;
  // } else if (x > 150) {
  //   toggle_x = 1;
  //   x = 150;
  // }



  // Solve inverse kinematics given the coordinates x and y, z, the desired gripping offset, tool angle and the list of lengths for the arm.
  // Note that for 3D movements, we use the solve2 method instead of solve.
  if (isPrinting)
  {
    Serial.print(x);
    Serial.print(", ");
    Serial.print(y);
    Serial.print(", ");
    Serial.print(z);
    Serial.print("\t");
  }
  if (fabrik2D.solve2(x, y, z, -M_PI / 2.5, 0, lengths)) {
    // Angles are printed in degrees.
    // The function calls below shows how easy it is to get the results from the inverse kinematics solution.

    float base = convertToBraccioBase(fabrik2D.getBaseAngle());
    float shoulder = convertToBraccioShoulder(fabrik2D.getAngle(0) * 57296 / 1000);
    float elbow = convertToBraccioElbow(fabrik2D.getAngle(1) * 57296 / 1000);
    float wristPitch = convertToBraccioWristPitch(fabrik2D.getAngle(2) * 57296 / 1000);

    if (isPrinting) {
      Serial.print(base);
      Serial.print("\t");
      Serial.print(shoulder);
      Serial.print("\t");
      Serial.print(elbow);
      Serial.print("\t");
      Serial.print(wristPitch);
      Serial.print("\t");
      Serial.print(fabrik2D.getX(0));
      Serial.print("\t");
      Serial.print(fabrik2D.getY(0));
      Serial.print("\t");
      Serial.print(fabrik2D.getX(1));
      Serial.print("\t");
      Serial.print(fabrik2D.getY(1));
      Serial.print("\t");
      Serial.print(fabrik2D.getX(2));
      Serial.print("\t");
      Serial.print(fabrik2D.getY(2));
      Serial.print("\t");
      Serial.print(fabrik2D.getX(3));
      Serial.print("\t");
      Serial.println(fabrik2D.getY(3));
    }

    Braccio.move(6).to(base);
    Braccio.move(5).to(shoulder);
    Braccio.move(4).to(elbow);
    Braccio.move(3).to(wristPitch);

  } else {
    Serial.println("Could not converge");
  }

  // if (toggle_z == 0) {
  //   z++;
  // } else {
  //   z--;
  // }

  // if (toggle_x == 0) {
  //   x++;
  // } else {
  //   x--;
  // }

  delay(2000);
  i++;
  if (i > amountOfLocations) {
    isDone = true;
  }
}

/**************************************************************************************
 * SETUP/LOOP
 **************************************************************************************/

void setup() {
  Serial.begin(115200);
  for (auto const start = millis(); !Serial && ((millis() - start) < 5000); delay(10)) {}

  if (!Braccio.begin(customMenu)) {
    if (Serial) Serial.println("Braccio.begin() failed.");
    for (;;) {}
  }

  fabrik_setup();

  Braccio.moveTo(home_position[0], home_position[1], home_position[2], home_position[3], home_position[4], home_position[5]);
  delay(1000);
  Braccio.setAngularVelocity(45.0f); /* 45 deg/sec */
}

void loop() {
  if (move_joint) {
    Braccio.move(4).to((SmartServoClass::MAX_ANGLE / 2.0f) - 45.0f);
    delay(2000);
    Braccio.move(4).to((SmartServoClass::MAX_ANGLE / 2.0f) + 45.0f);
    delay(2000);
  }

  fabrik_loop();
}
