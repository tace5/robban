#define TURNING_SPEED 52 
#define TURNING_ROBOT_DIAMETER 11
#define TURNING_WHEEL_DIAMETER 5.6
#define TURNING_360 (360*(TURNING_ROBOT_DIAMETER/TURNING_WHEEL_DIAMETER))
#define TURNING_90 TURNING_360/4
#define MOVE_100CM (360*5.68410511042)
#define MS_15 15

void WallMan(byte engine, byte sensor);
void Movement_Distance(byte engine, byte sensor, int distance); 
void Movement_Rotate(byte engine, int direction, int angle);
int Sensor_GetClosestObject(byte engine, byte sensor);

/*
  Rotate engines until distance away from object straight infront of robot.

  @Parameters
    byte[] engine   Engine(s) to rotate.
    byte[] sensor   Sensor to read.
    int    distance Distance to object.
  @return void
*/
void Movement_Distance(byte engine, byte sensor, int distance) {
  RotateMotorEx(engine, 70, 360*((SensorUS(sensor)-distance)/(5.6*PI)),
                0, false, true);
  Wait(MS_20);
}

/*
  Rotate engines until robot has rotated angle amount.
  NOTE: NO ERROR CHECKING FOR DIRECTION PARAMETER, CAN LEAD TO UNDEFINED
        BEHAVIOUR.

  @Parameters
    byte[] engine    Engine(s) to rotate.
    int    direction Direction to rotate, either 1 or -1.
    int    angle     Degrees to rotate.
  @return void
*/
void Movement_Rotate(byte engine, int direction, int angle) {
  RotateMotorEx(engine, TURNING_SPEED, angle, -100*direction, true, true);
  Wait(MS_20);
}

/*
  Rotate until nearest object is found, then face it.

  @Parameters
    byte[]    engine    Engine(s) to rotate.
    byte[]    sensor    Sensor to read.
  @return void
*/
void WallMan(byte engine, byte sensor) {
  int closest_point = Sensor_GetClosestObject(engine, sensor);
  int distance = 255;
  int last_distance = 255;

  OnFwd(engine, TURNING_SPEED);
  while (true) {
    distance = SensorUS(sensor);
    if (distance < last_distance && MotorPower(engine) > 35) {
      OnFwd(engine, MotorPower(engine) - 3); 
    } else if (distance > last_distance &&
               MotorPower(engine) <= TURNING_SPEED) {
      OnFwd(engine, MotorPower(engine) + 3);
    }
    if (distance <= closest_point) {
      Off(engine);
      break;
    }
    last_distance = distance;
    Wait(MS_15);
  }
  Wait(MS_20);
}

/*
  Rotate 1.25 rotations return closest object. 

  @Parameters
    byte[]    engine    Engine(s) to rotate.
    byte[]    sensor    Sensor to read.
  @return
    int closest_object  Closest object. 
*/
int Sensor_GetClosestObject(byte engine, byte sensor) {
  int closest_object = (SensorUS(sensor) == 0) ? 255 : SensorUS(S1);
  ResetRotationCount(engine);
  Wait(MS_20);
  OnFwd(engine, TURNING_SPEED);
  while (MotorRotationCount(engine) <= TURNING_360*3) {
    if (SensorUS(sensor) < closest_object) {
      closest_object = SensorUS(sensor); 
    }
    Wait(MS_15);
  }
  Off(engine);
  return closest_object;
}
