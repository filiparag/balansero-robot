enum  dimension {X, Y, Z};

double reference = 90;
double angle = 0;

double Kp = 1000;
double Kd = 20;
double Ki = 10000;

double error = 0;
double last_error = 0;
double derror = 0;
double ierror = 0;
double u = 0;

int Pin_PWM = 9;
int Pin_DIR = 8;

namespace IMU {

  char  imu_buffer[32];
  short imu_buffer_end = -1;

  struct Values {
    float X, Y, Z;
  } Values;

  void parse_buffer () {

    imu_buffer_end -= 3;
    
    short comma1 = -1,
          comma2 = -1;

    for (short it = 0; it <= imu_buffer_end; ++it) {
      if (imu_buffer[it] == ',') {
        if (comma1 == -1) {
          comma1 = it;
        } else {
          comma2 = it;
        }
      }
    }

    char  buffer_x[comma1],
          buffer_y[comma2 - comma1 - 1],
          buffer_z[imu_buffer_end - comma2];

    for (short it = 0; it < comma1; ++it)
      buffer_x[it] = imu_buffer[it];

    for (short it = 0; it < comma2 - comma1 - 1; ++it)
      buffer_y[it] = imu_buffer[comma1 + 1 + it];

    for (short it = 0; it <= imu_buffer_end - comma2; ++it)
      buffer_z[it] = imu_buffer[comma2 + 1 + it];

    Values.X = atof(buffer_x);
    Values.Y = atof(buffer_y);
    Values.Z = atof(buffer_z);
    
  }

  void  buffer_add (char c) {

    if (c != '#' && c != 'Y' && c != 'P' && c != 'R' && c != '/n') {
      if (c == '=') {
        parse_buffer();
        imu_buffer_end = 0;
      } else
      if (imu_buffer_end > -1) {
        imu_buffer[imu_buffer_end] = c;
        ++imu_buffer_end;
      }
    }
    
  }
 
}

void setup () {
  Serial.begin(57600);
  pinMode(Pin_PWM, OUTPUT);
  pinMode(Pin_DIR, OUTPUT);
}

void loop () {
          
   if (Serial.available() > 0) 
    IMU::buffer_add(Serial.read());

   angle = IMU::Values.X;
   error = reference - angle;
   derror = last_error - error;
   ierror = ierror + error;
   u = error*Kp+error*Kd+error*Ki;
   Motor(round(u));
   last_error = error;
 
}

void Motor(int input)
{
  int voltage = 0;
  int dir = 0;
  
  if(input>0) dir = 1;
  if(input<0) dir = 0;
  if(input>255) voltage = 255;
  if(input<-255) voltage = 255;

  analogWrite(Pin_PWM,voltage);
  digitalWrite(Pin_DIR,dir);
}
