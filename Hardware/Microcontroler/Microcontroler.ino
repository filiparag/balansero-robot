enum  dimension {X, Y, Z};

double reference = -0.65;
double angle = 0;

double Kp = 8;
double Kd = 0;
double Ki = 0;
double range = 1; //Ako je range = 0, koeficijenti mogu da idu od 0 do 5

double error = 0;
double last_error = 0;
double derror = 0;
double ierror = 0;
double u = 0;

int Pin_PWM = 9;
int Pin_DIR = 8;

bool manual_pid = true;

namespace IMU {

char  imu_buffer[32];
short imu_buffer_end = -1;

bool new_measurement = false;

struct Values {
  double X, Y, Z;
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

  new_measurement = true;
}

void  buffer_add (char c) {

  if (c != '#' && c != 'Y' && c != 'P' && c != 'R' && c != '/n') {
    if (c == '=') {
      parse_buffer();
      imu_buffer_end = 0;
    } else if (imu_buffer_end > -1) {
      imu_buffer[imu_buffer_end] = c;
      ++imu_buffer_end;
    }
  }
}

double read_value (dimension d) {

  new_measurement = false;

  switch (d) {
    case X:
      return Values.X;
    case Y:
      return Values.Y;
    case Z:
      return Values.Z;
  }

}

}

void setup () {
  
  Serial.begin(57600);
  pinMode(Pin_PWM, OUTPUT);
  pinMode(Pin_DIR, OUTPUT);

  if (manual_pid) {
      Kp = analogRead(A3) / 886.0 * 30.0;
      Kd = analogRead(A1) / 886.0 * 50.0;
//      Serial.println(Kp);
  }
  
}

void loop () {

  if (Serial.available() > 0)
    IMU::buffer_add(Serial.read());

  if (IMU::new_measurement) {
    
    angle = IMU::read_value(Z);

//        Serial.println(angle);



    //   Kp = analogRead(A0)*(5.0/1023.0)*range;
    //   Kd = analogRead(A1)*(5.0/1023.0)*range;
    //   Ki = analogRead(A2)*(5.0/1023.0)*range;

    error = reference - angle;
    derror = error - last_error;
    ierror = ierror + error;
    u = error * Kp + derror * Kd + ierror * Ki;
    Motor(round(u));
    last_error = error;
  }
}

void Motor(int input)
{
  int voltage = input;
  int dir = 0;

  if (input > 0) dir = 0;
  if (input < 0) dir = 1;
  if (input > 255) voltage = 255;
  if (input < -255) voltage = -255;

  voltage = abs(voltage);

  analogWrite(Pin_PWM, voltage);
  digitalWrite(Pin_DIR, dir);

      Serial.print(angle);
      Serial.print(',');
      Serial.print(Kp);
      Serial.print(',');
      Serial.print(Kd);
      Serial.print(',');
      Serial.println(u);
}
