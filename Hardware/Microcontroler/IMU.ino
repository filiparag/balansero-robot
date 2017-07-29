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
