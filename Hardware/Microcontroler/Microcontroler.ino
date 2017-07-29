enum  dimension {X, Y, Z};

void setup () {
  Serial.begin(57600);    
}

void loop () {
          
   if (Serial.available() > 0) 
    buffer_add(Serial.read());
 
}
