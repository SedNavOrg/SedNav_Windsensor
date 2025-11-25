// Calculations of wind speed and wind direction
// Data are defined as global data

// Round float to int
int roundFloat2Int(float x)
{
  if(x>0) return (int)(x + 0.5);
  return (int)(x - 0.5);
}

// Convert °C to °F
float convertCtoF(float value){
 float fahrenheit = (value * 9 / 5) + 32;
 return fahrenheit;
}

// Dewpoint calculation
// Refer: https://de.wikipedia.org/wiki/Taupunkt
float dewp(float temp, float humidity) {
//  float K1 = 6.112;   // [hPa]
  float K2 = 17.62;   // [1]
  float K3 = 243.12;  // [°C]
  float term1 = (K2 * temp) / (K3 + temp);
  float term2 = (K2 * K3) / (K3 + temp);
  float term3 = log(humidity / 100);                  // Humidiy range 0...1
  float dewp = K3 *((term1 + term3)/(term2 - term3));
  return dewp;
}

void calculationData(){
  // Copy necessary data
  float local_temperature;
  float local_quality;
  float local_fieldstrength;
  float local_magnitude;
  float local_magsensor;
  float local_rawwinddirection;
  float local_airtemperature;
  float local_airpressure;
  float local_airhumidity;
  float local_altitude;
  float local_dewpoint;
  float local_winddirection;
  float local_winddirection2;
  float local_dirresolution;
  float local_windspeed_hz;
  float local_windspeed_mps;
  NO_INTERRUPTS;
  float local_time1 = time1;
  float local_time1_avg = time1_avg;
  float local_time2_avg = time2_avg;
  INTERRUPTS;

  // Is connected with extern WLAN network
  if(WiFi.localIP().toString() != "0.0.0.0"){
    local_fieldstrength = float(WiFi.RSSI());
    if(local_fieldstrength > 0){
      local_fieldstrength = -100.0;
    }
    local_quality = 100  - (((local_fieldstrength * -1) - 50) * 2);
    if(local_quality < 0){
      local_quality = 0;
    }
    if(local_quality > 100){
      local_quality = 100;
    }
  }
  else{
    local_fieldstrength = 0;
    local_quality = 0;
  }

  // Read device temperature 1Wire DS18B20
  // The DS18B20 neeed a temperature compensation because the data rate is 2 Hz and heats up the sensor
  if(String(actconf.tempSensorType) == "DS18B20"){
    DS18B20->requestTemperatures();
    if(String(actconf.tempUnit) == "C"){
      local_temperature = float(DS18B20->getTempCByIndex(0) - 6.0);            // With temperature compensation
    }
    else{
      local_temperature = float(DS18B20->getTempFByIndex(0) - (6.0 * 9 / 5));  // With temperature compensation
    }
  }

  // time1 = time in [ms] for one rotation
  // time2 = time in [ms] between wind speed sensor and wind direction sensor
  if(local_time1_avg == 0){
    local_time1_avg = 0.1;
  }

  // Calculate wind direction
  switch (actconf.windSensorType)
  {
  case WIND_SENSOR_WIFI_1000:
    // Calculate only wind direction when time values ok
    if(local_time1_avg < 1000 && local_time2_avg < 1000){
      // Raw wind direction 0...360°, dir[°] = time2[ms] / time1[ms] *360
      local_rawwinddirection = local_time2_avg / local_time1_avg * 360;
    }
    local_magnitude = 0; // Set values for AS5600
    local_magsensor = 0;
    break;
  
  case WIND_SENSOR_YACHTA:
  case WIND_SENSOR_JUKOLEIN:
  case WIND_SENSOR_SEDNAV_C6:
    // Read only magnetic values if the I2C device is ready
    if(i2creadyAS5600){
      local_magnitude = ams5600.getMagnitude();
      local_magsensor = ams5600.getRawAngle() * 0.087; // 0...4096 which is 0.087 of a degree
      // Limiting values outer range
      if(local_magsensor < 0){
        local_magsensor = 0;
      } else if(local_magsensor > 360){
        local_magsensor = 360;
      }
    }
    else{
      local_magnitude = 0;
      local_magsensor = 0;
    }
    local_rawwinddirection = local_magsensor;
    break;

  // Attention! Inverse rotation because the MT6701 measure counter clock
  case WIND_SENSOR_YACHTA_2_0:
    // Read only magnetic values if the I2C device is ready
    if(i2creadyMT6701){
      local_magnitude = 0;                              // 0...16384 which is 0.0219 of a degree
      local_magsensor = 360 - mt6701.getDegreesAngle(); // value in degree
      // Limiting values outer range
      if(local_magsensor < 0){
        local_magsensor = 0;
      } else if(local_magsensor > 360){
        local_magsensor = 360;
      }
    }
    else{
      local_magnitude = 0;
      local_magsensor = 0;
    }
    local_rawwinddirection = local_magsensor;
    break;
  
  // Attention! Inverse rotation because the AS5600 measure on bottom side
  case WIND_SENSOR_VENTUS:
    // Read only magnetic values if the I2C device is ready
    if(i2creadyAS5600){
      local_magnitude = ams5600.getMagnitude();
      local_magsensor = 360 - ams5600.getRawAngle() * 0.087; // 0...4096 which is 0.087 of a degree
      // Limiting values outer range
      if(local_magsensor < 0){
        local_magsensor = 0;
      } else if(local_magsensor > 360){
        local_magsensor = 360;
      }
    }
    else{
      local_magnitude = 0;
      local_magsensor = 0;
    }
    local_rawwinddirection = local_magsensor;
    
    if(i2creadyBME280 && String(actconf.tempSensorType) == "BME280"){
      if(String(actconf.tempUnit) == "C"){
        local_airtemperature = bme.readTemperature();
      }
      else{
        local_airtemperature = convertCtoF(bme.readTemperature());
      }
      local_airpressure = bme.readPressure() / 100;
      // J'en suis ici !
      local_airhumidity = bme.readHumidity();
      local_dewpoint = dewp(local_airtemperature, local_airhumidity);
      local_altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);
    }
    else{
      local_airtemperature = 0;
      local_airpressure = 0;
      local_airhumidity = 0;
      local_dewpoint = 0;
      local_altitude = 0;
    }
    local_rawwinddirection = local_magsensor;
    break;
  }
  
  // Wind direction with offset
  if((local_rawwinddirection + actconf.offset) >= 0 && (local_rawwinddirection + actconf.offset) <= 360){
    local_winddirection = local_rawwinddirection + actconf.offset;
  }
  if((local_rawwinddirection + actconf.offset) > 360){
    local_winddirection = local_rawwinddirection + actconf.offset - 360;
  }
  if((local_rawwinddirection + actconf.offset) < 0){
    local_winddirection = 360 - (abs(actconf.offset) - local_rawwinddirection);
  }
  
  // Limiting max deviations between two measuring values of wind direction
  if(abs(local_winddirection - winddirection_old) > maxwinddirdev && local_winddirection > maxwinddirdev && local_winddirection < 360 - maxwinddirdev){
    if(local_winddirection - winddirection_old > 0){
      local_winddirection = winddirection_old + maxwinddirdev;
    }
    else{
      local_winddirection = winddirection_old - maxwinddirdev;
    }
  }
  winddirection_old = local_winddirection;
  
  // Wind direction 0...180° for each boat side
  if(local_winddirection >= 0 && local_winddirection <= 180){
    local_winddirection2 = local_winddirection;
  }
  else{
    local_winddirection2 = 360 - local_winddirection;
  }

  // Calculate wind direction resolution
  switch (actconf.windSensorType)
  {
  case WIND_SENSOR_WIFI_1000:
    // Wind direction resolution res[°] = 360 / time1
    local_dirresolution = 360 / (local_time1 * 10);  // now 100us counter
    if(local_dirresolution > 20.0){
      local_dirresolution = 0.0;
    }
    break;
  
  case WIND_SENSOR_YACHTA:
  case WIND_SENSOR_JUKOLEIN:
  case WIND_SENSOR_VENTUS:
  case WIND_SENSOR_SEDNAV_C6:
    local_dirresolution = 0.087;
    break;
  
  case WIND_SENSOR_YACHTA_2_0:
    local_dirresolution = 0.0219;
    break;
  }

  // Calculate only wind speed when time values ok
  if(local_time1_avg < 1000 && local_time2_avg < 1000){
    switch (actconf.windSensorType)
    {
    case WIND_SENSOR_WIFI_1000:
    case WIND_SENSOR_VENTUS:
      // Wind speed n[Hz] = 1 / time1[ms] *1000  // 1 pulse per round
      local_windspeed_hz = 1.0 / local_time1_avg * 1000;
      break;
    
    case WIND_SENSOR_YACHTA:
    case WIND_SENSOR_YACHTA_2_0:
    case WIND_SENSOR_JUKOLEIN:
    case WIND_SENSOR_SEDNAV_C6:
      // Wind speed n[Hz] = 1 / time1[ms] *1000 / 2
      local_windspeed_hz = 1.0 / local_time1_avg * 1000 / 2; // 2 pulses per round
      break;
    }
  }

  // Eleminate the big start value direct after wind sensor start
  if(local_windspeed_hz > 100){
    local_windspeed_hz = 0;
  }

  // If zero wind speed the set wind speed to 0 Hz
  // Controlled via Timer4 routine
  if(flag3){
    local_windspeed_hz = 0.0;
  }

  // Calculate wind speed based on sensor type
  switch (actconf.windSensorType)
  {
  case WIND_SENSOR_WIFI_1000:
    // Wind speed, v[m/s] = (2 * Pi * n[Hz] * r[m]) / lamda[1]
    local_windspeed_mps = (2 * pi * local_windspeed_hz * radius) / lamda;
    break;
  
  case WIND_SENSOR_YACHTA:
  case WIND_SENSOR_YACHTA_2_0:
  case WIND_SENSOR_JUKOLEIN:
  case WIND_SENSOR_SEDNAV_C6:
    // Wind speed, v[m/s] = (2 * Pi * n[Hz] * r[m]) / lamda[1]
    local_windspeed_mps = (2 * pi * local_windspeed_hz * radius2) / lamda;
    break;
  
  case WIND_SENSOR_VENTUS:
    // Wind speed, v[m/s] = (2 * Pi * n[Hz] * r[m]) / lamda[1]
    local_windspeed_mps = (2 * pi * local_windspeed_hz * radius3) / lamda;
    break;
  }
  
  // Calibration of wind speed data
  local_windspeed_mps = local_windspeed_mps * actconf.calslope + actconf.caloffset;
  if(local_windspeed_mps < 0){
    local_windspeed_mps = 0;
  }

  // Wind speed, v[kn] = v[m/s] * 1.94384
  float local_windspeed_kn = local_windspeed_mps * 1.94384;
  float v2 = local_windspeed_kn * local_windspeed_kn;
  float term3 = 0.0000222 * v2 * local_windspeed_kn;
  float term2 = 0.0034132 * v2;
  float term1 = 0.2981666 * local_windspeed_kn;
  // Wind speed v[bft] = 0.0000222 * v³[kn] - 0.0034132 * v²[kn] + 0.2981666 * v[kn] + 0.1467082
  int local_windspeed_bft = roundFloat2Int(term3 - term2 + term1 + 0.1467082);
  // Limiting wind speed for bft lower than 12
  if(local_windspeed_bft > 12){
    local_windspeed_bft = 12;
  }

  // Store new data
  NO_INTERRUPTS;
  fieldstrength = local_fieldstrength;
  quality = local_quality;
  temperature = local_temperature;
  time1_avg = local_time1_avg;
  magnitude = local_magnitude;
  magsensor = local_magsensor;
  rawwinddirection = local_rawwinddirection;
  airtemperature = local_airtemperature;
  airpressure = local_airpressure;
  airhumidity = local_airhumidity;
  altitude = local_altitude;
  dewpoint = local_dewpoint;
  winddirection = local_winddirection;
  winddirection2 = local_winddirection2;
  dirresolution = local_dirresolution;
  windspeed_hz = local_windspeed_hz;
  windspeed_mps = local_windspeed_mps;
  windspeed_kph = local_windspeed_mps * 3.6; // Wind speed, v[km/h] = v[m/s] * 3.6
  windspeed_kn = local_windspeed_kn;
  windspeed_bft = local_windspeed_bft;
  INTERRUPTS;
}

void simulationData(){
  // Atomic Block (not interruptible)
  NO_INTERRUPTS;
  int i = 0;
  int speedmps;         // Actual calculated speed in [m/s]
  int winddir;          // Actual calculated wind direction in [°]
  int steps = 600;      // Number of steps for one pointer round
                        // Time for oune round is steps * 500ms
  
  fieldstrength = -100;    // No signal
    quality = 100  - (((fieldstrength * -1) - 50) * 2);
    
    if(String(actconf.tempUnit) == "C"){
    //Basis unit is °C
    temperature = float(random(210, 220)) / 10;
    }
    else{
    temperature = float(random(210, 220)) / 10;
    //(0 °C × 9/5) + 32 = 32 °F
    temperature = (temperature * 9 / 5) + 32;
    }
    sensor1 =  int(random(0, 2));
    sensor2 =  int(random(0, 2));

    magnitude = int(random(300, 450));
    magsensor = int(random(0, 360));

    airtemperature = float(random(210, 230)) / 10;
    airpressure = float(random(9000, 10100)) / 10;
    airhumidity = float(random(700, 800)) / 10;
    dewpoint = float(random(10, 150)) / 10;
    altitude = float(random(500, 560)) / 10;
    
//**************************************************************************

  // Calculate demo data in timearrays with random numbers for DEMO mode
  randomSeed(micros());
  for (i = 0; i < average; i++){
    // Calculate demo data for wind speed
    speedmps = ((demoSet % steps) * 38 / steps) + 1;
    // Add 40% noise
    speedmps += speedmps * 0.4 * float(random(0, 10)) / 10;
    // t1[ms] = (2 * Pi * 1000 * radius[m]) / (speed[m/s] * lamda)
    time1 = (2 * pi * 1000 * radius) / (speedmps * lamda);
    timearray1[i] = time1;
    // Calculate demo data for wind direction
    winddir = ((demoSet % steps) * 360 / steps);
    // Add 40% noise
    winddir += winddir * 0.4 * float(random(0, 10)) / 10;
    // t2[ms] = (t1 * winddir[°]) / 360°
    time2 = (timearray1[i] * winddir) / 360;
    timearray2[i] = time2;
  }
  demoSet ++;
  /*
  Serial.print("Ratio: ");
  Serial.print((timearray2[0]/timearray1[0]));
  Serial.print("   1: " );
  Serial.print(timearray1[0]);
  Serial.print("   2: " );
  Serial.println(timearray2[0]);
  */

//**************************************************************************

  // time1 = time in [ms] for one rotation
  // time2 = time in [ms] between wind speed sensor and wind direction sensor
  if(time1_avg == 0){
    time1_avg = 0.1;
  }

  // Calculate only wind direction when time values ok
  if(time1_avg < 1000 && time2_avg < 1000){
    // Raw wind direction 0...360°, dir[°] = time2[ms] / time1[ms] *360
    rawwinddirection = time2_avg / time1_avg * 360;
  }
  // Wind direction with offset
  if((rawwinddirection + actconf.offset) > 360){
    winddirection = rawwinddirection + actconf.offset - 360;
  }
  else{
    winddirection = rawwinddirection + actconf.offset;
  }
  
  // Wind direction 0...180° for each boat side
  if(winddirection >= 0 && winddirection <= 180){
    winddirection2 = winddirection;
  }
  else{
    winddirection2 = 360 - winddirection;
  }
  // Wind direction resolution res[°] = 360 / time1
  dirresolution = 360 / (time1 * 10);  // now 100us counter
  if(dirresolution > 20.0){
    dirresolution = 0.0;
  }
  // Calculate only wind speed when time values ok
  if(time1_avg < 1000 && time2_avg < 1000){
    // Wind speed n[Hz] = 1 / time1[ms] *1000
    windspeed_hz = 1.0 / time1_avg * 1000;
  }

  // Eleminate the big start value direct after wind sensor start
  if(windspeed_hz > 100){
    windspeed_hz = 0;
  }

  // Wind speed, v[m/s] = (2 * Pi * n[Hz] * r[m]) / lamda[1]
  windspeed_mps = (2 * pi * windspeed_hz * radius) / lamda;
  // Calibration of wind speed data
  windspeed_mps = windspeed_mps * actconf.calslope + actconf.caloffset;
  // Wind speed, v[km/h] = v[m/s] * 3.6
  windspeed_kph = windspeed_mps * 3.6;
  // Wind speed, v[kn] = v[m/s] * 1.94384
  windspeed_kn = windspeed_mps * 1.94384;
  float v2 = windspeed_kn * windspeed_kn;
  float term3 = 0.0000222 * v2 * windspeed_kn;
  float term2 = 0.0034132 * v2;
  float term1 = 0.2981666 * windspeed_kn;
  // Wind speed v[bft] = 0.0000222 * v³[kn] - 0.0034132 * v²[kn] + 0.2981666 * v[kn] + 0.1467082
  windspeed_bft = roundFloat2Int(term3 - term2 + term1 + 0.1467082);
  // Limiting wind speed for bft lower than 12
  if(windspeed_bft > 12){
    windspeed_bft = 12;
  }
  // End Atomic Block (not interruptible)
  INTERRUPTS;
}
