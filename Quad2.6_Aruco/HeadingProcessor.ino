// tilt compensation, refining, and error scrubbing
float processHeading(Vector mag)
{  
  // convert them to pitch and roll
  float roll;
  float pitch;

  roll = asin(-ypr[2]);   // we've got this info already from the dmp
  pitch = asin(ypr[1]);

  if (roll > 0.6 || roll < -0.6 || pitch > 0.6 || pitch < -0.6)
    return prevHeading;

    // Some of these are used twice, so rather than computing them twice in the algorithem we precompute them before hand.
  float cosRoll = cos(roll);  
  float sinRoll = sin(roll);  
  float cosPitch = cos(pitch);
  float sinPitch = sin(pitch);
  
  // Tilt compensation
  float Xh = mag.XAxis * cosPitch + mag.ZAxis * sinPitch;
  float Yh = mag.XAxis * sinRoll * sinPitch + mag.YAxis * cosRoll - mag.ZAxis * sinRoll * cosPitch;
 
  float process_heading = atan2(Yh, Xh);

  // adjust for declination angle
  declinationAngle = (-3.0 + (11.0 / 60.0)) / (180 / M_PI);   // this is declination angle for uiuc!
  process_heading += declinationAngle;

  process_heading = correctAngle(process_heading);    // correct angle for if its above 2pi or less than 0
  process_heading = process_heading * 180/M_PI;     // convert to degrees

  // Fix magnetometer issue with angles to make it more accurate
  float fixedHeadingDegrees;
  if (process_heading >= 1 && process_heading < 240)
  {
    fixedHeadingDegrees = map(process_heading, 0, 239, 0, 179);
  } else
  if (process_heading >= 240)
  {
    fixedHeadingDegrees = map(process_heading, 240, 360, 180, 360);
  }
  process_heading = fixedHeadingDegrees;

  // check for unusual values
  int processHeadingInt = process_heading;
  int prevHeadingInt = prevHeading;
  int angleChange = ((processHeadingInt - prevHeadingInt + 180 + 360)%360) - 180;   // calculate difference, but keep in mind degrees!
  if((angleChange > 60) && (antiGlitch < 5))  // if difference > 50 and we haven't been throwing out values too long..
  {
    process_heading = prevHeading;
    antiGlitch++;
  }
  else
    antiGlitch = 0;

  // more often than not, a heading of 0 is erroneous. If it's 0, just make it previous heading.
  if (process_heading == 0)
    process_heading = prevHeading;
  
  return process_heading;   // return the smoothed, calibrated heading!
}

// Correct angle
float correctAngle(float process_heading)
{
  if (process_heading < 0) { process_heading += 2 * PI; }
  if (process_heading > 2 * PI) { process_heading -= 2 * PI; }

  return process_heading;
}

// Get a signed difference between two headings
int getHeadingDiff(float initial, float dest)
{
  if (initial > 360 || initial < 0 || dest > 360 || dest < 0)
  {
    Serial.print(F("Heading Diff Error!"));
    return 0;
  }

  int signed_diff = dest - initial;
  int abs_diff = abs(dest - initial);

  if (abs_diff <= 180)
    return abs_diff == 180 ? abs_diff : signed_diff;
  else if (dest-initial > 0)
    return abs_diff - 360;
  else
    return 360 - abs_diff;
}
