/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.util.Color;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorSensorV3.RawColor;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;


public class Robot extends TimedRobot {
  /**
   * Change the I2C port below to match the connection of your color sensor
   */
  private final I2C.Port i2cPort = I2C.Port.kOnboard;

  /**
   * A Rev Color Sensor V3 object is constructed with an I2C port as a 
   * parameter. The device will be automatically initialized with default 
   * parameters.
   */
  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);
  
 // public static class ColorSensorV3.RawColor
 private final ColorMatch m_colorMatcher = new ColorMatch();

 /**
  * Note: Any example colors should be calibrated as the user needs, these
  * are here as a basic example.
  */
 private final Color kBlueTarget = ColorMatch.makeColor(0.143, 0.427, 0.429);
 private final Color kGreenTarget = ColorMatch.makeColor(0.197, 0.561, 0.240);
 private final Color kRedTarget = ColorMatch.makeColor(0.561, 0.232, 0.114);
 private final Color kYellowTarget = ColorMatch.makeColor(0.361, 0.524, 0.113);

 @Override
 public void robotInit() {
   m_colorMatcher.addColorMatch(kBlueTarget);
   m_colorMatcher.addColorMatch(kGreenTarget);
   m_colorMatcher.addColorMatch(kRedTarget);
   m_colorMatcher.addColorMatch(kYellowTarget);    
 }


  @Override
  public void robotPeriodic() {
    /**
     * The method GetColor() returns a normalized color value from the sensor and can be
     * useful if outputting the color to an RGB LED or similar. To
     * read the raw color, use GetRawColor().
     * 
     * The color sensor works best when within a few inches from an object in
     * well lit conditions (the built in LED is a big help here!). The farther
     * an object is the more light from the surroundings will bleed into the 
     * measurements and make it difficult to accurately determine its color.
     */
    String colorString;
    Color detectedColor = m_colorSensor.getColor();
    RawColor detectedRawColor = m_colorSensor.getRawColor();
    ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);

    /**
     * The sensor returns a raw IR value of the infrared light detected.
     */
    double IR = m_colorSensor.getIR();

    /**
     * Open Smart Dashboard or Shuffleboard to see the color detected by the 
     * sensor.
     */
    //Color detectedrawcolor = GetRawColor(detectedColor);
    //SmartDashboard.putNumber("Leslie", detectedColor.getRawColor());
 
  boolean booleanRed = false;
  boolean booleanGreen = false;
  boolean booleanBlue = false;
  boolean booleanYellow = false;
  if (match.color == kBlueTarget){
    colorString = "Blue";
    booleanBlue =true;
  }else if (match.color == kRedTarget){
    colorString = "Red";
    booleanRed = true;
  }else if (match.color == kGreenTarget){
    colorString = "Green";
    booleanGreen = true;}
  else if (match.color == kYellowTarget){
    colorString = "Yellow";
    booleanYellow = true;}
    else {
      colorString = "unknown";
    }
    SmartDashboard.putBoolean("isRed", booleanRed);
    SmartDashboard.putBoolean("isBlue", booleanBlue);
    SmartDashboard.putBoolean("isYellow", booleanYellow);
    SmartDashboard.putBoolean("isGreen", booleanGreen);
    /**
     * In addition to RGB IR values, the color sensor can also return an 
     * infrared proximity value. The chip contains an IR led which will emit
     * IR pulses and measure the intensity of the return. When an object is 
     * close the value of the proximity will be large (max 2047 with default
     * settings) and will approach zero when the object is far away.
     * 
     * Proximity can be used to roughly approximate the distance of an object
     * or provide a threshold for when an object is close enough to provide
     * accurate color values.
     */
    int proximity = m_colorSensor.getProximity();

    SmartDashboard.putNumber("Proximity", proximity);

//This reads the game data from FMS and then on the dashboard/Shuffleboard it outputs it
    String gameData;
  gameData = DriverStation.getInstance().getGameSpecificMessage();
  if(gameData.length() > 0)
  {
    switch (gameData.charAt(0))
    {
    case 'B' :
      //Blue case code
      SmartDashboard.putString("FMS Color", "Blue");
      break;
    case 'G' :
      //Green case code
      SmartDashboard.putString("FMS Color", "Green");
      break;
    case 'R' :
      //Red case code 
      SmartDashboard.putString("FMS Color", "Red");
      break;
    case 'Y' :
      //Yellow case code
      SmartDashboard.putString("FMS Color", "Yellow");
      break;
    default :
      //This is corrupt data
      break;
  }
} else {
  //Code for no data received yet
}

  }
}
