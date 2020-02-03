/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;


import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatch;

//tihs sets the values for the color sensor 
public class SubColorSensor extends SubsystemBase {

  public final I2C.Port i2cPort = I2C.Port.kOnboard;
  public final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);
  public final ColorMatch m_colorMatcher = new ColorMatch();

  public final Color kBlueTarget = ColorMatch.makeColor(0.143, 0.427, 0.429);
  public final Color kGreenTarget = ColorMatch.makeColor(0.197, 0.561, 0.240);
  public final Color kRedTarget = ColorMatch.makeColor(0.561, 0.232, 0.114);
  public final Color kYellowTarget = ColorMatch.makeColor(0.361, 0.524, 0.113);

  public void colorMatch(){
  m_colorMatcher.addColorMatch(kBlueTarget);
  m_colorMatcher.addColorMatch(kGreenTarget);
  m_colorMatcher.addColorMatch(kRedTarget);
  m_colorMatcher.addColorMatch(kYellowTarget);
  }

  Color detectedColor = m_colorSensor.getColor();
  String colorString;
  //ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);

  //public void ColorMatchResult(){
    //match = m_colorMatcher.matchClosestColor(detectedColor);
  //}

  public void colorDash(){
    //SmartDashboard.putNumber("Red", detectedColor.red);
    //SmartDashboard.putNumber("Green", detectedColor.green);
    //SmartDashboard.putNumber("Blue", detectedColor.blue);
    //SmartDashboard.putNumber("Confidence", match.confidence);
    SmartDashboard.putString("Detected Color", colorString);
  }
}
