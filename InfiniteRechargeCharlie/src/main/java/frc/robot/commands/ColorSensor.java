/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;

public class ColorSensor extends Command {
  public ColorSensor() {
    requires(Robot.cs);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.cs.m_colorMatcher.addColorMatch(Robot.cs.kBlueTarget);
    Robot.cs.m_colorMatcher.addColorMatch(Robot.cs.kGreenTarget);
    Robot.cs.m_colorMatcher.addColorMatch(Robot.cs.kRedTarget);
    Robot.cs.m_colorMatcher.addColorMatch(Robot.cs.kYellowTarget); 
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Color detectedColor = Robot.cs.m_colorSensor.getColor();

    /**
     * Run the color match algorithm on our detected color
     */
    String colorString;
    ColorMatchResult match = Robot.cs.m_colorMatcher.matchClosestColor(detectedColor);

    if (match.color == Robot.cs.kBlueTarget) {
      colorString = "Blue";
    } else if (match.color == Robot.cs.kRedTarget) {
      colorString = "Red";
    } else if (match.color == Robot.cs.kGreenTarget) {
      colorString = "Green";
    } else if (match.color == Robot.cs.kYellowTarget) {
      colorString = "Yellow";
    } else {
      colorString = "Unknown";
    }

    /**
     * Open Smart Dashboard or Shuffleboard to see the color detected by the 
     * sensor.
     */
    SmartDashboard.putNumber("Red", detectedColor.red);
    SmartDashboard.putNumber("Green", detectedColor.green);
    SmartDashboard.putNumber("Blue", detectedColor.blue);
    SmartDashboard.putNumber("Confidence", match.confidence);
    SmartDashboard.putString("Detected Color", colorString);

    System.out.println(colorString);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
