/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.Constants;

public class Shooting extends Command {
  double velo;
  boolean m_isFinished;
  
  public Shooting() {
    requires(Robot.lm);
    requires(Robot.st);
 
  }

  @Override
  protected void initialize() {
    m_isFinished = false;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    double velo = (Constants.kHorisontalDistance + Constants.kGoalWallDist + Robot.lm.distanceToTarget())
    /(Math.sqrt((2/Constants.kGravity)*(Constants.kWallHeight-Constants.kBallHeight-((Math.sin(69)*(Constants.kHorisontalDistance + Constants.kGoalWallDist + Robot.lm.distanceToTarget()))/Math.cos(69))))*Math.cos(69));
  
    if(Robot.lm.validTarget()){
      Robot.st.setShootSpeed(velo);
      System.out.println(velo);
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    
    if (!Robot.lm.validTarget()){
      return true;
    }
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
