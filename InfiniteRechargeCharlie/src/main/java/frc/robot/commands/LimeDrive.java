/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;
import frc.robot.Robot;
import frc.robot.OI;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.Timer;

public class LimeDrive extends Command{
  /*
  Uses data from the Limelight subsystem to correct
  the robot's horizontal offset from the target
  */
  public LimeDrive(){
    requires(Robot.dt);
    requires(Robot.lm);
  }

  @Override
  protected void initialize(){
  }

  @Override
  protected void execute(){
    /*
      Calculates level of adjustment required to shift the robot to the target on-screen
      Remember to reconfigure Kp and minPower depending on the driving surface,
      esp. before competition
    */
    double headingError = Robot.lm.getHorizontalOffset();
    double steeringAdjust = 0.0f;
    double Kp = 0.05;
    double minPower = 0.0;

    if (Robot.lm.validTarget()){
      if (Robot.lm.getHorizontalOffset() > 1){
        steeringAdjust = Kp * headingError - minPower;
      }else if (headingError < 1){
        steeringAdjust = Kp * headingError + minPower;
      }
    }

    Robot.dt.ArcadeDrive(OI.driver.getRawAxis(1), steeringAdjust);
    Timer.delay(.005);
  }

  @Override
  protected boolean isFinished(){
    if (Robot.lm.getHorizontalOffset() == 0){
      return true;
    }
    else{
    return false;
    }
  }

  @Override
  protected void end(){
  }

  @Override
  protected void interrupted(){
  }
}