/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj2.command.CommandBase;

//import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Limelight;

public class LimeDrive extends CommandBase{
  private final DriveTrain drivetrain;
  private final Limelight limelight;
  /*
  Uses data from the Limelight subsystem to correct
  the robot's horizontal offset from the target
  */
  public LimeDrive(DriveTrain drive, Limelight lime){
    drivetrain = drive;
    limelight = lime;

    addRequirements(drive);
    addRequirements(lime);
  }

  @Override
  public void initialize(){
  }

  @Override
  public void execute(){
    /*
      Calculates level of adjustment required to shift the robot to the target on-screen
      Remember to reconfigure Kp and minPower depending on the driving surface,
      esp. before competition
    */
    double headingError = limelight.getHorizontalOffset();
    double steeringAdjust = 0.0f;
    double Kp = 0.05;
    double minPower = 0.0;

    if (limelight.validTarget()){
      if (limelight.getHorizontalOffset() > 1){
        steeringAdjust = Kp * headingError - minPower;
      }else if (headingError < 1){
        steeringAdjust = Kp * headingError + minPower;
      }
    }

    drivetrain.arcadeDrive(RobotContainer.driver.getRawAxis(1), steeringAdjust);
    Timer.delay(.005);
  }

  @Override
  public void end(boolean interrupted){
  }

  @Override
  public boolean isFinished(){
    if (limelight.getHorizontalOffset() == 0){
      return true;
    }
    else{
    return false;
    }
  }
  
}