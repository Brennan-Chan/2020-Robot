/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drivingCommands;


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;


public class MotionMagic extends CommandBase {
  private final DriveTrain drivetrain;
  private final Double m_right;
  private final Double m_left;

  public boolean magic_isFinished;

  public MotionMagic(Double right, Double left, DriveTrain drive) {
    drivetrain = drive;
    m_right = right;
    m_left = left;
    
    addRequirements(drivetrain);
    
    magic_isFinished = false;
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //configure the motor for motion magic 
    drivetrain.config4MotionMagic();
    drivetrain.resetEncoders();
    magic_isFinished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //set the motionmagic revolutions for each wheel and watch the magic happen
    if(!magic_isFinished){
    drivetrain.driveMotionMagic(m_right, m_left);
    }

    if ((Math.abs(drivetrain.getRightDistance()*2) >= Math.abs(m_right)) && (Math.abs(drivetrain.getLeftDistance()*2) >= Math.abs(m_left))){
      System.out.println("Finishing and turning off");
      magic_isFinished = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.configStandardDrive();
    drivetrain.resetEncoders();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //TODO: find if setting this true makes reconfigure for regular drive 
    return magic_isFinished;
  }
}
