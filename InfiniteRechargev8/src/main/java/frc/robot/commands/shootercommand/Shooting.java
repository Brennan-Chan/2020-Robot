/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.shootercommand;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;

public class Shooting extends CommandBase {
  private final Limelight m_limelight;
  private final Shooter m_shooter;
  double time;
  double endTime;
  

  public Shooting(Limelight limelight, Shooter shooter, double time) {
    m_limelight = limelight;
    m_shooter = shooter;
    
    addRequirements(m_limelight);
    addRequirements(m_shooter);
  }
  @Override
  public void initialize() {
    //start the timer 
    long startTime = System.currentTimeMillis();
    endTime = startTime + time;

    m_shooter.configHoodClosedLoop();
    m_shooter.resetHoodEncoder();
    m_shooter.configClosedLoop();
  }

  
  @Override
  public void execute() {
    //convert the encoder units into velocity 
    double convertedVelocity = m_limelight.setShooterVelocity() * (60/16); 

    //aim 
    m_shooter.cheetingAngle(m_shooter.hoodAngleTable());
    if( (m_shooter.getencoderAngle() >= (m_shooter.hoodAngleTable() - 0.2) ) && (m_shooter.getencoderAngle() <= (m_shooter.hoodAngleTable() + 0.2) )){
      m_shooter.setPower(0);
    }

    //fire 
    m_shooter.setShootSpeed(convertedVelocity);
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.cheetingAngle(0);
    m_shooter.setPower(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
    if (System.currentTimeMillis() >= endTime){
      System.out.println("The time has ended");
      return true;
    }
    else{
      return false;
    }
  }
}