/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.shootercommand;

import edu.wpi.first.wpilibj2.command.CommandBase;

//import subsystems 
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;

//import commands

public class FlyWheel extends CommandBase {
  private final Limelight m_limelight;
  private final Shooter m_shooter;
  private double balls;

  /**
   * Creates a new FlyWheel.
   */
  public FlyWheel(Limelight limelight, Shooter shooter) {
    m_limelight = limelight;
    m_shooter = shooter;

    addRequirements(m_limelight);
    addRequirements(m_shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_shooter.configClosedLoop();
    balls = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    //convert the encoder units into velocity 
    double convertedVelocity = m_limelight.setShooterVelocity() *(60/16); 

    //fire at will 
    m_shooter.setShootSpeed(convertedVelocity);

    if(!(convertedVelocity == m_shooter.getTheoShooterVelo())){
      ++balls;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.setPower(0.0);
  
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(!m_limelight.validTarget()){
      return true;
    }
    else{
      return false;
    }
  }
}
