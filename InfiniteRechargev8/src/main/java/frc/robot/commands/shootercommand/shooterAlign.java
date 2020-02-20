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

public class shooterAlign extends CommandBase {
  private final Limelight m_limelight;
  private final Shooter m_shooter;
  private final Double m_angle;
  private boolean hood_isFinished;
  
  public shooterAlign(double angle, Limelight limelight, Shooter shooter) {
    m_limelight = limelight;
    m_shooter = shooter;
    m_angle = angle;

    addRequirements(m_limelight);
    addRequirements(m_shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_shooter.configHoodClosedLoop();
    //m_shooter.resetHoodEncoder();
    hood_isFinished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    m_shooter.setHoodWithMagic(m_angle);

    if (m_shooter.getencoderAngle() >= m_angle){
      hood_isFinished = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
   m_shooter.setHoodPower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if( m_shooter.getencoderAngle() >= m_angle || !m_limelight.validTarget()){
      hood_isFinished = true;
      System.out.println("Hood is finished");
      return hood_isFinished;
    }
    else{
      return hood_isFinished;
    }
  }
}
