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

public class GoDown extends CommandBase {
  private final Limelight m_limelight;
  private final Shooter m_shooter;
  private boolean hood_isFinished;
  
  public GoDown(Limelight limelight, Shooter shooter) {
    m_limelight = limelight;
    m_shooter = shooter;

    addRequirements(m_limelight);
    addRequirements(m_shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    hood_isFinished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    m_shooter.cheetingAngle(1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.setHoodPower(0);
    m_shooter.resetHoodEncoder();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    /**
    if((m_shooter.getencoderAngle() == 1) || m_shooter.activeLimitSwitch()) {
      hood_isFinished = true;
      System.out.println("Hood is finished...reseting the angle");
      m_shooter.resetHoodEncoder();
      return hood_isFinished;
    }
    */
    if((m_shooter.getencoderAngle() <= 1) || m_shooter.activeLimitSwitch()) {
      hood_isFinished = true;
      System.out.println("Hood is finished");
      return hood_isFinished;
    }
    else{
      return hood_isFinished;
    }
  }
}
