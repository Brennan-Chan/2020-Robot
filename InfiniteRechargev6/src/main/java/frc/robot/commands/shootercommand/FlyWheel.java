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

public class FlyWheel extends CommandBase {
  private final Limelight m_limelight;
  private final Shooter m_shooter;
  double targetvelocity;
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
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    //convert the encoder units into velocity 
    double targetVelocity = (m_limelight.OptimalAngularVelocity() * 4096 / 600)/2;
    m_shooter.setShootSpeed(targetVelocity);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.setPower(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
