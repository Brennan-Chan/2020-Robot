/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.BallFeeder;

public class Feeder extends CommandBase {
  private final BallFeeder m_ballfeeder;

  public Feeder(BallFeeder feed) {
    m_ballfeeder = feed;
    addRequirements(m_ballfeeder);
  }

  
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    m_ballfeeder.feederOn();
    m_ballfeeder.shiftFeeder();
  
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_ballfeeder.shiftFeeder();
    m_ballfeeder.feederOFF();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
