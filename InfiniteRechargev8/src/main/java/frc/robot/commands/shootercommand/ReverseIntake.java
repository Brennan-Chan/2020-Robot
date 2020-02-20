/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.shootercommand;

import edu.wpi.first.wpilibj2.command.CommandBase;

//instanciate subsystems 
import frc.robot.subsystems.Conveyor;

public class ReverseIntake extends CommandBase {
  private final Conveyor Conveyor;
  double time;
  double endTime;

  public ReverseIntake(Conveyor m_Conveyor, double time) {
    Conveyor = m_Conveyor;
    addRequirements(m_Conveyor);

  }

  @Override
  public void initialize() {
    long startTime = System.currentTimeMillis();
    endTime = startTime + time;
  }

  @Override
  public void execute() {
    Conveyor.reverseIntake();
  }

  @Override
  public void end(boolean interrupted) {
    Conveyor.shitStop();
  }

  @Override
  public boolean isFinished() {
    if (System.currentTimeMillis() >= endTime){
      System.out.println("The time has ended");
      return false;
    }
    return false;
  }
}
