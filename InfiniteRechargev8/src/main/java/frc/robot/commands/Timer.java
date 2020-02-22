/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class Timer extends CommandBase {
  double time;
  double endTime;

  public Timer(double time) {
  }

  @Override
  public void initialize() {
    long startTime = System.currentTimeMillis();
    endTime = startTime + time;
  }

  @Override
  public void execute() {
  }

  @Override
  public void end(boolean interrupted) {
    
  }

  @Override
  public boolean isFinished() {
    if (System.currentTimeMillis() >= endTime){
      System.out.println("The time has ended");
      return true;
    }
    return false;
  }
}
