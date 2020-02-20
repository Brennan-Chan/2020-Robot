/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.groups;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;

//import subsystems 
import frc.robot.subsystems.*;
//import commands 
import frc.robot.commands.groups.MotionMagicAutoA;
import frc.robot.commands.Feeder;

public class Auto_A extends ParallelRaceGroup {
 
  public Auto_A(BallFeeder feed, Limelight lime, DriveTrain drive, Shooter shoot) {
    super(
      new Feeder(feed),
      new MotionMagicAutoA(drive, lime, shoot, feed)
    );
  }
}
