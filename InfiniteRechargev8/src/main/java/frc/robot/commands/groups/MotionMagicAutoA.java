/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.groups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.*;

//import commands 
import frc.robot.commands.drivingCommands.MotionMagic;
// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class MotionMagicAutoA extends SequentialCommandGroup {
  /**
   * Creates a new MotionMagicAutoA.
   */
  public MotionMagicAutoA(DriveTrain drive, Limelight lime, Shooter shoot, BallFeeder feed) {
    
    //TODO: see if the addCommand feature works instead of super 
    super(
    //new MotionMagic(1.3, -1.3, drive),// 90 degree turn 
    new MotionMagic(-11.0, -11.0, drive),
    new MotionMagic(-.65, .65, drive),
    new MotionMagic(3.9, 3.9, drive),
    new MotionMagic(.65, -.65, drive),
    new MotionMagic(5.0, 5.0, drive)
    );
  }
}
