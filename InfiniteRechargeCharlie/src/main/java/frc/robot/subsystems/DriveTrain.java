/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import frc.robot.commands.Drive;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

public class DriveTrain extends Subsystem {
  public WPI_TalonFX Left1 = new WPI_TalonFX(RobotMap.LEFT1);
  public WPI_TalonFX Left2 = new WPI_TalonFX(RobotMap.LEFT2);
 
  public WPI_TalonFX Right1 = new WPI_TalonFX(RobotMap.RIGHT1);
  public WPI_TalonFX Right2 = new WPI_TalonFX(RobotMap.RIGHT2);

  public DifferentialDrive dd = new DifferentialDrive(Left1, Right1);

  public DriveTrain(){
    Left1.setInverted(false);
    Left2.setInverted(false);
  
    Right2.follow(Right1);  
    Left2.follow(Left1);
    
  }
  public void ArcadeDrive ( double x, double rotation) {
    dd.arcadeDrive(-x, rotation);
  }
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
