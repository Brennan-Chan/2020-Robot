/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

import com.ctre.phoenix.motorcontrol.can.*;

public class Intake extends Subsystem {
  
  //public WPI_TalonFX feeder = new WPI_TalonFX(RobotMap.Intake);
  public WPI_TalonSRX feeder = new WPI_TalonSRX(RobotMap.Intake1);

  public Intake(){

  }

  public void setPower(double power){
    feeder.set(power);
  }

  public void stop(){
    feeder.set(0.0);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
