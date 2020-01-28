/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;



public class Conveyor extends SubsystemBase {
  //instanciate talons 
  public final WPI_TalonSRX conveyor_one = new WPI_TalonSRX(19);
  
  public Conveyor() {

  }

  //make go forward 
  public void intaketoShooter(){
    conveyor_one.set(.5);
    return;
  }
  //make go backward
  public void reverseIntake(){
    conveyor_one.set(-.5);
  }

  //stop 
  public void shitStop(){
    conveyor_one.set(0.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
