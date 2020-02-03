/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;


//this is pretty simple it can't get more bare bones than this 
public class Conveyor extends SubsystemBase {
  //instanciate talons 
  public final WPI_TalonSRX conveyor_one = new WPI_TalonSRX(35);
  public final WPI_TalonSRX conveyor_two = new WPI_TalonSRX(32);
  public final WPI_TalonSRX conveyor_three = new WPI_TalonSRX(33);
  public final WPI_TalonSRX conveyor_four = new WPI_TalonSRX(34);
  
  public Conveyor() {

  }

  //make go forward 
  public void intaketoShooter(){
    conveyor_one.set(.5);
    conveyor_two.set(.5);
    conveyor_three.set(.5);
    conveyor_four.set(-.7);
    return;
  }
  //make go backward
  public void reverseIntake(){
    conveyor_one.set(-.5);
    conveyor_two.set(-.5);
    conveyor_three.set(-.5);
    conveyor_four.set(.7);
  }

  //stop 
  public void shitStop(){
    conveyor_one.set(0.0);
    conveyor_two.set(0.0);
    conveyor_three.set(0.0);
    conveyor_four.set(0.0);
  }
 
}