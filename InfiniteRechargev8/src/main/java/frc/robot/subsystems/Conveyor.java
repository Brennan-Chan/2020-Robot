/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.Timer;

//this is pretty simple it can't get more bare bones than this 
public class Conveyor extends SubsystemBase {
  //instanciate talons 
  public final WPI_TalonSRX conveyor_one = new WPI_TalonSRX(35);
  public final WPI_TalonSRX conveyor_two = new WPI_TalonSRX(32);
  public final WPI_TalonFX converyor_up = new WPI_TalonFX(31);
  
  public Conveyor() {

  }

  //make go forward 
  public void intaketoShooter(){
    conveyor_one.set(.4);
    conveyor_two.set(-.2);
    converyor_up.set(0.6);
  }
  //make go backward
  public void reverseIntake(){
    conveyor_one.set(-0.4);
    conveyor_two.set(0.2);
    converyor_up.set(-0.4);
  }

  //stop 
  public void shitStop(){
    conveyor_one.set(0.0);
    conveyor_two.set(0.0);
    converyor_up.set(0.0);
  }
}
  