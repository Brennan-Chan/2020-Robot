/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;


import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DoubleSolenoid;

public class BallFeeder extends SubsystemBase {

  public WPI_TalonSRX feeder_1 = new WPI_TalonSRX(21);
  
  private DoubleSolenoid sook = new DoubleSolenoid(2,3);

  public BallFeeder() {

  feeder_1.set(ControlMode.PercentOutput, 0);
  feeder_1.setNeutralMode(NeutralMode.Brake);
    
  }

  public void shiftFeeder(){
    switch (sook.get()){
      case kOff:
        sook.set(DoubleSolenoid.Value.kForward);
       break;
      case kForward:
        sook.set(DoubleSolenoid.Value.kReverse);
       break;
      case kReverse:
        sook.set(DoubleSolenoid.Value.kForward);
        break;
    }
  }

  public void feederOn(){
    feeder_1.set(0.6);
    return;
  }

  public void feederOFF(){
    feeder_1.set(0.0);
  }

  public void feederBACK(){
    feeder_1.set(-0.6);
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
