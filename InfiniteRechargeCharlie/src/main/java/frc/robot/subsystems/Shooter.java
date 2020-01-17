/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.*;

public class Shooter extends Subsystem {
  public WPI_TalonSRX Shooter = new WPI_TalonSRX(RobotMap.shoot);
  private double velo = 0.0;
  
  public Shooter(){
    configureMotors();
  }

  
  //I spliced in the important parts from the elevator command
  //that did not subequently fail in the compiler 
 
 
  private void configureMotors() {
    //TODO: set inverted thing if its not inverted
    Shooter.setInverted(true);
    Shooter.configOpenloopRamp(1.0, 0);
    Shooter.overrideLimitSwitchesEnable(false);
    Shooter.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
    Shooter.setSensorPhase(true); 
    Shooter.setSelectedSensorPosition(0, 0, 0);
    
  }
  public int getElevatorPosition() {
    return Shooter.getSelectedSensorPosition(0);
  }
  
  public void setShootSpeed(double velocity){
    velo = velocity;
    if (velo < 1){
      velo = 0;
    }
    Shooter.set(ControlMode.Velocity, velo);
  }

  public void setPower(double power){
    Shooter.configNominalOutputForward(0.0, 0);
    Shooter.configNominalOutputReverse(0.0, 0);
    Shooter.setNeutralMode(NeutralMode.Brake);
    
    Shooter.set(ControlMode.Velocity, 0.0);
    Shooter.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
    Shooter.set(power);
  }
 
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
