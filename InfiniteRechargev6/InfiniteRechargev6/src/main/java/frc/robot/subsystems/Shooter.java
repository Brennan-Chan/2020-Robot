/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
//import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.*;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  //flywheel motors 
  WPI_TalonFX Shooter = new WPI_TalonFX(17);
  WPI_TalonFX Shooter2 = new WPI_TalonFX(18);
  
  //hood angle adjuster
  public WPI_TalonSRX angleMan = new WPI_TalonSRX(20);

  //hood encoders 
  //public Encoder freedomEncoder = new Encoder(0,1); 
  
  //instanciate doubles 
  private double velo = 0.0;
  private double m_angle = 0.0;
  
  public Shooter(){
    //reset the motors
    Shooter.configFactoryDefault();
    Shooter2.configFactoryDefault();
    angleMan.configFactoryDefault();

    //configure the motors
    configureMotors();
    
    //create the followers
    Shooter2.follow(Shooter);
  }
 
  /**
  * 
  * MOTOR CONFIGURATIONS 
  * 
  */

  //default configurations for the flywheel motors 
  private void configureMotors(){
    //configure the shooter1
    Shooter.setInverted(false);
    Shooter.configOpenloopRamp(2.0, 0);
    Shooter.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0,0);
    Shooter.setSensorPhase(true); 
    //configure the shooter2
    Shooter2.setInverted(true);
    Shooter2.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0,0);
    Shooter.configOpenloopRamp(2.0, 0);
    Shooter2.setSensorPhase(true); 
  }

  //configure the motors for the hood angle
 

  //Configure the motors so that it can use velocity
  public void configClosedLoop(){
    //set the faclons to read the integrated encoders 
    Shooter.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);
    Shooter2.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);

    //configure other settings 
    Shooter.configClosedloopRamp(2.0);
    Shooter2.configClosedloopRamp(2.0);
    Shooter.configAllowableClosedloopError(0, 50);
    Shooter2.configAllowableClosedloopError(0,50);

    //configure the kF to do something PID related...im not really sure what we are doing with our feed forward 
    Shooter.config_kF(0, 1023/12760);
    Shooter.config_kP(0, 0);
    Shooter.config_kD(0, 0);
    Shooter.config_kI(0, .0205);

    Shooter2.config_kF(0, 1023/12760);
    Shooter2.config_kP(0, 0);
    Shooter2.config_kD(0, 0);
    Shooter2.config_kI(0, .0205);

    /** 
      Old PID's used for the test shooter 
    Shooter.config_kF(0, 1023/12760);
    Shooter.config_kP(0, .15);
    Shooter.config_kD(0, 1.7);
    Shooter.config_kI(0, .001);

    Shooter2.config_kF(0, 1023/12760);
    Shooter2.config_kP(0, .15);
    Shooter2.config_kD(0, 1.7);
    Shooter2.config_kI(0, .001);
    */
  }

  /**
   * 
   *  CALCULATE THE FLYWHEEL VALUES AND SET THE SPEED
   * 
   */

  //find the shooters velocity
  public int getShooterVelo(){
    return (Shooter.getSelectedSensorVelocity(0)*4096/600)/2;
  }

  //find the shooter2 velocity
  public int getShooter2Velo(){
    return (Shooter2.getSelectedSensorVelocity(0)*4096/600)/2;
  }

  //get the motor current output percent 
  public double motorOutput(){
    return Shooter.getMotorOutputPercent();
  }

  //find the shooters velocity 
  public double getShooterVelocity(){
    return velo;
  }

  public double getShooterkF(double RPM){
    //change maxRPM to the shootervelo max value 
    double maxRPM = 6200;
    double percentOutput = RPM/maxRPM;

    double kF = (percentOutput*Constants.tickPerRev)/RPM;
    
    return kF;
  }
  
  //set the angular velocity of the shooter 
  public void setShootSpeed(double velocity){
    configClosedLoop();
    velo = velocity;
    Shooter.set(TalonFXControlMode.Velocity, velo);
    Shooter2.set(TalonFXControlMode.Velocity, velo);
  }

  public void setPower(double power){
    Shooter.configNominalOutputForward(0.0, 0);
    Shooter.configNominalOutputReverse(0.0, 0);
    Shooter.setNeutralMode(NeutralMode.Coast);
    Shooter.set(power);

    Shooter2.configNominalOutputForward(0.0, 0);
    Shooter2.configNominalOutputReverse(0.0, 0);
    Shooter2.setNeutralMode(NeutralMode.Coast);
    Shooter2.set(power);
  }
  /**
   * 
   * CALCULATE THE VALUES FOR THE HOODANGLE AND SET THE COMMANDS
   * 
   */

  //find the distance the encoders have travled 
  public double distancePerPulse(){
    return (Math.PI*Constants.axilDiameter/Constants.cyclesPerRev);
  }

  //find the angle of the encoder
  //public double getencoderAngle(){
    //System.out.println(freedomEncoder.getRaw());
    //return freedomEncoder.getRaw();
  //}


}