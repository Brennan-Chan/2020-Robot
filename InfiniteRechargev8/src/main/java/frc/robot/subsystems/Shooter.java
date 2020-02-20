/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.*;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.Limelight;

public class Shooter extends SubsystemBase {
  //flywheel motors 
  WPI_TalonFX Shooter = new WPI_TalonFX(15);
  WPI_TalonFX Shooter2 = new WPI_TalonFX(16);
  
  //hood angle adjuster
  public WPI_TalonSRX angleMan = new WPI_TalonSRX(20);
  
  //instanciate doubles 
  private double velo = 0.0;

  //initiate subsystems 
  private Limelight m_limelight = new Limelight();

  //create limit switch 
  public static DigitalInput limitSwitch1 = new DigitalInput(1);

  public Shooter(){
    //reset the motors
    Shooter.configFactoryDefault();
    Shooter2.configFactoryDefault();
    angleMan.configFactoryDefault();

    //configure the motors
    configureMotors();
    configClosedLoop();
    
    //create the followers
    Shooter2.follow(Shooter);
  }
 
  /**
  * 
  * MOTOR CONFIGURATIONS 
  * 
  */

  //default configurations for the flywheel motors 
  public void configureMotors(){
    //configure the shooter1
    Shooter.setInverted(false);
    Shooter.configOpenloopRamp(2.0, 0);
    Shooter.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0,0);
    Shooter.setSensorPhase(true); 
    
    
    Shooter2.setInverted(true);
    Shooter2.configOpenloopRamp(2.0, 0);
    Shooter2.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0,0);
    Shooter2.setSensorPhase(true); 

    Shooter.configClosedloopRamp(1.0);
    Shooter2.configClosedloopRamp(1.0);
  }

  //configure the motors for the hood angle
 

  //Configure the motors so that it can use velocity
  public void configClosedLoop(){
    //set the faclons to read the integrated encoders 
    Shooter.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);
    Shooter2.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);

    //configure other settings 
    Shooter.configClosedloopRamp(0.2);
    Shooter2.configClosedloopRamp(0.2);
    Shooter.configAllowableClosedloopError(0, 0);
    Shooter2.configAllowableClosedloopError(0, 0);

    //About: Create a horzontal asymatote for the velocity to reach by configuring the PID & F values 
    //TIP: change the PID loop back to zero if things start to fail 
    //Shooter.config_kF(0, getShooterkF(5642*((2048*60)/(600*16)))); 
    Shooter.config_kF(0, getShooterkF(m_limelight.setShooterVelocity()*((2048*60)/(600*16)))); 
    Shooter.config_kP(0, 0.002); 
    Shooter.config_kD(0, 0);
    Shooter.config_kI(0, 0);

    //Shooter2.config_kF(0, getShooterkF(5642*((2048*60)/(600*16))));
    Shooter2.config_kF(0, getShooterkF(m_limelight.setShooterVelocity()*((2048*60)/(600*16))));
    Shooter2.config_kP(0, 0.002);
    Shooter2.config_kD(0, 0);
    Shooter2.config_kI(0, 0);
  }

  public void config4Power(){
    Shooter.configNominalOutputForward(0.0, 0);
    Shooter.configNominalOutputReverse(0.0, 0);
    Shooter.setNeutralMode(NeutralMode.Coast);
    
    Shooter2.configNominalOutputForward(0.0, 0);
    Shooter2.configNominalOutputReverse(0.0, 0);
    Shooter2.setNeutralMode(NeutralMode.Coast);
  }

  /**
   * 
   *  CALCULATE THE FLYWHEEL VALUES AND SET THE SPEED
   * 
   */

  //find the shooters velocity
  public int getShooterVelo(){
    return Shooter.getSelectedSensorVelocity(0);
  }

  //find the shooter2 velocity
  public int getShooter2Velo(){
    return Shooter2.getSelectedSensorVelocity(0);
  }

  public int getTheoShooterVelo(){
    return (Shooter.getSelectedSensorVelocity(0)*4096/600)/2;
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
    double maxRPM = 19700; //possibly * 2048
    double percentOutput = RPM/maxRPM; //possibly (RPM * 2048)

    double kF = (percentOutput*m_limelight.setShooterVelocity())/RPM;
    //double kF = (percentOutput*5642)/RPM; //possibly (RPM * 2048)
    
    return kF;
  }
  
  //set the angular velocity of the shooter 
  public void setShootSpeed(double velocity){
    //configClosedLoop();
    velo = velocity;
    Shooter.set(TalonFXControlMode.Velocity, velo);
    Shooter2.set(TalonFXControlMode.Velocity, velo);
  }

  public int ballsLaunched(){
    int balls = 0;
    if( !(m_limelight.setShooterVelocity() == getTheoShooterVelo()) ){
       balls = balls - 1;
     }
    return balls; 
  }

  //set the power for the shooter 
  public void setPower(double power){   
    config4Power();
    Shooter.set(ControlMode.PercentOutput, power);
    Shooter2.set(ControlMode.PercentOutput, power);
  }

  /**
   * 
   * CALCULATE THE VALUES FOR THE HOODANGLE AND SET THE COMMANDS
   * 
   */

  public void configHoodClosedLoop(){

    //configure the voltage 
    angleMan.configVoltageCompSaturation(12.0,0);
    angleMan.enableVoltageCompensation(true);

    //set the limits of the shooter 
    angleMan.configReverseSoftLimitThreshold(1024);
    angleMan.configNominalOutputForward(0);

    //set the speed limits 
    angleMan.configPeakOutputForward(.6);
    angleMan.configPeakOutputReverse(-.6);

    angleMan.setInverted(true);

    //tell it how long it should take to get there 
    angleMan.configClosedloopRamp(0.10);
    
    //set the motor into positon mode and set the encoder
    angleMan.set(ControlMode.MotionMagic, 0.0);
    angleMan.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder,0,0);

    //configure the PID loop 
    angleMan.config_kP(0, 0.2);
    angleMan.config_kI(0, 0);
    angleMan.config_kD(0, 0);
    //angleMan.config_kF(0, .2);
    angleMan.config_kF(0, HoodKF());

    //set the neutral mode
    angleMan.setNeutralMode(NeutralMode.Brake);
    angleMan.configMotionAcceleration(100);
    angleMan.configMotionCruiseVelocity(50000);
  }

  public void configPositionClosedLoop(){

    //configure the voltage 
    angleMan.configFactoryDefault();

    //set the inverts 
    angleMan.setInverted(true);
    angleMan.setSensorPhase(true);
    
    //set the motor into positon mode and set the encoder
    angleMan.set(ControlMode.Position, 0.0);
    angleMan.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder,0,0);
    angleMan.setSelectedSensorPosition(0, 0, 0);

    //configure the PID loop 
    angleMan.config_kP(0, 0.2);
    angleMan.config_kI(0, 0);
    angleMan.config_kD(0, 0);
    //angleMan.config_kF(0, .2);
    angleMan.config_kF(0, HoodKF());

    //set the neutral mode
    angleMan.setNeutralMode(NeutralMode.Brake);

    //set the output 
    angleMan.configNominalOutputForward(0);
    angleMan.configNominalOutputReverse(0);
    angleMan.configPeakOutputForward(1);
    angleMan.configPeakOutputReverse(-1);
  }

  public double HoodKF(){
    double Velo = (1024/36);
    double percentOut = 100;
    double Kf = (percentOut*1023)/Velo;

    return Kf; 
  }

  public double HoodVelocity(){
    return angleMan.getSelectedSensorVelocity();
  }

  //set the hood angle with the position control mode
  public void setHoodWithAngle(double m_position){

    double targetPosition = m_position * 4096;
    
    angleMan.set(ControlMode.Position, targetPosition);
  }

  //set the angle of the hood with motion magic 
  public void setHoodWithMagic(double kHoodangle){
    double targetAngle = (1024 *  kHoodangle)/100;
     
    angleMan.set(ControlMode.MotionMagic, -targetAngle*2);
  } 
  
  public void setHoodPower(double power){
    angleMan.set(ControlMode.PercentOutput, 0);
  }
  
  public void Configcheetangle(){

    //configure the voltage 
    angleMan.configVoltageCompSaturation(12.0,0);
    angleMan.enableVoltageCompensation(true);

    //set the limits of the shooter 
    angleMan.configReverseSoftLimitThreshold(1024);
    angleMan.configNominalOutputForward(0);

    //set the speed limits 
    angleMan.configPeakOutputForward(1);
    angleMan.configPeakOutputReverse(-1);

    angleMan.setInverted(true);

    //tell it how long it should take to get there 
    angleMan.configClosedloopRamp(0.10);
    
    //set the motor into positon mode and set the encoder
    angleMan.set(ControlMode.PercentOutput, 0.0);
    angleMan.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder,0,0);

  }

  //Creates its own kinda position mode and makes it hit a certian position 
  public void cheetingAngle(double hoodAngle){

    if((getencoderAngle() <= hoodAngle)){
    
      while(getencoderAngle() < hoodAngle - .1){

        angleMan.set(-0.8);  
      }
      if(getencoderAngle() >= hoodAngle){
        
        angleMan.set(0.0);
      }
    }
    else if((getencoderAngle() >= hoodAngle)){

      while(getencoderAngle() > hoodAngle + .1){

        angleMan.set(0.8);         
      }
      if(getencoderAngle() <= hoodAngle){

        angleMan.set(0.0);
      }
    }
  }



  //find the angle of the encoder
  public double getencoderAngle(){
    //find the encoder angle based on degrees 
    return angleMan.getSelectedSensorPosition(0)* 0.0878906;
  }

  //returns the angle in raw encoder ticks 
  public double actualEncoderAngle(){
    return angleMan.getSelectedSensorPosition(0);
  }

  //reset the angle of the encoder 
  public void resetHoodEncoder(){
    angleMan.setSelectedSensorPosition(0);
  }

  //see if the limit switch is active 
  public boolean activeLimitSwitch(){
    boolean m_limitActive;
    
    m_limitActive = false;
    while (limitSwitch1.get() == false){
      m_limitActive = true;
    }
    while (limitSwitch1.get() == true){
      m_limitActive = false;
    }
    return m_limitActive;
  }

  //table to set the hood angle cause arrays are hard with frc
  public double hoodAngleTable(){
    double setAngle = 0;

    if ((m_limelight.distanceToTarget() >= 0 ) && (m_limelight.distanceToTarget() < .5 )){
      setAngle = 3;
    }
    else if ((m_limelight.distanceToTarget() >= .5 ) && (m_limelight.distanceToTarget() < 3 )){
      setAngle = 27;
    }
    else if ((m_limelight.distanceToTarget() >= 3 ) && (m_limelight.distanceToTarget() < 4.5 )){
      setAngle = 41;
    }
    else if ((m_limelight.distanceToTarget() >= 4.5 ) && (m_limelight.distanceToTarget() < 7.5 )){
      setAngle = 61;
    }
    else if ((m_limelight.distanceToTarget() >= 7.5 ) && (m_limelight.distanceToTarget() < 9.5 )){
      setAngle = 45;
    }
    else if ((m_limelight.distanceToTarget() >= 9.5 ) && (m_limelight.distanceToTarget() < 11.5 )){
      setAngle = 45;
    }
    else if ((m_limelight.distanceToTarget() >= 11.5 ) && (m_limelight.distanceToTarget() < 13.5 )){
      setAngle = 42;
    }
    else if ((m_limelight.distanceToTarget() >= 13.5 ) && (m_limelight.distanceToTarget() < 14.5 )){
      setAngle = 32.5; //tech 28.5
    }
    else if ((m_limelight.distanceToTarget() >= 14.5 ) && (m_limelight.distanceToTarget() < 15.5 )){
      setAngle = 60;
    }
    else if ((m_limelight.distanceToTarget() >= 15.5 ) && (m_limelight.distanceToTarget() < 17.5 )){
      setAngle = 56; //tech 22.5
    }
    else if ((m_limelight.distanceToTarget() >= 17.5 ) && (m_limelight.distanceToTarget() < 20 )){
      setAngle = 59;
    }
    else if ((m_limelight.distanceToTarget() >= 20 ) && (m_limelight.distanceToTarget() < 23 )){
      setAngle = 44;
    }
    else if ((m_limelight.distanceToTarget() >= 23 ) && (m_limelight.distanceToTarget() < 26 )){
      setAngle = 68;
    }
    else if ((m_limelight.distanceToTarget() >= 26 ) && (m_limelight.distanceToTarget() < 29 )){
      setAngle = 60;
    }
    else if ((m_limelight.distanceToTarget() >= 29 ) && (m_limelight.distanceToTarget() < 30)){
      setAngle = 60;
    }
    //System.out.println("Set Shooter velocity" + setAngle);
    
    return setAngle;
  }
  
  @Override
  public void periodic() {
    //About: Display all of the values you want to monitor 
    SmartDashboard.putNumber("Shooter Velocity", getTheoShooterVelo());
    SmartDashboard.putNumber("Theoretical Hood Angle", getencoderAngle());
    SmartDashboard.putNumber("Encoder tick Hood Angle:", actualEncoderAngle());
    SmartDashboard.putNumber("Predicted Angle", hoodAngleTable());
    SmartDashboard.putNumber("Hood Velocity", HoodVelocity());
    SmartDashboard.putBoolean("Hood Limit Active", activeLimitSwitch());
  }
}