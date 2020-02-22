/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;


import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.ColorSensorV3;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class ColorSensor extends SubsystemBase {

  //About: config the color sensor for the ports onthe RIO
  public final I2C.Port i2cPort = I2C.Port.kOnboard;
  public final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);
  public final ColorMatch m_colorMatcher = new ColorMatch();

  //About: config the colors so set the values
  public final Color kBlueTarget = ColorMatch.makeColor(0.143, 0.427, 0.429);
  public final Color kGreenTarget = ColorMatch.makeColor(0.197, 0.561, 0.240);
  public final Color kRedTarget = ColorMatch.makeColor(0.561, 0.232, 0.114);
  public final Color kYellowTarget = ColorMatch.makeColor(0.361, 0.524, 0.113);
                                
  //About: set the color sensor to detect a a color based on the preset values 
  Color detectedColor = m_colorSensor.getColor(); 
  String colorString;    
  ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);

  //About: instanciate the motors for spinning the wheel
  WPI_TalonSRX wheelSpinner = new WPI_TalonSRX(24);

  //---------------------------Place Getters Here-------------------------------
  
  //Name: Brennan
  //About: get the result of the color sensor 
  public void ColorMatchResult(){
    match = m_colorMatcher.matchClosestColor(detectedColor);
  }

  //Name: Brennan 
  //About: get the kF
  public double getKf(int RPM){
    //TODO: get the max RPM of the motor to get the correct kF
    double Velo = (1024/RPM);
    double percentOut = 100;
    double Kf = (percentOut * 1024)/Velo;

    return Kf;
  }

  //Name: Brennan 
  //About: get the wheel rotations for the correct color 
  public double misfortuneRotations(){
    return 0.0;
  }

  //---------------------------Place Setters Here-------------------------------

  //Name: Brennan
  //About: make the sensor match the color 
  public void colorMatch(){
    m_colorMatcher.addColorMatch(kBlueTarget);
    m_colorMatcher.addColorMatch(kGreenTarget);
    m_colorMatcher.addColorMatch(kRedTarget);
    m_colorMatcher.addColorMatch(kYellowTarget);
  }

  //Name: Brennan 
  //About: Config the motor into motion magic mode 
  public void setWheelMotionMagic(){
    //reset the motor and config the encoder 
    wheelSpinner.configFactoryDefault();
    wheelSpinner.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    wheelSpinner.configClosedloopRamp(0);
    
    //configure the PID 
    wheelSpinner.config_kF(0, getKf(200));
    wheelSpinner.config_kP(0, .02);
    wheelSpinner.config_kI(0, 0);
    wheelSpinner.config_kD(0, 0);

    //config everything else 
    wheelSpinner.setInverted(false);
    wheelSpinner.setNeutralMode(NeutralMode.Brake);
  }
  
  //---------------------------Place Others Here-------------------------------

  //Name: Brennan
  //About: display the color on the dash board
  public void colorDash(){
    SmartDashboard.putString("Detected Color", colorString);
  }

  //Name: Brennan 
  //About: set the wheel to rotate a certain amount of rotations 
  public void wheelofMisfortune(double rotations){
    double target = Constants.tickPerRev * rotations;

    wheelSpinner.set(ControlMode.MotionMagic, target);
  }

  //Name: Brennan 
  //About: set the wheel to rotate based on power  
  public void powertheWheel(double power){
    wheelSpinner.set(ControlMode.PercentOutput, power);
  }

  @Override
  public void periodic() {
    //About: Find the color that is being detected Display the colorMatcher
    colorDash();
    colorMatch();
  }

}
