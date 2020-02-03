/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
//import edu.wpi.first.wpilibj.interfaces.Gyro;
//import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveTrain extends SubsystemBase {

  //Lets create the motorcontrollers
  public WPI_TalonFX leftMaster = new WPI_TalonFX(12);
  public WPI_TalonFX leftSlaveOne = new WPI_TalonFX(13);

  public WPI_TalonFX rightMaster = new WPI_TalonFX(10);
  public WPI_TalonFX rightSlaveOne = new WPI_TalonFX(11);
  
  //Create the Gyro
  PigeonIMU gyro = new PigeonIMU(19);
  AHRS ahrs = new AHRS(SPI.Port.kMXP);
  //Gyro dumbgyro = new ADXRS450_Gyro(SPI.Port.kOnboardCS0);
  //Create the Drive Train for regular driving
  DifferentialDrive dt = new DifferentialDrive(rightMaster, leftMaster);

  public DoubleSolenoid sook = new DoubleSolenoid(0,1);
  public Compressor ruuuuuuuum = new Compressor();
  //Initializers for Ramsete
  DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Constants.trackWidthMeters);
  DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(getHeading());

  SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.kS, Constants.kV, Constants.kA);
  //Creates internal PID controllers for both sides of the drivetrain
  PIDController leftPIDController = new PIDController(Constants.kP, 0, 0);
  PIDController rightPIDController = new PIDController(Constants.kP, 0, 0);

  Pose2d pose;

  //things for motion magic 
  StringBuilder _sb = new StringBuilder();
  int _smoothing = 0;
  int _pov = -1;

  public DriveTrain() {
  
    leftMaster.configFactoryDefault();
    leftSlaveOne.configFactoryDefault();
    rightMaster.configFactoryDefault();
    rightSlaveOne.configFactoryDefault();
    //Configures the MagEncoders into Relative mode
    leftMaster.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    rightMaster.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    leftMaster.setSelectedSensorPosition(0);
    rightMaster.setSelectedSensorPosition(0);

    //Live life on the edge and turn off safety mode
    dt.setSafetyEnabled(false);

    //Set the sensor phase
    leftMaster.setSensorPhase(true);
    rightMaster.setSensorPhase(true);



    //Set inverted
    rightMaster.setInverted(true);
    rightSlaveOne.setInverted(true);
    leftMaster.setInverted(false);
    leftSlaveOne.setInverted(false);

    //This makes the slave controllers follow the output values of the master controllers
    leftSlaveOne.follow(leftMaster);
    rightSlaveOne.follow(rightMaster);

    //Enables voltage compensation, it will take the battery voltage into account when trying to drive the robot.
    leftMaster.enableVoltageCompensation(false);
    rightMaster.enableVoltageCompensation(false);
    leftMaster.configVoltageCompSaturation(Constants.voltageSaturation);
    rightMaster.configVoltageCompSaturation(Constants.voltageSaturation);


    leftMaster.setNeutralMode(NeutralMode.Coast);
    rightMaster.setNeutralMode(NeutralMode.Coast);
    leftSlaveOne.setNeutralMode(NeutralMode.Coast);
    rightSlaveOne.setNeutralMode(NeutralMode.Coast);


  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
     odometry.update(getHeading(),
     getLeftDistance(), 
     getRightDistance());
    
     pose = getPose();
     ramseteDash();

  }

  public void compressorON(){
    ruuuuuuuum.setClosedLoopControl(true);
  }
  public void shiftPiston(){
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
/*-------------------------------------------------------*/
/*------------INSERT OTHER BITS OF CODE HERE-------------*/
/*-------------------------------------------------------*/

public void arcadeDrive(double speed, double turn){
  dt.arcadeDrive(speed, -turn);
  return;
}

//set the max output which the drive train will be constrained 
public void setMaxOutput(double maxOutput){
  dt.setMaxOutput(maxOutput);
}

public void ebrake(){
  leftMaster.setNeutralMode(NeutralMode.Brake);
  rightMaster.setNeutralMode(NeutralMode.Brake);
  leftSlaveOne.setNeutralMode(NeutralMode.Brake);
  rightSlaveOne.setNeutralMode(NeutralMode.Brake);
}public void noebrake(){
  leftMaster.setNeutralMode(NeutralMode.Coast);
  rightMaster.setNeutralMode(NeutralMode.Coast);
  leftSlaveOne.setNeutralMode(NeutralMode.Coast);
  rightSlaveOne.setNeutralMode(NeutralMode.Coast);
}
/*-------------------------------------------------------*/
/*--------------------RAMSETE STUFF----------------------*/
/*-------------------------------------------------------*/


  public void resetGyro(){
  //gyro.setYaw(0);
    ahrs.reset();
  }


  //Stores the angle of the gyro in radians
  public Rotation2d getHeading(){
    //double heading = getAngle();
    return Rotation2d.fromDegrees(ahrs.getAngle());
  }

  //turn rate of the robot in degrees per second 
  public double getTurnRate(){
    return ahrs.getRate()*(true ? -1.0 : 1.0);
  }

  //public double getHeading(){
  //  return Math.IEEEremainder(getAngle(), 360) * (false ? -1.0 : 1.0);
  //}



  //Obtains the speeds of both sides of the drive train in m/s
  public DifferentialDriveWheelSpeeds getSpeeds() {
    return new DifferentialDriveWheelSpeeds(
      leftMaster.getSelectedSensorVelocity()*Constants.metersPerPulse*10, //Converts units/100ms to m/s
      rightMaster.getSelectedSensorVelocity()*Constants.metersPerPulse*10);
  }

  //Obtains the position of the robot in a 2d space
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public SimpleMotorFeedforward getFeedForward(){
    return feedforward;
  }

  public PIDController getleftPIDController(){
    return leftPIDController;
  }

  public PIDController getrightPIDController(){
    return rightPIDController;
  }

  public DifferentialDriveKinematics getKinematics(){
    return kinematics;
  }

  //This is what the controller will use to set the power to all the motors
  public void setOutput(double leftVoltage, double rightVoltage){
    leftMaster.setVoltage(rightVoltage);
    //one of the sides is negative 
    rightMaster.setVoltage(-leftVoltage);
    return;
  }



/*------------------HELPERS FOR RAMSETE-------------------*/

  //Resets the Pose when needed
  public void resetOdometry(Pose2d pose){
    resetEncoders();
    odometry.resetPosition(pose, Rotation2d.fromDegrees(ahrs.getAngle()));
    return;
  }

  //Resets the Encoders
  public void resetEncoders(){
    leftMaster.setSelectedSensorPosition(0);
    rightMaster.setSelectedSensorPosition(0);
    return;
  }

  //Gives us a way to get the left and right wheel distances in meters
  public double getLeftDistance(){
    return leftMaster.getSelectedSensorPosition()/Constants.pulsesPerMeter;
  }
  public double getRightDistance(){
    return rightMaster.getSelectedSensorPosition()/Constants.pulsesPerMeter;
  }

  public void ramseteDash(){
    SmartDashboard.putNumber("Robot Angle", ahrs.getAngle());
    SmartDashboard.putNumber("Left Encoder Distance", getLeftDistance());
    SmartDashboard.putNumber("Right Encoder Distance", getRightDistance());
    SmartDashboard.putNumber("Left Encoder Speed", leftMaster.getSelectedSensorVelocity()*Constants.metersPerPulse*10);
    SmartDashboard.putNumber("Right Encoder Speed", rightMaster.getSelectedSensorVelocity()*Constants.metersPerPulse*10);
    SmartDashboard.putNumber("Encoder Units", leftMaster.getSelectedSensorPosition());

    SmartDashboard.putNumber("rightMaster Current", rightMaster.getSupplyCurrent());
    SmartDashboard.putNumber("rightSlaveOne Current", rightSlaveOne.getSupplyCurrent());

    SmartDashboard.putNumber("Left Master Output", leftMaster.getMotorOutputPercent());
    SmartDashboard.putNumber("Right Master Output", rightMaster.getMotorOutputPercent());
  }

  /**
   *
   * MOTION MAGIC CODE (EXPIRAMENTAL)
   * 
   */

  public void config4MotionMagic(){
    //set the motors to factory default 
    leftMaster.configFactoryDefault();
    leftSlaveOne.configFactoryDefault();
    rightMaster.configFactoryDefault();
    rightSlaveOne.configFactoryDefault();

    //Configures the MagEncoders into Relative mode
    leftMaster.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    rightMaster.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    leftMaster.setSelectedSensorPosition(0);
    rightMaster.setSelectedSensorPosition(0);

    //Live life on the edge and turn off safety mode
    dt.setSafetyEnabled(false);

    //Set the sensor phase
    leftMaster.setSensorPhase(false);
    rightMaster.setSensorPhase(false);

    //Set inverted
    rightMaster.setInverted(true);
    rightSlaveOne.setInverted(true);
    leftMaster.setInverted(false);
    leftSlaveOne.setInverted(false);

    //This makes the slave controllers follow the output values of the master controllers
    leftSlaveOne.follow(leftMaster);
    rightSlaveOne.follow(rightMaster);

    //Enables voltage compensation, it will take the battery voltage into account when trying to drive the robot.
    leftMaster.enableVoltageCompensation(false);
    rightMaster.enableVoltageCompensation(false);
    leftMaster.configVoltageCompSaturation(Constants.voltageSaturation);
    rightMaster.configVoltageCompSaturation(Constants.voltageSaturation);

    //config the outputs 
    leftMaster.configNominalOutputForward(0, 0);
    rightMaster.configNominalOutputForward(0, 0);

    leftMaster.configPeakOutputForward(.6, 0);
    rightMaster.configPeakOutputForward(.6, 0);

    //config the neuteral mode 
    leftMaster.setNeutralMode(NeutralMode.Brake);
    rightMaster.setNeutralMode(NeutralMode.Brake);
    leftSlaveOne.setNeutralMode(NeutralMode.Brake);
    rightSlaveOne.setNeutralMode(NeutralMode.Brake);
  }

  public void driveMotionMagic(double wheelrot){
    double targetPosition = Constants.ticksPerRev * wheelrot;

    rightMaster.set(ControlMode.MotionMagic, targetPosition);
    leftMaster.set(ControlMode.MotionMagic, targetPosition);
  }

}