/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {
  // The motors on the left side of the drive.
  private final WPI_TalonFX m_leftMotor1 = new WPI_TalonFX(DriveConstants.kLeftMotor1Port);
  private final WPI_TalonFX m_leftMotor2 = new WPI_TalonFX(DriveConstants.kLeftMotor2Port);

  private final WPI_TalonFX m_rightMotor1 = new WPI_TalonFX(DriveConstants.kRightMotor1Port);
  private final WPI_TalonFX m_rightMotor2 = new WPI_TalonFX(DriveConstants.kRightMotor2Port);

  // The robot's drive
  private final DifferentialDrive m_drive;

  // The left-side drive encoder

  // The gyro sensor
  private final AHRS m_gyro = new AHRS(SPI.Port.kMXP);

  // Odometry class for tracking robot pose
  private final DifferentialDriveOdometry m_odometry;

  /**
   * Creates a new DriveSubsystem.
   */
  public DriveSubsystem() {
    m_leftMotor1.configFactoryDefault(10);
    m_leftMotor2.configFactoryDefault(10);
    m_rightMotor1.configFactoryDefault(10);
    m_rightMotor2.configFactoryDefault(10);

    m_leftMotor1.setInverted(false);
    m_leftMotor2.setInverted(false);
    m_rightMotor1.setInverted(false);
    m_rightMotor2.setInverted(false);

    m_leftMotor2.follow(m_leftMotor1);
    m_rightMotor2.follow(m_rightMotor1);

    m_leftMotor1.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 10);
    m_rightMotor1.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 10);
    m_leftMotor1.setSelectedSensorPosition(0, 0, 10);
    m_rightMotor1.setSelectedSensorPosition(0, 0, 10);

    m_drive = new DifferentialDrive(m_leftMotor1, m_rightMotor1);

    resetEncoders();
    m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getYaw()));
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(Rotation2d.fromDegrees(getYaw()), m_leftMotor1.getSelectedSensorPosition(), m_rightMotor1.getSelectedSensorPosition());
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(m_leftMotor1.getSelectedSensorVelocity(0), m_rightMotor1.getSelectedSensorVelocity(0));
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(pose, Rotation2d.fromDegrees(getYaw()));
  }

  /**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  public void arcadeDrive(double fwd, double rot) {
    m_drive.arcadeDrive(fwd, rot);
  }

  /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts  the commanded left output
   * @param rightVolts the commanded right output
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    m_leftMotor1.set(leftVolts);
    m_rightMotor1.set(rightVolts);
  }

  /**
   * Resets the drive encoders to currently read a position of 0.
   */
  public void resetEncoders() {
    m_leftMotor1.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 10);
    m_rightMotor1.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 10);
    m_leftMotor1.setSelectedSensorPosition(0, 0, 10);
    m_rightMotor1.setSelectedSensorPosition(0, 0, 10);
  }

  /**
   * Gets the average distance of the two encoders.
   *
   * @return the average of the two encoder readings
   */
  public double getAverageEncoderDistance() {
    return (m_leftMotor1.getSelectedSensorPosition() + m_rightMotor1.getSelectedSensorPosition()) / 2.0;
  }

  /**
   * Sets the max output of the drive. Useful for scaling the drive to drive more
   * slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }

  /**
   * Zeroes the heading of the robot.
   */
  public void zeroHeading() {
    m_gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from 180 to 180
   */
  public double getYaw() {
    return m_gyro.getYaw() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }
}
