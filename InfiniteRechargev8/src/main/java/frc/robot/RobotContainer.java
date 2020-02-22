/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

//import java.util.Arrays;
import java.util.List;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.*;

//import commands 
import frc.robot.commands.drivingCommands.ArcadeDrive;
import frc.robot.commands.drivingCommands.LimeDrive;
import frc.robot.commands.drivetrainShifters;
import frc.robot.commands.shootercommand.Align2;
import frc.robot.commands.shootercommand.FeedToWheel;
import frc.robot.commands.shootercommand.FlyWheel;
import frc.robot.commands.shootercommand.GoDown;
import frc.robot.commands.shootercommand.ReverseIntake;
import frc.robot.commands.shootercommand.hoodPositionAlign;
import frc.robot.commands.shootercommand.shooterAlign;
import frc.robot.commands.groups.Auto_A;
import frc.robot.commands.groups.MotionMagicAutoA;
//import subsystems 
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.BallFeeder;
import frc.robot.subsystems.Conveyor;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private DriveTrain drive = new DriveTrain();
  private Limelight m_limelight = new Limelight();
  private Shooter m_shooter = new Shooter();
  private Conveyor m_conveyor = new Conveyor();
  private BallFeeder m_feeder = new BallFeeder();

  //Driver:1
  public static final XboxController driver = new XboxController(0);
  //Driver:2
  //TODO: ask their preference on buttons so that I can put them in  
  public static final XboxController operator = new XboxController(1);



  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    //Create the ability to drive the robot
    drive.setDefaultCommand(new ArcadeDrive(
      ()-> driver.getY(Hand.kLeft),
      ()-> driver.getX(Hand.kRight),
      drive
    ));    
  }

  private void configureButtonBindings() {

    new JoystickButton(driver, Button.kA.value).whenHeld(new LimeDrive(drive, m_limelight));
    new JoystickButton(driver, Button.kBumperRight.value).toggleWhenPressed(new FlyWheel(m_limelight, m_shooter));
    new JoystickButton(driver, Button.kBumperLeft.value)
      .whenPressed(()-> drive.setMaxOutput(0.3))
      .whenReleased(()-> drive.setMaxOutput(1));
    new JoystickButton(driver, Button.kStickLeft.value)
      .whenPressed(()-> drive.ebrake())
      .whenReleased(()-> drive.noebrake()); 
    new JoystickButton(driver, Button.kB.value).toggleWhenPressed(new FeedToWheel(m_conveyor));
    new JoystickButton(driver, Button.kBumperLeft.value).and(new JoystickButton (driver, Button.kB.value))
      .whenActive(new ReverseIntake(m_conveyor));
    new JoystickButton(driver, Button.kX.value).whenPressed(new Align2(m_limelight, m_shooter));
    //new JoystickButton(driver, Button.kX.value).whenPressed(new hoodPositionAlign(m_limelight, m_shooter));
    new JoystickButton(driver, Button.kY.value).whenPressed(new GoDown(m_limelight, m_shooter));
    new JoystickButton(driver, Button.kStickLeft.value).and(new JoystickButton (driver, Button.kStickRight.value)).whenActive(new drivetrainShifters(drive));
    new JoystickButton(driver, Button.kBumperLeft.value).and(new JoystickButton (driver, Button.kBumperRight.value))
      .whenActive(new MotionMagicAutoA(drive, m_limelight, m_shooter, m_feeder));
      
  }
    


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    
    //create a voltage constraint so we don't accelerate too fast 
      //var autoVoltageConstraint =
        //new DifferentialDriveVoltageConstraint(
          //new SimpleMotorFeedforward(feedforward, kinematics, maxVoltage),
          //drivekinematics, 11);
          
    //Create the trajectory config
    drive.resetGyro();
    drive.resetEncoders();
    TrajectoryConfig config = new TrajectoryConfig(
      Units.feetToMeters(Constants.autoMaxVelocity),      //Set Max Velocity
      Units.feetToMeters(Constants.autoMaxAcceleration)); //Set Max Acceleration
    //Set Drivetrain kinematics to create optimal paths

    //This is a sample trajectory that will move the robot 1 Meter Forward
    config.setKinematics(drive.getKinematics());
    Trajectory testTrajectory = TrajectoryGenerator.generateTrajectory(
      new Pose2d(0, 0, new Rotation2d(0)),
      // Pass through these two interior waypoints, making an 's' curve path
      List.of(
          new Translation2d(1, 1),
          new Translation2d(2, -1)
      ),
      // End 3 meters straight ahead of where we started, facing forward
      new Pose2d(3, 0, new Rotation2d(0)),
        config                                    //Add our trajectory configuration at the end
        );
      config.setKinematics(drive.getKinematics());

    //Create the Ramsete Controller that will move our robot
    RamseteCommand pathFollower = new RamseteCommand(
      testTrajectory,                                       //Input the trajectory here
      drive::getPose,                                       //Get the pose of the robot
      new RamseteController(Constants.b, Constants.zeta),   //Set up the following controller
      drive.getFeedForward(),                               //Get the feedforward values for the robot
      drive.getKinematics(),                                //Get the Kinematics for the robot
      drive::getSpeeds,                                     //Get the wheel speeds for the robot
      drive.getleftPIDController(),                         //Get the PID Controllers for the drivetrain
      drive.getrightPIDController(),                        
      drive::setOutput,                                     //This controls the robot
      drive                                                 //Sets the requirement of needing to have the drivetrain
    );
    //return pathFollower
    return new MotionMagicAutoA(drive, m_limelight, m_shooter, m_feeder);
    //return new Auto_A(m_feeder, m_limelight, drive, m_shooter);
  }
}
