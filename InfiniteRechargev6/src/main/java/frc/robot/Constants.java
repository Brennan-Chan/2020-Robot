/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    //Drive Train Kinematics

    public static double trackWidthMeters = 0.77;//In meters
    public static double wheelSize = 6.5; //In inches
    public static int voltageSaturation = 10; //Voltage Compensation Saturation Constant


    //FRC Characterization Constants
    //Run the tool and then plug in the values
    public static double kS = 2.43;
    public static double kV = 1.19;
    public static double kA = 0.261;
    public static double kP = 5.0;
    public static double kD = 0;

    //Set these for how fast you want the robot to run in auto (in feet per second)
    public static double autoMaxVelocity = 2;
    public static double autoMaxAcceleration = 0.3;

    //Ramsete constants
    //Do not change these unless you read the docs!
    public static double b = 2;
    public static double zeta = 0.7;


    //Math Variables for various things
    public static int ticksPerRev = 2048;
    public static double pulsesPerMeter = (8.45*ticksPerRev)/(Math.PI*Units.inchesToMeters(wheelSize));

    public static double metersPerPulse = (Math.PI*Units.inchesToMeters(wheelSize))/(ticksPerRev*8.45);

    public static final double kEncoderDistancePerPulse =
    // Assumes the encoders are directly mounted on the wheel shafts
    (Units.inchesToMeters(wheelSize) * Math.PI) / ((double) ticksPerRev*8.45);

    //limelight constants 
    public static final double kBallHeight = 0.5; //meters (change to final value when robot gets built )
    public static final double kHorisontalDistance = .2; //meters ^^
    public static final double kGoalWallDist = 0.0254; //meters 
    public static final double kWallHeight = 2.4892; //meters 
    public static final double kShooterAngle = 69; //radians 
    public static final double kGravity = 9.81; //meters per second

    //shooter constants
    public static double tickPerRev = 1024;

    //hood contants 
    public static double cyclesPerRev = 360;
    public static double axilDiameter = 1;

    //difine distance table 
    public TableElement[] tableElementArray = new TableElement[10];

    
}