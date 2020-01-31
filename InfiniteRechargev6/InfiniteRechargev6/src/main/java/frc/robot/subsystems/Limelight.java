/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Config;
import frc.robot.Constants;

import java.lang.Math;

public class Limelight extends SubsystemBase {
  //Instance NetworkTable to obtain realtime data from the Limelight
  public NetworkTable ll = NetworkTableInstance.getDefault().getTable("limelight");

  public NetworkTableEntry target = ll.getEntry("tv");
  public NetworkTableEntry horizontalOffset = ll.getEntry("tx");
  public NetworkTableEntry verticalOffset = ll.getEntry("ty");
  public NetworkTableEntry targetArea = ll.getEntry("ta");
  public NetworkTableEntry skew = ll.getEntry("ts");
  public NetworkTableEntry led = ll.getEntry("ledMode");
  public NetworkTableEntry cameraMode = ll.getEntry("camMode");

  //Create the numbers
  double offsetAngle = getVerticalOffset();
  double BigTarget = LargestTarget();
  //double tabledistance = arrayDistanceChooser();

  /**
   * 
   * LIMELIGHT CONSTANTS AND VARIABLES
   * 
   */
  
  //Returns x offset angle of the robot from the target
  public double getHorizontalOffset(){
    //System.out.println("horizontal offset:" + horizontalOffset.getDouble(0.0));
    return horizontalOffset.getDouble(0.0);
  }

  //Returns y offset angle of the robot from the target
  public double getVerticalOffset(){
    return verticalOffset.getDouble(0.0);
  }
  
  public double getSkew(){
    return skew.getDouble(0.0);
  }

  public void switchLED(int mode){
    //Sets LED mode (1: Off, 2: Blink, 3: On)
    switch (mode){
      case 1:
        led.setDouble(1);
        break;
      case 2:
        led.setDouble(2);
        break;
      case 3:
        led.setDouble(3);
        break;
      default:
        led.setDouble(0);
        break;
    }
    return;
  }
  public void camMode(int mode){
    //Toggles vision processing on the Limelight (0: On, 1: Off with Driver Mode for increased exposure)
    if (mode == 1) cameraMode.setDouble(1);
    else cameraMode.setDouble(0);
    return;
  }

  public double rotatetoTarget(double PID){
    //Calculates power necessary to shift drivetrain and align with the target
    double power = horizontalOffset.getDouble(0)*PID;
    return power;
  }

  public Boolean validTarget(){
    //Returns true if target is in frame, false is no valid target
    if (target.getDouble(0) == 1) return true;
    else return false;
  }

  public double LargestTarget(){
    return targetArea.getDouble(0.0);
  }

  /** 
   * 
   *  USE THE ARRAY VALUES 
   * 
  */

  //picking the correct distance in the array 
  //public double arrayDistanceChooser(){
    //TODO: make it so that it increases every array increase 
    //if(distanceToTarget() <= Config.karrayDistance[0]){
      //tabledistance = Config.karrayDistance[0];
      //return tabledistance;
    //}
    //return tabledistance;
  //}

  /** 
   * 
   *  LIMELIGHT AND OTHER EQUATIONS GIVEN BY MIKE AND DANTE
   * 
  */

  //calc the distance to the target
  public double distanceToTarget(){
    double offsetAngle = verticalOffset.getDouble(0);
    System.out.println("Distance to target:" + (Config.hpHeight-Config.cameraHeight)/Math.tan(Config.mountAngle +offsetAngle));
    return (Config.hpHeight-Config.cameraHeight)/Math.tan(Config.mountAngle + offsetAngle);
  }

  //calc the Optimal velocity based on distane and Dante's equation
  public double OptimalVelocity(){
    return (Constants.kHorisontalDistance + Constants.kGoalWallDist + distanceToTarget())
    /(Math.sqrt((2/Constants.kGravity)*(Constants.kWallHeight-Constants.kBallHeight-((Math.sin(69)*(Constants.kHorisontalDistance + Constants.kGoalWallDist + distanceToTarget()))/Math.cos(69))))*Math.cos(69));
  }
  
  //calc the optimal angular velocity(RPM)
  public double OptimalAngularVelocity(){
    return (OptimalVelocity())/(0.0508*Math.PI);
  }

}