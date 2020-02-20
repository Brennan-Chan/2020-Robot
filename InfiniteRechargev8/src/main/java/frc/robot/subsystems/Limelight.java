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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
   *  LIMELIGHT AND OTHER EQUATIONS GIVEN BY MIKE AND DANTE
   * 
  */

  //calc the distance to the target
  public double distanceToTarget(){
    double offsetAngle = verticalOffset.getDouble(0);
    //System.out.println("Distance to target:" + (Config.goalHeight-Config.cameraHeight)/Math.tan(Config.mountAngle + ((offsetAngle*2*Math.PI)/360)));
    return (Config.goalHeight-Config.cameraHeight)/Math.tan(Config.mountAngle + ((offsetAngle*2*Math.PI)/360));
  }

  //calc the Optimal velocity based on distane and Dante's equation
  public double OptimalVelocity(){
    return (Constants.kHorisontalDistance + Constants.kGoalWallDist + distanceToTarget())
    /(Math.sqrt((2/Constants.kGravity)*(Constants.kWallHeight-Constants.kBallHeight-((Math.sin(69)*(Constants.kHorisontalDistance + Constants.kGoalWallDist + distanceToTarget()))/Math.cos(69))))*Math.cos(69));
  }
  
  //calc the optimal angular velocity(RPM)
  public double OptimalAngularVelocity(){
    System.out.println("Optimal RPM"+(OptimalVelocity())/(0.0508*Math.PI));
    return (OptimalVelocity())/(0.0508*Math.PI);
  }

  //set the velocity of the shooter
  public double setShooterVelocity(){
    double setVelocity = 0;

    if ((distanceToTarget() >= 0 ) && (distanceToTarget() < .5 )){
      setVelocity = 1089.095;
    }
    else if (( distanceToTarget() >= .5) && ( distanceToTarget() < 3)){
      setVelocity = 1243.94;
    }
    else if (( distanceToTarget() >= 3) && ( distanceToTarget() < 4.5)){
      setVelocity = 2717.19/2;
    }
    else if ((distanceToTarget() >= 4.5) &&( distanceToTarget() < 7.5)){
      setVelocity = (4929.93/2);
    }
    else if ((distanceToTarget() >= 7.5) &&( distanceToTarget() < 9.5)){
      setVelocity = (4012.73/2);
    }
    else if ((distanceToTarget() >= 9.5) &&( distanceToTarget() < 11.5)){
      setVelocity = (2400.00);
    }
    else if ((distanceToTarget() >= 11.5) &&( distanceToTarget() < 13.5)){
      setVelocity = 4643.31/2;
    }
    else if ((distanceToTarget() >= 13.5) &&( distanceToTarget() < 14.5)){
      setVelocity = 4929.93/2;
    }
    else if ((distanceToTarget() >= 14.5) &&( distanceToTarget() < 15.5)){
      setVelocity = 5044.58/2;
    }
    else if ((distanceToTarget() >= 15.5) &&( distanceToTarget() < 17.5)){
      setVelocity = 6305.73/2;
    }
    else if ((distanceToTarget() >= 17.5) &&( distanceToTarget() < 20.0)){
      setVelocity = 6191.08/2;
    }
    else if ((distanceToTarget() >= 20.0) &&( distanceToTarget() < 23.0)){
      setVelocity = 6191.08/2;
    }
    else if ((distanceToTarget() >= 23.0) &&( distanceToTarget() < 26.0)){
      setVelocity = 6191.08/2;
    }
    else if ((distanceToTarget() >= 26.0) &&( distanceToTarget() < 29.0)){
      setVelocity = 6191.08/2;
    }
    else if ((distanceToTarget() >= 29.0) &&( distanceToTarget() < 30.0)){
      setVelocity = 6191.08/2;
    }
    else{
      setVelocity = 0;
    }

    //System.out.println("Set Shooter velocity" + setVelocity);
    return setVelocity;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Distance to Target", distanceToTarget());
  }

}