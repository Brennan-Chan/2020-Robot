/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;
import frc.robot.commands.*;


import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.*;

import frc.robot.RobotMap;
/*
This class is the glue that binds the controls on the physical operator
interface (controller) to the commands and command groups that allow control of the robot.
*/
public class OI {
	//Driver: Drivetrain control, Operator: Other functions
	public static Joystick driver = new Joystick(0);
	public static Joystick operator = new Joystick(1);
	public static Joystick autonomousSelect = new Joystick(2);
	
	/* Logitech controller binds (replaces Start & Select)
	static Button Back = new JoystickButton(driver, 7);
	static Button Start = new JoystickButton(driver, 8);
	*/

	//Attach raw button inputs to their corresponding names
	Button A = new JoystickButton(driver, 1);
	Button B = new JoystickButton(driver, 2);
	Button X = new JoystickButton(driver, 3);
	Button Y = new JoystickButton(driver, 4);
	Button LB = new JoystickButton(driver, 5);
	Button RB = new JoystickButton(driver, 6);
	Button Back = new JoystickButton(driver, 7);
	Button Start = new JoystickButton(driver, 8);
	Button LeftStick = new JoystickButton(driver, 9);
	Button RightStick = new JoystickButton(driver, 10);

	Button opA = new JoystickButton(operator, 1);
	Button opB = new JoystickButton(operator, 2);
	Button opX = new JoystickButton(operator, 3);
	Button opY = new JoystickButton(operator, 4);
	Button opLB = new JoystickButton(operator, 5);
	Button opRB = new JoystickButton(operator, 6);
	Button opStart = new JoystickButton(operator, 7);
	Button opSelect = new JoystickButton(operator, 8);

	Button dUp = new POVButton(driver, 0);
	Button dUpRight = new POVButton(driver, 45);
	Button dRight = new POVButton(driver, 90);
	Button dDownRight = new POVButton(driver, 135);
	Button dDown = new POVButton(driver, 180);
	Button dDownLeft = new POVButton(driver, 225);
	Button dLeft = new POVButton(driver, 270);
	Button dUpLeft = new POVButton(driver, 325);
	
	Button opdUp = new POVButton(operator, 0);
	Button opdUpRight = new POVButton(operator, 45);
	Button opdRight = new POVButton(operator, 90);
	Button opdDownRight = new POVButton(operator, 135);
	Button opdDown = new POVButton(operator, 180);
	Button opdDownLeft = new POVButton(operator, 225);
	Button opdLeft = new POVButton(operator, 270);
	Button opdUpLeft = new POVButton(operator, 325);

	public static int leftX = 0;
	public static int leftY = 1;
	public static int rightX = 4;
	public static int rightY = 5;
	public static int leftTrigger = 2;
  	public static int rightTrigger = 3;
	  
  public OI(){
	
	

	
  }	
}
