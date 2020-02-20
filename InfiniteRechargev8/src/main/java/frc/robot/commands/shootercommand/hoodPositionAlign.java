/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.shootercommand;

import edu.wpi.first.wpilibj2.command.CommandBase;

//import subystems 
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;

public class hoodPositionAlign extends CommandBase {
  private final Limelight m_limelight;
  private final Shooter m_shooter;
  private boolean hood_isFinished;
  
  public hoodPositionAlign( Limelight limelight, Shooter shooter) {
    m_limelight = limelight;
    m_shooter = shooter;
 
    addRequirements(m_limelight);
    addRequirements(m_shooter);
  }

  @Override
  public void initialize() {
    //config and prep the motors 
    m_shooter.configPositionClosedLoop();
    hood_isFinished = false;
  }

  @Override
  public void execute() {
    //set the position with the hood angle table 
    m_shooter.setHoodWithAngle(m_shooter.hoodAngleTable());
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("Hood is finished @ the angle:" + m_shooter.getencoderAngle());
  }

  @Override
  public boolean isFinished() {
    //it will finish once the angle = the desired angle 
    if(m_shooter.getencoderAngle() == m_shooter.hoodAngleTable()){
      hood_isFinished = true;
      
      return true;
    }
    else{
      return hood_isFinished;
    }
  }
}