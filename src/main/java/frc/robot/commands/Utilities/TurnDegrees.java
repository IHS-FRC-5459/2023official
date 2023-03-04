// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Utilities;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class TurnDegrees extends CommandBase {
  /** Creates a new TurnDegrees. */
  private double goalAngle, currentAng, power, powerL, powerR, powerM;
  
  public TurnDegrees(double pwr, double goalAngle) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.goalAngle = goalAngle;
    power = pwr;
    powerM = 1;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Robot.m_robotContainer.m_DriveSub.resetYaw();

    Robot.m_robotContainer.m_DriveSub.setDrive(0, 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   
    Robot.m_robotContainer.m_DriveSub.setDrive(0, 0);

    currentAng = Robot.m_robotContainer.m_DriveSub.getYaw();

    //double yawIntensity = Robot.m_robotContainer.m_DriveSub.getYaw()/100;

    
    if(goalAngle < 0){
      if(Math.abs(goalAngle) - Math.abs(currentAng) <8){
        powerM = 0.5;
      }
      Robot.m_robotContainer.m_DriveSub.setDrive(-power * powerM, power * powerM);
    } else{
      if(goalAngle - currentAng <8){
        powerM = 0.5;
      }
      Robot.m_robotContainer.m_DriveSub.setDrive(power * powerM, -power * powerM);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
    Robot.m_robotContainer.m_DriveSub.setDrive(0, 0);
    Robot.m_robotContainer.m_DriveSub.resetYaw();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(goalAngle > 0){
      if(currentAng > goalAngle){
        return true;
      } 
    }else{
      if(currentAng < goalAngle){
        return true;
      }
    }
    return false;
  }
}
