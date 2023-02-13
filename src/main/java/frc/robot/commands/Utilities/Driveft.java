// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Utilities;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class Driveft extends CommandBase {
  /** Creates a new Drive3ft. */
  private double power;
  private double goalDist, currentDist;
  private boolean isPos = true;
  public Driveft(double pwr, double distance) {
    power = pwr;
    goalDist = distance;
    if(power > 0){isPos = true;}
    else if(power < 0){isPos = false;}
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Robot.m_robotContainer.m_DriveSub.setDrive(0, 0);
    Robot.m_robotContainer.m_DriveSub.resetEncoders();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentDist = Robot.m_robotContainer.m_DriveSub.getAverageEncoderDistance();
    if(isPos){
      if(currentDist < goalDist){
        Robot.m_robotContainer.m_DriveSub.setDrive(power, power);
      }
    } else{
      if(currentDist > goalDist){
        Robot.m_robotContainer.m_DriveSub.setDrive(-power, -power);
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot.m_robotContainer.m_DriveSub.setDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(isPos){
      if(currentDist >= goalDist){
        return true;
      }
    } else{
      if(currentDist <= goalDist){
        return true;
      }
    }
    
    return false;
  }
}
