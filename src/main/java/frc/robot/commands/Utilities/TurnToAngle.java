// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Utilities;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;


public class TurnToAngle extends CommandBase {
  private static double goalAngle, startAngle, power;
  private static boolean posAng = true;
  private static double currentAng = Robot.m_robotContainer.m_DriveSub.getYaw() % 360;
  /** Creates a new TurnToAngle. */
  public TurnToAngle(double ang, double pwr) {
    // Use addRequirements() here to declare subsystem dependencies.
    goalAngle = ang;
    power = pwr;
    // sees if input is posotive
    if(goalAngle < 0){
      posAng = false;
    }
    // gets start angle of robot, before turning
    startAngle = Robot.m_robotContainer.m_DriveSub.getYaw();
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // set both sides to 0
    Robot.m_robotContainer.m_DriveSub.setDrive(0, 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if input angle is posotive
    if(posAng){
      // sees if robot is too far tilted right
      while(currentAng > goalAngle){
        
        Robot.m_robotContainer.m_DriveSub.setDrive(-power, power); // turns robot left
        currentAng = Robot.m_robotContainer.m_DriveSub.getYaw() % 360;// updates currentAngle
      }
      // sees if robot it too far tilted left
      while(currentAng < goalAngle){
        
        Robot.m_robotContainer.m_DriveSub.setDrive(power, -power); // turns robot right
        currentAng = Robot.m_robotContainer.m_DriveSub.getYaw() % 360;//updates currentAngle
      }
      
    }
    // if input angle is negative
    else{
      // too far tilted left
      while(currentAng < goalAngle){
        
        Robot.m_robotContainer.m_DriveSub.setDrive(power, -power); // turns robot right
        currentAng = Robot.m_robotContainer.m_DriveSub.getYaw() % 360; // updates curentAng
        
      }
      // too far tilted right
      while(currentAng > goalAngle){
        Robot.m_robotContainer.m_DriveSub.setDrive(-power, power); // turns robot left
        currentAng = Robot.m_robotContainer.m_DriveSub.getYaw() % 360;//updates currentAng
      }

    }
    
  }
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
// ran when isFinish
Robot.m_robotContainer.m_DriveSub.setDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(currentAng == goalAngle)// checks if  robot has reached desired angle
    {
      return true; // ends program
    }
    return false;
  }
}
