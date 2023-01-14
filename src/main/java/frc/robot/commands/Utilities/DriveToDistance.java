// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Utilities;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class DriveToDistance extends CommandBase {
  double goalDistance, pwr, startDistance, startYaw;
  /** Creates a new DriveToDistance. */
  public DriveToDistance(double goalDistance, double pwr) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.goalDistance = goalDistance;// IN INCHES
    this.pwr = pwr;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Robot.m_robotContainer.m_DriveSub.setDrive(0, 0);
    startDistance = Robot.m_robotContainer.m_DriveSub.getDistance();
    startYaw = Robot.m_robotContainer.m_DriveSub.getYaw();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double yaw = (Robot.m_robotContainer.m_DriveSub.getYaw() - startYaw)/100;
    if(pwr < 0)
    {
      Robot.m_robotContainer.m_DriveSub.setDrive(pwr + yaw, pwr - yaw);

    }else{
      Robot.m_robotContainer.m_DriveSub.setDrive(pwr - yaw, pwr + yaw);

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
    if(pwr < 0)
    {
      if(Robot.m_robotContainer.m_DriveSub.getDistance() <= (startDistance - goalDistance))
      {
        return true;
      }
    } else {
      if(Robot.m_robotContainer.m_DriveSub.getDistance() >= (goalDistance + startDistance) ){
        return true;
      }
    }

    return false;
  }
}
