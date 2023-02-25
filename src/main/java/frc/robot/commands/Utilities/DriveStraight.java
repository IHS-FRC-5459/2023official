// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Utilities;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class DriveStraight extends CommandBase {
  /** Creates a new DriveStraight. */
  double pwr;
  public DriveStraight(double pwr) {
    this.pwr = pwr;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Robot.m_robotContainer.m_DriveSub.setDrive(0, 0);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double yawIntensity = -1 * (Robot.m_robotContainer.m_DriveSub.getYaw())/100;
    if(pwr >0){
        Robot.m_robotContainer.m_DriveSub.setDrive(pwr + yawIntensity, pwr - yawIntensity);
    }else{
      Robot.m_robotContainer.m_DriveSub.setDrive(pwr + yawIntensity, pwr - yawIntensity);

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
    return false;
  }
}
