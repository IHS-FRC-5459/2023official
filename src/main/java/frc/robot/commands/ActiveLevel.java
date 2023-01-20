// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class ActiveLevel extends CommandBase {
  double deadspace, sensitivity;
  /** Creates a new ActiveLevel. */
  public ActiveLevel(double deadspace, double sensitivity) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.deadspace = deadspace;
    this.sensitivity = sensitivity;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Robot.m_robotContainer.m_DriveSub.setDrive(0, 0);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double pitch = Robot.m_robotContainer.m_DriveSub.getPitch();
    if(pitch < deadspace && pitch > -deadspace)
    {
      Robot.m_robotContainer.m_DriveSub.setDrive(0, 0);
    } else {
      double power = sensitivity * (Constants.drivetrain_kA * 9.81 * Math.sin(Math.toRadians(pitch)));
      Robot.m_robotContainer.m_DriveSub.setDrive(power, power);
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
