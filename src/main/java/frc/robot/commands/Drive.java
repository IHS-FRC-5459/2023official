// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DriveSub;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;


public class Drive extends CommandBase {
//joystick declarations
  private Joystick leftJoystick = Robot.m_robotContainer.getleftStick();
    private Joystick rightJoystick = Robot.m_robotContainer.getrightStick();

  /** Creates a new Drive. */
  public Drive() {
    // Use addRequirements() here to declare subsystem dependencies.

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}
    
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double s = Robot.m_robotContainer.m_DriveSub.getSlow() ? 1 : 0.5;
    // sets motors to joystick

    if(Robot.m_robotContainer.m_DriveSub.getDirection() < 0)
    {
      Robot.m_robotContainer.m_DriveSub.setDrive(  s * rightJoystick.getY(),   s * leftJoystick.getY());
    } else {
      Robot.m_robotContainer.m_DriveSub.setDrive(s* -leftJoystick.getY(), s* -rightJoystick.getY());
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // shuts motors off
    Robot.m_robotContainer.m_DriveSub.setDrive(0.0, 0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  
}
