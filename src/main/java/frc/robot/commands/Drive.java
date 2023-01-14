// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSub;

public class Drive extends CommandBase {
//joystick declarations
  private Joystick leftJoystick = RobotContainer.getInstance().getleftStick();
    private Joystick rightJoystick = RobotContainer.getInstance().getrightStick();
    private final DriveSub m_DriveSub;

  /** Creates a new Drive. */
  public Drive(DriveSub driveSub) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_DriveSub = driveSub;
    addRequirements(m_DriveSub);  

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_DriveSub.setDrive(-leftJoystick.getY(), -rightJoystick.getY());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_DriveSub.setDrive(0.0, 0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  
}
