// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Utilities;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class SlowSwitch extends CommandBase {
  /** Creates a new SlowSwitch. */
  public SlowSwitch() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    boolean isS = Robot.m_robotContainer.m_DriveSub.getSlow();
    Robot.m_robotContainer.m_DriveSub.setSlow(!isS);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //Robot.m_robotContainer.m_DriveSub.setSlow(true);


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    boolean isS = Robot.m_robotContainer.m_DriveSub.getSlow();
    Robot.m_robotContainer.m_DriveSub.setSlow(!isS);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
