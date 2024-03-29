// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Utilities;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class MoveClaw extends CommandBase {
  double pwr;
  /** Creates a new MoveClaw. */
  public MoveClaw(double pwr) {

this.pwr = pwr;    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Robot.setOverrideClaw(true);
    Robot.setClawConstant(false);
    Robot.m_robotContainer.m_ClawSub.setClaw(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Robot.m_robotContainer.m_ClawSub.setClaw(pwr);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    Robot.m_robotContainer.m_ClawSub.setClaw(0);
    Robot.setOverrideClaw(false);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
