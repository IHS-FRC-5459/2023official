// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Mechanism;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class Claw extends CommandBase {
  double power;
  /** Creates a new Claw. */
  public Claw(double pwr) {
    // Use addRequirements() here to declare subsystem dependencies.
    power = pwr;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Robot.m_robotContainer.m_ClawSub.setClaw(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Robot.m_robotContainer.m_ClawSub.setClaw(power);

  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot.m_robotContainer.m_ClawSub.setClaw(0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
