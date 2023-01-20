// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Mechanism;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class Roller extends CommandBase {
  /** Creates a new Roller. */
  private static double power;
  public Roller(double pwr) {
    // Use addRequirements() here to declare subsystem dependencies.
    power = pwr;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // sets roller to 0 power
    Robot.m_robotContainer.m_RollerSub.setRoller(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
     //sets roller to the desired power
    Robot.m_robotContainer.m_RollerSub.setRoller(power);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // sets roller to 0 when done
    Robot.m_robotContainer.m_RollerSub.setRoller(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
