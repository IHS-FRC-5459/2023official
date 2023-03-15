// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Utilities;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Robot;
import frc.robot.commands.Mechanism.Arm;
import frc.robot.commands.Mechanism.Claw;
import frc.robot.commands.Mechanism.Roller;

public class MovePivot extends CommandBase {
  /** Creates a new PickUpGamepiece. */
    double pwr;
  public MovePivot(double pwr) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.pwr = pwr;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Robot.setOverridePivot(true);
    Robot.m_robotContainer.m_ArmSub.setPivot(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Robot.m_robotContainer.m_ArmSub.setPivot(pwr);

  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    Robot.m_robotContainer.m_ArmSub.setPivot(0);
    Robot.setOverridePivot(false);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
