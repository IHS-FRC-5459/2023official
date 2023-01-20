// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Utilities;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class FullyRetract extends CommandBase {
  double pwr;
  /** Creates a new FullyRetract. */
  public FullyRetract(double pwr) {
    this.pwr = pwr;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Robot.m_robotContainer.m_ArmSub.setExtend(0); // turns off arm
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Robot.m_robotContainer.m_ArmSub.setExtend(-pwr); // brings arm back 

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot.m_robotContainer.m_ArmSub.setExtend(0); // turns off arm

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(Robot.m_robotContainer.m_ArmSub.getLimitSwitch()){
      return true;
    }
    return false;
  }
}
