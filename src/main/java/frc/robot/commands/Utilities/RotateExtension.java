// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Utilities;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class RotateExtension extends CommandBase {
  int dir = 0;
  /** Creates a new RotateExtension. */

  // dir = -1 is back, dir = 1 is forward
  public RotateExtension(int dir) {
    this.dir = dir;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(dir < 0){
      Robot.m_robotContainer.m_ArmSub.subPos();
    }else{
      Robot.m_robotContainer.m_ArmSub.addPos();

    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {


  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
