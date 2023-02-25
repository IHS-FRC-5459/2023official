// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Utilities;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class RotateLED extends CommandBase {
  /** Creates a new RotateLED. */
  public RotateLED() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot.m_robotContainer.m_LEDSub.addPos();
    if(Robot.m_robotContainer.m_LEDSub.getSet() == 0)
    {
      if(Robot.m_robotContainer.m_DriveSub.getDirection() > 0)
      {
        Robot.m_robotContainer.m_LEDSub.setBlinkIn(0.77);

      } else {
        Robot.m_robotContainer.m_LEDSub.setBlinkIn(0.61);


      }
    } else { 
      Robot.m_robotContainer.m_LEDSub.setBlinkIn(Robot.m_robotContainer.m_LEDSub.getSet());


    }
       

    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
