// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Utilities;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class DriveToDistance extends CommandBase {
  double goalDistance, power;
  /** Creates a new DriveToDistance. */
  public DriveToDistance(double goalDistance, double pwr) {
    this.goalDistance = goalDistance;
    power = pwr;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {  
    Robot.m_robotContainer.m_DriveSub.setDrive(0, 0);
    Robot.m_robotContainer.m_DriveSub.zeroEnc();
    Robot.m_robotContainer.m_DriveSub.zeroYaw();



  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double distLeft = goalDistance - Robot.m_robotContainer.m_DriveSub.dist;
      double yawIntensity = -Robot.m_robotContainer.m_DriveSub.ang/100;
      if(distLeft <12)
      {
        power *= Math.pow(-1.05946, distLeft);
        if(power < 0.1)
        {
          power = 0.1;
        }
      }
      Robot.m_robotContainer.m_DriveSub.setDrive(power + yawIntensity, power - yawIntensity);
    

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot.m_robotContainer.m_DriveSub.setDrive(0, 0);
    Robot.m_robotContainer.m_DriveSub.zeroEnc();
    Robot.m_robotContainer.m_DriveSub.zeroYaw();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double distLeft = goalDistance - Robot.m_robotContainer.m_DriveSub.dist;


    if(goalDistance < 0)
    {
      if(distLeft >=0.5)
      {
        return true;
      }
    
    }else{
      if(distLeft <= 0.5)
      {
        return true;
      }
    }
    return false;
  }
}
