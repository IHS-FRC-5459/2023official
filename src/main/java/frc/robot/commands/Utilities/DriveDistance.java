// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Utilities;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class DriveDistance extends CommandBase {
  double distance, pwr;
  /** Creates a new DriveDistance. */
  public DriveDistance(double pwr, double inches) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.pwr = pwr;
    this.distance = inches * 0.0254;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Robot.m_robotContainer.m_DriveSub.setDrive(0,0);
    Robot.m_robotContainer.m_DriveSub.resetEncoders();
    Robot.m_robotContainer.m_DriveSub.resetYaw();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double yawIntensity = -1 * (Robot.m_robotContainer.m_DriveSub.getYaw())/100;
    if(pwr >0){
        Robot.m_robotContainer.m_DriveSub.setDrive(pwr + yawIntensity, pwr - yawIntensity);
    }else{
      Robot.m_robotContainer.m_DriveSub.setDrive(pwr - yawIntensity, pwr - yawIntensity);

    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot.m_robotContainer.m_DriveSub.setDrive(0,0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double distanceTravelled = Robot.m_robotContainer.m_DriveSub.getAverageEncoderDistance();
    if(pwr>0)
    {
      if(distanceTravelled >= distance){
        return true;
      }
    }else{
      if(distanceTravelled <= -distance){
        return true;
      }
    }
    return false;
  }
}
