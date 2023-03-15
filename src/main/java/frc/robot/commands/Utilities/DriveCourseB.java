// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Utilities;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class DriveCourseB extends CommandBase {
  private double pwr, course, distanceInInches;
  /** Creates a new DriveCourseB. */
  public DriveCourseB(double pwr, double course, double distanceInInches) {

    if(pwr > 0){
      pwr = -pwr;
    }
    if(distanceInInches > 0){
    distanceInInches = -distanceInInches;
    }

    this.pwr = pwr;
    this.course=course;
    this.distanceInInches=distanceInInches *  0.0254;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Robot.m_robotContainer.m_DriveSub.setDrive(0, 0);
    Robot.m_robotContainer.m_DriveSub.resetEncoders();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currentAng = Robot.m_robotContainer.m_DriveSub.getYaw();
    

    if(currentAng > course){
      double distance = currentAng - course;
      distance = (distance / 100 ) * 3;
      distance = 1 - distance;
      Robot.m_robotContainer.m_DriveSub.setDrive(pwr , pwr* distance);
    } else if(currentAng < course){
      double distanceO = course - currentAng;
      distanceO = (distanceO / 100 ) * 3;
      distanceO = 1 - distanceO;
      Robot.m_robotContainer.m_DriveSub.setDrive(pwr * distanceO, pwr);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot.m_robotContainer.m_DriveSub.setDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double distanceTraveled = Robot.m_robotContainer.m_DriveSub.getAverageEncoderDistance();
    if(distanceTraveled <= distanceInInches){  
      return true;
    }
    return false;
  }
}
