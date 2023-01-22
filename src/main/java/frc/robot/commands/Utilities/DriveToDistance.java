// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Utilities;

import javax.naming.spi.DirStateFactory;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class DriveToDistance extends CommandBase {
  /*
  take in angle we want to turn to (expectedAngle)
  set expected angle as our "zero" angle to turn onto
  when get angle, try to turn to expected ange
   */
  double goalDistance, power, expectedAngle, leftBias, rightBias;
  /** Creates a new DriveToDistance. */
  public DriveToDistance(double goalDistance, double expectedAngle, double pwr) {
    this.goalDistance = goalDistance;
    power = pwr;
    this.expectedAngle = expectedAngle;

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

    double distLeft = goalDistance - Robot.m_robotContainer.m_DriveSub.getDist();
      double yawIntensity = (-Robot.m_robotContainer.m_DriveSub.ang  - expectedAngle)/100;
     double newpower = power;
      if((Math.abs(distLeft) < 12 && Math.abs(distLeft) > 6))
      {
         newpower = 0.3* power *  (-Math.pow(1.05946, 12 - distLeft) + 2);
         if(distLeft < 0)
         {
          newpower *= -1;
         }
         System.out.println(distLeft + " " +newpower);

          
        Robot.m_robotContainer.m_DriveSub.setDrive(newpower + yawIntensity , newpower - yawIntensity );

      }else if(Math.abs(distLeft) >= 12){
        Robot.m_robotContainer.m_DriveSub.setDrive(power + yawIntensity , power - yawIntensity );

      }

      


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
    double distLeft = goalDistance - Robot.m_robotContainer.m_DriveSub.getDist();


    if(Math.abs(distLeft) <= 6)
    {
      return true; 
    }
    
    return false;
  }
}
