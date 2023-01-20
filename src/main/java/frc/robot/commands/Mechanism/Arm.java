// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Mechanism;

import javax.swing.text.Utilities;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.commands.Utilities.FullyRetract;

public class Arm extends CommandBase {
  static double power;
  static int mode;
  static int level;
  static int ticks;
  static boolean hasReached = false;

  //parameter action: 0 => pivot 1=> extend to x ticks 2=> extend to level
  //if not using tks, for action1, set tks to 0;
  // if not using lvl, for action2, set lvl to 0;
  public Arm(double pwr, int action, int tks, int lvl) {
    // Use addRequirements() here to declare subsystem dependencies.
    power = pwr;
    mode = action;
    level = lvl;
    ticks = tks;
  }

  // pwr = arm power, action => 2 by default, level => 2 or 3 for mid/high
  
  
  /*public Arm(double pwr, int action, int level)
  {
    power = pwr;
    action = 2;
    this.level = level;

                                                                                        }         Why do we need this???????         */
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    switch(mode){
      case 0://pivot
        Robot.m_robotContainer.m_ArmSub.setPivot(0);
        break;
      case 1://arm
        Robot.m_robotContainer.m_ArmSub.setExtend(0);  
        break;
      case 2://to level
        Robot.m_robotContainer.m_ArmSub.setExtend(0);
        break;
      

    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    switch(mode){
      case 0://pivot
        Robot.m_robotContainer.m_ArmSub.setPivot(power);
        break;
      case 1:// to ticks
        extendToTicks(ticks, power);
        break;
      case 2://to level
        if(level==2)
        {
          extendToTicks(Constants.ticksToMid, power);  
        } else {
          extendToTicks(Constants.ticksToHigh, power);  
        }
        break;

    }
  }

    //level: 2 => mid 3=> high
    public void toLevel(int level, double pwr)
    {
      switch(level)
      {
        case 2:
          extendToTicks(Constants.ticksToMid, pwr);
          break;
        case 3:
          extendToTicks(Constants.ticksToHigh, pwr);
          break;
      }
    }
  
    public void extendToTicks(double ticks, double pwr)
    {
      double startTicks = Robot.m_robotContainer.m_ArmSub.getTicks();
      double goalTicks = ticks + startTicks;
  
      while(Robot.m_robotContainer.m_ArmSub.getTicks() < goalTicks)
      {
        Robot.m_robotContainer.m_ArmSub.setExtend(pwr);
      }

      hasReached = true;
    }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    switch(mode){
      case 0://pivot
        Robot.m_robotContainer.m_ArmSub.setPivot(0);
        break;
      case 1://arm
        Robot.m_robotContainer.m_ArmSub.setExtend(0);  
        break;
      case 2://to ticks
        Robot.m_robotContainer.m_ArmSub.setExtend(0);
        break;

    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(hasReached)
    {
      return true;
    }
    return false;
  }
}
