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

public class PickUpGamepiece extends CommandBase {
  /** Creates a new PickUpGamepiece. */
  private double pP, rP, cP;
  public PickUpGamepiece(double pP, double rP, double cP) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.pP = pP;
    this.rP = rP;
    this.cP = cP;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
     Robot.m_robotContainer.m_ArmSub.resetEncoder();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
  }

  public SequentialCommandGroup getPickupPiece(){
    return new SequentialCommandGroup(
      
        new Roller(-rP),
        new Claw(cP),
        new WaitCommand(0.08),
        new Claw(-cP),
        new Roller(rP),
        new WaitCommand(0.08),
        new Arm(-pP, 0, 0, 0)
      
    );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
