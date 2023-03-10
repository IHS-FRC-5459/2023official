// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Utilities;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Mechanism.Arm;
import frc.robot.commands.Mechanism.Claw;

public class PlaceGamepiece extends CommandBase {
  /** Creates a new PlaceGamepiece. */
  private double aP, cP;
  private int level;
  public PlaceGamepiece(double aP, double cP, int level) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.level = level;
    this.aP = aP;
    this.cP = cP;
   
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute(){}
  public SequentialCommandGroup getPickupPiece(){
    return new SequentialCommandGroup(
      new Arm(aP, 2, 0, level),
      new Claw(-cP),
      new WaitCommand(0.1),
      new Claw(0),
      new FullyRetract(aP)
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
