// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoRoutes;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Mechanism.Arm;
import frc.robot.commands.Mechanism.Claw;
import frc.robot.commands.Utilities.DriveToDistance;
import frc.robot.commands.Utilities.FullyRetract;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PlaceOne extends SequentialCommandGroup {
  /** Creates a new PlaceOne. */
  public PlaceOne() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());


    /*
     *X - Extend arm to high
     * X- Open claw
     * -   retract 
     * - drive back 8ft // 96in
     */
    addCommands(
      new Arm(0.3, 2, 3),
      new ParallelRaceGroup(new WaitCommand(0.5), new Claw(-0.2)),
      new FullyRetract(-0.3),
      new DriveToDistance(140, 0.5)
    );
  }
}
