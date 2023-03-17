// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Utilities;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoScore extends SequentialCommandGroup {
  /** Creates a new AutoScore. */
  public AutoScore() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(

      new ParallelRaceGroup(new WaitCommand(0.3), new MoveClaw(-0.25)),
      new ParallelRaceGroup(new WaitCommand(0.1), new RotateExtension(-1),new RotateExtension(-1),new RotateExtension(-1)),
      new WaitCommand(0.5),
      new ParallelRaceGroup(new WaitCommand(0.6), new MovePivot(0.2))
      //new ParallelCommandGroup(new ParallelRaceGroup(new WaitCommand(0.62), new MoveClaw(0.2)))
    );
  }
}
