// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Utilities;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

/** Add your docs here. */
public class AutoIntakeCubes extends SequentialCommandGroup{
    public AutoIntakeCubes(){
        addCommands(
            /*//new ParallelRaceGroup(new WaitCommand(0.4), new MovePivot(0.2)),
            //new ParallelRaceGroup(new WaitCommand(0.25), new MoveClaw(-0.4))
            //new ParallelRaceGroup(new WaitCommand(0.3), new MovePivot(-0.3), new MoveIntake(0.35))
            */
            new MoveIntake(0.1),
            new ParallelRaceGroup(
                new WaitCommand(0.8), 
                new MovePivot(0.25),
                new ParallelRaceGroup(new WaitCommand( 0.62), new MoveClaw(-0.2))
                )
                
            /*
             * new ParallelRaceGroup(new WaitCommand(0.62), MoveClaw(0.2)),
             * new ParallelRaceGroup(new WaitCommand(0.8), new MovePivot(-0.25))
             * new MoveIntake(0)
            */
        );
    }
}
