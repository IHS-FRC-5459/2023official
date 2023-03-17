// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoRoutes;

import javax.naming.PartialResultException;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.ActiveLevel;
import frc.robot.commands.Utilities.AutoIntakeCubes;
import frc.robot.commands.Utilities.DriveCourseB;
import frc.robot.commands.Utilities.DriveCourseF;
import frc.robot.commands.Utilities.PickUpGamepiece;
import frc.robot.commands.Utilities.PlaceGamepiece;
import frc.robot.commands.Utilities.MoveIntake;
import frc.robot.commands.Utilities.TurnDegrees;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Utilities.DriveDistance;
import frc.robot.commands.Utilities.MoveClaw;
import frc.robot.commands.Utilities.MovePivot;
import frc.robot.commands.Utilities.RotateExtension;


/** Add your docs here. */
public class Num1  extends SequentialCommandGroup {
    public Num1(){
        addCommands(
            new ParallelRaceGroup(new WaitCommand(0.38), new MovePivot(-0.2)),
            new ParallelRaceGroup(new WaitCommand(0.2), new RotateExtension(1), new RotateExtension(1), new RotateExtension(1)),
            new WaitCommand(1.6),
            new ParallelRaceGroup(new WaitCommand(0.5), new MoveClaw(-0.15)),
            new ParallelRaceGroup(new WaitCommand(0.2), new RotateExtension(-1), new RotateExtension(-1), new RotateExtension(-1)),
            new DriveDistance(0.35, 194.40)
             
        );
    }
}
