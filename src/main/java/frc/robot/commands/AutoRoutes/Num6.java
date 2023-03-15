// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoRoutes;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.ActiveLevel;
import frc.robot.commands.Utilities.DriveCourseB;
import frc.robot.commands.Utilities.DriveCourseF;
import frc.robot.commands.Utilities.DriveDistance;
import frc.robot.commands.Utilities.MoveClaw;
import frc.robot.commands.Utilities.MovePivot;
import frc.robot.commands.Utilities.PickUpGamepiece;
import frc.robot.commands.Utilities.PlaceGamepiece;
import frc.robot.commands.Utilities.RotateExtension;
import frc.robot.commands.Utilities.TurnDegrees;

/** Add your docs here. */
public class Num6   extends SequentialCommandGroup {
    public Num6(){
        System.out.println("Hi!");
        addCommands (
            new ParallelRaceGroup(new WaitCommand(0.38), new MovePivot(-0.2)),
            new ParallelRaceGroup(new WaitCommand(0.2), new RotateExtension(1), new RotateExtension(1), new RotateExtension(1)),
                  // new WaitCommand(1.75),
                  new ParallelRaceGroup(new WaitCommand(0.5), new MoveClaw(-0.25)),
                   new ParallelRaceGroup(new WaitCommand(0.2), new RotateExtension(-1), new RotateExtension(-1), new RotateExtension(-1)),
                    new DriveDistance(0.35, 194.40)
            
        );
    }
    }
