// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoRoutes.Blue;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Utilities.PlaceGamepiece;

/** Add your docs here. */
public class Top  extends SequentialCommandGroup {
    public Top(){
        addCommands(
            new PlaceGamepiece(0.3, 0.1, 3)
        );
    }
}
