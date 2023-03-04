// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoRoutes;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Utilities.DriveDistance;

/** Add your docs here. */
public class threeftog extends SequentialCommandGroup{
    public threeftog(){
        addCommands(
            
            new DriveDistance(0.2, 150),
            new DriveDistance(-0.2, 150)
            

    );
}
}
