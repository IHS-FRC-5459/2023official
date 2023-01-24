// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoRoutes;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.ActiveLevel;
import frc.robot.commands.Mechanism.Arm;
import frc.robot.commands.Mechanism.Claw;
import frc.robot.commands.Mechanism.Roller;
import frc.robot.commands.Utilities.FullyRetract;

/** Add your docs here. */
public class MiddleLevelAuto extends SequentialCommandGroup{
    public MiddleLevelAuto(){
        //17
    addCommands(/* 
      // place cone
      new Arm(0.3, 2, 0, 3),
      new ParallelRaceGroup( new WaitCommand(2), new Claw(-0.2)),
      new FullyRetract(-0.3),
      new ParallelRaceGroup(new WaitCommand(4), new Roller(0.5)),
      // go to cone
  //    new DriveToDistance(189, 2),
      // pick up cone
      new ParallelRaceGroup(new Arm(0.3, 1, Constants.distForArmToExToGetConeInTicks, 0)),
      new ParallelRaceGroup( new WaitCommand(2), new Claw(0.2)),
      new FullyRetract(-0.3),
      new Arm(3, 0, 0, 0),
      // drive back
 //     new ParallelRaceGroup(new DriveToDistance(-189, 0.5)),
      // place cone
      new Arm(0.3, 2, 0, 2),
      new ParallelRaceGroup( new WaitCommand(2), new Claw(-0.2)),
      
      // active leveling
      // go there
      new ParallelRaceGroup(new FullyRetract(-0.3)),
 //     new DriveToDistance(93.69,0.5),
      // level
      new ActiveLevel(0, 1)//IDK  what deadspace, sensitivity values should be, or really what any of these values should be. I just plugged in random numbers, tmeporarily

   */ );
    }
}
