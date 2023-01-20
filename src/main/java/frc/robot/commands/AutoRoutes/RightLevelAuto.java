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
import frc.robot.commands.Utilities.DriveToDistance;
import frc.robot.commands.Utilities.FullyRetract;
import frc.robot.commands.Utilities.TurnToAngle;

/** Add your docs here. */
public class RightLevelAuto  extends SequentialCommandGroup {
  public RightLevelAuto(){

    /// needs to be updated
    addCommands(
        // place cone
        new Arm(0.3, 2, 0, 3),
        new ParallelRaceGroup( new WaitCommand(2), new Claw(-0.2)),
        new FullyRetract(-0.3),
        // go to cone
        new DriveToDistance(12, 0.5),
        new TurnToAngle(-4.74, 0.5),
        new ParallelRaceGroup(new WaitCommand(4), new Roller(0.5)),
        new DriveToDistance(193.66, 0.5),
        // get cone
        new ParallelRaceGroup(new Arm(0.3, 1, Constants.distForArmToExToGetConeInTicks, 0)),
        new ParallelRaceGroup( new WaitCommand(2), new Claw(0.2)),
        new FullyRetract(-0.3),
        /// go back
        new ParallelRaceGroup(new DriveToDistance(-193.66, 0.5)),
        new Arm(3, 0, 0, 0),
        new TurnToAngle(0, 0.5),
        new Arm(0, 0, 0, 0),
        new DriveToDistance(-12, 0.5),
        // place cone
        new ParallelRaceGroup(new Arm(0.3, 2, 0, 2)),
        new ParallelRaceGroup( new WaitCommand(2), new Claw(-0.2)),
        new ParallelRaceGroup(new FullyRetract(-0.3)),
      // active leveling
      // go to charging station 
      new TurnToAngle(-53.8, 0.5),
      
      new DriveToDistance(49, 0.5),
      
      new TurnToAngle(0, 0.5),
      new DriveToDistance(36,0.5),
      // level
      new ActiveLevel(0, 1)//IKS  what deadspace, sensitivity values should be
    );
  }

}
