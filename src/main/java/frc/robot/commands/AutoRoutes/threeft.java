// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoRoutes;

import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.commands.ActiveLevel;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class threeft extends SequentialCommandGroup {
  /** Creates a new threeft. */
  public threeft() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());


    PathPlannerTrajectory examplePath = PathPlanner.loadPath("Example Path", new PathConstraints(4, 3));
    HashMap<String, Command> eventMap = new HashMap<>();
    eventMap.put("marker1", new ActiveLevel(0, 0));
    FollowPathWithEvents command = new FollowPathWithEvents(
      getPathFollowingCommand(examplePath),
      examplePath.getMarkers(),
      eventMap
  );

    addCommands(
      command

    );
  }

  private Command getPathFollowingCommand(PathPlannerTrajectory examplePath) {
    return Robot.m_robotContainer.m_DriveSub.followTrajectoryCommand(examplePath, true);
  }
}
