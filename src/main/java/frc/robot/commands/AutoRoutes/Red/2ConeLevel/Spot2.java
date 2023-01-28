// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.



import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.commands.ActiveLevel;
import frc.robot.commands.Utilities.PickUpGamepiece;
import frc.robot.commands.Utilities.PlaceGamepiece;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Spot2 extends SequentialCommandGroup {
  /** Creates a new threeft. */
  public Spot2() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());


    PathPlannerTrajectory examplePath = PathPlanner.loadPath("/Paths/Red/2ConeLevel/Spot2", new PathConstraints(3, 1));
    HashMap<String, Command> eventMap = new HashMap<>();

    //events
    eventMap.put("Start/place cone 1", new PlaceGamepiece(0.5, 0.2, 3));
    eventMap.put("Pick up cone 2", new PickUpGamepiece(0.7, 0.2, 0.3));
    eventMap.put("Place cone 2", new PlaceGamepiece(0.5, 0.2, 3));
    eventMap.put("Level", new ActiveLevel(0, 1));


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
