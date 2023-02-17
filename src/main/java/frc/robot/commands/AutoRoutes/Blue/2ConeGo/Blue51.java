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
import frc.robot.commands.ActiveLevel;
import frc.robot.commands.Utilities.PickUpGamepiece;
import frc.robot.commands.Utilities.PlaceGamepiece;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Blue51 extends SequentialCommandGroup {
  /** Creates a new threeft. */
  public Blue51() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());


    PathPlannerTrajectory examplePath = PathPlanner.loadPath("/Paths/Blue/2ConeGo/Spot5", new PathConstraints(3, 1));
    HashMap<String, Command> eventMap = new HashMap<>();

    //events
    eventMap.put("PlaceCone1", new PlaceGamepiece(0.5, 0.2, 3));
    eventMap.put("PickUpCone2", new PickUpGamepiece(0.7, 0.2, 0.3));
    eventMap.put("PlaceCone2", new PlaceGamepiece(0.5, 0.2, 3));
    eventMap.put("PickUpFinalCone", new PickUpGamepiece(0.7, 0.2, 0.3));
    


    FollowPathWithEvents command = new FollowPathWithEvents(
      getPathFollowingCommand(examplePath),
      examplePath.getMarkers(),
      eventMap
  );

    addCommands(/* 
    new DriveToDistance(48, 15, 0.25),
     new DriveToDistance(-48,-15,  -0.25)
*/
command

    );
  }

  private Command getPathFollowingCommand(PathPlannerTrajectory examplePath) {
    return null;
  }
}
