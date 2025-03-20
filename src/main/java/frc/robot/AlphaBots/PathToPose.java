package frc.robot.AlphaBots;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.AlphaBots.AprilTagSystem.PoseFinder;
import frc.robot.AlphaBots.AprilTagSystem.config;
import frc.robot.commands.C_Align;

public class PathToPose {

    public static PathConstraints constraints = new PathConstraints(4.2, 9.0, 2 * Math.PI, 4 * Math.PI);

    public static Command C_OnTheFlyAlign(Pose2d PosePositionGoal){
        //if you want the pose to flip when on red use pathfindToPoseFlipped
      Command pathfindingCommand = AutoBuilder.pathfindToPose(
        PosePositionGoal,
        PathToPose.constraints,
        0.0);
    
      return pathfindingCommand;
    }

    public static Command C_OnTheFlyWaypointAlign(Pose2d PosePositionGoal){
      Pose2d TargetPose = PosePositionGoal;
    
       List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
              //MantaState.DriveTrain.getState().Pose,
              new Pose2d(2.12,6.28,Rotation2d.fromDegrees(-53)),
              PosePositionGoal
      );
    
      PathPlannerPath path = new PathPlannerPath(
        waypoints,
        PathToPose.constraints,
        null, // The ideal starting state, this is only relevant for pre-planned paths, so can be null for on-the-fly paths.
        new GoalEndState(0.0, TargetPose.getRotation()) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect
        );
        path.preventFlipping = true;
        
        return AutoBuilder.followPath(path);
    }

    public static SequentialCommandGroup ontheFlyThenPidReverseStraightAlign(int TagID)
    {
      return PathToPose.ontheFlyThenPidReverseStraightAlign(TagID, config.ontheFlyDistanceFromCorrect);
    }

    public static SequentialCommandGroup ontheFlyThenPidReverseStraightAlign(int TagID, double OntheFlyPadding)
    {
      return new SequentialCommandGroup(
        new PrintCommand("Reverse Straight Aligned To Tag ID " + TagID + "!"),
        C_OnTheFlyAlign(PoseFinder.getReverseStraightOutLoc(1,config.ontheFlyDistanceFromCorrect)),
        new C_Align(PoseFinder.getReverseStraightOutLoc(1,0.0))
      );
    }

    
    
}
