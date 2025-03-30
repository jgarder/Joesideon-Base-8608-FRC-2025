package frc.robot.AlphaBots;

import java.util.List;
import java.util.Set;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.AlphaBots.AprilTagSystem.PoseFinder;
import frc.robot.AlphaBots.AprilTagSystem.config;
import frc.robot.commands.C_Align;
import frc.robot.subsystems.MantaState;

public class PathToPose {

    public static PathConstraints constraints = new PathConstraints(5.0, 9.0, 2 * Math.PI, 4 * Math.PI);

    public static Command C_OnTheFlyAlign(Pose2d PosePositionGoal){
        //if you want the pose to flip when on red use pathfindToPoseFlipped
      Command pathfindingCommand = AutoBuilder.pathfindToPose(
        PosePositionGoal,
        PathToPose.constraints,
        0.0);
    
      return pathfindingCommand;
    }

    //this command cannot be bound unless it is a defeered command, it need to be constructed and ran at runtime not at boot during normal construction. 
    //new DeferredCommand(()->{return C_OnTheFlyWaypointAlign(PoseFinder.getReverseStraightOutLoc(TagID,config.ontheFlyDistanceFromCorrect));},Set.of(MantaState.DriveTrain)),

    public static Command C_OnTheFlyWaypointAlign(Pose2d PosePositionGoal){
      Pose2d TargetPose = PosePositionGoal;
    
       List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
              AutoBuilder.getCurrentPose(),
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
    public static double DoNotPathPlannInRangemeters = Units.inchesToMeters(48);
    public static double PathplannerSpeedSettleTimerSeconds = 0.0;
    
    public static SequentialCommandGroup ontheFlyThenPidReverseStraightAlign(int TagID, double OntheFlyPadding)
    {
      return new SequentialCommandGroup(
        new PrintCommand("Reverse Straight Aligned To Tag ID " + TagID + "!"),
        new DeferredCommand(()->{return C_OnTheFlyWaypointAlign(PoseFinder.getReverseStraightOutLoc(TagID,config.ontheFlyDistanceFromCorrect)).andThen(new WaitCommand(PathplannerSpeedSettleTimerSeconds));},Set.of(MantaState.DriveTrain))
            .unless(()->{return PoseFinder.getDistanceToTagID(MantaState.DriveTrain.getState().Pose, TagID) < DoNotPathPlannInRangemeters;})
            .unless(()->{return DriverStation.isAutonomous();}),
        new C_Align(PoseFinder.getReverseStraightOutLoc(TagID,0.0))
      );
    }
    public static SequentialCommandGroup ontheFlyThenPidReefSideAlign(int TagID,boolean positiveTrueLeft)
    {
      return PathToPose.ontheFlyThenPidReefSideAlign(TagID, config.ontheFlyDistanceFromCorrect, positiveTrueLeft);
    }

    public static SequentialCommandGroup ontheFlyThenPidReefSideAlign(int TagID, double OntheFlyPadding,boolean positiveTrueLeft)
    {
      return new SequentialCommandGroup(
        new PrintCommand("Reef Aligned (LeftSide = "+ positiveTrueLeft + ") To Tag ID " + TagID + "!"),
        new DeferredCommand(()->{return C_OnTheFlyWaypointAlign(PoseFinder.getOffSet90Loc(TagID,config.ontheFlyDistanceFromCorrect,config.ReefWidthCenterOffset,positiveTrueLeft)).andThen(new WaitCommand(PathplannerSpeedSettleTimerSeconds));},Set.of(MantaState.DriveTrain))
            .unless(()->{return PoseFinder.getDistanceToTagID(MantaState.DriveTrain.getState().Pose, TagID) < DoNotPathPlannInRangemeters;}),
        new C_Align(PoseFinder.getOffSet90Loc(TagID,0.0,config.ReefWidthCenterOffset,positiveTrueLeft))
      );
    }

    
    
}
