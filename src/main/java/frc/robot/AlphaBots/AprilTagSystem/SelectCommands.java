package frc.robot.AlphaBots.AprilTagSystem;

import java.util.Map;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import frc.robot.AlphaBots.PathToPose;
import frc.robot.AlphaBots.AprilTagSystem.AprilTag.TagType;
import frc.robot.commands.C_Align;

public class SelectCommands {

    public static int selectSource() {
    return PoseFinder.getClosestTagofTypeToRobotCenter(config.drivetrain.getState().Pose,TagType.Source).ID;
    }

    public static int selectProcessor() {
      return PoseFinder.getClosestTagofTypeToRobotCenter(config.drivetrain.getState().Pose,TagType.Processor).ID;
    }

    public static int selectReef() {
    return PoseFinder.getClosestTagofTypeToRobotCenter(config.drivetrain.getState().Pose,TagType.Reef).ID;
    }

    public static int selectBarge() {
      TagType BargeToSelect = DriverStation.getAlliance().equals(Alliance.Blue) ? TagType.RedBarge:TagType.BlueBarge;
      return PoseFinder.getClosestTagofTypeToRobotCenter(config.drivetrain.getState().Pose,BargeToSelect).ID;
    }

    public static final SelectCommand C_SourceSelectCommand(){
       return new SelectCommand<>(
            // Maps selector values to commands
            Map.ofEntries(
                Map.entry(1, PathToPose.ontheFlyThenPidReverseStraightAlign(1)),
    
                Map.entry(2, PathToPose.ontheFlyThenPidReverseStraightAlign(2)),
                
                Map.entry(12, PathToPose.ontheFlyThenPidReverseStraightAlign(12)),
    
                Map.entry(13, PathToPose.ontheFlyThenPidReverseStraightAlign(13))
                ),
    
            ()->{return selectSource();});
    
    }
    
    public static final SelectCommand C_BargeSelectCommand(DoubleSupplier yAxisOverride){
        return new SelectCommand<>(
              // Maps selector values to commands
              Map.ofEntries(
                  Map.entry(4, new PrintCommand("Command 4 blue was selected!")
                  .alongWith(new C_Align(PoseFinder.getStraightOutLoc(4,0.0),yAxisOverride))),
      
                  Map.entry(14, new PrintCommand("Command 14 blue was selected!")
                  .alongWith(new C_Align(PoseFinder.getStraightOutLoc(14,0),yAxisOverride))),
                  
                  Map.entry(5, new PrintCommand("Command 5 red was selected!")
                  .alongWith(new C_Align(PoseFinder.getStraightOutLoc(5,0),yAxisOverride))),
      
                  Map.entry(15, new PrintCommand("Command 15 red was selected!")
                  .alongWith(new C_Align(PoseFinder.getStraightOutLoc(15,0),yAxisOverride)))
                  ),
      
              ()->{return selectBarge();});
      
    }

    public static final SelectCommand C_ProcessorSelectCommand(){
      return new SelectCommand<>(
            // Maps selector values to commands
            Map.ofEntries(
                Map.entry(3, new PrintCommand("Processor red 3 was selected!")
                .alongWith(new C_Align(PoseFinder.getStraightOutLoc(3,0.0)))),
    
                Map.entry(16, new PrintCommand("Processor blue 16 was selected!")
                .alongWith(new C_Align(PoseFinder.getStraightOutLoc(16,0.0))))
            ),
            ()->{return selectProcessor();});
    
    }

    public static final SelectCommand C_ReefLeftSelectCommand(){ 
      return new SelectCommand<>(
              // Maps selector values to commands
              Map.ofEntries(
                  Map.entry(6, new PrintCommand("Command 6 was selected!")
                  .alongWith(new C_Align(PoseFinder.getOffSet90Loc(6,0.0,config.ReefWidthCenterOffset,true)))),
    
                  Map.entry(7, new PrintCommand("Command 7 was selected!")
                  .alongWith(new C_Align(PoseFinder.getOffSet90Loc(7,0,config.ReefWidthCenterOffset,true)))),
                  
                  Map.entry(8, new PrintCommand("Command 8 was selected!")
                  .alongWith(new C_Align(PoseFinder.getOffSet90Loc(8,0,config.ReefWidthCenterOffset,true)))),
    
                  Map.entry(9, new PrintCommand("Command 9 was selected!")
                  .alongWith(new C_Align(PoseFinder.getOffSet90Loc(9,0,config.ReefWidthCenterOffset,true)))),
                  Map.entry(10, new PrintCommand("Command 10 was selected!")
                  .alongWith(new C_Align(PoseFinder.getOffSet90Loc(10,0,config.ReefWidthCenterOffset,true)))),
                  Map.entry(11, new PrintCommand("Command 11 was selected!")
                  .alongWith(new C_Align(PoseFinder.getOffSet90Loc(11,0,config.ReefWidthCenterOffset,true)))),
    
                  Map.entry(17, new PrintCommand("Command 17 was selected!")
                  .alongWith(new C_Align(PoseFinder.getOffSet90Loc(17,0,config.ReefWidthCenterOffset,true)))),
                  Map.entry(18, new PrintCommand("Command 18 was selected!")
                  .alongWith(new C_Align(PoseFinder.getOffSet90Loc(18,0,config.ReefWidthCenterOffset,true)))),
                  Map.entry(19, new PrintCommand("Command 19 was selected!")
                  .alongWith(new C_Align(PoseFinder.getOffSet90Loc(19,0,config.ReefWidthCenterOffset,true)))),
                  Map.entry(20, new PrintCommand("Command 20 was selected!")
                  .alongWith(new C_Align(PoseFinder.getOffSet90Loc(20,0,config.ReefWidthCenterOffset,true)))),
                  Map.entry(21, new PrintCommand("Command 21 was selected!")
                  .alongWith(new C_Align(PoseFinder.getOffSet90Loc(21,0,config.ReefWidthCenterOffset,true)))),
                  Map.entry(22, new PrintCommand("Command 22 was selected!")
                  .alongWith(new C_Align(PoseFinder.getOffSet90Loc(22,0,config.ReefWidthCenterOffset,true))))
                  
                  ),
                  
    
              ()->{return selectReef();});
    }

    public static final SelectCommand C_ReefRightSelectCommand(){ 
        return
          new SelectCommand<>(
              // Maps selector values to commands
              Map.ofEntries(
                  Map.entry(6, new PrintCommand("Command 6 Right was selected!")
                  .alongWith(new C_Align(PoseFinder.getOffSet90Loc(6,0.0,config.ReefWidthCenterOffset,false)))),
    
                  Map.entry(7, new PrintCommand("Command 7 Right was selected!")
                  .alongWith(new C_Align(PoseFinder.getOffSet90Loc(7,0,config.ReefWidthCenterOffset,false)))),
                  
                  Map.entry(8, new PrintCommand("Command 8 was selected!")
                  .alongWith(new C_Align(PoseFinder.getOffSet90Loc(8,0,config.ReefWidthCenterOffset,false)))),
    
                  Map.entry(9, new PrintCommand("Command 9 was selected!")
                  .alongWith(new C_Align(PoseFinder.getOffSet90Loc(9,0,config.ReefWidthCenterOffset,false)))),
                  Map.entry(10, new PrintCommand("Command 10 was selected!")
                  .alongWith(new C_Align(PoseFinder.getOffSet90Loc(10,0,config.ReefWidthCenterOffset,false)))),
                  Map.entry(11, new PrintCommand("Command 11 was selected!")
                  .alongWith(new C_Align(PoseFinder.getOffSet90Loc(11,0,config.ReefWidthCenterOffset,false)))),
    
                  Map.entry(17, new PrintCommand("Command 17 was selected!")
                  .alongWith(new C_Align(PoseFinder.getOffSet90Loc(17,0,config.ReefWidthCenterOffset,false)))),
                  Map.entry(18, new PrintCommand("Command 18 was selected!")
                  .alongWith(new C_Align(PoseFinder.getOffSet90Loc(18,0,config.ReefWidthCenterOffset,false)))),
                  Map.entry(19, new PrintCommand("Command 19 was selected!")
                  .alongWith(new C_Align(PoseFinder.getOffSet90Loc(19,0,config.ReefWidthCenterOffset,false)))),
                  Map.entry(20, new PrintCommand("Command 20 was selected!")
                  .alongWith(new C_Align(PoseFinder.getOffSet90Loc(20,0,config.ReefWidthCenterOffset,false)))),
                  Map.entry(21, new PrintCommand("Command 21 was selected!")
                  .alongWith(new C_Align(PoseFinder.getOffSet90Loc(21,0,config.ReefWidthCenterOffset,false)))),
                  Map.entry(22, new PrintCommand("Command 22 was selected!")
                  .alongWith(new C_Align(PoseFinder.getOffSet90Loc(22,0,config.ReefWidthCenterOffset,false))))
                  
                  ),
                  
    
              ()->{return selectReef();});
    }

    public static final SelectCommand C_ReefCenterAlgaeSelectCommand(){ 
        return
          new SelectCommand<>(
              // Maps selector values to commands
              Map.ofEntries(
                  Map.entry(6, new PrintCommand("Command 6 Center was selected!")
                  .alongWith(new C_Align(PoseFinder.getStraightOutLoc(6,config.ExtraMetersoffsetForAlgaePickup)))),
    
                  Map.entry(7, new PrintCommand("Command 7  Center was selected!")
                  .alongWith(new C_Align(PoseFinder.getStraightOutLoc(7,config.ExtraMetersoffsetForAlgaePickup)))),
                  
                  Map.entry(8, new PrintCommand("Command 8 was selected!")
                  .alongWith(new C_Align(PoseFinder.getStraightOutLoc(8,config.ExtraMetersoffsetForAlgaePickup)))),
    
                  Map.entry(9, new PrintCommand("Command 9 was selected!")
                  .alongWith(new C_Align(PoseFinder.getStraightOutLoc(9,config.ExtraMetersoffsetForAlgaePickup)))),
                  Map.entry(10, new PrintCommand("Command 10 was selected!")
                  .alongWith(new C_Align(PoseFinder.getStraightOutLoc(10,config.ExtraMetersoffsetForAlgaePickup)))),
                  Map.entry(11, new PrintCommand("Command 11 was selected!")
                  .alongWith(new C_Align(PoseFinder.getStraightOutLoc(11,config.ExtraMetersoffsetForAlgaePickup)))),
    
                  Map.entry(17, new PrintCommand("Command 17 was selected!")
                  .alongWith(new C_Align(PoseFinder.getStraightOutLoc(17,config.ExtraMetersoffsetForAlgaePickup)))),
                  Map.entry(18, new PrintCommand("Command 18 was selected!")
                  .alongWith(new C_Align(PoseFinder.getStraightOutLoc(18,config.ExtraMetersoffsetForAlgaePickup)))),
                  Map.entry(19, new PrintCommand("Command 19 was selected!")
                  .alongWith(new C_Align(PoseFinder.getStraightOutLoc(19,config.ExtraMetersoffsetForAlgaePickup)))),
                  Map.entry(20, new PrintCommand("Command 20 was selected!")
                  .alongWith(new C_Align(PoseFinder.getStraightOutLoc(20,config.ExtraMetersoffsetForAlgaePickup)))),
                  Map.entry(21, new PrintCommand("Command 21 was selected!")
                  .alongWith(new C_Align(PoseFinder.getStraightOutLoc(21,config.ExtraMetersoffsetForAlgaePickup)))),
                  Map.entry(22, new PrintCommand("Command 22 was selected!")
                  .alongWith(new C_Align(PoseFinder.getStraightOutLoc(22,config.ExtraMetersoffsetForAlgaePickup))))
                  
                  ),
                  
    
              ()->{return selectReef();});
    }

    public static final SelectCommand C_ReefL1CenterSelectCommand(){ 
    return
      new SelectCommand<>(
          // Maps selector values to commands
          Map.ofEntries(
              Map.entry(6, new PrintCommand("L1Alignment 6 Center was selected!")
              .alongWith(new C_Align(
                new Pose2d(
                  PoseFinder.getOffSet90Loc(6,0,config.L1AlignmentOffsetMeters,false).getX(),
                  PoseFinder.getOffSet90Loc(6,0,config.L1AlignmentOffsetMeters,false).getY(),
                  Rotation2d.fromDegrees(PoseFinder.getOffSet90Loc(6,0,config.L1AlignmentOffsetMeters,false).getRotation().getDegrees() + config.L1TwistOffsetDegrees)
                  )
              ))),
    
              Map.entry(7, new PrintCommand("L1Alignment 7  Center was selected!")
              .alongWith(new C_Align(
                new Pose2d(
                  PoseFinder.getOffSet90Loc(7,0,config.L1AlignmentOffsetMeters,false).getX(),
                  PoseFinder.getOffSet90Loc(7,0,config.L1AlignmentOffsetMeters,false).getY(),
                  Rotation2d.fromDegrees(PoseFinder.getOffSet90Loc(7,0,config.L1AlignmentOffsetMeters,false).getRotation().getDegrees() + config.L1TwistOffsetDegrees)
                  )
              ))),
              
              Map.entry(8, new PrintCommand("L1Alignment 8 was selected!")
              .alongWith(new C_Align(
                new Pose2d(
                  PoseFinder.getOffSet90Loc(8,0,config.L1AlignmentOffsetMeters,false).getX(),
                  PoseFinder.getOffSet90Loc(8,0,config.L1AlignmentOffsetMeters,false).getY(),
                  Rotation2d.fromDegrees(PoseFinder.getOffSet90Loc(8,0,config.L1AlignmentOffsetMeters,false).getRotation().getDegrees() + config.L1TwistOffsetDegrees)
                  )
              ))),    
              Map.entry(9, new PrintCommand("L1Alignment 9 was selected!")
              .alongWith(new C_Align(
                new Pose2d(
                  PoseFinder.getOffSet90Loc(9,0,config.L1AlignmentOffsetMeters,false).getX(),
                  PoseFinder.getOffSet90Loc(9,0,config.L1AlignmentOffsetMeters,false).getY(),
                  Rotation2d.fromDegrees(PoseFinder.getOffSet90Loc(9,0,config.L1AlignmentOffsetMeters,false).getRotation().getDegrees() + config.L1TwistOffsetDegrees)
                  )
              ))),                  
              Map.entry(10, new PrintCommand("L1Alignment 10 was selected!")
              .alongWith(new C_Align(
                new Pose2d(
                  PoseFinder.getOffSet90Loc(10,0,config.L1AlignmentOffsetMeters,false).getX(),
                  PoseFinder.getOffSet90Loc(10,0,config.L1AlignmentOffsetMeters,false).getY(),
                  Rotation2d.fromDegrees(PoseFinder.getOffSet90Loc(10,0,config.L1AlignmentOffsetMeters,false).getRotation().getDegrees() + config.L1TwistOffsetDegrees)
                  )
              ))),                  
              Map.entry(11, new PrintCommand("L1Alignment 11 was selected!")
              .alongWith(new C_Align(
                new Pose2d(
                  PoseFinder.getOffSet90Loc(11,0,config.L1AlignmentOffsetMeters,false).getX(),
                  PoseFinder.getOffSet90Loc(11,0,config.L1AlignmentOffsetMeters,false).getY(),
                  Rotation2d.fromDegrees(PoseFinder.getOffSet90Loc(11,0,config.L1AlignmentOffsetMeters,false).getRotation().getDegrees() + config.L1TwistOffsetDegrees)
                  )
              ))),
              Map.entry(17, new PrintCommand("L1Alignment 17 was selected!")
              .alongWith(new C_Align(
                new Pose2d(
                  PoseFinder.getOffSet90Loc(17,0,config.L1AlignmentOffsetMeters,false).getX(),
                  PoseFinder.getOffSet90Loc(17,0,config.L1AlignmentOffsetMeters,false).getY(),
                  Rotation2d.fromDegrees(PoseFinder.getOffSet90Loc(17,0,config.L1AlignmentOffsetMeters,false).getRotation().getDegrees() + config.L1TwistOffsetDegrees)
                  )
              ))),                  
              Map.entry(18, new PrintCommand("L1Alignment 18 was selected!")
              .alongWith(new C_Align(
                new Pose2d(
                  PoseFinder.getOffSet90Loc(18,0,config.L1AlignmentOffsetMeters,false).getX(),
                  PoseFinder.getOffSet90Loc(18,0,config.L1AlignmentOffsetMeters,false).getY(),
                  Rotation2d.fromDegrees(PoseFinder.getOffSet90Loc(18,0,config.L1AlignmentOffsetMeters,false).getRotation().getDegrees() + config.L1TwistOffsetDegrees)
                  )
              ))),                  
              Map.entry(19, new PrintCommand("L1Alignment 19 was selected!")
              .alongWith(new C_Align(
                new Pose2d(
                  PoseFinder.getOffSet90Loc(19,0,config.L1AlignmentOffsetMeters,false).getX(),
                  PoseFinder.getOffSet90Loc(19,0,config.L1AlignmentOffsetMeters,false).getY(),
                  Rotation2d.fromDegrees(PoseFinder.getOffSet90Loc(19,0,config.L1AlignmentOffsetMeters,false).getRotation().getDegrees() + config.L1TwistOffsetDegrees)
                  )
              ))),                  
              Map.entry(20, new PrintCommand("L1Alignment 20 was selected!")
              .alongWith(new C_Align(
                new Pose2d(
                  PoseFinder.getOffSet90Loc(20,0,config.L1AlignmentOffsetMeters,false).getX(),
                  PoseFinder.getOffSet90Loc(20,0,config.L1AlignmentOffsetMeters,false).getY(),
                  Rotation2d.fromDegrees(PoseFinder.getOffSet90Loc(20,0,config.L1AlignmentOffsetMeters,false).getRotation().getDegrees() + config.L1TwistOffsetDegrees)
                  )
              ))),                  
              Map.entry(21, new PrintCommand("L1Alignment 21 was selected!")
              .alongWith(new C_Align(
                new Pose2d(
                  PoseFinder.getOffSet90Loc(21,0,config.L1AlignmentOffsetMeters,false).getX(),
                  PoseFinder.getOffSet90Loc(21,0,config.L1AlignmentOffsetMeters,false).getY(),
                  Rotation2d.fromDegrees(PoseFinder.getOffSet90Loc(21,0,config.L1AlignmentOffsetMeters,false).getRotation().getDegrees() + config.L1TwistOffsetDegrees)
                  )
              ))),                  
              Map.entry(22, new PrintCommand("L1Alignment 22 was selected!")
              .alongWith(new C_Align(
                new Pose2d(
                  PoseFinder.getOffSet90Loc(22,0,config.L1AlignmentOffsetMeters,false).getX(),
                  PoseFinder.getOffSet90Loc(22,0,config.L1AlignmentOffsetMeters,false).getY(),
                  Rotation2d.fromDegrees(PoseFinder.getOffSet90Loc(22,0,config.L1AlignmentOffsetMeters,false).getRotation().getDegrees() + config.L1TwistOffsetDegrees)
                  )
              )))                  
              ),
              
    
          ()->{return selectReef();});
    }
    
}
