package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.StructEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.AlphaBots.AprilTagSystem.AprilTag.TagType;
import frc.robot.AlphaBots.AprilTagSystem.PoseFinder;
import frc.robot.AlphaBots.AprilTagSystem.config;
import frc.robot.AlphaBots.NT;

public class ChassisTelemetryUpdater extends SubsystemBase
  {
    StructEntry<Pose2d> NT_myloc = NT.getStructEntry_Pose2D("Poses","TagLoc",new Pose2d());
    StructEntry<Pose2d> NT_myStraightloc = NT.getStructEntry_Pose2D("Poses","StraightLoc",new Pose2d());//straight out from the april tag. a centered pick for de-algae/processor/pickup 
    StructEntry<Pose2d> NT_Leftloc = NT.getStructEntry_Pose2D("Poses","leftLoc",new Pose2d());//LEFT FROM Robot TOWARDS tag view!
    StructEntry<Pose2d> NT_rightloc = NT.getStructEntry_Pose2D("Poses","rightLoc",new Pose2d());//Right Given View Robot TOWARDS tag!
    StructEntry<Pose2d> NT_ClosestTag = NT.getStructEntry_Pose2D("Poses","ClosestTag",new Pose2d());//shows closest tag to robotchassis
    StructEntry<Pose2d> NT_Simloc = NT.getStructEntry_Pose2D("Poses","ChassisLoc",new Pose2d());//NT_Simloc
    StructEntry<Pose2d> NT_ClosestSource = NT.getStructEntry_Pose2D("Poses","ClosestSource",new Pose2d());
    StructEntry<Pose2d> NT_ClosestReef = NT.getStructEntry_Pose2D("Poses","ClosestReef",new Pose2d());
    StructEntry<Pose2d> NT_ClosestProcessor = NT.getStructEntry_Pose2D("Poses","ClosestProcessor",new Pose2d());
    StructEntry<Pose2d> NT_ClosestBarge = NT.getStructEntry_Pose2D("Poses","ClosestBarge",new Pose2d());
    //DoubleEntry NT_tagID = NT.getDoubleEntry("", "TagID", chosenAprilTagID);
    
  
    public ChassisTelemetryUpdater(CommandSwerveDrivetrain _drivetrain)
    {
      config.drivetrain =_drivetrain;//this is required to be able to use SelectCommands that will need our drivetrains position for Getting Closest Tag. 
    } 

    
    @Override
    public void periodic() {
      //NT_myloc.set(ourtag.Pose);
      Pose2d SimRobotChassisLoc = config.drivetrain.getState().Pose;
     
      // NT_myStraightloc.set(AprilTagManager.getStraightOutLoc(chosenAprilTagID, Units.inchesToMeters(6)));
      // NT_Leftloc.set(getOffSet90Loc(chosenAprilTagID, Units.inchesToMeters(6), constants.ReefWidthCenteronCenter,true));
      // NT_rightloc.set(getOffSet90Loc(chosenAprilTagID, Units.inchesToMeters(6), constants.ReefWidthCenteronCenter,false));
      if(DriverStation.isDSAttached())
      {
        NT_Simloc.set(SimRobotChassisLoc);
        NT_ClosestSource.set(PoseFinder.getClosestTagofTypeToRobotCenter(SimRobotChassisLoc,TagType.Source).Pose);
        NT_ClosestProcessor.set(PoseFinder.getClosestTagofTypeToRobotCenter(SimRobotChassisLoc,TagType.Processor).Pose);
        NT_ClosestReef.set(PoseFinder.getClosestTagofTypeToRobotCenter(SimRobotChassisLoc,TagType.Reef).Pose);
        NT_ClosestBarge.set(PoseFinder.getClosestTagofTypeToRobotCenter(SimRobotChassisLoc,(DriverStation.getAlliance().isPresent() & DriverStation.getAlliance().get().equals(Alliance.Blue))? TagType.BlueBarge:TagType.RedBarge).Pose);
        NT_ClosestTag.set(PoseFinder.getClosestTagToRobotCenter(SimRobotChassisLoc).Pose);
      }
  
    }
  }