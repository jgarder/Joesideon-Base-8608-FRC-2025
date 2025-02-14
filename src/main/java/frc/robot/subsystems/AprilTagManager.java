package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meter;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.StructEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.AlphaBots.Tools;
import frc.robot.constants;
import frc.robot.AlphaBots.AprilTag;
import frc.robot.AlphaBots.AprilTag.TagType;
import frc.robot.AlphaBots.NT;

public class AprilTagManager extends SubsystemBase
  {
    //public static ArrayList<AprilTag> tagList = new ArrayList<AprilTag>(22);
    private static double RobotDefaultOffset = .25;//adds a 1/4 inch extra space at locations. 
    private static double bumperthickness = 3.5;
    private static double robotmetersdistToCenter = Units.inchesToMeters(RobotDefaultOffset+(26.75+bumperthickness*2)/2);
    public static AprilTag getTagbyID(int _ID)
    {
      for (AprilTag aprilTag : tagList) {
        if (aprilTag.ID == _ID)
        {
          return aprilTag;
        }
      }
        return new AprilTag(0, "NotFound", new Pose2d(), 0);
    }
    public static AprilTag getClosestTagToRobotCenter(Pose2d RobotLoc)
    {
      double closestSoFar = 99999;
      AprilTag closesAprilTag = new AprilTag(0, "NotFound", new Pose2d(), 0);
      for (AprilTag aprilTag : tagList) {
        double distToThisPose = Tools.getdistancetopose(RobotLoc, aprilTag.Pose);
        if (distToThisPose < closestSoFar)
        {
          closestSoFar = distToThisPose;
          closesAprilTag = aprilTag;
        }
      }
        return closesAprilTag;
    }
    public static AprilTag getClosestTagofTypeToRobotCenter(Pose2d RobotLoc,TagType Type)
    {
      double closestSoFar = 99999;
      AprilTag closesAprilTag = new AprilTag(0, "NotFound", new Pose2d(), 0);
      for (AprilTag aprilTag : tagList) {
        double distToThisPose = Tools.getdistancetopose(RobotLoc, aprilTag.Pose);
        if (aprilTag.tagType == Type && distToThisPose < closestSoFar)
        {
          closestSoFar = distToThisPose;
          closesAprilTag = aprilTag;
        }
      }
        return closesAprilTag;
    }
    // public static AprilTag getClosestSourceToRobotCenter(Pose2d RobotLoc)
    // {
    //   AprilTag LeftAprilTag = getTagbyID(GetLeftSourceID());
    //   AprilTag RightAprilTag = getTagbyID(GetRightSourceID());
    //   double LeftdistToThisPose = Tools.getdistancetopose(RobotLoc, LeftAprilTag.Pose);
    //   double RightdistToThisPose = Tools.getdistancetopose(RobotLoc, RightAprilTag.Pose);
      
        
    //     if (LeftdistToThisPose < RightdistToThisPose)
    //     {
    //       return LeftAprilTag;
    //     }
    //     return RightAprilTag;
    // }
    
    public static int GetLeftSourceID(){return (DriverStation.getAlliance().isPresent() & DriverStation.getAlliance().get().equals(Alliance.Blue))? 13:1;}//first num blue second red.
    public static int GetRightSourceID(){return (DriverStation.getAlliance().isPresent() & DriverStation.getAlliance().get().equals(Alliance.Blue))? 12:2;}//first num blue second red.

    public static int GetProcessorID(){return (DriverStation.getAlliance().isPresent() & DriverStation.getAlliance().get().equals(Alliance.Blue))? 16:3;}//first num blue second red.
    public static int GetBargeID(){return (DriverStation.getAlliance().isPresent() & DriverStation.getAlliance().get().equals(Alliance.Blue))? 14:5;}//first num blue second red.
    
    public static Pose2d getStraightOutLoc(int TagID,double MetersFromAprilTag)
    {
        AprilTag Thistag = getTagbyID(TagID);
        return getPose2DStraightLocTranslation(Thistag,MetersFromAprilTag);
    }
    public static Pose2d getOffSet90Loc(int TagID,double MetersFromAprilTag,double offcenter90distMeters,boolean positive)
    {
        AprilTag Thistag = getTagbyID(TagID);
        double offsetangle = positive ? -Math.PI/2 : Math.PI/2;//offset 90 degrees from tag ID. and if positive or not.

        Pose2d StraightLoc = getPose2DStraightLocTranslation(Thistag,MetersFromAprilTag);
        Pose2d offsetLoc = getPose2DOffset90LocTranslation(Thistag,MetersFromAprilTag,offcenter90distMeters,offsetangle);
        return new Pose2d(StraightLoc.getX() +offsetLoc.getX(), StraightLoc.getY() + offsetLoc.getY(), Thistag.Pose.getRotation().plus(Rotation2d.fromDegrees(180)));
    }
    public static Pose2d getPose2DStraightLocTranslation(AprilTag Thistag,double MetersFromAprilTag)
    {
      double numberwithrobotdepth = MetersFromAprilTag+robotmetersdistToCenter;
        double newX = Thistag.Pose.getX() + ((Thistag.extraOffsetWhenTargeting + numberwithrobotdepth)* Math.cos(Thistag.Pose.getRotation().getRadians()));
        double newY = Thistag.Pose.getY() + ((Thistag.extraOffsetWhenTargeting + numberwithrobotdepth)* Math.sin(Thistag.Pose.getRotation().getRadians()));
        return new Pose2d(newX, newY, Thistag.Pose.getRotation().plus(Rotation2d.fromDegrees(180)));
    }
    public static Pose2d getPose2DOffset90LocTranslation(AprilTag Thistag,double MetersFromAprilTag,double offcenter90distMeters,double offsetangle)
    {

      double offcenterX = ((Thistag.offset90Offset + offcenter90distMeters)* Math.cos(Thistag.Pose.getRotation().getRadians()+offsetangle));
      double offcenterY = ((Thistag.offset90Offset + offcenter90distMeters)* Math.sin(Thistag.Pose.getRotation().getRadians()+offsetangle));
        return new Pose2d(offcenterX, offcenterY, Thistag.Pose.getRotation());
    }


    public static List<AprilTag> tagList = Arrays.asList(
        //Blue Side
        new AprilTag(13,"LeftSource",new Pose2d(Units.inchesToMeters(33.51),Units.inchesToMeters(291.20),Rotation2d.fromDegrees(306)),0).Withoffset90(Units.inchesToMeters(10)).WithType(TagType.Source),
        new AprilTag(12,"RightSource",new Pose2d(Units.inchesToMeters(33.51),Units.inchesToMeters(25.80),Rotation2d.fromDegrees(54)),0).Withdepthoffset(Units.inchesToMeters(24)).WithType(TagType.Source),
        new AprilTag(16,"Processor",new Pose2d(Units.inchesToMeters(235.73),Units.inchesToMeters(-0.15),Rotation2d.fromDegrees(90)),0).WithType(TagType.Processor),
        new AprilTag(14,"blueBarge",new Pose2d(Units.inchesToMeters(325.68),Units.inchesToMeters(241.64),Rotation2d.fromDegrees(180)),30).WithType(TagType.BlueBarge),
        new AprilTag(15,"redBarge",new Pose2d(Units.inchesToMeters(325.68),Units.inchesToMeters(75.39),Rotation2d.fromDegrees(180)),30).WithType(TagType.RedBarge),
        new AprilTag(22,"reefSE",new Pose2d(Units.inchesToMeters(193.10),Units.inchesToMeters(130.17),Rotation2d.fromDegrees(300)),0).WithAlgaeOnUpper().WithType(TagType.Reef),
        new AprilTag(21,"reefE",new Pose2d(Units.inchesToMeters(209.49),Units.inchesToMeters(158.50),Rotation2d.fromDegrees(0)),0).WithType(TagType.Reef),
        new AprilTag(20,"reefNE",new Pose2d(Units.inchesToMeters(193.10),Units.inchesToMeters(186.83),Rotation2d.fromDegrees(60)),0).WithAlgaeOnUpper().WithType(TagType.Reef),
        new AprilTag(19,"reefNW",new Pose2d(Units.inchesToMeters(160.39),Units.inchesToMeters(186.83),Rotation2d.fromDegrees(120)),0).WithType(TagType.Reef),
        new AprilTag(18,"reefW",new Pose2d(Units.inchesToMeters(144.00),Units.inchesToMeters(158.50),Rotation2d.fromDegrees(180)),0).WithAlgaeOnUpper().WithType(TagType.Reef),
        new AprilTag(17,"reefSW",new Pose2d(Units.inchesToMeters(160.39),Units.inchesToMeters(130.17),Rotation2d.fromDegrees(240)),0).WithType(TagType.Reef),
        //Red Side
        new AprilTag(1,"LeftSource",new Pose2d(Units.inchesToMeters(657.37),Units.inchesToMeters(25.80),Rotation2d.fromDegrees(126)),0).WithType(TagType.Source),
        new AprilTag(2,"RightSource",new Pose2d(Units.inchesToMeters(657.37),Units.inchesToMeters(291.20),Rotation2d.fromDegrees(234)),0).WithType(TagType.Source),
        new AprilTag(3,"Processor",new Pose2d(Units.inchesToMeters(455.15),Units.inchesToMeters(317.15),Rotation2d.fromDegrees(270)),0).WithType(TagType.Processor),
        new AprilTag(4,"blueBarge",new Pose2d(Units.inchesToMeters(365.2),Units.inchesToMeters(241.64),Rotation2d.fromDegrees(0)),30).WithType(TagType.BlueBarge),
        new AprilTag(5,"redBarge",new Pose2d(Units.inchesToMeters(365.20),Units.inchesToMeters(75.39),Rotation2d.fromDegrees(0)),30).WithType(TagType.RedBarge),
        new AprilTag(6,"reefSE",new Pose2d(Units.inchesToMeters(530.49),Units.inchesToMeters(130.17),Rotation2d.fromDegrees(300)),0).WithType(TagType.Reef),
        new AprilTag(7,"reefE",new Pose2d(Units.inchesToMeters(546.87),Units.inchesToMeters(158.50),Rotation2d.fromDegrees(0)),0).WithAlgaeOnUpper().WithType(TagType.Reef),
        new AprilTag(8,"reefNE",new Pose2d(Units.inchesToMeters(530.49),Units.inchesToMeters(186.83),Rotation2d.fromDegrees(60)),0).WithType(TagType.Reef),
        new AprilTag(9,"reefNW",new Pose2d(Units.inchesToMeters(497.77),Units.inchesToMeters(186.83),Rotation2d.fromDegrees(120)),0).WithAlgaeOnUpper().WithType(TagType.Reef),
        new AprilTag(10,"reefW",new Pose2d(Units.inchesToMeters(481.39),Units.inchesToMeters(158.50),Rotation2d.fromDegrees(180)),0).WithType(TagType.Reef),
        new AprilTag(11,"reefSW",new Pose2d(Units.inchesToMeters(497.77),Units.inchesToMeters(130.17),Rotation2d.fromDegrees(240)),0).WithAlgaeOnUpper().WithType(TagType.Reef)
        );

  StructEntry<Pose2d> NT_myloc = NT.getStructEntry_Pose2D("Poses","TagLoc",new Pose2d());
  StructEntry<Pose2d> NT_myStraightloc = NT.getStructEntry_Pose2D("Poses","StraightLoc",new Pose2d());//straight out from the april tag. a centered pick for de-algae/processor/pickup 
  StructEntry<Pose2d> NT_Leftloc = NT.getStructEntry_Pose2D("Poses","leftLoc",new Pose2d());//LEFT FROM Robot TOWARDS tag view!
  StructEntry<Pose2d> NT_rightloc = NT.getStructEntry_Pose2D("Poses","rightLoc",new Pose2d());//Right Given View Robot TOWARDS tag!
  StructEntry<Pose2d> NT_ClosestTag = NT.getStructEntry_Pose2D("Poses","ClosestTag",new Pose2d());//shows closest tag to robotchassis
  StructEntry<Pose2d> NT_Simloc = NT.getStructEntry_Pose2D("Poses","ChassisLoc",new Pose2d());
  StructEntry<Pose2d> NT_ClosestSource = NT.getStructEntry_Pose2D("Poses","ClosestSource",new Pose2d());
  StructEntry<Pose2d> NT_ClosestReef = NT.getStructEntry_Pose2D("Poses","ClosestReef",new Pose2d());
  StructEntry<Pose2d> NT_ClosestProcessor = NT.getStructEntry_Pose2D("Poses","ClosestProcessor",new Pose2d());
  StructEntry<Pose2d> NT_ClosestBarge = NT.getStructEntry_Pose2D("Poses","ClosestBarge",new Pose2d());
  //DoubleEntry NT_tagID = NT.getDoubleEntry("", "TagID", chosenAprilTagID);
  public CommandSwerveDrivetrain drivetrain;
  public int chosenAprilTagID = 0;
  public AprilTagManager(CommandSwerveDrivetrain _drivetrain)
  {
    drivetrain =_drivetrain;
  } 
  @Override
  public void periodic() {
    //NT_myloc.set(ourtag.Pose);
    Pose2d SimRobotChassisLoc = drivetrain.getState().Pose;
   
    // NT_myStraightloc.set(AprilTagManager.getStraightOutLoc(chosenAprilTagID, Units.inchesToMeters(6)));
    // NT_Leftloc.set(getOffSet90Loc(chosenAprilTagID, Units.inchesToMeters(6), constants.ReefWidthCenteronCenter,true));
    // NT_rightloc.set(getOffSet90Loc(chosenAprilTagID, Units.inchesToMeters(6), constants.ReefWidthCenteronCenter,false));
    if(DriverStation.isDSAttached())
    {
      NT_Simloc.set(SimRobotChassisLoc);
      NT_ClosestSource.set(getClosestTagofTypeToRobotCenter(SimRobotChassisLoc,TagType.Source).Pose);
      NT_ClosestProcessor.set(getClosestTagofTypeToRobotCenter(SimRobotChassisLoc,TagType.Processor).Pose);
      NT_ClosestReef.set(getClosestTagofTypeToRobotCenter(SimRobotChassisLoc,TagType.Reef).Pose);
      NT_ClosestBarge.set(getClosestTagofTypeToRobotCenter(SimRobotChassisLoc,(DriverStation.getAlliance().isPresent() & DriverStation.getAlliance().get().equals(Alliance.Blue))? TagType.BlueBarge:TagType.RedBarge).Pose);
      NT_ClosestTag.set(getClosestTagToRobotCenter(SimRobotChassisLoc).Pose);
    }

  }
  }