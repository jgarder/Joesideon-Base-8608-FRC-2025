package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meter;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.StructEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.AlphaBots.Tools;
import frc.robot.commands.C_Align;
import frc.robot.constants;
import frc.robot.AlphaBots.AprilTag;
import frc.robot.AlphaBots.AprilTag.TagType;
import frc.robot.AlphaBots.NT;

public class AprilTagManager extends SubsystemBase
  {
    //public static ArrayList<AprilTag> tagList = new ArrayList<AprilTag>(22);
    public static double RobotDefaultOffset = -1.00;//adds a 1/4 inch extra space at locations. 
    public static double bumperthickness = 3.00*2; //real 3.75"
    public static double robotsize = 30.25/2;//size no bumpers divided by 2
    public static double robotmetersdistToCenter = Units.inchesToMeters(RobotDefaultOffset+robotsize+bumperthickness);
    public static AprilTag getTagbyID(int _ID)
    {
      for (AprilTag aprilTag : tagList) {
        if (aprilTag.ID == _ID)
        {
          return aprilTag;
        }
      }
      System.err.println("No Tag found for ID :" + _ID);
        return new AprilTag(0, "NotFound", new Pose2d(), 0,Alliance.Blue);
    }
    public static AprilTag getClosestTagToRobotCenter(Pose2d RobotLoc)
    {
      double closestSoFar = 99999;
      AprilTag closesAprilTag = new AprilTag(0, "NotFound", new Pose2d(), 0,Alliance.Blue);
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
      AprilTag closesAprilTag = new AprilTag(0, "NotFound", new Pose2d(), 0,Alliance.Blue);
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
    /// get the tag that is the closest to the robots center, for a given tag type, and for the robots alliance.
    public static AprilTag getClosestTagofTypeToRobotCenterForAlliance(Pose2d RobotLoc,TagType Type)
    {
      double closestSoFar = 99999;
      AprilTag closesAprilTag = new AprilTag(0, "NotFound", new Pose2d(), 0,Alliance.Blue);
      if(!DriverStation.getAlliance().isPresent()){return closesAprilTag;}//no alliance? return default. 

      Alliance thisAlliance =  DriverStation.getAlliance().get();
      
      for (AprilTag aprilTag : tagList) {
        double distToThisPose = Tools.getdistancetopose(RobotLoc, aprilTag.Pose);
        if (aprilTag.tagType == Type && aprilTag.tagsAlliance == thisAlliance && distToThisPose < closestSoFar)
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
    
    //public static int GetLeftSourceID(){return (DriverStation.getAlliance().isPresent() & DriverStation.getAlliance().get().equals(Alliance.Blue))? 13:1;}//first num blue second red.
    //public static int GetRightSourceID(){return (DriverStation.getAlliance().isPresent() & DriverStation.getAlliance().get().equals(Alliance.Blue))? 12:2;}//first num blue second red.

    //public static int GetProcessorID(){return (DriverStation.getAlliance().isPresent() & DriverStation.getAlliance().get().equals(Alliance.Blue))? 16:3;}//first num blue second red.
    //public static int GetBargeID(){return (DriverStation.getAlliance().isPresent() & DriverStation.getAlliance().get().equals(Alliance.Blue))? 14:5;}//first num blue second red.

    public static Pose2d getStraightOutLoc(int TagID,double MetersFromAprilTag)
    {
        AprilTag Thistag = getTagbyID(TagID);
        return getPose2DStraightLocTranslation(Thistag,MetersFromAprilTag);
    }
    public static Pose2d getReverseStraightOutLoc(int TagID,double MetersFromAprilTag)
    {
        AprilTag Thistag = getTagbyID(TagID);
        return getReversePose2DStraightLocTranslation(Thistag,MetersFromAprilTag);
    }
    public static Pose2d getReversePose2DStraightLocTranslation(AprilTag Thistag,double MetersFromAprilTag)
    {
      Pose2d results = getPose2DStraightLocTranslation(Thistag,MetersFromAprilTag);
      return new Pose2d(results.getX(),results.getY(),Rotation2d.fromDegrees(results.getRotation().getDegrees() +180));
    }
    public static Pose2d getOffSet90Loc(int TagID,double MetersFromAprilTag,double offcenter90distMeters,boolean positive)
    {
        AprilTag Thistag = getTagbyID(TagID);
       return getOffSet90Loc(Thistag,MetersFromAprilTag,offcenter90distMeters,positive);
    }
    
    public static Pose2d getOffSet90Loc(AprilTag Thistag,double MetersFromAprilTag,double offcenter90distMeters,boolean positive)
    {
        double offsetangle = positive ? -Math.PI/2 : Math.PI/2;//offset 90 degrees from tag ID. and if positive or not.

        Pose2d StraightLoc = getPose2DStraightLocTranslation(Thistag,MetersFromAprilTag);
        Pose2d offsetLoc = getPose2DOffset90LocTranslation(Thistag,offcenter90distMeters,offsetangle);
        return new Pose2d(StraightLoc.getX() +offsetLoc.getX(), StraightLoc.getY() + offsetLoc.getY(), Thistag.Pose.getRotation().plus(Rotation2d.fromDegrees(180)));
    }
    public static Pose2d getPose2DStraightLocTranslation(AprilTag Thistag,double MetersFromAprilTag)
    {
      double numberwithrobotdepth = MetersFromAprilTag+robotmetersdistToCenter;
        double newX = Thistag.Pose.getX() + ((Thistag.extraOffsetWhenTargeting + numberwithrobotdepth)* Math.cos(Thistag.Pose.getRotation().getRadians()));
        double newY = Thistag.Pose.getY() + ((Thistag.extraOffsetWhenTargeting + numberwithrobotdepth)* Math.sin(Thistag.Pose.getRotation().getRadians()));
        return new Pose2d(newX, newY, Thistag.Pose.getRotation().plus(Rotation2d.fromDegrees(180)));
    }
    public static Pose2d getPose2DOffset90LocTranslation(AprilTag Thistag,double offcenter90distMeters,double offsetangle)
    {

      double offcenterX = ((Thistag.offset90Offset + offcenter90distMeters)* Math.cos(Thistag.Pose.getRotation().getRadians()+offsetangle));
      double offcenterY = ((Thistag.offset90Offset + offcenter90distMeters)* Math.sin(Thistag.Pose.getRotation().getRadians()+offsetangle));
        return new Pose2d(offcenterX, offcenterY, Thistag.Pose.getRotation());
    }


    public static List<AprilTag> tagList = Arrays.asList(
        //Blue Side
        new AprilTag(13,"LeftSource",new Pose2d(Units.inchesToMeters(33.51),Units.inchesToMeters(291.20),Rotation2d.fromDegrees(306)),0,Alliance.Blue).WithType(TagType.Source)
        //.Withdepthoffset(Units.inchesToMeters(60)),//SOURCE test
        .Withdepthoffset(Units.inchesToMeters(-.55)),  //SOURCE LIVE
        new AprilTag(12,"RightSource",new Pose2d(Units.inchesToMeters(33.51),Units.inchesToMeters(25.80),Rotation2d.fromDegrees(54)),0,Alliance.Blue).WithType(TagType.Source)
        //.Withdepthoffset(Units.inchesToMeters(60)),//SOURCE test
        .Withdepthoffset(Units.inchesToMeters(-.55)),  //SOURCE LIVE
        new AprilTag(16,"Processor",new Pose2d(Units.inchesToMeters(235.73),Units.inchesToMeters(-0.15),Rotation2d.fromDegrees(90)),0,Alliance.Blue).WithType(TagType.Processor)
        .Withdepthoffset(Units.inchesToMeters(2)),
        new AprilTag(14,"blueBarge",new Pose2d(Units.inchesToMeters(325.68),Units.inchesToMeters(241.64),Rotation2d.fromDegrees(180)),30,Alliance.Blue).WithType(TagType.BlueBarge),
        new AprilTag(15,"redBarge",new Pose2d(Units.inchesToMeters(325.68),Units.inchesToMeters(75.39),Rotation2d.fromDegrees(180)),30,Alliance.Blue).WithType(TagType.RedBarge),
        new AprilTag(22,"reefSE",new Pose2d(Units.inchesToMeters(193.10),Units.inchesToMeters(130.17),Rotation2d.fromDegrees(300)),0,Alliance.Blue).WithAlgaeOnUpper().WithType(TagType.Reef),
        new AprilTag(21,"reefE",new Pose2d(Units.inchesToMeters(209.49),Units.inchesToMeters(158.50),Rotation2d.fromDegrees(0)),0,Alliance.Blue).WithType(TagType.Reef),
        new AprilTag(20,"reefNE",new Pose2d(Units.inchesToMeters(193.10),Units.inchesToMeters(186.83),Rotation2d.fromDegrees(60)),0,Alliance.Blue).WithAlgaeOnUpper().WithType(TagType.Reef),
        new AprilTag(19,"reefNW",new Pose2d(Units.inchesToMeters(160.39),Units.inchesToMeters(186.83),Rotation2d.fromDegrees(120)),0,Alliance.Blue).WithType(TagType.Reef),
        new AprilTag(18,"reefW",new Pose2d(Units.inchesToMeters(144.00),Units.inchesToMeters(158.50),Rotation2d.fromDegrees(180)),0,Alliance.Blue).WithAlgaeOnUpper().WithType(TagType.Reef),
        new AprilTag(17,"reefSW",new Pose2d(Units.inchesToMeters(160.39),Units.inchesToMeters(130.17),Rotation2d.fromDegrees(240)),0,Alliance.Blue).WithType(TagType.Reef),
        //Red Side
        new AprilTag(1,"LeftSource",new Pose2d(Units.inchesToMeters(657.37),Units.inchesToMeters(25.80),Rotation2d.fromDegrees(126)),0,Alliance.Red).WithType(TagType.Source)
        //.Withdepthoffset(Units.inchesToMeters(60)),  //SOURCE test
        .Withdepthoffset(Units.inchesToMeters(-.55)),  //SOURCE LIVE
        new AprilTag(2,"RightSource",new Pose2d(Units.inchesToMeters(657.37),Units.inchesToMeters(291.20),Rotation2d.fromDegrees(234)),0,Alliance.Red).WithType(TagType.Source)
        //.Withdepthoffset(Units.inchesToMeters(60)),// SOURCE test
        .Withdepthoffset(Units.inchesToMeters(-.55)), //SOURCE LIVE
        new AprilTag(3,"Processor",new Pose2d(Units.inchesToMeters(455.15),Units.inchesToMeters(317.15),Rotation2d.fromDegrees(270)),0,Alliance.Red).WithType(TagType.Processor)
        .Withdepthoffset(Units.inchesToMeters(2)),
        new AprilTag(4,"blueBarge",new Pose2d(Units.inchesToMeters(365.2),Units.inchesToMeters(241.64),Rotation2d.fromDegrees(0)),30,Alliance.Red).WithType(TagType.BlueBarge),
        new AprilTag(5,"redBarge",new Pose2d(Units.inchesToMeters(365.20),Units.inchesToMeters(75.39),Rotation2d.fromDegrees(0)),30,Alliance.Red).WithType(TagType.RedBarge),
        new AprilTag(6,"reefSE",new Pose2d(Units.inchesToMeters(530.49),Units.inchesToMeters(130.17),Rotation2d.fromDegrees(300)),0,Alliance.Red).WithType(TagType.Reef),
        new AprilTag(7,"reefE",new Pose2d(Units.inchesToMeters(546.87),Units.inchesToMeters(158.50),Rotation2d.fromDegrees(0)),0,Alliance.Red).WithAlgaeOnUpper().WithType(TagType.Reef),
        new AprilTag(8,"reefNE",new Pose2d(Units.inchesToMeters(530.49),Units.inchesToMeters(186.83),Rotation2d.fromDegrees(60)),0,Alliance.Red).WithType(TagType.Reef),
        new AprilTag(9,"reefNW",new Pose2d(Units.inchesToMeters(497.77),Units.inchesToMeters(186.83),Rotation2d.fromDegrees(120)),0,Alliance.Red).WithAlgaeOnUpper().WithType(TagType.Reef),
        new AprilTag(10,"reefW",new Pose2d(Units.inchesToMeters(481.39),Units.inchesToMeters(158.50),Rotation2d.fromDegrees(180)),0,Alliance.Red).WithType(TagType.Reef),
        new AprilTag(11,"reefSW",new Pose2d(Units.inchesToMeters(497.77),Units.inchesToMeters(130.17),Rotation2d.fromDegrees(240)),0,Alliance.Red).WithAlgaeOnUpper().WithType(TagType.Reef)
        );

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
  private int selectSource() {

    return AprilTagManager.getClosestTagofTypeToRobotCenter(this.drivetrain.getState().Pose,TagType.Source).ID;
  }
  private int selectProcessor() {

    return AprilTagManager.getClosestTagofTypeToRobotCenter(this.drivetrain.getState().Pose,TagType.Processor).ID;
  }
  private int selectReef() {

  return AprilTagManager.getClosestTagofTypeToRobotCenter(this.drivetrain.getState().Pose,TagType.Reef).ID;

  }

public final SelectCommand C_SourceSelectCommand(){
     return new SelectCommand<>(
          // Maps selector values to commands
          Map.ofEntries(
              Map.entry(1, new PrintCommand("Command 1 was selected!")
              .alongWith(new C_Align(AprilTagManager.getReverseStraightOutLoc(1,0.0)))),

              Map.entry(2, new PrintCommand("Command 2 was selected!")
              .alongWith(new C_Align(AprilTagManager.getReverseStraightOutLoc(2,0)))),
              
              Map.entry(12, new PrintCommand("Command 12 was selected!")
              .alongWith(new C_Align(AprilTagManager.getReverseStraightOutLoc(12,0)))),

              Map.entry(13, new PrintCommand("Command 13 was selected!")
              .alongWith(new C_Align(AprilTagManager.getReverseStraightOutLoc(13,0))))
              ),

          ()->{return selectSource();});

    }
    public final SelectCommand C_ProcessorSelectCommand(){
      return new SelectCommand<>(
           // Maps selector values to commands
           Map.ofEntries(
               Map.entry(3, new PrintCommand("Processor red 3 was selected!")
               .alongWith(new C_Align(AprilTagManager.getStraightOutLoc(3,0.0)))),
 
               Map.entry(16, new PrintCommand("Processor blue 16 was selected!")
               .alongWith(new C_Align(AprilTagManager.getStraightOutLoc(16,0.0))))
           ),
           ()->{return selectProcessor();});
 
     }
    public final SelectCommand C_ReefLeftSelectCommand()
        { return
          new SelectCommand<>(
              // Maps selector values to commands
              Map.ofEntries(
                  Map.entry(6, new PrintCommand("Command 6 was selected!")
                  .alongWith(new C_Align(AprilTagManager.getOffSet90Loc(6,0.0,constants.ReefWidthCenterOffset,true)))),
    
                  Map.entry(7, new PrintCommand("Command 7 was selected!")
                  .alongWith(new C_Align(AprilTagManager.getOffSet90Loc(7,0,constants.ReefWidthCenterOffset,true)))),
                  
                  Map.entry(8, new PrintCommand("Command 8 was selected!")
                  .alongWith(new C_Align(AprilTagManager.getOffSet90Loc(8,0,constants.ReefWidthCenterOffset,true)))),
    
                  Map.entry(9, new PrintCommand("Command 9 was selected!")
                  .alongWith(new C_Align(AprilTagManager.getOffSet90Loc(9,0,constants.ReefWidthCenterOffset,true)))),
                  Map.entry(10, new PrintCommand("Command 10 was selected!")
                  .alongWith(new C_Align(AprilTagManager.getOffSet90Loc(10,0,constants.ReefWidthCenterOffset,true)))),
                  Map.entry(11, new PrintCommand("Command 11 was selected!")
                  .alongWith(new C_Align(AprilTagManager.getOffSet90Loc(11,0,constants.ReefWidthCenterOffset,true)))),

                  Map.entry(17, new PrintCommand("Command 17 was selected!")
                  .alongWith(new C_Align(AprilTagManager.getOffSet90Loc(17,0,constants.ReefWidthCenterOffset,true)))),
                  Map.entry(18, new PrintCommand("Command 18 was selected!")
                  .alongWith(new C_Align(AprilTagManager.getOffSet90Loc(18,0,constants.ReefWidthCenterOffset,true)))),
                  Map.entry(19, new PrintCommand("Command 19 was selected!")
                  .alongWith(new C_Align(AprilTagManager.getOffSet90Loc(19,0,constants.ReefWidthCenterOffset,true)))),
                  Map.entry(20, new PrintCommand("Command 20 was selected!")
                  .alongWith(new C_Align(AprilTagManager.getOffSet90Loc(20,0,constants.ReefWidthCenterOffset,true)))),
                  Map.entry(21, new PrintCommand("Command 21 was selected!")
                  .alongWith(new C_Align(AprilTagManager.getOffSet90Loc(21,0,constants.ReefWidthCenterOffset,true)))),
                  Map.entry(22, new PrintCommand("Command 22 was selected!")
                  .alongWith(new C_Align(AprilTagManager.getOffSet90Loc(22,0,constants.ReefWidthCenterOffset,true))))
                  
                  ),
                  

              ()->{return selectReef();});
        }
        public final SelectCommand C_ReefRightSelectCommand()
        { return
          new SelectCommand<>(
              // Maps selector values to commands
              Map.ofEntries(
                  Map.entry(6, new PrintCommand("Command 6 Right was selected!")
                  .alongWith(new C_Align(AprilTagManager.getOffSet90Loc(6,0.0,constants.ReefWidthCenterOffset,false)))),
    
                  Map.entry(7, new PrintCommand("Command 7 Right was selected!")
                  .alongWith(new C_Align(AprilTagManager.getOffSet90Loc(7,0,constants.ReefWidthCenterOffset,false)))),
                  
                  Map.entry(8, new PrintCommand("Command 8 was selected!")
                  .alongWith(new C_Align(AprilTagManager.getOffSet90Loc(8,0,constants.ReefWidthCenterOffset,false)))),
    
                  Map.entry(9, new PrintCommand("Command 9 was selected!")
                  .alongWith(new C_Align(AprilTagManager.getOffSet90Loc(9,0,constants.ReefWidthCenterOffset,false)))),
                  Map.entry(10, new PrintCommand("Command 10 was selected!")
                  .alongWith(new C_Align(AprilTagManager.getOffSet90Loc(10,0,constants.ReefWidthCenterOffset,false)))),
                  Map.entry(11, new PrintCommand("Command 11 was selected!")
                  .alongWith(new C_Align(AprilTagManager.getOffSet90Loc(11,0,constants.ReefWidthCenterOffset,false)))),

                  Map.entry(17, new PrintCommand("Command 17 was selected!")
                  .alongWith(new C_Align(AprilTagManager.getOffSet90Loc(17,0,constants.ReefWidthCenterOffset,false)))),
                  Map.entry(18, new PrintCommand("Command 18 was selected!")
                  .alongWith(new C_Align(AprilTagManager.getOffSet90Loc(18,0,constants.ReefWidthCenterOffset,false)))),
                  Map.entry(19, new PrintCommand("Command 19 was selected!")
                  .alongWith(new C_Align(AprilTagManager.getOffSet90Loc(19,0,constants.ReefWidthCenterOffset,false)))),
                  Map.entry(20, new PrintCommand("Command 20 was selected!")
                  .alongWith(new C_Align(AprilTagManager.getOffSet90Loc(20,0,constants.ReefWidthCenterOffset,false)))),
                  Map.entry(21, new PrintCommand("Command 21 was selected!")
                  .alongWith(new C_Align(AprilTagManager.getOffSet90Loc(21,0,constants.ReefWidthCenterOffset,false)))),
                  Map.entry(22, new PrintCommand("Command 22 was selected!")
                  .alongWith(new C_Align(AprilTagManager.getOffSet90Loc(22,0,constants.ReefWidthCenterOffset,false))))
                  
                  ),
                  

              ()->{return selectReef();});
        }
        public  final SelectCommand C_ReefCenterAlgaeSelectCommand()
        { return
          new SelectCommand<>(
              // Maps selector values to commands
              Map.ofEntries(
                  Map.entry(6, new PrintCommand("Command 6 Center was selected!")
                  .alongWith(new C_Align(AprilTagManager.getStraightOutLoc(6,constants.ExtraMetersoffsetForAlgaePickup)))),
    
                  Map.entry(7, new PrintCommand("Command 7  Center was selected!")
                  .alongWith(new C_Align(AprilTagManager.getStraightOutLoc(7,constants.ExtraMetersoffsetForAlgaePickup)))),
                  
                  Map.entry(8, new PrintCommand("Command 8 was selected!")
                  .alongWith(new C_Align(AprilTagManager.getStraightOutLoc(8,constants.ExtraMetersoffsetForAlgaePickup)))),
    
                  Map.entry(9, new PrintCommand("Command 9 was selected!")
                  .alongWith(new C_Align(AprilTagManager.getStraightOutLoc(9,constants.ExtraMetersoffsetForAlgaePickup)))),
                  Map.entry(10, new PrintCommand("Command 10 was selected!")
                  .alongWith(new C_Align(AprilTagManager.getStraightOutLoc(10,constants.ExtraMetersoffsetForAlgaePickup)))),
                  Map.entry(11, new PrintCommand("Command 11 was selected!")
                  .alongWith(new C_Align(AprilTagManager.getStraightOutLoc(11,constants.ExtraMetersoffsetForAlgaePickup)))),

                  Map.entry(17, new PrintCommand("Command 17 was selected!")
                  .alongWith(new C_Align(AprilTagManager.getStraightOutLoc(17,constants.ExtraMetersoffsetForAlgaePickup)))),
                  Map.entry(18, new PrintCommand("Command 18 was selected!")
                  .alongWith(new C_Align(AprilTagManager.getStraightOutLoc(18,constants.ExtraMetersoffsetForAlgaePickup)))),
                  Map.entry(19, new PrintCommand("Command 19 was selected!")
                  .alongWith(new C_Align(AprilTagManager.getStraightOutLoc(19,constants.ExtraMetersoffsetForAlgaePickup)))),
                  Map.entry(20, new PrintCommand("Command 20 was selected!")
                  .alongWith(new C_Align(AprilTagManager.getStraightOutLoc(20,constants.ExtraMetersoffsetForAlgaePickup)))),
                  Map.entry(21, new PrintCommand("Command 21 was selected!")
                  .alongWith(new C_Align(AprilTagManager.getStraightOutLoc(21,constants.ExtraMetersoffsetForAlgaePickup)))),
                  Map.entry(22, new PrintCommand("Command 22 was selected!")
                  .alongWith(new C_Align(AprilTagManager.getStraightOutLoc(22,constants.ExtraMetersoffsetForAlgaePickup))))
                  
                  ),
                  

              ()->{return selectReef();});
        }
        public static double L1AlignmentOffsetMeters = Units.inchesToMeters(24);
        public static double L1TwistOffsetDegrees = 15;
        public  final SelectCommand C_ReefL1CenterSelectCommand()
        { return
          new SelectCommand<>(
              // Maps selector values to commands
              Map.ofEntries(
                  Map.entry(6, new PrintCommand("L1Alignment 6 Center was selected!")
                  .alongWith(new C_Align(
                    new Pose2d(
                      AprilTagManager.getOffSet90Loc(6,0,L1AlignmentOffsetMeters,false).getX(),
                      AprilTagManager.getOffSet90Loc(6,0,L1AlignmentOffsetMeters,false).getY(),
                      Rotation2d.fromDegrees(AprilTagManager.getOffSet90Loc(6,0,L1AlignmentOffsetMeters,false).getRotation().getDegrees() + L1TwistOffsetDegrees)
                      )
                  ))),
    
                  Map.entry(7, new PrintCommand("L1Alignment 7  Center was selected!")
                  .alongWith(new C_Align(
                    new Pose2d(
                      AprilTagManager.getOffSet90Loc(7,0,L1AlignmentOffsetMeters,false).getX(),
                      AprilTagManager.getOffSet90Loc(7,0,L1AlignmentOffsetMeters,false).getY(),
                      Rotation2d.fromDegrees(AprilTagManager.getOffSet90Loc(7,0,L1AlignmentOffsetMeters,false).getRotation().getDegrees() + L1TwistOffsetDegrees)
                      )
                  ))),
                  
                  Map.entry(8, new PrintCommand("L1Alignment 8 was selected!")
                  .alongWith(new C_Align(
                    new Pose2d(
                      AprilTagManager.getOffSet90Loc(8,0,L1AlignmentOffsetMeters,false).getX(),
                      AprilTagManager.getOffSet90Loc(8,0,L1AlignmentOffsetMeters,false).getY(),
                      Rotation2d.fromDegrees(AprilTagManager.getOffSet90Loc(8,0,L1AlignmentOffsetMeters,false).getRotation().getDegrees() + L1TwistOffsetDegrees)
                      )
                  ))),    
                  Map.entry(9, new PrintCommand("L1Alignment 9 was selected!")
                  .alongWith(new C_Align(
                    new Pose2d(
                      AprilTagManager.getOffSet90Loc(9,0,L1AlignmentOffsetMeters,false).getX(),
                      AprilTagManager.getOffSet90Loc(9,0,L1AlignmentOffsetMeters,false).getY(),
                      Rotation2d.fromDegrees(AprilTagManager.getOffSet90Loc(9,0,L1AlignmentOffsetMeters,false).getRotation().getDegrees() + L1TwistOffsetDegrees)
                      )
                  ))),                  
                  Map.entry(10, new PrintCommand("L1Alignment 10 was selected!")
                  .alongWith(new C_Align(
                    new Pose2d(
                      AprilTagManager.getOffSet90Loc(10,0,L1AlignmentOffsetMeters,false).getX(),
                      AprilTagManager.getOffSet90Loc(10,0,L1AlignmentOffsetMeters,false).getY(),
                      Rotation2d.fromDegrees(AprilTagManager.getOffSet90Loc(10,0,L1AlignmentOffsetMeters,false).getRotation().getDegrees() + L1TwistOffsetDegrees)
                      )
                  ))),                  
                  Map.entry(11, new PrintCommand("L1Alignment 11 was selected!")
                  .alongWith(new C_Align(
                    new Pose2d(
                      AprilTagManager.getOffSet90Loc(11,0,L1AlignmentOffsetMeters,false).getX(),
                      AprilTagManager.getOffSet90Loc(11,0,L1AlignmentOffsetMeters,false).getY(),
                      Rotation2d.fromDegrees(AprilTagManager.getOffSet90Loc(11,0,L1AlignmentOffsetMeters,false).getRotation().getDegrees() + L1TwistOffsetDegrees)
                      )
                  ))),
                  Map.entry(17, new PrintCommand("L1Alignment 17 was selected!")
                  .alongWith(new C_Align(
                    new Pose2d(
                      AprilTagManager.getOffSet90Loc(17,0,L1AlignmentOffsetMeters,false).getX(),
                      AprilTagManager.getOffSet90Loc(17,0,L1AlignmentOffsetMeters,false).getY(),
                      Rotation2d.fromDegrees(AprilTagManager.getOffSet90Loc(17,0,L1AlignmentOffsetMeters,false).getRotation().getDegrees() + L1TwistOffsetDegrees)
                      )
                  ))),                  
                  Map.entry(18, new PrintCommand("L1Alignment 18 was selected!")
                  .alongWith(new C_Align(
                    new Pose2d(
                      AprilTagManager.getOffSet90Loc(18,0,L1AlignmentOffsetMeters,false).getX(),
                      AprilTagManager.getOffSet90Loc(18,0,L1AlignmentOffsetMeters,false).getY(),
                      Rotation2d.fromDegrees(AprilTagManager.getOffSet90Loc(18,0,L1AlignmentOffsetMeters,false).getRotation().getDegrees() + L1TwistOffsetDegrees)
                      )
                  ))),                  
                  Map.entry(19, new PrintCommand("L1Alignment 19 was selected!")
                  .alongWith(new C_Align(
                    new Pose2d(
                      AprilTagManager.getOffSet90Loc(19,0,L1AlignmentOffsetMeters,false).getX(),
                      AprilTagManager.getOffSet90Loc(19,0,L1AlignmentOffsetMeters,false).getY(),
                      Rotation2d.fromDegrees(AprilTagManager.getOffSet90Loc(19,0,L1AlignmentOffsetMeters,false).getRotation().getDegrees() + L1TwistOffsetDegrees)
                      )
                  ))),                  
                  Map.entry(20, new PrintCommand("L1Alignment 20 was selected!")
                  .alongWith(new C_Align(
                    new Pose2d(
                      AprilTagManager.getOffSet90Loc(20,0,L1AlignmentOffsetMeters,false).getX(),
                      AprilTagManager.getOffSet90Loc(20,0,L1AlignmentOffsetMeters,false).getY(),
                      Rotation2d.fromDegrees(AprilTagManager.getOffSet90Loc(20,0,L1AlignmentOffsetMeters,false).getRotation().getDegrees() + L1TwistOffsetDegrees)
                      )
                  ))),                  
                  Map.entry(21, new PrintCommand("L1Alignment 21 was selected!")
                  .alongWith(new C_Align(
                    new Pose2d(
                      AprilTagManager.getOffSet90Loc(21,0,L1AlignmentOffsetMeters,false).getX(),
                      AprilTagManager.getOffSet90Loc(21,0,L1AlignmentOffsetMeters,false).getY(),
                      Rotation2d.fromDegrees(AprilTagManager.getOffSet90Loc(21,0,L1AlignmentOffsetMeters,false).getRotation().getDegrees() + L1TwistOffsetDegrees)
                      )
                  ))),                  
                  Map.entry(22, new PrintCommand("L1Alignment 22 was selected!")
                  .alongWith(new C_Align(
                    new Pose2d(
                      AprilTagManager.getOffSet90Loc(22,0,L1AlignmentOffsetMeters,false).getX(),
                      AprilTagManager.getOffSet90Loc(22,0,L1AlignmentOffsetMeters,false).getY(),
                      Rotation2d.fromDegrees(AprilTagManager.getOffSet90Loc(22,0,L1AlignmentOffsetMeters,false).getRotation().getDegrees() + L1TwistOffsetDegrees)
                      )
                  )))                  
                  ),
                  

              ()->{return selectReef();});
        }
  }