package frc.robot.AlphaBots.AprilTagSystem;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.AlphaBots.Tools;
import frc.robot.AlphaBots.AprilTagSystem.AprilTag.TagType;

public class PoseFinder {

    public static AprilTag getTagbyID(int _ID)
    {
      for (AprilTag aprilTag : TagList.tagList) {
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
      for (AprilTag aprilTag : TagList.tagList) {
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
      for (AprilTag aprilTag : TagList.tagList) {
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
      
      for (AprilTag aprilTag : TagList.tagList) {
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
      double numberwithrobotdepth = MetersFromAprilTag+config.robotmetersdistToCenter;
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
    
}
