package frc.robot.AlphaBots.AprilTagSystem;

import java.util.Arrays;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.AlphaBots.AprilTagSystem.AprilTag.TagType;

public class TagList {
    public static List<AprilTag> tagList = Arrays.asList(
        //Blue Side
        new AprilTag(13,"LeftSource",
        new Pose2d(Units.inchesToMeters(33.51),Units.inchesToMeters(291.20),Rotation2d.fromDegrees(306)),0,Alliance.Blue)
        .WithType(TagType.Source)
        .Withdepthoffset(config.SourceOffset),
        new AprilTag(12,"RightSource",
        new Pose2d(Units.inchesToMeters(33.51),Units.inchesToMeters(25.80),Rotation2d.fromDegrees(54)),0,Alliance.Blue)
        .WithType(TagType.Source)
        .Withdepthoffset(config.SourceOffset), 
        new AprilTag(16,"Processor",
        new Pose2d(Units.inchesToMeters(235.73),Units.inchesToMeters(-0.15),Rotation2d.fromDegrees(90)),0,Alliance.Blue)
        .WithType(TagType.Processor)
        .Withdepthoffset(config.ProcessorDepthOffset),
        new AprilTag(14,"blueBarge",
        new Pose2d(Units.inchesToMeters(325.68),Units.inchesToMeters(241.64),Rotation2d.fromDegrees(180)),30,Alliance.Blue)
        .WithType(TagType.BlueBarge)
        .Withdepthoffset(config.BargeOffset),
        new AprilTag(15,"redBarge",
        new Pose2d(Units.inchesToMeters(325.68),Units.inchesToMeters(75.39),Rotation2d.fromDegrees(180)),30,Alliance.Red)
        .WithType(TagType.RedBarge)
        .Withdepthoffset(config.BargeOffset),
        new AprilTag(22,"reefSE",
        new Pose2d(Units.inchesToMeters(193.10),Units.inchesToMeters(130.17),Rotation2d.fromDegrees(300)),0,Alliance.Blue)
        .WithType(TagType.Reef)
        .WithAlgaeOnUpper(),
        new AprilTag(21,"reefE",
        new Pose2d(Units.inchesToMeters(209.49),Units.inchesToMeters(158.50),Rotation2d.fromDegrees(0)),0,Alliance.Blue)
        .WithType(TagType.Reef),
        new AprilTag(20,"reefNE",
        new Pose2d(Units.inchesToMeters(193.10),Units.inchesToMeters(186.83),Rotation2d.fromDegrees(60)),0,Alliance.Blue)
        .WithType(TagType.Reef).WithAlgaeOnUpper(),
        new AprilTag(19,"reefNW",
        new Pose2d(Units.inchesToMeters(160.39),Units.inchesToMeters(186.83),Rotation2d.fromDegrees(120)),0,Alliance.Blue)
        .WithType(TagType.Reef),
        new AprilTag(18,"reefW",
        new Pose2d(Units.inchesToMeters(144.00),Units.inchesToMeters(158.50),Rotation2d.fromDegrees(180)),0,Alliance.Blue)
        .WithType(TagType.Reef).WithAlgaeOnUpper(),
        new AprilTag(17,"reefSW",
        new Pose2d(Units.inchesToMeters(160.39),Units.inchesToMeters(130.17),Rotation2d.fromDegrees(240)),0,Alliance.Blue)
        .WithType(TagType.Reef),
        //
        //Red Side
        //
        new AprilTag(1,"LeftSource",
        new Pose2d(Units.inchesToMeters(657.37),Units.inchesToMeters(25.80),Rotation2d.fromDegrees(126)),0,Alliance.Red)
        .WithType(TagType.Source)
        .Withdepthoffset(config.SourceOffset),
        new AprilTag(2,"RightSource",
        new Pose2d(Units.inchesToMeters(657.37),Units.inchesToMeters(291.20),Rotation2d.fromDegrees(234)),0,Alliance.Red)
        .WithType(TagType.Source)
        .Withdepthoffset(config.SourceOffset), 
        new AprilTag(3,"Processor",
        new Pose2d(Units.inchesToMeters(455.15),Units.inchesToMeters(317.15),Rotation2d.fromDegrees(270)),0,Alliance.Red)
        .WithType(TagType.Processor)
        .Withdepthoffset(config.ProcessorDepthOffset),
        new AprilTag(4,"blueBarge",
        new Pose2d(Units.inchesToMeters(365.2),Units.inchesToMeters(241.64),Rotation2d.fromDegrees(0)),30,Alliance.Blue)
        .WithType(TagType.BlueBarge)
        .Withdepthoffset(config.BargeOffset),
        new AprilTag(5,"redBarge",
        new Pose2d(Units.inchesToMeters(365.20),Units.inchesToMeters(75.39),Rotation2d.fromDegrees(0)),30,Alliance.Red)
        .WithType(TagType.RedBarge)
        .Withdepthoffset(config.BargeOffset),
        new AprilTag(6,"reefSE",
        new Pose2d(Units.inchesToMeters(530.49),Units.inchesToMeters(130.17),Rotation2d.fromDegrees(300)),0,Alliance.Red)
        .WithType(TagType.Reef),
        new AprilTag(7,"reefE",
        new Pose2d(Units.inchesToMeters(546.87),Units.inchesToMeters(158.50),Rotation2d.fromDegrees(0)),0,Alliance.Red)
        .WithType(TagType.Reef)
        .WithAlgaeOnUpper(),
        new AprilTag(8,"reefNE",
        new Pose2d(Units.inchesToMeters(530.49),Units.inchesToMeters(186.83),Rotation2d.fromDegrees(60)),0,Alliance.Red)
        .WithType(TagType.Reef),
        new AprilTag(9,"reefNW",
        new Pose2d(Units.inchesToMeters(497.77),Units.inchesToMeters(186.83),Rotation2d.fromDegrees(120)),0,Alliance.Red)
        .WithType(TagType.Reef)
        .WithAlgaeOnUpper(),
        new AprilTag(10,"reefW",
        new Pose2d(Units.inchesToMeters(481.39),Units.inchesToMeters(158.50),Rotation2d.fromDegrees(180)),0,Alliance.Red)
        .WithType(TagType.Reef),
        new AprilTag(11,"reefSW",
        new Pose2d(Units.inchesToMeters(497.77),Units.inchesToMeters(130.17),Rotation2d.fromDegrees(240)),0,Alliance.Red)
        .WithType(TagType.Reef)
        .WithAlgaeOnUpper()
        );
    
}
