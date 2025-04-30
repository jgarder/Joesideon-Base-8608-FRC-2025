package frc.robot.AlphaBots.AprilTagSystem;

import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class config {
    //YOU MUST SET drivetrain BEFORE using!
    public static CommandSwerveDrivetrain drivetrain;//this is required because then the SelectCommands will need our drivetrains position for Getting Closest Tag. 

    public static double BargeOffset = Units.inchesToMeters(3.5);//-0.5; //LIVE
    public static double SourceOffset = Units.inchesToMeters(2.5);//-0.5; //LIVE
    //public static double SourceOffset =  Units.inchesToMeters(60); //TESTING ONLY

    public static double RobotDefaultOffset = -1.00;//adds a 1/4 inch extra space at locations. 
    public static double bumperthickness = 3.00*2; //real 3.75"
    public static double robotsize = 30.25/2;//size no bumpers divided by 2
    public static double robotmetersdistToCenter = Units.inchesToMeters(RobotDefaultOffset+robotsize+bumperthickness);

    
    public static int chosenAprilTagID = 0;
    
    public static double L1AlignmentOffsetMeters = Units.inchesToMeters(24);
    public static double L1TwistOffsetDegrees = 15;

    public static final double ReefWidthCenterOffset = Units.inchesToMeters(12.94)/2;// used during test Units.inchesToMeters(12.875)/2; //Reef Width CenteronCenter divided in half

    public static final double SourcePickupWidthCenterOffset = Units.inchesToMeters(24)/2; //Reef Width CenteronCenter divided in half

    public static final double ExtraMetersoffsetForAlgaePickup = Units.inchesToMeters(6.25);

    public static final double ontheFlyDistanceFromCorrect = Units.inchesToMeters(12);

    public static double ProcessorDepthOffset = Units.inchesToMeters(2);
    
}
