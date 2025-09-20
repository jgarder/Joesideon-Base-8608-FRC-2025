package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;

import edu.wpi.first.math.util.Units;
import frc.robot.generated.TunerConstants;

public class constants {

    public static class drivetrainThings{
        public static final double translationDeadbandPercent = 0.1;//0.025 = 2.5%
        public static final double rotationalDeadband = 0.05;//0.05 = 5% deadband
        public static final SwerveRequest.FieldCentric TeleOpDrive = new SwerveRequest.FieldCentric()
        .withDeadband(constants.drivetrainThings.MaxSpeed * translationDeadbandPercent).withRotationalDeadband(constants.drivetrainThings.MaxAngularRate * rotationalDeadband) // Add a 10% deadband
        .withDriveRequestType(DriveRequestType.Velocity); 

        public static final SwerveRequest.FieldCentric FCdriveAuton = new SwerveRequest.FieldCentric().withForwardPerspective(ForwardPerspectiveValue.BlueAlliance);
        public static final SwerveRequest.FieldCentric StopDrivetrain = FCdriveAuton.withVelocityX(0 ) // Drive forward with // negative Y (forward)
    .withVelocityY(0 ) // Drive left with negative X (left)
    .withRotationalRate(0);
        // private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
        // private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
        // private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.Velocity);

        public static final double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
        public static final double MaxAngularRate = RotationsPerSecond.of(0.3).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
        //public static final double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

        public static final double minXposeErrorMetersToCorrect = Units.inchesToMeters(.9);//.6;
        public static final double minYposeErrorMetersToCorrect = Units.inchesToMeters(.9);//.6;
        public static final double minRZErrorToCorrect = .9;//1;//.45;//0.5;//1;//2;//1.25;

        public static  double k_PoseX_P = 3.0;//2.6;//3.0;//2.1;//4;
        public static  double k_PoseX_I = 0.1;//.6;//0.0;//0.000001;//0.02;
        public static  double k_PoseX_D = 0.0;//.0;//0.06;

        public static  double k_PoseY_P = k_PoseX_P;//.5;//1.20;
        public static  double k_PoseY_I = k_PoseX_I;//0.000001;//0.02;
        public static  double k_PoseY_D = k_PoseX_D;//0.15;//0.002; 

        public static  double k_RZ_P = 0.11;//.05;
        public static  double k_RZ_I = 0.01;//0.00;
        public static  double k_RZ_D = 0.000000;//0.00;

        //if we are really far away lets keep pid from going insane.
        public static final double maxYvelocity = 1.0;
        public static final double maxXvelocity = 1.0;
        public static final double maxRZvelocity = MaxAngularRate /2;

        


    }
    
    public class CanBus {

        public static final String RioCANBusName = "rio";
        public static final String CanivoreCANBusName = "8608ChassisCan";
        public static final double canBusUpdateFrequency = 50;
        public static final double canBusCanivoreUpdateFrequency = 100;

        public static final String limelightFrontName = "limelight-front";
        public static final String limelightBackName = "limelight-back";

        public static final int elevatorMotor1CanID = 40;
        public static final int elevatorMotor2CanID = 41;
        public static final int MantaRayMotorCanID = 44;

        public static final int armExtensionMotorCanID = 46;
        
        public static final int armPivotMotorCanID = 45;
        public static final int pivotAbsoluteEncoder = 35;

        public static final int josiahClimberCatchMotorCanID = 42;//42 on canivore
        public static final int josiahClimberSlideCanID = 43;

        public static final int MotorizedRearIntakeCanID = 47;
        public static final int CanRangeRearIntakeCanID = 3;

        public static final int CANdleID = 2;

        public static final int groundPivot = 60;
        public static final int groundAbsoluteEncoder = 36;
        public static final int intakeRoller = 48;
        public static final int rollerCANRange = 50;
       
        
    

    }
    public class groundIntake{
        //soft limits
        public static final double gearRatio = (50.0/9.0) * (48.0/16.0) * (48.0/16.0) * (38.0/17.0);
        public static final double maxStatorCurrent = 60;

        public static final double maxPosition = 0.5; //.5;
        public static final double minPosition = 0.0; //0;

        //PID+FF
        public static final double kP = 240;
        public static final double kI = 0;
        public static final double kD = 30;

        public static final double kS = 0;
        public static final double kA = 0.2;
        public static final double kV = 0;

        public static final double kG = 0;

        //Motion Magic
        public static final double Jerk = 0;
        public static final double Accel = 6.0;
        public static final double Cruise = 8.0;
        public static final double absoMagnetOffset = 0.338134765625;
        public static final double PIDtolerance = 0.015;
        public static final double MovementDebounceTime = 0.02;


        public class positions{
            public static final double groundGrab = 0.5;
            public static final double climbReady = 0.27;
            public static final double handOffReady = 0.29;
            public static final double handOffStage2 = 0.16;
            public static final double L1Ready = 0.29;
            public static final double Park = minPosition;
        }

        public class intakeRoller{
            public static final double maxStatorCurrent = 50;

            public static final double kP = 20;
            public static final double kI = 0;
            public static final double kD = 0;

            public static final double kS = 0.1;
            public static final double kA = 0.0;
            public static final double kV = 0;

            public static final double fullSpeed = 7000;
            public static final double L1ShootSpeed = 5000;
            public static final double fastButNotFull = 4000;
            public static final double somewhatSlowSpeed = 2000;
            public static final double slowRunIn = 200;
        }
    }
    
    public class MantaRay {
        public static final double IntakeDutyCycle = .4;
        public static final double intakeAmpLimit = 60;

        public static final double intakeAmpCutoffThreshold = 35;//20;//17;
        public static final double intakeAmpLimittime = 0.125;//.125;
    }

    public class RearMotorizedIntake {
        public static final double gearRatio = 3*1; //27:1 gear reduction (was60)
        public static final double maxStatorCurrent = 40;

        public static final double kP = 3.0;
        public static final double kI = 0.3;
        public static final double kD = 0;

        public static final double kA = 0;
        public static final double kS = 1.0;//2.12
        public static final double kV = 0.047;
       
        public static double ReversingdutyCyclePercent = .16;
       // public static double dutyCyclePercent = .14; // different from wanted an rps, its just volt out. 
        public static  double IntakeRps = 17.25;//24;//25; // this is "WantedRPM * 60" to make the RPM into RPS
        //public static double feedforwardsamps = 12.5; //kt = .0198
    }
 
    public class Elevator {
        public static final double kP = 60.0;//30.0; //15 //MotionMagic
        public static final double kI = 20.0; //6
        public static final double kD = 8.0; //0

        public static final double kS = 0.0;
        public static final double kG = 17;
        public static final double kV = 0.0;
        public static final double kA = 0.0;
        

        //higher accel number is more accel for this subsystem
        public static final double Accel = 250.0;
        public static final double Jerk = 1200;//0;//480.0;
        public static final double Cruise = 500;//180.0;



        
        public static final double gearRatio = (60/11);//60T / 11T (driven/drive) = 5.45454545

        public static final double maxElevatorheight = 26.43;//26.43 measured actual
        public static final double minElevatorHeight = 0;

        public static final double maxStatorCurrent = 100;
        
        public static final double ElevatorBrakeParkTolerance = 1.0;
        public static final double MoveTolerance = .7;//.5;

        public static final double CannotPivotParkBelowElevatorPosition = 18;//when elevator is above this, pivot must be in travel position for elevator to go below this number
        public static final double CannotPivotParkAboveElevatorPosition = 2.0;//when elevator is below this, pivot must be in TPos for elevator to go above this number. 
        
        public static final double groundPickup = minElevatorHeight;
        public static final double AngledgroundPickup = 7.0;
        public static final double ProcessorPos = 4.0;
        public static final double l1Position = 6.5;//5.5;
        public static final double l2Position = 9.0;
        public static final double l2Algae = 12.7;//15.5;
        public static final double l3Position = 18.0;
        public static final double l3algae = 20.0;
        public static final double l4Position = maxElevatorheight;
        public static final double BargePosition = maxElevatorheight;
    }

    public class PlasmaPivot {

        public static final double gearRatio = 4*9*(52/26);//is now 4:1 on 9:1 on 26T driving gear to 52T driven gear - 72:1 gear ratio.
        public static final double absoMagnetOffset = -0.480712890625;//0.4716796875;//
       
        public static final double minPositionToBeSafeFromStage1Crossbar = .18;//.03;////3.55; //cant be folded up too much when elevator goes up or else head crashes.
        public static final double maxPositionToBeSafeFromSmashingintoReef = .107;//0.060;//7.1;// cant be pointing down too much when elevator goes down or else head crashes. 
        //public static final double maxPositionToBeSafeFromSmashingintoSelf = .192;//.119;//14;// cant be pointing down too much when elevator goes down or else head crashes. 
        
        public static final double MoveTolerance = .015;//.003;
        public static final double MovementDebounceTime = .02;//.1;
        public static final double maxposition = 0.241;
        public static final double minposition = -0.06;//-0.247;
        public static final double AddedOffsetToMinPositionToHold = 0.01;
        public static final double ParkPosition = maxposition;
        public static final double rearintakePos = maxposition;//-0.242;//minposition;
        public static final double SafteyFromWobbleAmount = -.05;
        public static final double TravelPosition = minPositionToBeSafeFromStage1Crossbar + SafteyFromWobbleAmount; //.13;//-.160;//-.120;//.0319;//3.75;
        
        public static final double AlgaeReefPickup = 0.01;
        public static final double processorPivot = .050;
        public static final double GroundPickupPosition = 0.00;//12.0;
        public static final double AngledGroundPickupPos = -0.06;//-0.247;
        public static final double BargePosition = .17;

        public static final double CoralGroundPickup = 0.06;

        public static final double l1ReadyPosition = .033; //.02;
        public static final double l2ReadyPosition = .117;
        public static final double l3ReadyPosition = .107;
        public static final double l4ReadyPosition = l3ReadyPosition + .025;

        public static final double newIntakeReadyPosition = 0.14;


        //public static final double l1ScorePosition = l3ReadyPosition + -.04;
        //public static final double l2ScorePosition = l2ReadyPosition + -.04;

        
        public static final double SideScorePosition = 0.05;
        public static final double L4CoralDropPivotAmount = -.08; //when scoring l4 how should the pivot change. 
        
        

        public static final double maxStatorCurrent = 120;

        public static final double kP = 1500;   //320;//350;//270.0;
        public static final double kI = 0.0;    //400;//160;//60.0;
        public static final double kD = 400.0;  //83;//80.0;

        public static final double kA = 0.276;
        public static final double kV = 0.0;
        public static final double kS = 0.0;//4.0;
        public static final double lowkG = 12.5;
        public static final double highkG = 20;

        public static final double Accel = 30;//30.0;        //9.0;//10.0;
        public static final double Jerk = 0;//120;//0;        //400.0;
        public static final double Cruise = 120;//100.0;

        public static final double ExpokA = 0.1;
        public static final double ExpokV = 0.12;
  
        
        


       

        
    }

    public class PlasmaExtension {
        //CURRENTLY EACH "1" POSITION is .33" approx more etenxsion.
        public static double LiveOffset = 2.0;//-2.0; 
        //0.7407407
        public static final double gearRatio = 4*5;//3*3*3; //27:1 gear reduction (was60)
  
        public static final double maxStatorCurrent = 40;
        public static final double maxposition = 36.29;//117;
        public static final double minposition = 0;
        public static final double kP = .37;//1.0
        public static final double kI = 0.0;
        public static final double kD = 0;
        public static final double MoveTolerance = .3;//.5;

        //public static final double positionAddIfCoralOffset = 6;
        public static final double maxPositionToBeSafeFromStage1Crossbar = 0;
        public static final double parkPostion = 0.0;//0.37;
        public static final double rearintakePos = 0.0;//0.37;//.5;
        public static final double l1ReadyPosition =  25.18;//34.0;//uses LiveOffset //the ready position in the reef is right before we swoop to score.
        public static final double l2ReadyPosition = 28.14;//38;//uses LiveOffset
        public static final double l3ReadyPosition =  27;//41.0;//uses LiveOffset //the ready position in the reef is right before we swoop to score.

        public static final double l1ScorePosition = 11.644;//15.55;//21.0;//uses LiveOffset
        public static final double l2ScorePosition = 0;//uses LiveOffset
        public static final double l4ScorePosition = 28.8;//39;//uses LiveOffset
        public static final double ReefAlgaePickupExtension = maxposition - 3;//-7.4;
        public static final double GroundPickupExtension = 28;//38;
         


        public static final double climbExtension = 9.6291;//13;

        public static final double processorExtension = 7.40;//10;
    }
        



    public class Climber {
        //how many seconds left in the match for the climb lights to show
        public static final double secondsToClimb = 20; //20 for actual matches, 130 for testing without waiting 2:10

        public static final double maxStatorCurrent = 120;
        public static final double maxSupplyCurrent = 120;
        public static final double kP = 0.5;
        public static final double kI = 0.04;
        public static final double kD = 0;

        public static final double MoveTolerance = 0.5;

        public static class CatchSide {
            public static final double gearRatio = 45; //9:1 on a 5:1
            public static final double maxPostion = 5.25;//4.12; //max pos will be hook position
            public static final double minPostion = -9.5; //min position will be negative and will be prolly be full climb position
            public static final double startPos = 0; // we start at flat across with tips almost touching. this is out start pos
            public static final double LoadPostion = maxPostion; //the position this motor is in when we are in load mode catch side is at max hook height slide is folder back over to allow cage in
            public static final double FullClimbPostion = minPostion;
        }

        public static class SlideSide {
            public static final double gearRatio = 45; //9:1 on a 5:1
            public static final double maxPostion = 21.0;
            public static final double minPostion = -9;
            public static final double startPos = 0;
            public static final double LoadPostion = maxPostion;
            public static final double FullClimbPostion = minPostion;
        }



        
    }


}
