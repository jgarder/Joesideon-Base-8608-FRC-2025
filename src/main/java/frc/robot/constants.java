package frc.robot;

import edu.wpi.first.math.util.Units;

public class constants {

    public static final double ReefWidthCenteronCenter = Units.inchesToMeters(15);
    
    public static class drivetrainThings{
        public static final double minXposeErrorToCorrect = .06;
        public static final double minYposeErrorToCorrect = .06;
        public static final double minRZErrorToCorrect = 2;//1.25;

        public static final double k_PoseX_P = .5;//.5;//1.20;
        public static final double k_PoseX_I = .0000005;//0.000001;//0.02;
        public static final double k_PoseX_D = .06;//0.15;//0.0020;

        public static final double k_PoseY_P = .5;//.5;//1.20;
        public static final double k_PoseY_I = .0000005;//0.000001;//0.02;
        public static final double k_PoseY_D = .06;//0.15;//0.002; 

        public static final double k_RZ_P = 0.009000;//.05;
        public static final double k_RZ_I = 0.000000;//0.00;
        public static final double k_RZ_D = 0.000000;//0.00;

        //if we are really far away lets keep pid from going insane.
        public static final double maxYvelocity = .75;
        public static final double maxXvelocity = .75;
        public static final double maxRZvelocity = 3;

        public static final double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity


    }
    
    public class CanBus {

        public static final String RioCANBusName = "rio";
        public static final String CanivoreCANBusName = "8608ChassisCan";
        public static final double canBusUpdateFrequency = 50;
        public static final double canBusCanivoreUpdateFrequency = 100;

        public static final String limelightFrontName = "limelight";
        public static final String limelightBackName = "limelight-back";

        public static final int elevatorMotor1CanID = 40;
        public static final int elevatorMotor2CanID = 41;
        public static final int MantaRayMotorCanID = 44;

        public static final int armExtensionMotorCanID = 46;
        
        public static final int armPivotMotorCanID = 45;
        public static final int pivotAbsoluteEncoder = 35;

        public static final int josiahClimberCatchMotorCanID = 42;//42 on canivore
        public static final int josiahClimberSlideCanID = 43;

        public static final int fryscoopIntakeMotorCanID = 0;

        public static final int CANdleID = 2;
        

    }

    public class MantaRay {
        public static final double IntakeDutyCycle = .3;
        public static final double intakeAmpLimit = 40;

        public static final double intakeAmpCutoffThreshold = 15; //20
        public static final double intakeAmpLimittime = .10;
    }
    public class Elevator {
        public static final double kP = 5.0; //MotionMagic
        public static final double kI = 0.0;
        public static final double kD = 0.0;

        public static final double kS = 23.0;
        public static final double kG = 10;
        public static final double kV = 0.0;
        

        public static final double Accel = 90.0;
        public static final double Jerk = 400.0;
        public static final double Cruise = 30.0;



        
        public static final double gearRatio = (60/11);//60T / 11T (driven/drive) = 5.45454545

        public static final double maxElevatorheight = 26.5;
        public static final double minElevatorHeight = 0;

        public static final double maxStatorCurrent = 60;
        
        public static final double ElevatorBrakeParkTolerance = 1.0;
        public static final double MoveTolerance = .5;

        public static final double CannotPivotParkBelowElevatorPosition = 18;//when elevator is above this, pivot must be in travel position for elevator to go below this number
        public static final double CannotPivotParkAboveElevatorPosition = 3;//when elevator is below this, pivot must be in TPos for elevator to go above this number. 
        
        public static final double ProcessorPos = 4.0;
        public static final double l1Position = 4.0;
        public static final double l2Position = 8.0;
        public static final double l3Position = 12.0;
        public static final double l4Position = maxElevatorheight-1;
        public static final double BargePosition = maxElevatorheight-1;
    }

    public class PlasmaPivot {

        public static final double gearRatio = 4*4*5; //3 stage gearbox with 4:1,4:1,5:1 stacked ontop of each other; creates a 80:1 gear ratio 
        public static final double absoMagnetOffset = .07667;//0.316162;
       
        public static final double minPositionToBeSafeFromStage1Crossbar = -.192;//.03;////3.55; //cant be folded up too much when elevator goes up or else head crashes.
        public static final double maxPositionToBeSafeFromSmashingintoReef = -.107;//0.060;//7.1;// cant be pointing down too much when elevator goes down or else head crashes. 
        public static final double maxPositionToBeSafeFromSmashingintoSelf = -.24;//.119;//14;// cant be pointing down too much when elevator goes down or else head crashes. 
        
        public static final double MoveTolerance = .03;
        public static final double maxposition = 0;
        public static final double minposition = -0.240234375;

        public static final double ParkPosition = minposition;
        public static final double TravelPosition = -.150;//.0319;//3.75;
        public static final double GroundPickupPosition = -0.05;//0.102;//12.0;

        public static final double l1ReadyPosition = -.107;//0.06388;
        public static final double l1ScorePosition = l1ReadyPosition + .008;

        public static final double l2ReadyPosition = -.117;//0.06388;
        public static final double l2ScorePosition = l2ReadyPosition + .008;

        public static final double l4ReadyPosition = l1ReadyPosition;

        public static final double maxStatorCurrent = 40;

        public static final double kP = 250.0;
        public static final double kI = 10.00;
        public static final double kD = 0.00;

        public static final double kV = 0.3;
        public static final double kS = 5.0;
        public static final double kG = -18.5;

        public static final double Accel = 5000.0;
        public static final double Jerk = 0.0;//800
        public static final double Cruise = 100.0;
        


       

        
    }

    public class PlasmaExtension {
        public static final double gearRatio = 3*3*3; //27:1 gear reduction (was60)
        public static final double maxPositionToBeSafeFromStage1Crossbar = 0;
        public static final double maxStatorCurrent = 40;
        public static final double maxposition = 49;//117;
        public static final double minposition = 0;
        public static final double kP = 0.06;
        public static final double kI = 0.01;
        public static final double kD = 0;
        public static final double MoveTolerance = 1.0;

        public static final double l1ReadyPosition = 30.0;//the ready position in the reef is right before we swoop to score. 
        public static final double l1ScorePosition = 20.0;

        public static final double l2ReadyPosition = l1ReadyPosition;
        public static final double l2ScorePosition = 0;
    }



    public class Climber {
        public static final double maxStatorCurrent = 120;
        public static final double kP = 0.5;
        public static final double kI = 0.04;
        public static final double kD = 0;

        public static class CatchSide {
            public static final double gearRatio = 45; //9:1 on a 5:1
            public static final double maxPostion = 10;//4.12; //max pos will be hook position
            public static final double minPostion = -9; //min position will be negative and will be prolly be full climb position
            public static final double startPos = 0; // we start at flat across with tips almost touching. this is out start pos
            public static final double LoadPostion = maxPostion; //the position this motor is in when we are in load mode catch side is at max hook height slide is folder back over to allow cage in
            public static final double FullClimbPostion = minPostion;
        }

        public static class SlideSide {
            public static final double gearRatio = 45; //9:1 on a 5:1
            public static final double maxPostion = 22;
            public static final double minPostion = -9;
            public static final double startPos = 0;
            public static final double LoadPostion = maxPostion;
            public static final double FullClimbPostion = minPostion;
        }



        
    }


}
