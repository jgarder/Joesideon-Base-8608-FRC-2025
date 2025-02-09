package frc.robot;

public class constants {
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

        public static final String roboRIO = "rio";
        public static final String kCANbusName = "8608ChassisCan";
        public static final double canBusUpdateFrequency = 50;
        public static final double canBusCanivoreUpdateFrequency = 100;

        public static final String limelightFrontName = "limelight";
        public static final String limelightBackName = "limelight-back";

        public static final int elevatorMotor1CanID = 40;
        public static final int elevatorMotor2CanID = 41;
        public static final int MantaRayMotorCanID = 42;

        public static final int armExtensionMotorCanID = 0;
        public static final int armPivotMotorCanID = 0;

        public static final int josiahClimberCatchMotorCanID = 6;
        public static final int josiahClimberSlideCanID = 7;

        public static final int fryscoopIntakeMotorCanID = 0;

        public static final int CANdleID = 11;

    }

    public class MantaRay {
        public static final double IntakeDutyCycle = .3;
        public static final double intakeAmpLimit = 40;

        public static final double intakeAmpCutoffThreshold = 15; //20
        public static final double intakeAmpLimittime = .10;
    }
    public class Elevator {
        public static final double gearRatio = (60/11);//60T / 11T (driven/drive) = 5.45454545

        public static final double maxElevatorheight = 25;
        public static final double minElevatorHeight = 0;

        public static final double maxStatorCurrent = 40;

        public static final double MoveTolerance = 0;

    }

    public class PlasmaPivot {

        public static final double gearRatio = 3*4*5; //3 stage gearbox with 3:1,4:1,5:1 stacked ontop of each other; creates a 60:1 gear ratio 

        public static final double minPositionToBeSafeFromStage1Crossbar = 5; //cant be folded up too much when elevator goes up or else head crashes.
        public static final double maxPositionToBeSafeFromSmashingintoSelf = 20;// cant be pointing down too much when elevator goes down or else head crashes.

        public static final double maxposition = 25;
        public static final double minposition = 0;

        public static final double maxStatorCurrent = 40;

        public static final double kP = 0.013;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double elevatorheightToFoldUp = 5;

        public static final double MoveTolerance = 0;
    }

    public class PlasmaExtension {
        public static final double gearRatio = 60;
        public static final double maxExtension = 10;
        public static final double minExtension = 0;
        public static final double maxPositionToBeSafeFromStage1Crossbar = 0;
        public static final double maxStatorCurrent = 0;
        public static final double maxposition = 0;
        public static final double minposition = 0;
        public static final double kP = 0;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double MoveTolerance = 0;
    }



    public class Climber {
        public static final double maxStatorCurrent = 120;
        public static final double kP = 0.05;
        public static final double kI = 0.04;
        public static final double kD = 0;

        public static class CatchSide {
            public static final double gearRatio = 45; //9:1 on a 5:1
            public static final double maxPostion = 4.12; //max pos will be hook position
            public static final double minPostion = -8; //min position will be negative and will be prolly be full climb position
            public static final double startPos = 0; // we start at flat across with tips almost touching. this is out start pos
            public static final double LoadPostion = maxPostion; //the position this motor is in when we are in load mode catch side is at max hook height slide is folder back over to allow cage in
            public static final double FullClimbPostion = minPostion;
        }

        public static class SlideSide {
            public static final double gearRatio = 45; //9:1 on a 5:1
            public static final double maxPostion = 22;
            public static final double minPostion = -8;
            public static final double startPos = 0;
            public static final double LoadPostion = maxPostion;
            public static final double FullClimbPostion = minPostion;
        }



        
    }


}
