package frc.robot;

public class constants {
    
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

        public static final int josiahClimberCatchMotorCanID = 0;
        public static final int josiahClimberSlideCanID = 0;

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
        public static final double maxElevatorheight = 25;
        public static final double minElevatorHeight = 0;

        public static final double maxStatorCurrent = 40;

    }

    public class Pivot {
        public static final double minPositionToBeSafeFromStage1Crossbar = 5; //cant be folded up too much when elevator goes up or else head crashes.
        public static final double maxPositionToBeSafeFromSmashingintoSelf = 20;// cant be pointing down too much when elevator goes down or else head crashes.

        public static final double maxposition = 25;
        public static final double minposition = 0;

        public static final double maxStatorCurrent = 40;

        public static final double kP = 0.013;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double elevatorheightToFoldUp = 5;
    }

    public class PlasmaExtension {

        public static final double maxExtension = 10;
        public static final double minExtension = 0;
        public static final double maxPositionToBeSafeFromStage1Crossbar = 0;
        public static final double maxStatorCurrent = 0;
        public static final double maxposition = 0;
        public static final double minposition = 0;
        public static final double kP = 0;
        public static final double kI = 0;
        public static final double kD = 0;
    }

    public class PlasmaPivot {

        public static final double maxPivot = 10;
        public static final double minPivot = 0;

    }

    public class Climber {
        public static final double maxStatorCurrent = 40;
        public class CatchSide {
            
            public static final double maxPostion = 10; //max pos will be hook position
            public static final double minPostion = 0; //min position will be negative and will be prolly be full climb position
            public static final double startPos = 0; // we start at flat across with tips almost touching. this is out start pos
            public static final double LoadPostion = 0; //the position this motor is in when we are in load mode catch side is at max hook height slide is folder back over to allow cage in
            public static final double FullClimbPostion = 0;
        }

        public class SlideSide {
            public static final double maxPostion = 20;
            public static final double minPostion = 0;
            public static final double startPos = 0;
            public static final double LoadPostion = 0;
            public static final double FullClimbPostion = 0;
        }



        
    }


}
