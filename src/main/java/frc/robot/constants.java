package frc.robot;

public class constants {
    
    public class CanBus {

        public static final String roboRIO = "rio";
        public static final String kCANbusName = "8608ChassisCan";

        public static final String limelightFrontName = "limelight";
        public static final String limelightBackName = "limelight-back";

        public static final int elevatorMotor1CanID = 40;
        public static final int elevatorMotor2CanID = 41;
        public static final int MantaRayMotorCanID = 42;

        public static final int armExtensionMotorCanID = 0;
        public static final int armPivotMotorCanID = 0;

        public static final int josiahClimberMotor1CanID = 0;
        public static final int josiahClimberMotor2CanID = 0;

        public static final int fryscoopIntakeMotorCanID = 0;

        public static final int CANdleID = 11;

    }

    public class MantaRay {
        public static final double IntakeDutyCycle = .3;
        public static final double intakeAmpLimit = 40;

        public static final double intakeAmpCutoffThreshold = 15; //20
        public static final double intakeAmpLimittime = .10;
    }
    public class PlasmaExtension {

        public static final double maxExtension = 10;
        public static final double minExtension = 0;
    }

    public class PlasmaPivot {

        public static final double maxPivot = 10;
        public static final double minPivot = 0;

    }

    public class josiahClimber {

        public class CatchSide {
            public static final double maxPostion = 10;
            public static final double minPostion = 0;
            public static final double LoadPostion = 0;
            public static final double FullClimbPostion = 0;
        }

        public class SlideSide {
            public static final double maxPostion = 20;
            public static final double minPostion = 0;
            public static final double LoadPostion = 0;
            public static final double FullClimbPostion = 0;
        }



        
    }


}
