package frc.robot;

public class constants {
    
    public class CanBus {

        public static final String roboRIO = "rio";
        public static final String kCANbusName = "8608ChassisCan";

        public static final String limelightFrontName = "limelight";
        public static final String limelightBackName = "limelight-back";

        public static final int elevatorMotor1CanID = 40;
        public static final int elevatorMotor2CanID = 41;
        public static final int TridentMotorCanID = 42;

        public static final int armExtensionMotorCanID = 0;
        public static final int armPivotMotorCanID = 0;

        public static final int josiahClimberMotor1CanID = 0;
        public static final int josiahClimberMotor2CanID = 0;

        public static final int fryscoopIntakeMotorCanID = 0;

        public static final int CANdleID = 11;

    }

    public class Trident {
        public static final double IntakeDutyCycle = .3;
        public static final double intakeAmpLimit = 40;

        public static final double intakeAmpCutoffThreshold = 15; //20
        public static final double intakeAmpLimittime = .10;
    }
}
