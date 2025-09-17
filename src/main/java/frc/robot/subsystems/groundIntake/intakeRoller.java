package frc.robot.subsystems.groundIntake;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants;

public class intakeRoller extends SubsystemBase{
    //Get ClassName to help network tables auto sort by creating a sub Table with the same name.
    String className = this.getClass().getSimpleName();

    public final TalonFX m_rollerMotor = new TalonFX(constants.CanBus.intakeRoller, constants.CanBus.RioCANBusName);
        TalonFXConfiguration configuration;

    public final CANrange canRange = new CANrange(constants.CanBus.rollerCANRange, constants.CanBus.RioCANBusName);
        CANrangeConfiguration rangeConfiguration;

    private VelocityTorqueCurrentFOC rollerRequest = new VelocityTorqueCurrentFOC(0).withSlot(1).withFeedForward(0);

    private Boolean hasCoral = false;


    public intakeRoller(){
        System.out.println("Creating " + className + " object");

        configuration = buildMotorConfig();
        rangeConfiguration = buildCANRangeConfig();
        frc.robot.AlphaBots.Tools.SetConfigToTalonFX(m_rollerMotor,configuration,className);
        canRange.getConfigurator().apply(rangeConfiguration);
    }

    public TalonFXConfiguration buildMotorConfig(){
    TalonFXConfiguration _configuration = new TalonFXConfiguration();

    _configuration.withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));

    //_configuration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    _configuration.Slot1.kP = constants.groundIntake.kP;
    _configuration.Slot1.kI = constants.groundIntake.kI;
    _configuration.Slot1.kD = constants.groundIntake.kD;

    _configuration.Slot1.kA = constants.groundIntake.kA;
    _configuration.Slot1.kV = constants.groundIntake.kV;

    _configuration.Slot1.kS = constants.groundIntake.kS;
    _configuration.Slot1.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;

    _configuration.CurrentLimits.StatorCurrentLimitEnable = true;
    _configuration.CurrentLimits.StatorCurrentLimit = constants.groundIntake.maxStatorCurrent;

    _configuration.TorqueCurrent.PeakForwardTorqueCurrent = constants.groundIntake.maxStatorCurrent;
    _configuration.TorqueCurrent.PeakReverseTorqueCurrent = constants.groundIntake.maxStatorCurrent;

    _configuration.TorqueCurrent.TorqueNeutralDeadband = 0.0;
    
    _configuration.MotorOutput.withNeutralMode(NeutralModeValue.Brake);

    return _configuration;
  }
  
  public CANrangeConfiguration buildCANRangeConfig(){
    CANrangeConfiguration _rangeConfiguration = new CANrangeConfiguration();

    _rangeConfiguration.ProximityParams.ProximityThreshold = 0;

    return _rangeConfiguration;
  }
}
