package frc.robot.subsystems.groundIntake;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants;
import frc.robot.AlphaBots.Tools;
import frc.robot.subsystems.MantaState;

public class groundPivot extends SubsystemBase{
      //Get ClassName to help network tables auto sort by creating a sub Table with the same name.
  String className = this.getClass().getSimpleName();
  
  public final TalonFX m_armMotor = new TalonFX(constants.CanBus.groundPivot, constants.CanBus.RioCANBusName);
    private final StatusSignal<Angle> motorPos = m_armMotor.getPosition(false);
    private final StatusSignal<AngularVelocity> motorVel = m_armMotor.getVelocity(false);
  public final CANcoder armAbsoluteEncoder = new CANcoder(constants.CanBus.groundAbsoluteEncoder, constants.CanBus.RioCANBusName);

  private MotionMagicTorqueCurrentFOC armRequest = new MotionMagicTorqueCurrentFOC(0).withSlot(1).withFeedForward(0);

  TalonFXConfiguration configuration;
  public double gearRatio = constants.groundIntake.gearRatio;

  public double armSetpoint;

  public enum ArmState{
    inPosition,
    outOfPostion,
    unknown
  }

  //we dont know the arm state on boot, does the same thing as outOfPosition
  ArmState armState = ArmState.unknown;

  public groundPivot() {
    System.out.println("Creating " + className + " object"); 

    // NT_PGain.set(constants.PlasmaPivot.kP);
    // NT_IGain.set(constants.PlasmaPivot.kI);
    // NT_DGain.set(constants.PlasmaPivot.kD);

    // NT_AGain.set(constants.PlasmaPivot.kA);
    // NT_VGain.set(constants.PlasmaPivot.kV);

    // NT_SGain.set(constants.PlasmaPivot.kS);
    // NT_GGain.set(constants.PlasmaPivot.lowkG);

    // NT_Acceleration.set(constants.PlasmaPivot.Accel);
    // NT_Jerk.set(constants.PlasmaPivot.Jerk);
    // NT_Cruise.set(constants.PlasmaPivot.Cruise);

    // NT_RequestedPosition.set(requestedPosition);

    configuration = buildMotorConfig();

    frc.robot.AlphaBots.Tools.SetConfigToTalonFX(m_armMotor,configuration,className);
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

    _configuration.Slot1.kG = 0.0;
    //_configuration.Slot1.GravityType = GravityTypeValue.Arm_Cosine;

    _configuration.Slot1.kS = constants.groundIntake.kS;
    _configuration.Slot1.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;

    _configuration.Feedback.RotorToSensorRatio = constants.groundIntake.gearRatio;

    _configuration.CurrentLimits.StatorCurrentLimitEnable = true;
    _configuration.CurrentLimits.StatorCurrentLimit = constants.groundIntake.maxStatorCurrent;

    _configuration.TorqueCurrent.PeakForwardTorqueCurrent = constants.groundIntake.maxStatorCurrent;
    _configuration.TorqueCurrent.PeakReverseTorqueCurrent = constants.groundIntake.maxStatorCurrent;

    _configuration.TorqueCurrent.TorqueNeutralDeadband = 0.0;

    _configuration.MotionMagic.MotionMagicAcceleration = constants.groundIntake.Accel;
    _configuration.MotionMagic.MotionMagicJerk = constants.groundIntake.Jerk;
    _configuration.MotionMagic.MotionMagicCruiseVelocity = constants.groundIntake.Cruise;

    _configuration.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    _configuration.SoftwareLimitSwitch.ForwardSoftLimitThreshold = constants.groundIntake.maxPosition;

    _configuration.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    _configuration.SoftwareLimitSwitch.ReverseSoftLimitThreshold = constants.groundIntake.minPosition;
    
    _configuration.MotorOutput.withNeutralMode(NeutralModeValue.Brake);

      CANcoderConfiguration cc_cfg = new CANcoderConfiguration();
      //cc_cfg.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf; old 2024 and before way. 
      //Setting this to 1 makes the absolute position unsigned [0, 1)
      //Setting this to 0.5 makes the absolute position signed [-0.5, 0.5)
      //Setting this to 0 makes the absolute position always negative [-1, 0) 
      cc_cfg.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
      
      cc_cfg.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
      cc_cfg.MagnetSensor.MagnetOffset = constants.groundIntake.absoMagnetOffset;// ;
      armAbsoluteEncoder.getConfigurator().apply(cc_cfg);

    var AbsoluteEncoderFeedbackConfig = new FeedbackConfigs().withFeedbackRemoteSensorID(armAbsoluteEncoder.getDeviceID())
       .withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder)
       .withRotorToSensorRatio(constants.groundIntake.gearRatio).withSensorToMechanismRatio(1.0);

    _configuration.withFeedback(AbsoluteEncoderFeedbackConfig);

    return _configuration;
  }

   @Override
  public void periodic() {
    //sets the arbitrary feedforward
    armRequest = armRequest.withFeedForward(calculateArmkG(getPosition()));

    boolean ArmIsAtSetpoint = MathUtil.isNear(armSetpoint, getPosition(), constants.groundIntake.armPositionErrorTolerance);
    if(ArmIsAtSetpoint){
        armState = ArmState.inPosition;
    }else{armState = ArmState.outOfPostion;}

    // NT_CurrentPosition.set(currentPosition);
    // NT_SetpointPosition.set(setPointPosition);

    // // This method will be called once per scheduler run
    // NT_Rpm.set(m_PivotMotor.getVelocity().getValueAsDouble() * 60);
    // NT_MotorTemp.set(m_PivotMotor.getDeviceTemp().getValueAsDouble());
    // NT_StatorCurrent.set(m_PivotMotor.getStatorCurrent().getValueAsDouble());


    // NT_FoldedOut.set(IsPivotFoldedOut.getAsBoolean());
    // //NT_FoldedUpEnough.set(IsPivotFoldedFarOut.getAsBoolean());
    // NT_FoldedUpFromReef.set(IsPivotAwayFromReef.getAsBoolean());
    // NT_ElevatorTravelPosition.set(IsPivotinTravelPosition.getAsBoolean());

    // //feedback
    // double p = NT_PGain.getAsDouble();
    // double i = NT_IGain.getAsDouble();
    // double d = NT_DGain.getAsDouble();

    // //feedforward
    // double a = NT_AGain.getAsDouble();
    // double v = NT_VGain.getAsDouble();
    // double s = NT_SGain.getAsDouble();
    // double g = NT_GGain.getAsDouble();

    // double mA = NT_Acceleration.getAsDouble();
    // double mJ = NT_Jerk.getAsDouble();
    // double mC = NT_Cruise.getAsDouble();
    // boolean motorNeedsConfig = false;

    // if((p != configuration.Slot1.kP)) { configuration.Slot1.kP = p; motorNeedsConfig = true; }
    // if((i != configuration.Slot1.kI)) { configuration.Slot1.kI = i; motorNeedsConfig = true; }
    // if((d != configuration.Slot1.kD)) { configuration.Slot1.kD = d; motorNeedsConfig = true; }
  
    // if((a != configuration.Slot1.kA)) { configuration.Slot1.kA = a; motorNeedsConfig = true; }
    // if((v != configuration.Slot1.kV)) { configuration.Slot1.kV = v; motorNeedsConfig = true; }
    // if((s != configuration.Slot1.kS)) { configuration.Slot1.kS = s; motorNeedsConfig = true; }
    // //if((g != tempkG)) { tempkG = g; motorNeedsConfig = true; }

    // if((mA != configuration.MotionMagic.MotionMagicAcceleration)) { configuration.MotionMagic.MotionMagicAcceleration = mA; motorNeedsConfig = true; }
    // if((mJ != configuration.MotionMagic.MotionMagicJerk)) { configuration.MotionMagic.MotionMagicJerk = mJ; motorNeedsConfig = true; }
    // if((mC != configuration.MotionMagic.MotionMagicCruiseVelocity)) { configuration.MotionMagic.MotionMagicCruiseVelocity = mC; motorNeedsConfig = true; }
    
    // if (motorNeedsConfig){Tools.SetConfigToTalonFX(m_armMotor,configuration,className);}

    //doTravelIfelevatormoving(requestedPosition);
  }
  public double calculateArmkG(double armCurrentPosition){
    //changes kG depending on whether we need more force, for example if the arm is extended or we have a coral
    // InterpolatingDoubleTreeMap gravityExtensionTable = new InterpolatingDoubleTreeMap();
    //   gravityExtensionTable.put(constants.PlasmaExtension.minposition, constants.PlasmaPivot.lowkG);
    //   gravityExtensionTable.put(constants.PlasmaExtension.maxposition, constants.PlasmaPivot.highkG);
    //converts arm position to radians 
    double armCurrentPositionRadians = Units.degreesToRadians(armCurrentPosition * 360);
    //calculates the arbitrary feedforward (used as kG) to be sent to the motor
    double armKG = constants.groundIntake.kG * Math.cos(armCurrentPositionRadians);//gravityExtensionTable.get(armExtension)
    //NT_GGain.set(armKG);
    return armKG;
  }

  public void setArmRequest(double position){
      m_armMotor.setControl(armRequest.withPosition(position));
  }

  /**
   * Returns Latency Compensated Position of the arm
   * @return Double
   */
  public double getPosition(){
    BaseStatusSignal.refreshAll(motorPos, motorVel);
    return BaseStatusSignal.getLatencyCompensatedValueAsDouble(motorPos, motorVel);
  }
}
