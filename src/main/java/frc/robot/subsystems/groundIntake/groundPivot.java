package frc.robot.subsystems.groundIntake;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

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
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants;
import frc.robot.AlphaBots.NT;
import frc.robot.AlphaBots.Tools;
import frc.robot.subsystems.MantaState;

public class groundPivot extends SubsystemBase{
      //Get ClassName to help network tables auto sort by creating a sub Table with the same name.
  String className = this.getClass().getSimpleName();
  
  public final TalonFX m_armMotor = new TalonFX(constants.CanBus.groundPivot, constants.CanBus.CanivoreCANBusName);
    private final StatusSignal<Angle> motorPos = m_armMotor.getPosition(false);
    private final StatusSignal<AngularVelocity> motorVel = m_armMotor.getVelocity(false);
  public final CANcoder armAbsoluteEncoder = new CANcoder(constants.CanBus.groundAbsoluteEncoder, constants.CanBus.CanivoreCANBusName);

  private MotionMagicTorqueCurrentFOC armRequest = new MotionMagicTorqueCurrentFOC(0).withSlot(1).withFeedForward(0);

  TalonFXConfiguration configuration;
  public double gearRatio = constants.groundIntake.gearRatio;

  double tempkG = 0;
  LoggedNetworkNumber sfda = new LoggedNetworkNumber("Tuning/" + className + "/" + "KG", tempkG);
  // DoubleEntry NT_GGain = NT.getDoubleEntry(className , "G Gain",0);

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

    // NT_GGain.set(constants.groundIntake.kG);

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
      //Setting this to 1 makes the absolute position unsigned [0, 1)
      //Setting this to 0.5 makes the absolute position signed [-0.5, 0.5)
      //Setting this to 0 makes the absolute position always negative [-1, 0) 
      cc_cfg.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
      
      cc_cfg.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
      cc_cfg.MagnetSensor.MagnetOffset = constants.groundIntake.absoMagnetOffset;
      armAbsoluteEncoder.getConfigurator().apply(cc_cfg);

    var AbsoluteEncoderFeedbackConfig = new FeedbackConfigs().withFeedbackRemoteSensorID(armAbsoluteEncoder.getDeviceID())
       .withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder)
       .withRotorToSensorRatio(constants.groundIntake.gearRatio).withSensorToMechanismRatio(1.0);

    _configuration.withFeedback(AbsoluteEncoderFeedbackConfig);

    return _configuration;
  }

  @Override
  public void periodic() {
    tempkG = sfda.get();
    //sets the arbitrary feedforward
    double calculatedArmkG = calculateArmkG(getPosition());

    armRequest = armRequest.withFeedForward(calculatedArmkG);

    boolean ArmIsAtSetpoint = MathUtil.isNear(armSetpoint, getPosition(), constants.groundIntake.PIDtolerance);
    if(ArmIsAtSetpoint){
        armState = ArmState.inPosition;
    }else{armState = ArmState.outOfPostion;}

    /*
    copy paste this for ease
     Logger.recordOutput(className + "/" + "", null);
     */
    Logger.recordOutput(className + "/" + "motor Pos no latency compensation", m_armMotor.getPosition().getValueAsDouble());
    Logger.recordOutput(className + "/" +"motorPos Latency Compensated", getPosition());
    Logger.recordOutput(className + "/" + "Aboslutely positioned", armAbsoluteEncoder.getAbsolutePosition().getValueAsDouble());
    Logger.recordOutput(className + "/" + "PickupIsAtSetpoint", ArmIsAtSetpoint);
    Logger.recordOutput(className + "/" + "ArmState", armState);  
    Logger.recordOutput(className + "/" + "arm KG", calculatedArmkG);
    Logger.recordOutput(className + "/" + "Stator current draw", m_armMotor.getStatorCurrent().getValueAsDouble());
    Logger.recordOutput(className + "/" + "motor Temp", m_armMotor.getDeviceTemp().getValueAsDouble());
    

    // double g = NT_GGain.getAsDouble();
    // if((g != tempkG)) { tempkG = g;}
  }

  public double calculateArmkG(double armCurrentPosition){
    //changes kG depending on whether we need more force, for example if the arm is extended or we have a coral
    // InterpolatingDoubleTreeMap gravityExtensionTable = new InterpolatingDoubleTreeMap();
    //   gravityExtensionTable.put(constants.PlasmaExtension.minposition, constants.PlasmaPivot.lowkG);
    //   gravityExtensionTable.put(constants.PlasmaExtension.maxposition, constants.PlasmaPivot.highkG);
    //converts arm position to radians 
    double armCurrentPositionRadians = Units.degreesToRadians(armCurrentPosition * 360);
    //calculates the arbitrary feedforward (used as kG) to be sent to the motor
    double armKG =  tempkG * Math.cos(armCurrentPositionRadians);//gravityExtensionTable.get(armExtension)
    //NT_GGain.set(armKG);
    return armKG;
  }

  public void setArmRequest(double position){
      m_armMotor.setControl(armRequest.withPosition(position));
      armSetpoint = position;
  }

  /**
   * Returns Latency Compensated Position of the arm
   * @return Double
   */
  public double getPosition(){
    BaseStatusSignal.refreshAll(motorPos, motorVel);
    return BaseStatusSignal.getLatencyCompensatedValueAsDouble(motorPos, motorVel);
  }

  public ArmState getArmState(){
      return armState;
  }
}
