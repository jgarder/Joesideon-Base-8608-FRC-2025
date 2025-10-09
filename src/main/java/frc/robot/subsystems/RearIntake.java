package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants;
import frc.robot.AlphaBots.NT;
import frc.robot.AlphaBots.Tools;

public class RearIntake extends SubsystemBase {
 
  //Get ClassName to help network tables auto sort by creating a sub Table with the same name.
  String className = this.getClass().getSimpleName();
  
  public final TalonFX m_Motor = new TalonFX(constants.CanBus.CanBusIDs.MotorizedRearIntakeCanID.id, constants.CanBus.RioCANBusName);
  
  public final CANrange m_CANRange = new CANrange(constants.CanBus.CanBusIDs.CanRangeRearIntakeCanID.id, constants.CanBus.RioCANBusName);
  TalonFXConfiguration configuration;
  
  BooleanEntry NT_IsLoaded = NT.getBooleanEntry(className , "CanRangedIsLoaded",false);
  DoubleEntry NT_Rps =  NT.getDoubleEntry(className ,"RPS",0);
  DoubleEntry NT_MotorTemp =  NT.getDoubleEntry(className,"MotorTemp",0);
  DoubleEntry NT_StatorCurrent = NT.getDoubleEntry(className, "StatorCurrent", 0);
  
  DoubleEntry NT_PGain = NT.getDoubleEntry(className , "P Gain",0);
  DoubleEntry NT_IGain = NT.getDoubleEntry(className, "I Gain",0);
  DoubleEntry NT_DGain = NT.getDoubleEntry(className , "D Gain",0);

  DoubleEntry NT_AGain = NT.getDoubleEntry(className , "A Gain",0);
  DoubleEntry NT_SGain = NT.getDoubleEntry(className , "S Gain",0);
  DoubleEntry NT_VGain = NT.getDoubleEntry(className , "V Gain",0);

  DoubleEntry NT_SetpointVelocity = NT.getDoubleEntry(className , "SetpointRPS",0.0);
  BooleanEntry NT_BrakeEnabled = NT.getBooleanEntry(className , "BrakeOn",false);

  public double DetectionThresholdMeters = .07;//.01 == 10mm; 1 = 1meter == 1000 millimeters.
  
  public RearIntake() {
    System.out.println("Creating " + className + " object"); 
    configuration = buildMotorConfig();
    frc.robot.AlphaBots.Tools.SetConfigToTalonFX(m_Motor,configuration,className);

    NT_PGain.set(constants.RearMotorizedIntake.kP);
    NT_IGain.set(constants.RearMotorizedIntake.kI);
    NT_DGain.set(constants.RearMotorizedIntake.kD);

    NT_AGain.set(constants.RearMotorizedIntake.kA);
    NT_SGain.set(constants.RearMotorizedIntake.kS);
    NT_VGain.set(constants.RearMotorizedIntake.kV);

    SetCANrangeConfiguration();
  }

  private void SetCANrangeConfiguration() {
    CANrangeConfiguration CANrangeConfiguration = new CANrangeConfiguration();
    CANrangeConfiguration.ProximityParams.ProximityThreshold = DetectionThresholdMeters;

    m_CANRange.getConfigurator().apply(CANrangeConfiguration);
  }
  public Trigger CoralInRearIntake = new Trigger(()->{return LaserDetectsCoral();});
  public boolean LaserDetectsCoral(){
    return m_CANRange.getIsDetected().getValue();
  }
  public TalonFXConfiguration buildMotorConfig(){
    TalonFXConfiguration _configuration = new TalonFXConfiguration();
    _configuration.withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive));
    //configuration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    _configuration.Slot0.kP = NT_PGain.get();
    _configuration.Slot0.kI = NT_IGain.get();
    _configuration.Slot0.kD = NT_DGain.get();

    _configuration.Slot0.kA = NT_AGain.get();
    _configuration.Slot0.kS = NT_SGain.get();
    _configuration.Slot0.kV = NT_VGain.get();


    _configuration.CurrentLimits.StatorCurrentLimitEnable = true;
    _configuration.CurrentLimits.StatorCurrentLimit = constants.RearMotorizedIntake.maxStatorCurrent;

    _configuration.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
    _configuration.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
    
    return _configuration;
  }

  @Override
  public void periodic() {
    NT_IsLoaded.set(LaserDetectsCoral());
    NT_Rps.set(m_Motor.getVelocity().getValueAsDouble());
    NT_MotorTemp.set(m_Motor.getDeviceTemp().getValueAsDouble());
    NT_StatorCurrent.set(m_Motor.getStatorCurrent().getValueAsDouble());

    double p = NT_PGain.getAsDouble();
    double i = NT_IGain.getAsDouble();
    double d = NT_DGain.getAsDouble();

    double a = NT_AGain.getAsDouble();
    double s = NT_SGain.getAsDouble();
    double v = NT_VGain.getAsDouble();

          
    if((p != configuration.Slot0.kP)) { configuration.Slot0.kP = p; Tools.SetConfigToTalonFX(m_Motor,configuration,className); }
    if((i != configuration.Slot0.kI)) { configuration.Slot0.kI = i; Tools.SetConfigToTalonFX(m_Motor,configuration,className); }
    if((d != configuration.Slot0.kD)) { configuration.Slot0.kD = d; Tools.SetConfigToTalonFX(m_Motor,configuration,className); }
  
    if((a != configuration.Slot0.kA)) { configuration.Slot0.kA = a; Tools.SetConfigToTalonFX(m_Motor,configuration,className); }
    if((s != configuration.Slot0.kS)) { configuration.Slot0.kS = s; Tools.SetConfigToTalonFX(m_Motor,configuration,className); }
    if((v != configuration.Slot0.kV)) { configuration.Slot0.kV = v; Tools.SetConfigToTalonFX(m_Motor,configuration,className); }

  }


  public Command C_GotoPositon(double positon) {
      return new InstantCommand(()->{
        GotoVelocity(positon);
      });
  }
  public InstantCommand C_Stop() {
    return new InstantCommand(()->{
      BRAKE();
    });
  }

  public void HoldPosition(){ 
      GotoVelocity(0);
  }
  
 
  public void GotoVelocity(double wantedRPS){ 
    NT_BrakeEnabled.set(false);
    NT_SetpointVelocity.set(wantedRPS);
    m_Motor.setControl(
            new VelocityTorqueCurrentFOC(wantedRPS)//.withFeedForward(constants.RearMotorizedIntake.feedforwardsamps)
            .withSlot(0).withOverrideCoastDurNeutral(true)
        );
  }

  public void GotoDutyCycle(double wantedDutyCycle){ 
    NT_BrakeEnabled.set(false);
    NT_SetpointVelocity.set(wantedDutyCycle);
    m_Motor.setControl(
            new DutyCycleOut(wantedDutyCycle)
        );
  }

  public void COAST(){
    NT_BrakeEnabled.set(true);
    NT_SetpointVelocity.set(0);
    m_Motor.setControl(new CoastOut());
  }  
  public void BRAKE(){
    NT_BrakeEnabled.set(true);
    NT_SetpointVelocity.set(0);
    m_Motor.setControl(new StaticBrake());
  }  
}