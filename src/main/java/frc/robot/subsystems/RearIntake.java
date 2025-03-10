package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
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
  
  public final TalonFX m_Motor = new TalonFX(constants.CanBus.MotorizedRearIntakeCanID, constants.CanBus.RioCANBusName);
  
  TalonFXConfiguration configuration;
  

  DoubleEntry NT_Rps =  NT.getDoubleEntry(className ,"RPS",0);
  DoubleEntry NT_MotorTemp =  NT.getDoubleEntry(className,"MotorTemp",0);
  DoubleEntry NT_StatorCurrent = NT.getDoubleEntry(className, "StatorCurrent", 0);
  
  DoubleEntry NT_PGain = NT.getDoubleEntry(className , "P Gain",0);
  DoubleEntry NT_IGain = NT.getDoubleEntry(className, "I Gain",0);
  DoubleEntry NT_DGain = NT.getDoubleEntry(className , "D Gain",0);

  DoubleEntry NT_SetpointVelocity = NT.getDoubleEntry(className , "SetpointRPS",0.0);
  BooleanEntry NT_BrakeEnabled = NT.getBooleanEntry(className , "BrakeOn",false);

  
  public RearIntake() {
    System.out.println("Creating " + className + " object"); 
    configuration = buildMotorConfig();
    frc.robot.AlphaBots.Tools.SetConfigToTalonFX(m_Motor,configuration,className);

    NT_PGain.set(constants.RearMotorizedIntake.kP);
    NT_IGain.set(constants.RearMotorizedIntake.kI);
    NT_DGain.set(constants.RearMotorizedIntake.kD);

  }

  public TalonFXConfiguration buildMotorConfig(){
    TalonFXConfiguration _configuration = new TalonFXConfiguration();
    _configuration.withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive));
    //configuration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    _configuration.Slot0.kP = NT_PGain.get();
    _configuration.Slot0.kI = NT_IGain.get();
    _configuration.Slot0.kD = NT_DGain.get();

    _configuration.CurrentLimits.StatorCurrentLimitEnable = true;
    _configuration.CurrentLimits.StatorCurrentLimit = constants.RearMotorizedIntake.maxStatorCurrent;

    _configuration.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
    _configuration.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
    
    return _configuration;
  }

  @Override
  public void periodic() {
    
    NT_Rps.set(m_Motor.getVelocity().getValueAsDouble());
    NT_MotorTemp.set(m_Motor.getDeviceTemp().getValueAsDouble());
    NT_StatorCurrent.set(m_Motor.getStatorCurrent().getValueAsDouble());

    double p = NT_PGain.getAsDouble();
    double i = NT_IGain.getAsDouble();
    double d = NT_DGain.getAsDouble();
          
    if((p != configuration.Slot0.kP)) { configuration.Slot0.kP = p; Tools.SetConfigToTalonFX(m_Motor,configuration,className); }
    if((i != configuration.Slot0.kI)) { configuration.Slot0.kI = i; Tools.SetConfigToTalonFX(m_Motor,configuration,className); }
    if((d != configuration.Slot0.kD)) { configuration.Slot0.kD = d; Tools.SetConfigToTalonFX(m_Motor,configuration,className); }
  
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
            new VelocityTorqueCurrentFOC(wantedRPS).withFeedForward(constants.RearMotorizedIntake.feedforwardsamps)
            .withSlot(0).withOverrideCoastDurNeutral(true)
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