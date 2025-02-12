package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.BooleanTopic;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants;
import frc.robot.AlphaBots.NT;
import frc.robot.AlphaBots.Tools;

public class josiahClimber extends SubsystemBase {
 
  //Get ClassName to help network tables auto sort by creating a sub Table with the same name.
  String className = this.getClass().getSimpleName();
  
  public final TalonFX m_CatchMotor = new TalonFX(constants.CanBus.josiahClimberCatchMotorCanID, constants.CanBus.kCANbusName);
  public final TalonFX m_SlideMotor = new TalonFX(constants.CanBus.josiahClimberSlideCanID, constants.CanBus.kCANbusName);
  
  TalonFXConfiguration catchConfig;
  TalonFXConfiguration slideConfig;
  private double catchPosition = 0;
  private double slidePosition = 0;

  DoubleEntry NT_CatchRpm =  NT.getDoubleEntry(className ,"C_RPM",0);
  DoubleEntry NT_CatchMotorTemp =  NT.getDoubleEntry(className,"C_MotorTemp",0);
  DoubleEntry NT_Catchposition = NT.getDoubleEntry(className, "C_position",0);
  DoubleEntry NT_CatchStatorCurrent = NT.getDoubleEntry(className, "C_StatorCurrent", 0);
  DoubleEntry NT_CatchPGain = NT.getDoubleEntry(className , "P Gain",0.0);
  DoubleEntry NT_CatchIGain = NT.getDoubleEntry(className, "I Gain",0.0);
  DoubleEntry NT_CatchDGain = NT.getDoubleEntry(className , "D Gain",0.0);
  DoubleEntry NT_CatchSetpointPosition = NT.getDoubleEntry(className , "C_SetpointPosition",0.0);
  BooleanEntry NT_CatchBrakeEnabled = NT.getBooleanEntry(className , "BrakeOn",false);

  DoubleEntry NT_SlideRpm =  NT.getDoubleEntry(className ,"S_RPM",0);
  DoubleEntry NT_SlideMotorTemp =  NT.getDoubleEntry(className,"S_MotorTemp",0);
  DoubleEntry NT_Slideposition = NT.getDoubleEntry(className, "S_position",0);
  DoubleEntry NT_SlideStatorCurrent = NT.getDoubleEntry(className, "S_StatorCurrent", 0);
  //DoubleEntry NT_SlidePGain = NT.getDoubleEntry(className , "P Gain",constants.PlasmaExtension.kP);
  //DoubleEntry NT_SlideIGain = NT.getDoubleEntry(className, "I Gain",constants.PlasmaExtension.kI);
  //DoubleEntry NT_SlideDGain = NT.getDoubleEntry(className , "D Gain",constants.PlasmaExtension.kD);
  DoubleEntry NT_SlideSetpointPosition = NT.getDoubleEntry(className , "S_SetpointPosition",0.0);
  //BooleanEntry NT_SlideBrakeEnabled = NT.getBooleanEntry(className , "S_BrakeOn",false);
  public josiahClimber() {
    System.out.println("Creating " + className + " object"); 
    catchConfig = buildCatchMotorConfig();
    slideConfig = buildSlideConfig();
    frc.robot.AlphaBots.Tools.SetConfigToTalonFX(m_CatchMotor,catchConfig,className);
    frc.robot.AlphaBots.Tools.SetConfigToTalonFX(m_SlideMotor,slideConfig,className);
    NT_CatchPGain.set(constants.Climber.kP);
    NT_CatchIGain.set(constants.Climber.kI);
    NT_CatchDGain.set(constants.Climber.kD);

  }

  public TalonFXConfiguration buildCatchMotorConfig(){
    TalonFXConfiguration _configuration = new TalonFXConfiguration();
    //_configuration.withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));
    _configuration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    _configuration.Slot1.kP = constants.Climber.kP;
    _configuration.Slot1.kI = constants.Climber.kI;
    _configuration.Slot1.kD = constants.Climber.kD;

    _configuration.CurrentLimits.StatorCurrentLimitEnable = true;
    _configuration.CurrentLimits.StatorCurrentLimit = constants.Climber.maxStatorCurrent;

    _configuration.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    _configuration.SoftwareLimitSwitch.ForwardSoftLimitThreshold = constants.Climber.CatchSide.maxPostion;

    _configuration.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    _configuration.SoftwareLimitSwitch.ReverseSoftLimitThreshold = constants.Climber.CatchSide.minPostion;
    
    return _configuration;
  }
  public TalonFXConfiguration buildSlideConfig(){
    TalonFXConfiguration _configuration = new TalonFXConfiguration();
    //_configuration.withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));
    _configuration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    _configuration.Slot1.kP = constants.Climber.kP;
    _configuration.Slot1.kI = constants.Climber.kI;
    _configuration.Slot1.kD = constants.Climber.kD;
    
    // _configuration.Feedback.withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor);
    // _configuration.Feedback.withRotorToSensorRatio(constants.Climber.SlideSide.gearRatio);
    // _configuration.Feedback.withSensorToMechanismRatio(constants.Climber.SlideSide.gearRatio);

    _configuration.CurrentLimits.StatorCurrentLimitEnable = true;
    _configuration.CurrentLimits.StatorCurrentLimit = constants.Climber.maxStatorCurrent;

    _configuration.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    _configuration.SoftwareLimitSwitch.ForwardSoftLimitThreshold = constants.Climber.SlideSide.maxPostion;

    _configuration.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    _configuration.SoftwareLimitSwitch.ReverseSoftLimitThreshold = constants.Climber.SlideSide.minPostion;
    
    return _configuration;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    catchPosition = m_CatchMotor.getPosition().getValueAsDouble();
    slidePosition = m_SlideMotor.getPosition().getValueAsDouble();

    NT_CatchRpm.set(m_CatchMotor.getVelocity().getValueAsDouble() * 60);
    NT_CatchMotorTemp.set(m_CatchMotor.getDeviceTemp().getValueAsDouble());
    NT_CatchStatorCurrent.set(m_CatchMotor.getStatorCurrent().getValueAsDouble());
    NT_Catchposition.set(catchPosition);

    NT_SlideRpm.set(m_SlideMotor.getVelocity().getValueAsDouble() * 60);
    NT_SlideMotorTemp.set(m_SlideMotor.getDeviceTemp().getValueAsDouble());
    NT_SlideStatorCurrent.set(m_SlideMotor.getStatorCurrent().getValueAsDouble());
    NT_Slideposition.set(slidePosition);

    double p = NT_CatchPGain.getAsDouble();
    double i = NT_CatchIGain.getAsDouble();
    double d = NT_CatchDGain.getAsDouble();
    
    SmartDashboard.putNumber(className + "catch", m_CatchMotor.getPosition().getValueAsDouble());
    SmartDashboard.putNumber(className + "slide", m_SlideMotor.getPosition().getValueAsDouble());
        
    if((p != catchConfig.Slot1.kP)) { 
      catchConfig.Slot1.kP = p; Tools.SetConfigToTalonFX(m_CatchMotor,catchConfig,className);
      slideConfig.Slot1.kP = p; Tools.SetConfigToTalonFX(m_SlideMotor,slideConfig,className); }
    if((i != catchConfig.Slot1.kI)) { 
      catchConfig.Slot1.kI = i; Tools.SetConfigToTalonFX(m_CatchMotor,catchConfig,className); 
      slideConfig.Slot1.kI = i; Tools.SetConfigToTalonFX(m_SlideMotor,slideConfig,className); }
    if((d != catchConfig.Slot1.kD)) { 
      catchConfig.Slot1.kD = d; Tools.SetConfigToTalonFX(m_CatchMotor,catchConfig,className); 
      slideConfig.Slot1.kD = d; Tools.SetConfigToTalonFX(m_SlideMotor,slideConfig,className); }
  }

  //this tells us if our extension is retracted enough to allow a fold up into the elevator
  public BooleanSupplier IsRetractedToGetPastStage1 = ()->{return catchPosition < constants.PlasmaExtension.maxPositionToBeSafeFromStage1Crossbar ? true:false;};
  
  //IsSafeToGoDown TODO: this needs a linear interpolation map because at 0 elevator we can only be 90. at mid height we can point down a bit. 
  //also extension will change this number but maybe just assume always extened (ie worst case scenario)

  public double getCatchPosition(){return catchPosition;}
  public double getSlidePosition(){return slidePosition;}
  
  public Trigger IsPivotOutPastStage1 = new Trigger(IsRetractedToGetPastStage1);

  public Command C_CatchGotoPositon(double positon) {
      return new InstantCommand(()->{
        CatchGotoPosition(positon);
      });
  }
  public Command C_SlideGotoPositon(double positon) {
    return new InstantCommand(()->{
      SlideGotoPosition(positon);
    });
}
  public InstantCommand C_Stop() {
    return new InstantCommand(()->{
      BRAKE();
    });
  }

  public void CatchHoldPosition(){ 
      CatchGotoPosition(catchPosition-(m_CatchMotor.getVelocity().getValueAsDouble()/constants.CanBus.canBusUpdateFrequency));
  }
  

  public void CatchGotoPosition(double wantedposition){ 
    NT_CatchBrakeEnabled.set(false);
    NT_CatchSetpointPosition.set(wantedposition);
    m_CatchMotor.setControl(
        new PositionDutyCycle(wantedposition)
        .withEnableFOC(true)
        .withSlot(1)
    );
  }
  public void SlideGotoPosition(double wantedposition){ 
    //NT_CatchBrakeEnabled.set(false);
    //NT_CatchSetpointPosition.set(wantedposition);
    m_SlideMotor.setControl(
        new PositionDutyCycle(wantedposition)
        .withEnableFOC(true)
        .withSlot(1)
    );
  }

  public void BRAKE(){
    NT_CatchBrakeEnabled.set(true);
    m_CatchMotor.setControl(new StaticBrake());
    m_SlideMotor.setControl(new StaticBrake());
  }  
}