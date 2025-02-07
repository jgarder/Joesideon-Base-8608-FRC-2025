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

public class Pivot extends SubsystemBase {
 
  //Get ClassName to help network tables auto sort by creating a sub Table with the same name.
  String className = this.getClass().getSimpleName();
  
  public final TalonFX m_PivotMotor = new TalonFX(constants.CanBus.armPivotMotorCanID);
  
  TalonFXConfiguration configuration;
  
  private double LastPosition = 0;
  public double gearRatio = constants.PlasmaPivot.gearRatio;

  DoubleEntry NT_Rpm =  NT.getDoubleEntry(className ,"RPM",0);
  DoubleEntry NT_MotorTemp =  NT.getDoubleEntry(className,"MotorTemp",0);
  DoubleEntry NT_position = NT.getDoubleEntry(className, "position",0);
  DoubleEntry NT_StatorCurrent = NT.getDoubleEntry(className, "StatorCurrent", 0);
  DoubleEntry NT_PGain = NT.getDoubleEntry(className , "P Gain",constants.PlasmaPivot.kP);
  DoubleEntry NT_IGain = NT.getDoubleEntry(className, "I Gain",constants.PlasmaPivot.kI);
  DoubleEntry NT_DGain = NT.getDoubleEntry(className , "D Gain",constants.PlasmaPivot.kD);
  DoubleEntry NT_SetpointPosition = NT.getDoubleEntry(className , "SetpointPosition",0.0);
  BooleanEntry NT_BrakeEnabled = NT.getBooleanEntry(className , "BrakeOn",false);

  DoubleSupplier elevatorposition;
  InterpolatingDoubleTreeMap heightMaxPivotMap;
  public Pivot(DoubleSupplier elevatorposition) {
    System.out.println("Creating " + className + " object"); 
    elevatorposition = elevatorposition;
    configuration = buildMotorConfig();
    frc.robot.AlphaBots.Tools.SetConfigToTalonFX(m_PivotMotor,configuration,className);

    InterpolatingDoubleTreeMap heightMaxPivotMap = new InterpolatingDoubleTreeMap();
    heightMaxPivotMap.put(0.0,10.0);
    heightMaxPivotMap.put(5.0, 14.0);
  }

  public TalonFXConfiguration buildMotorConfig(){
    TalonFXConfiguration _configuration = new TalonFXConfiguration();
    _configuration.withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));
    //configuration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    _configuration.Slot1.kP = NT_PGain.get();
    _configuration.Slot1.kI = NT_IGain.get();
    _configuration.Slot1.kD = NT_DGain.get();
    _configuration.Feedback.RotorToSensorRatio = constants.PlasmaPivot.gearRatio;

    _configuration.CurrentLimits.StatorCurrentLimitEnable = true;
    _configuration.CurrentLimits.StatorCurrentLimit = constants.PlasmaPivot.maxStatorCurrent;

    _configuration.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    _configuration.SoftwareLimitSwitch.ForwardSoftLimitThreshold = constants.PlasmaPivot.maxposition;

    _configuration.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    _configuration.SoftwareLimitSwitch.ReverseSoftLimitThreshold = constants.PlasmaPivot.minposition;
    
    return _configuration;
  }

  @Override
  public void periodic() {
    LastPosition = m_PivotMotor.getPosition().getValueAsDouble();
    // This method will be called once per scheduler run
    NT_Rpm.set(m_PivotMotor.getVelocity().getValueAsDouble() * 60);
    NT_MotorTemp.set(m_PivotMotor.getDeviceTemp().getValueAsDouble());
    NT_StatorCurrent.set(m_PivotMotor.getStatorCurrent().getValueAsDouble());
    NT_position.set(LastPosition);

    double p = NT_PGain.getAsDouble();
    double i = NT_IGain.getAsDouble();
    double d = NT_DGain.getAsDouble();
          
    if((p != configuration.Slot1.kP)) { configuration.Slot1.kP = p; Tools.SetConfigToTalonFX(m_PivotMotor,configuration,className); }
    if((i != configuration.Slot1.kI)) { configuration.Slot1.kI = i; Tools.SetConfigToTalonFX(m_PivotMotor,configuration,className); }
    if((d != configuration.Slot1.kD)) { configuration.Slot1.kD = d; Tools.SetConfigToTalonFX(m_PivotMotor,configuration,className); }
  }

  //is the elevator height low enough that we can fit under the stafe 1 cross bar when retracting (does not account for extension)
  public BooleanSupplier CanPivotFoldUp = ()->{return getPosition() < constants.PlasmaPivot.elevatorheightToFoldUp ? true:false;}; 
  public BooleanSupplier IsOutPastPastStage1 = ()->{return getPosition() > constants.PlasmaPivot.minPositionToBeSafeFromStage1Crossbar ? true:false;};
  
  //IsSafeToGoDown TODO: this needs a linear interpolation map because at 0 elevator we can only be 90. at mid height we can point down a bit. 
  //also extension will change this number but maybe just assume always extened (ie worst case scenario)

  public BooleanSupplier IsSafeToGoDown = ()->{return getPosition() < constants.PlasmaPivot.maxPositionToBeSafeFromSmashingintoSelf ? true:false;};  
  
  public Trigger IsPivotOutPastStage1 = new Trigger(IsOutPastPastStage1);

  public double getPosition()
  {
    return LastPosition;//m_PivotMotor.getPosition().getValueAsDouble(); // / gearRatio;
  }
  public Command C_GotoPositon(double positon) {
      return new InstantCommand(()->{
        GotoPosition(positon);
      });
  }
  public InstantCommand C_Stop() {
    return new InstantCommand(()->{
      BRAKE();
    });
  }

  public void HoldPosition(){ 
      LastPosition = m_PivotMotor.getPosition().getValueAsDouble();
      GotoPosition(getPosition()-(m_PivotMotor.getVelocity().getValueAsDouble()/constants.CanBus.canBusUpdateFrequency));
  }
  

  public void GotoPosition(double wantedposition){ 
    NT_BrakeEnabled.set(false);
    NT_SetpointPosition.set(wantedposition);
    m_PivotMotor.setControl(
        new PositionDutyCycle(wantedposition)
        .withEnableFOC(true)
        .withSlot(1)
    );
  }

  public void BRAKE(){
    NT_BrakeEnabled.set(true);
    m_PivotMotor.setControl(new StaticBrake());
  }  
}