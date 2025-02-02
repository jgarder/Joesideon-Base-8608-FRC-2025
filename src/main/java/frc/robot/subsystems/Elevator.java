package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants;
import frc.robot.AlphaBots.NT;

public class Elevator extends SubsystemBase {
 
    //This will make smartdashboardPuts goto the classes subfolder in the network tables. the / does the subfoldering.
  String className = this.getClass().getSimpleName()+"/";
  
  public final TalonFX m_ElevatorMotor1 = new TalonFX(constants.CanBus.elevatorMotor1CanID);
  public final TalonFX m_ElevatorMotor2 = new TalonFX(constants.CanBus.elevatorMotor2CanID);


  private final com.ctre.phoenix6.controls.PositionDutyCycle m_positionDC = new PositionDutyCycle(0);
  private final com.ctre.phoenix6.controls.DutyCycleOut m_DutyCycle = new DutyCycleOut(constants.MantaRay.IntakeDutyCycle);

  private final StaticBrake m_s_Brake = new StaticBrake();
  public double LastPosition = 0;
  public double kP = 0.013;
  public double kI = 0.0;
  public double kD = 0.0;

  TalonFXConfiguration configuration;
  public DoubleSupplier currentHeight = ()->{return LastPosition;};

  DoubleTopic RpmTopic = NT.table.getDoubleTopic(className + " rpm");
  DoublePublisher RpmPub =  RpmTopic.publish();
  DoublePublisher MotorTemp =  NT.table.getDoubleTopic(className + "MotorTemp").publish();
  DoublePublisher Motor2Temp =  NT.table.getDoubleTopic(className + "Motor2Temp").publish();
  DoublePublisher CurrentPosition = NT.table.getDoubleTopic(className + "CurrentPosition").publish();

  public Elevator() {
    System.out.println("Creating " + className + " object"); 
    setMotorConfig();
    m_ElevatorMotor2.setControl(
      new StrictFollower(constants.CanBus.elevatorMotor1CanID)
  );
    SmartDashboard.putNumber(className +" P Gain", kP);
    SmartDashboard.putNumber(className +" I Gain", kI);
    SmartDashboard.putNumber(className +" D Gain", kD);
  }

  



  public void setMotorConfig(){
    configuration = new TalonFXConfiguration();
    configuration.withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));
    //configuration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    configuration.Slot1.kP = kP;
    configuration.Slot1.kI = kI;
    configuration.Slot1.kD = kD;

    configuration.CurrentLimits.StatorCurrentLimitEnable = true;
    configuration.CurrentLimits.StatorCurrentLimit = constants.Elevator.maxStatorCurrent;

    configuration.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    configuration.SoftwareLimitSwitch.ForwardSoftLimitThreshold = constants.Elevator.maxElevatorheight;

    configuration.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    configuration.SoftwareLimitSwitch.ReverseSoftLimitThreshold = constants.Elevator.minElevatorHeight;
    
    
    frc.robot.AlphaBots.Tools.SetConfigToTalonFX(m_ElevatorMotor1,configuration,className);
    frc.robot.AlphaBots.Tools.SetConfigToTalonFX(m_ElevatorMotor2,configuration,className);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    RpmPub.set(m_ElevatorMotor1.getVelocity().getValueAsDouble() * 60);
    MotorTemp.set(m_ElevatorMotor1.getDeviceTemp().getValueAsDouble());
    Motor2Temp.set(m_ElevatorMotor2.getDeviceTemp().getValueAsDouble());
    
    //SmartDashboard.putNumber(className + " rpm", (m_TridentMotor.getVelocity().getValueAsDouble() * 60));
    SmartDashboard.putNumber(className + " MotorTemp", m_ElevatorMotor1.getDeviceTemp().getValueAsDouble());
    SmartDashboard.putNumber(className + " StatorCurrent", m_ElevatorMotor1.getStatorCurrent().getValueAsDouble());

    SmartDashboard.putNumber(className + " Motor2Temp", m_ElevatorMotor2.getDeviceTemp().getValueAsDouble());
    SmartDashboard.putNumber(className + " Stator2Current", m_ElevatorMotor2.getStatorCurrent().getValueAsDouble());

    double p = SmartDashboard.getNumber(className +" P Gain", kP);
    double i = SmartDashboard.getNumber(className +" I Gain", kI);
    double d = SmartDashboard.getNumber(className +" D Gain", kD);
          
    if((p != kP)) { configuration.Slot1.kP = p; kP = p; frc.robot.AlphaBots.Tools.SetConfigToTalonFX(m_ElevatorMotor1,configuration,className); }
    if((i != kI)) { configuration.Slot1.kI = i; kI = i; frc.robot.AlphaBots.Tools.SetConfigToTalonFX(m_ElevatorMotor1,configuration,className); }
    if((d != kD)) { configuration.Slot1.kD = d; kD = d; frc.robot.AlphaBots.Tools.SetConfigToTalonFX(m_ElevatorMotor1,configuration,className); }

    double currentRotorposition = m_ElevatorMotor1.getPosition().getValueAsDouble();
    SmartDashboard.putNumber(className + "CurrentPosition", currentRotorposition);
    CurrentPosition.set(currentRotorposition);//what is the performance hit of getting this publisher over and over?
    // if (LastPosition != currentRotorposition) {
    //     LastPosition = currentRotorposition;
    // }
    //

  }


    public Command GotoPositonCommand(double positon) {
        double rpmgoal = positon;
        return new InstantCommand(()->{
          GotoPosition(rpmgoal);
        });

    }

    public double canBusUpdateFrequency = 50;

    public InstantCommand Stop() {
      return new InstantCommand(()->{
        BRAKE();
      });
    }
    public void HoldPosition(){ 
        double currentRotorposition = m_ElevatorMotor1.getPosition().getValueAsDouble();
        LastPosition = currentRotorposition;
        GotoPosition(currentRotorposition-(m_ElevatorMotor1.getVelocity().getValueAsDouble()/canBusUpdateFrequency));
    }
    public void GotoPosition(double wantedposition){ 
        SmartDashboard.putNumber(className + "SetpointPosition", LastPosition);
        m_ElevatorMotor1.setControl(
            new PositionDutyCycle(wantedposition)
            .withEnableFOC(true)
            .withSlot(1)
        );
    }

    public void BRAKE(){
      m_ElevatorMotor1.setControl(m_s_Brake);
    }
    
}