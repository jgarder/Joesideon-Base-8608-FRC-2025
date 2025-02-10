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

import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.MantaState;
import frc.robot.constants;
import frc.robot.AlphaBots.NT;

public class Elevator extends SubsystemBase {
 
  //This will make smartdashboardPuts goto the classes subfolder in the network tables. the / does the subfoldering.
  private final String className = this.getClass().getSimpleName()+"/";
  
  public final TalonFX m_ElevatorMotor1 = new TalonFX(constants.CanBus.elevatorMotor1CanID);
  public final TalonFX m_ElevatorMotor2 = new TalonFX(constants.CanBus.elevatorMotor2CanID);


  private final com.ctre.phoenix6.controls.PositionDutyCycle m_positionDC = new PositionDutyCycle(0);
  private final com.ctre.phoenix6.controls.DutyCycleOut m_DutyCycle = new DutyCycleOut(constants.MantaRay.IntakeDutyCycle);

  private final StaticBrake m_s_Brake = new StaticBrake();
  


  private double currentPosition = 0;
  private double requestedPosition = 0;
  private double setPointPosition = 0;


  TalonFXConfiguration configuration;
  public DoubleSupplier currentHeight = ()->{return currentPosition;};

  DoubleTopic RpmTopic = NT.table.getDoubleTopic(className + " rpm");
  DoublePublisher RpmPub =  RpmTopic.publish();
  DoublePublisher MotorTemp =  NT.table.getDoubleTopic(className + "MotorTemp").publish();
  DoublePublisher Motor2Temp =  NT.table.getDoubleTopic(className + "Motor2Temp").publish();
  DoublePublisher NT_CurrentPosition = NT.table.getDoubleTopic(className + "CurrentPosition").publish();
  DoubleEntry NT_SetpointPosition = NT.getDoubleEntry(className , "SetpointPosition",0.0);
  DoubleEntry NT_RequestedPosition = NT.getDoubleEntry(className , "RequestedPosition",0.0);
  DoubleEntry NT_PGain = NT.getDoubleEntry(className , "P Gain",0);
  DoubleEntry NT_IGain = NT.getDoubleEntry(className, "I Gain",0);
  DoubleEntry NT_DGain = NT.getDoubleEntry(className , "D Gain",0);

  public Elevator() {
    System.out.println("Creating " + className + " object"); 
    setMotorConfig();
    m_ElevatorMotor2.setControl(
      new StrictFollower(constants.CanBus.elevatorMotor1CanID)
    );
    NT_PGain.set(constants.Elevator.kP);
    NT_IGain.set(constants.Elevator.kI);
    NT_DGain.set(constants.Elevator.kD);
  }



  public void setMotorConfig(){
    configuration = new TalonFXConfiguration();
    configuration.withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));
    //configuration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    configuration.Slot1.kP = constants.Elevator.kP;
    configuration.Slot1.kI = constants.Elevator.kI;
    configuration.Slot1.kD = constants.Elevator.kD;

    configuration.CurrentLimits.StatorCurrentLimitEnable = true;
    configuration.CurrentLimits.StatorCurrentLimit = constants.Elevator.maxStatorCurrent;

    configuration.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    configuration.SoftwareLimitSwitch.ForwardSoftLimitThreshold = constants.Elevator.maxElevatorheight;

    configuration.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    configuration.SoftwareLimitSwitch.ReverseSoftLimitThreshold = constants.Elevator.minElevatorHeight;
    
    
    frc.robot.AlphaBots.Tools.SetConfigToTalonFX(m_ElevatorMotor1,configuration,className);
    frc.robot.AlphaBots.Tools.SetConfigToTalonFX(m_ElevatorMotor2,configuration,className);
  }


  @Override // This method will be called once per scheduler run
  public void periodic() {
    currentPosition = m_ElevatorMotor1.getPosition().getValueAsDouble();
    NT_CurrentPosition.set(currentPosition);

    RpmPub.set(m_ElevatorMotor1.getVelocity().getValueAsDouble() * 60);
    MotorTemp.set(m_ElevatorMotor1.getDeviceTemp().getValueAsDouble());
    Motor2Temp.set(m_ElevatorMotor2.getDeviceTemp().getValueAsDouble());
    
    //SmartDashboard.putNumber(className + " rpm", (m_TridentMotor.getVelocity().getValueAsDouble() * 60));
    SmartDashboard.putNumber(className + " MotorTemp", m_ElevatorMotor1.getDeviceTemp().getValueAsDouble());
    SmartDashboard.putNumber(className + " StatorCurrent", m_ElevatorMotor1.getStatorCurrent().getValueAsDouble());

    SmartDashboard.putNumber(className + " Motor2Temp", m_ElevatorMotor2.getDeviceTemp().getValueAsDouble());
    SmartDashboard.putNumber(className + " Stator2Current", m_ElevatorMotor2.getStatorCurrent().getValueAsDouble());

    double p = NT_PGain.getAsDouble();
    double i = NT_IGain.getAsDouble();
    double d = NT_DGain.getAsDouble();
          
    if((p != configuration.Slot1.kP)) { configuration.Slot1.kP = p;  frc.robot.AlphaBots.Tools.SetConfigToTalonFX(m_ElevatorMotor1,configuration,className); }
    if((i != configuration.Slot1.kI)) { configuration.Slot1.kI = i;  frc.robot.AlphaBots.Tools.SetConfigToTalonFX(m_ElevatorMotor1,configuration,className); }
    if((d != configuration.Slot1.kD)) { configuration.Slot1.kD = d;  frc.robot.AlphaBots.Tools.SetConfigToTalonFX(m_ElevatorMotor1,configuration,className); }

    if (setPointPosition != requestedPosition) {
      
      //if we are above the CannotFoldBelow position
      if(currentPosition > constants.Elevator.CannotPivotParkBelowElevatorPosition)
      {
        //if we are going below the cannot fold position
        if(requestedPosition < constants.Elevator.CannotPivotParkBelowElevatorPosition)
        {
          //check if pivot is in a safe travel position
          if(MantaState.ss_Pivot.IsPivotinTravelPosition.getAsBoolean())
          {
             //if/when we are folded out, set position to requested position
            //safe to goto requestion position
            GotoPosition(requestedPosition);
          }
          else{
            //if not IsPivotinTravelPosition, set position to "cannotfoldbelowPosition"
            //ONLY safe to goto CannotFoldBelowPosition
            GotoPosition(constants.Elevator.CannotPivotParkBelowElevatorPosition);
          }
        }
      }
      //////////////////
        //if we are below the CannotFoldabove position
        if(currentPosition < constants.Elevator.CannotPivotParkAboveElevatorPosition)
        {
          //if we are going above the cannot fold position
          if(requestedPosition > constants.Elevator.CannotPivotParkAboveElevatorPosition)
          {
            //check if pivot is in a safe travel position
            if(MantaState.ss_Pivot.IsPivotinTravelPosition.getAsBoolean())
            {
                //if/when we are folded out, set position to requested position
              //safe to goto requestion position
              GotoPosition(requestedPosition);
            }
            else{
              //if not IsPivotinTravelPosition, set position to "cannotfoldbelowPosition"
              //ONLY safe to goto CannotFoldBelowPosition
              GotoPosition(constants.Elevator.CannotPivotParkAboveElevatorPosition);
            }
          }
        }
    }
  }

  public double canBusUpdateFrequency = 50;
  public double getPosition()
  {
    return currentPosition;//m_ElevatorMotor1.getPosition().getValueAsDouble(); // / gearRatio;
  }

    public Command GotoPositonCommand(double positon) {
        double rpmgoal = positon;
        return new InstantCommand(()->{
          RequestPosition(rpmgoal);
        });
    }
    public InstantCommand Stop() {
      return new InstantCommand(()->{
        BRAKE();
      });
    }

    public void HoldPosition(){ 
        GotoPosition(currentPosition-(m_ElevatorMotor1.getVelocity().getValueAsDouble()/canBusUpdateFrequency));
    }
    
    public void RequestPosition(double wantedposition)
    {
      NT_RequestedPosition.set(wantedposition);
      requestedPosition = wantedposition;
    }
    private void GotoPosition(double wantedposition){
        setPointPosition = wantedposition;
        NT_SetpointPosition.set(wantedposition);
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