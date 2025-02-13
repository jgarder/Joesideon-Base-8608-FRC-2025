package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
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
import frc.robot.AlphaBots.Tools;
import frc.robot.commands.C_ElevateToPosition;

public class Elevator extends SubsystemBase {
 
  public static enum POSITION
  {
    parked,
    up,
  }
  public POSITION currentState = POSITION.parked;
  //This will make smartdashboardPuts goto the classes subfolder in the network tables. the / does the subfoldering.
  private final String className = this.getClass().getSimpleName(); //+"/"
  
  public final TalonFX m_ElevatorMotor1 = new TalonFX(constants.CanBus.elevatorMotor1CanID, constants.CanBus.CanivoreCANBusName);
  public final TalonFX m_ElevatorMotor2 = new TalonFX(constants.CanBus.elevatorMotor2CanID, constants.CanBus.CanivoreCANBusName);


  private final com.ctre.phoenix6.controls.PositionDutyCycle m_positionDC = new PositionDutyCycle(0);
  private final com.ctre.phoenix6.controls.DutyCycleOut m_DutyCycle = new DutyCycleOut(constants.MantaRay.IntakeDutyCycle);

  private final StaticBrake m_s_Brake = new StaticBrake();
  


  private double currentPosition = 0;
  private double requestedPosition = 0;
  private double setPointPosition = 0;


  TalonFXConfiguration configuration;
  public DoubleSupplier currentHeight = ()->{return currentPosition;};

  DoubleEntry NT_Rpm = NT.getDoubleEntry(className , " rpm",0.0);

  DoubleEntry NT_MotorTemp =  NT.getDoubleEntry(className , "MotorTemp",0.0);
  DoubleEntry NT_Motor2Temp =  NT.getDoubleEntry(className ,"Motor2Temp",0.0);

  DoubleEntry NT_StatorCurrent =  NT.getDoubleEntry(className , "StatorCurrent",0.0);
  DoubleEntry NT_StatorCurrent2 =  NT.getDoubleEntry(className ,"StatorCurrent2",0.0);

  DoubleEntry NT_CurrentPosition = NT.getDoubleEntry(className , "CurrentPosition",0.0);
  DoubleEntry NT_SetpointPosition = NT.getDoubleEntry(className , "SetpointPosition",0.0);
  DoubleEntry NT_RequestedPosition = NT.getDoubleEntry(className , "RequestedPosition",0.0);

  DoubleEntry NT_PGain = NT.getDoubleEntry(className , "P Gain",0);
  DoubleEntry NT_IGain = NT.getDoubleEntry(className, "I Gain",0);
  DoubleEntry NT_DGain = NT.getDoubleEntry(className , "D Gain",0);

  public Elevator() {
    System.out.println("Creating " + className + " object"); 
    m_ElevatorMotor1.setPosition(0,1);
    setMotorConfig();
    
    m_ElevatorMotor2.setControl(
      new StrictFollower(constants.CanBus.elevatorMotor1CanID)
    );

    NT_PGain.set(constants.Elevator.kP);
    NT_IGain.set(constants.Elevator.kI);
    NT_DGain.set(constants.Elevator.kD);

    NT_RequestedPosition.set(requestedPosition);
    NT_SetpointPosition.set(setPointPosition);
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
    NT_SetpointPosition.set(setPointPosition);
    NT_RequestedPosition.set(requestedPosition);

    NT_Rpm.set(m_ElevatorMotor1.getVelocity().getValueAsDouble() * 60);
    NT_MotorTemp.set(m_ElevatorMotor1.getDeviceTemp().getValueAsDouble());
    NT_Motor2Temp.set(m_ElevatorMotor2.getDeviceTemp().getValueAsDouble());
    NT_StatorCurrent.set(m_ElevatorMotor1.getStatorCurrent().getValueAsDouble());
    NT_StatorCurrent2.set(m_ElevatorMotor2.getStatorCurrent().getValueAsDouble());

    //SmartDashboard.putNumber(className + " rpm", (m_TridentMotor.getVelocity().getValueAsDouble() * 60));
    // SmartDashboard.putNumber(className + " MotorTemp", m_ElevatorMotor1.getDeviceTemp().getValueAsDouble());
    // SmartDashboard.putNumber(className + " StatorCurrent", m_ElevatorMotor1.getStatorCurrent().getValueAsDouble());

    // SmartDashboard.putNumber(className + " Motor2Temp", m_ElevatorMotor2.getDeviceTemp().getValueAsDouble());
    // SmartDashboard.putNumber(className + " Stator2Current", m_ElevatorMotor2.getStatorCurrent().getValueAsDouble());

    // SmartDashboard.putNumber(className + "Elevator position", m_ElevatorMotor1.getPosition().getValueAsDouble());

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
        if(requestedPosition <= constants.Elevator.CannotPivotParkBelowElevatorPosition)
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
        else{
          //if we are above the safe zone and staying above the safe zone then request the new position. 
          GotoPosition(requestedPosition);
        }
      }   //if we are below the CannotFoldabove position
      else if(currentPosition < constants.Elevator.CannotPivotParkAboveElevatorPosition)
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
        else{
          //if we are below the safe zone and going below the safe zone then request the new position. 
          GotoPosition(requestedPosition);
        }
      }//if we are not above the nogo and we are not below the nogo we are in the nogo. make sure we are in travel position and goto the called position
      else 
      {
        //check if pivot is in a safe travel position
        if(MantaState.ss_Pivot.IsPivotinTravelPosition.getAsBoolean())
        {
            //if/when we are folded out, set position to requested position
          //safe to goto requestion position
          GotoPosition(requestedPosition);
        }
        else{
          //if not IsPivotinTravelPosition, dont move we are in the No-go zone already. 
        }
      }
    }// else if we are close to parked and we are requesting a park. then just brake mode. 
    else if ((setPointPosition < ElevatorBrakeParkTolerance) & (requestedPosition < ElevatorBrakeParkTolerance) & Tools.isPosAtSetpoint(currentPosition, constants.Elevator.minElevatorHeight, ElevatorBrakeParkTolerance))
    {
      //System.out.println("elevator Braking");
      currentState = POSITION.parked;
      BRAKE();
    }
  }
  public double ElevatorBrakeParkTolerance = 0.5;
  public double canBusUpdateFrequency = 50;
  public double getPosition()
  {
    return currentPosition;//m_ElevatorMotor1.getPosition().getValueAsDouble(); // / gearRatio;
  }

    public Command GotoPositonCommand(double positon) {
      return new C_ElevateToPosition(this, positon);
        // return new InstantCommand(()->{
        //   RequestPosition(positon);
        // });
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
      requestedPosition = wantedposition;
    }
    private void GotoPosition(double wantedposition){
        setPointPosition = wantedposition;
        currentState = POSITION.up;
        m_ElevatorMotor1.setControl(
          // new MotionMagicTorqueCurrentFOC(wantedposition)
          // .withFeedForward(0)
          // .withSlot(1)
            new PositionDutyCycle(wantedposition)
            .withOverrideBrakeDurNeutral(true)
            .withEnableFOC(true)
            .withSlot(1)
        );
      
    }

    public void BRAKE(){
      m_ElevatorMotor1.setControl(m_s_Brake);
    }
    
}