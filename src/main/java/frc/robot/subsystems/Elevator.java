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
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
  
  public final TalonFX m_ElevatorMotor1 = new TalonFX(constants.CanBus.CanBusIDs.elevatorMotor1CanID.id, constants.CanBus.CanivoreCANBusName);
  public final TalonFX m_ElevatorMotor2 = new TalonFX(constants.CanBus.CanBusIDs.elevatorMotor2CanID.id, constants.CanBus.CanivoreCANBusName);


  private final com.ctre.phoenix6.controls.PositionDutyCycle m_positionDC = new PositionDutyCycle(0);
  private final com.ctre.phoenix6.controls.DutyCycleOut m_DutyCycle = new DutyCycleOut(constants.MantaRay.IntakeDutyCycle);

  private final StaticBrake m_s_Brake = new StaticBrake();
  
  public final BooleanSupplier elevatorisparked = ()->{return currentState.equals(Elevator.POSITION.parked);};

  private double currentPosition = 0;
  private double requestedPosition = 0;
  private double setPointPosition = 0;


  TalonFXConfiguration configuration;
  public DoubleSupplier setpointHeight = ()->{return setPointPosition;};
  public DoubleSupplier currentHeight = ()->{return currentPosition;};
  public DoubleSupplier requestedHeight = ()->{return requestedPosition;};
  public DoubleSupplier elevatorVelocity = ()->{return m_ElevatorMotor1.getVelocity().getValueAsDouble();};

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

  DoubleEntry NT_SGain = NT.getDoubleEntry(className , "S Gain",0);
  DoubleEntry NT_GGain = NT.getDoubleEntry(className , "G Gain",0);
  DoubleEntry NT_VGain = NT.getDoubleEntry(className , "V Gain",0);
  DoubleEntry NT_AGain = NT.getDoubleEntry(className , "A Gain",0);

  DoubleEntry NT_Acceleration = NT.getDoubleEntry(className , "Acceleration",0);
  DoubleEntry NT_Jerk = NT.getDoubleEntry(className , "Jerk",0);
  DoubleEntry NT_Cruise = NT.getDoubleEntry(className , "Cruise",0);

  public Elevator() {
    System.out.println("Creating " + className + " object"); 
    m_ElevatorMotor1.setPosition(0);
    setMotorConfig();
    
    m_ElevatorMotor2.setControl(
      new StrictFollower(constants.CanBus.CanBusIDs.elevatorMotor1CanID.id)
    );

    NT_PGain.set(constants.Elevator.kP);
    NT_IGain.set(constants.Elevator.kI);
    NT_DGain.set(constants.Elevator.kD);

    NT_VGain.set(constants.Elevator.kV);
    NT_AGain.set(constants.Elevator.kA);

    NT_SGain.set(constants.Elevator.kS);
    NT_GGain.set(constants.Elevator.kG);

    NT_Acceleration.set(constants.Elevator.Accel);
    NT_Jerk.set(constants.Elevator.Jerk);
    NT_Cruise.set(constants.Elevator.Cruise);

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

    configuration.Slot1.kV = constants.Elevator.kV;
    configuration.Slot1.kA = constants.Elevator.kA;
 

    configuration.Slot1.kG = constants.Elevator.kG;
    configuration.Slot1.GravityType = GravityTypeValue.Elevator_Static;

    configuration.Slot1.kS = constants.Elevator.kS;
    configuration.Slot1.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;

    configuration.CurrentLimits.StatorCurrentLimitEnable = true;
    configuration.CurrentLimits.StatorCurrentLimit = constants.Elevator.maxStatorCurrent;

    configuration.CurrentLimits.SupplyCurrentLimitEnable = false;
    configuration.CurrentLimits.SupplyCurrentLimit = constants.Elevator.maxStatorCurrent;

    configuration.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    configuration.SoftwareLimitSwitch.ForwardSoftLimitThreshold = constants.Elevator.maxElevatorheight;

    configuration.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    configuration.SoftwareLimitSwitch.ReverseSoftLimitThreshold = constants.Elevator.minElevatorHeight;
    
    configuration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    
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

    //feedback
    double p = NT_PGain.getAsDouble();
    double i = NT_IGain.getAsDouble();
    double d = NT_DGain.getAsDouble();

    //feedforward
    double a = NT_VGain.getAsDouble();    
    double v = NT_VGain.getAsDouble();
    double s = NT_SGain.getAsDouble();
    double g = NT_GGain.getAsDouble();

    double mA = NT_Acceleration.getAsDouble();
    double mJ = NT_Jerk.getAsDouble();
    double mC = NT_Cruise.getAsDouble();
    boolean motorNeedsConfig = false;

    if((p != configuration.Slot1.kP)) { configuration.Slot1.kP = p; motorNeedsConfig = true; }
    if((i != configuration.Slot1.kI)) { configuration.Slot1.kI = i; motorNeedsConfig = true; }
    if((d != configuration.Slot1.kD)) { configuration.Slot1.kD = d; motorNeedsConfig = true; }
  
    if((v != configuration.Slot1.kV)) { configuration.Slot1.kV = v; motorNeedsConfig = true; }
    if((s != configuration.Slot1.kS)) { configuration.Slot1.kS = s; motorNeedsConfig = true; }
    if((g != configuration.Slot1.kG)) { configuration.Slot1.kG = g; motorNeedsConfig = true; }
    if((a != configuration.Slot1.kA)) { configuration.Slot1.kA = a; motorNeedsConfig = true; }

    if((mA != configuration.MotionMagic.MotionMagicAcceleration)) { configuration.MotionMagic.MotionMagicAcceleration = mA; motorNeedsConfig = true; }
    if((mJ != configuration.MotionMagic.MotionMagicJerk)) { configuration.MotionMagic.MotionMagicJerk = mJ; motorNeedsConfig = true; }
    if((mC != configuration.MotionMagic.MotionMagicCruiseVelocity)) { configuration.MotionMagic.MotionMagicCruiseVelocity = mC; motorNeedsConfig = true; }
    
    if (motorNeedsConfig){Tools.SetConfigToTalonFX(m_ElevatorMotor1,configuration,className);}

    
  }
  
  
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
        GotoPosition(currentPosition-(m_ElevatorMotor1.getVelocity().getValueAsDouble()/constants.CanBus.canBusUpdateFrequency));
    }
    
    public void RequestPosition(double wantedposition)
    {
      requestedPosition = wantedposition;
    }
    public void GotoPosition(double wantedposition){
        setPointPosition = wantedposition;
        currentState = POSITION.up;
        m_ElevatorMotor1.setControl(
          new MotionMagicTorqueCurrentFOC(wantedposition)
          .withSlot(1)
          .withOverrideCoastDurNeutral(false)
            // new PositionDutyCycle(wantedposition)
            // .withOverrideBrakeDurNeutral(true)
            // .withEnableFOC(true)
            // .withSlot(1)
        );
      
    }

    public void BRAKE(){
      currentState = POSITION.parked;
      m_ElevatorMotor1.setControl(m_s_Brake);
    }
    
}