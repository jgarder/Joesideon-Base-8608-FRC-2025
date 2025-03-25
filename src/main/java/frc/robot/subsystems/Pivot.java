package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

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
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants;
import frc.robot.AlphaBots.NT;
import frc.robot.AlphaBots.Tools;
import frc.robot.commands.C_PivotToPosition;
import frc.robot.subsystems.Elevator.POSITION;

public class Pivot extends SubsystemBase {
 
  //Get ClassName to help network tables auto sort by creating a sub Table with the same name.
  String className = this.getClass().getSimpleName();
  
  public final TalonFX m_PivotMotor = new TalonFX(constants.CanBus.armPivotMotorCanID, constants.CanBus.RioCANBusName);
  public final CANcoder pivotAbsoluteEncoder = new CANcoder(constants.CanBus.pivotAbsoluteEncoder, constants.CanBus.RioCANBusName);

  TalonFXConfiguration configuration;
  


  public double gearRatio = constants.PlasmaPivot.gearRatio;

  DoubleEntry NT_Rpm =  NT.getDoubleEntry(className ,"RPM",0);
  DoubleEntry NT_MotorTemp =  NT.getDoubleEntry(className,"MotorTemp",0);
  DoubleEntry NT_StatorCurrent = NT.getDoubleEntry(className, "StatorCurrent", 0);

  DoubleEntry NT_PGain = NT.getDoubleEntry(className , "P Gain",0);
  DoubleEntry NT_IGain = NT.getDoubleEntry(className, "I Gain",0);
  DoubleEntry NT_DGain = NT.getDoubleEntry(className , "D Gain",0);

  DoubleEntry NT_SGain = NT.getDoubleEntry(className , "S Gain",0);
  DoubleEntry NT_GGain = NT.getDoubleEntry(className , "G Gain",0);
  DoubleEntry NT_AGain = NT.getDoubleEntry(className , "A Gain",0);
  DoubleEntry NT_VGain = NT.getDoubleEntry(className , "V Gain",0);


  DoubleEntry NT_Acceleration = NT.getDoubleEntry(className , "Acceleration",0);
  DoubleEntry NT_Jerk = NT.getDoubleEntry(className , "Jerk",0);
  DoubleEntry NT_Cruise = NT.getDoubleEntry(className , "Cruise",0);


  DoubleEntry NT_CurrentPosition = NT.getDoubleEntry(className, "position",0);
  DoubleEntry NT_SetpointPosition = NT.getDoubleEntry(className , "SetpointPosition",0.0);
  DoubleEntry NT_RequestedPosition = NT.getDoubleEntry(className , "RequestedPosition",0.0);
 
  BooleanEntry NT_BrakeEnabled = NT.getBooleanEntry(className , "BrakeOn",false);
  BooleanEntry NT_FoldedOut = NT.getBooleanEntry(className , "FoldedOut",false); //this refers to being folded out enough to NOT stage 1 Xbar.
  BooleanEntry NT_FoldedUpEnough = NT.getBooleanEntry(className , "FoldedUpEnough",false);//This refers to be being folded Up to not be out past striaght sticking out 90 =====<
  BooleanEntry NT_FoldedUpFromReef = NT.getBooleanEntry(className , "FoldedUpFromReef",false);//This refers to be being folded Up to not be out past striaght sticking out 90 =====<

  BooleanEntry NT_ElevatorTravelPosition = NT.getBooleanEntry(className , "InElevatorTravelPosition",false);// folded out past Stage 1 but folded up past chassis


  private double currentPosition = 0;
  private double requestedPosition = 0;
  private double setPointPosition = 0;

  DoubleSupplier elevatorposition;
  //InterpolatingDoubleTreeMap heightMaxPivotMap;

  public Pivot(DoubleSupplier _elevatorPosition) {
    System.out.println("Creating " + className + " object"); 

    NT_PGain.set(constants.PlasmaPivot.kP);
    NT_IGain.set(constants.PlasmaPivot.kI);
    NT_DGain.set(constants.PlasmaPivot.kD);

    NT_AGain.set(constants.PlasmaPivot.kA);
    NT_VGain.set(constants.PlasmaPivot.kV);

    NT_SGain.set(constants.PlasmaPivot.kS);
    NT_GGain.set(constants.PlasmaPivot.kG);

    NT_Acceleration.set(constants.PlasmaPivot.Accel);
    NT_Jerk.set(constants.PlasmaPivot.Jerk);
    NT_Cruise.set(constants.PlasmaPivot.Cruise);

    NT_RequestedPosition.set(requestedPosition);

    elevatorposition = _elevatorPosition;
    configuration = buildMotorConfig();

    frc.robot.AlphaBots.Tools.SetConfigToTalonFX(m_PivotMotor,configuration,className);

    // InterpolatingDoubleTreeMap heightMaxPivotMap = new InterpolatingDoubleTreeMap();
    // heightMaxPivotMap.put(0.0,10.0);
    // heightMaxPivotMap.put(5.0, 14.0);
    //BRAKE();//HoldPosition();
  }

  public TalonFXConfiguration buildMotorConfig(){
    TalonFXConfiguration _configuration = new TalonFXConfiguration();

    _configuration.withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive));

    //_configuration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    _configuration.Slot1.kP = constants.PlasmaPivot.kP;
    _configuration.Slot1.kI = constants.PlasmaPivot.kI;
    _configuration.Slot1.kD = constants.PlasmaPivot.kD;

    _configuration.Slot1.kA = constants.PlasmaPivot.kA;
    _configuration.Slot1.kV = constants.PlasmaPivot.kV;

    _configuration.Slot1.kG = constants.PlasmaPivot.kG;
    _configuration.Slot1.GravityType = GravityTypeValue.Arm_Cosine;

    _configuration.Slot1.kS = constants.PlasmaPivot.kS;
    _configuration.Slot1.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;

    _configuration.Feedback.RotorToSensorRatio = constants.PlasmaPivot.gearRatio;

    _configuration.CurrentLimits.StatorCurrentLimitEnable = true;
    _configuration.CurrentLimits.StatorCurrentLimit = constants.PlasmaPivot.maxStatorCurrent;

    _configuration.TorqueCurrent.PeakForwardTorqueCurrent = constants.PlasmaPivot.maxStatorCurrent;
    _configuration.TorqueCurrent.PeakReverseTorqueCurrent = constants.PlasmaPivot.maxStatorCurrent;

    _configuration.TorqueCurrent.TorqueNeutralDeadband = 0.0;

    _configuration.MotionMagic.MotionMagicAcceleration = constants.PlasmaPivot.Accel;
    _configuration.MotionMagic.MotionMagicJerk = constants.PlasmaPivot.Jerk;
    _configuration.MotionMagic.MotionMagicCruiseVelocity = constants.PlasmaPivot.Cruise;

    _configuration.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    _configuration.SoftwareLimitSwitch.ForwardSoftLimitThreshold = constants.PlasmaPivot.maxposition;

    _configuration.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    _configuration.SoftwareLimitSwitch.ReverseSoftLimitThreshold = constants.PlasmaPivot.minposition;
    
    _configuration.MotorOutput.withNeutralMode(NeutralModeValue.Brake);

      CANcoderConfiguration cc_cfg = new CANcoderConfiguration();
      //cc_cfg.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf; old 2024 and before way. 
      //Setting this to 1 makes the absolute position unsigned [0, 1)
      //Setting this to 0.5 makes the absolute position signed [-0.5, 0.5)
      //Setting this to 0 makes the absolute position always negative [-1, 0) 
      cc_cfg.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
      
      cc_cfg.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
      cc_cfg.MagnetSensor.MagnetOffset = constants.PlasmaPivot.absoMagnetOffset;// ;
      pivotAbsoluteEncoder.getConfigurator().apply(cc_cfg);

    var AbsoluteEncoderFeedbackConfig = new FeedbackConfigs().withFeedbackRemoteSensorID(pivotAbsoluteEncoder.getDeviceID())
       .withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder)
       .withRotorToSensorRatio(constants.PlasmaPivot.gearRatio).withSensorToMechanismRatio(1.0);

    _configuration.withFeedback(AbsoluteEncoderFeedbackConfig);

    return _configuration;
  }
  
  @Override
  public void periodic() {
    currentPosition = m_PivotMotor.getPosition().getValueAsDouble();

    NT_CurrentPosition.set(currentPosition);
    NT_SetpointPosition.set(setPointPosition);
    NT_RequestedPosition.set(requestedPosition);

    // This method will be called once per scheduler run
    NT_Rpm.set(m_PivotMotor.getVelocity().getValueAsDouble() * 60);
    NT_MotorTemp.set(m_PivotMotor.getDeviceTemp().getValueAsDouble());
    NT_StatorCurrent.set(m_PivotMotor.getStatorCurrent().getValueAsDouble());


    NT_FoldedOut.set(IsPivotFoldedOut.getAsBoolean());
    NT_FoldedUpEnough.set(IsPivotFoldedFarOut.getAsBoolean());
    NT_FoldedUpFromReef.set(IsPivotAwayFromReef.getAsBoolean());
    NT_ElevatorTravelPosition.set(IsPivotinTravelPosition.getAsBoolean());

    //feedback
    double p = NT_PGain.getAsDouble();
    double i = NT_IGain.getAsDouble();
    double d = NT_DGain.getAsDouble();

    //feedforward
    double a = NT_AGain.getAsDouble();
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
  
    if((a != configuration.Slot1.kA)) { configuration.Slot1.kA = a; motorNeedsConfig = true; }
    if((v != configuration.Slot1.kV)) { configuration.Slot1.kV = v; motorNeedsConfig = true; }
    if((s != configuration.Slot1.kS)) { configuration.Slot1.kS = s; motorNeedsConfig = true; }
    if((g != configuration.Slot1.kG)) { configuration.Slot1.kG = g; motorNeedsConfig = true; }

    if((mA != configuration.MotionMagic.MotionMagicAcceleration)) { configuration.MotionMagic.MotionMagicAcceleration = mA; motorNeedsConfig = true; }
    if((mJ != configuration.MotionMagic.MotionMagicJerk)) { configuration.MotionMagic.MotionMagicJerk = mJ; motorNeedsConfig = true; }
    if((mC != configuration.MotionMagic.MotionMagicCruiseVelocity)) { configuration.MotionMagic.MotionMagicCruiseVelocity = mC; motorNeedsConfig = true; }
    
    if (motorNeedsConfig){Tools.SetConfigToTalonFX(m_PivotMotor,configuration,className);}

    doTravelIfelevatormoving(requestedPosition);
  }
  public void doTravelIfelevatormoving(double _requestedPosition)
  {
    if (setPointPosition != _requestedPosition) {
      if(MantaState.ss_Elevator.currentState == POSITION.parked){GotoPosition(_requestedPosition);}
      else
      {
          if(_requestedPosition >= constants.PlasmaPivot.TravelPosition)
            {
              //check if pivot is in a safe travel position
             
                GotoPosition(_requestedPosition);
            }
              else{
                //if not IsPivotinTravelPosition, set position to "cannotfoldbelowPosition"
                //ONLY safe to goto CannotFoldBelowPosition
                GotoPosition(constants.PlasmaPivot.TravelPosition);
              }
          }
      }
  }
 

   public void doTravelIfInCorrectPosition(double _requestedPosition)
  {
    if (setPointPosition != _requestedPosition) {
      
      //if we are above the CannotFoldBelow position
      if(elevatorposition.getAsDouble() > constants.Elevator.CannotPivotParkBelowElevatorPosition)
      {
        //if we are going below the cannot fold position
        if(_requestedPosition >= constants.PlasmaPivot.TravelPosition)
        {
          //check if pivot is in a safe travel position
          if(MantaState.ss_Pivot.IsPivotFoldedOut.getAsBoolean()) //IsPivotinTravelPosition
          {
             //if/when we are folded out, set position to requested position
            //safe to goto requestion position
            GotoPosition(_requestedPosition);
          }
          else{
            //if not IsPivotinTravelPosition, set position to "cannotfoldbelowPosition"
            //ONLY safe to goto CannotFoldBelowPosition
            GotoPosition(constants.PlasmaPivot.TravelPosition);
          }
        }
        else{
          //if we are above the safe zone and staying above the safe zone then request the new position. 
          GotoPosition(_requestedPosition);
        }
      }   //if we are below the CannotFoldabove position
      else if(elevatorposition.getAsDouble() < constants.Elevator.CannotPivotParkAboveElevatorPosition)
      {
        //if we are going above the cannot fold position
        if(_requestedPosition < constants.Elevator.CannotPivotParkAboveElevatorPosition)
        {
          //check if pivot is in a safe travel position
          if(MantaState.ss_Pivot.IsPivotinTravelPosition.getAsBoolean())
          {
              //if/when we are folded out, set position to requested position
            //safe to goto requestion position
            GotoPosition(_requestedPosition);
          }
          else{
            //if not IsPivotinTravelPosition, set position to "cannotfoldbelowPosition"
            //ONLY safe to goto CannotFoldBelowPosition
            GotoPosition(constants.Elevator.CannotPivotParkAboveElevatorPosition);
          }
        }
        else{
          //if we are below the safe zone and going below the safe zone then request the new position. 
          GotoPosition(_requestedPosition);
        }
      }//if we are not above the nogo and we are not below the nogo we are in the nogo. make sure we are in travel position and goto the called position
      else 
      {
        //check if pivot is in a safe travel position
        if(MantaState.ss_Pivot.IsPivotinTravelPosition.getAsBoolean())
        {
            //if/when we are folded out, set position to requested position
          //safe to goto requestion position
          GotoPosition(_requestedPosition);
        }
        else{
          //if not IsPivotinTravelPosition, dont move we are in the No-go zone already. 
        }
      }
    }// else if we are close to parked and we are requesting a park. then just brake mode. 
    else if ((setPointPosition < constants.Elevator.ElevatorBrakeParkTolerance) 
          & (_requestedPosition < constants.Elevator.ElevatorBrakeParkTolerance)
          &  m_PivotMotor.getVelocity().getValueAsDouble() < 100
          & Tools.isPosAtSetpoint(currentPosition, constants.Elevator.minElevatorHeight, constants.Elevator.ElevatorBrakeParkTolerance))
    {
      //System.out.println("elevator Braking");
      
      //currentState = POSITION.parked;
      BRAKE();
    }
  }
  //is the elevator height low enough that we can fit under the stafe 1 cross bar when retracting (does not account for extension)
  //public BooleanSupplier CanPivotFoldUp = ()->{return getPosition() < constants.PlasmaPivot.elevatorheightToFoldUp ? true:false;}; 
  //public BooleanSupplier IsOutPastPastStage1 = ()->{return getPosition() > constants.PlasmaPivot.minPositionToBeSafeFromStage1Crossbar ? true:false;};
  
  public BooleanSupplier IsPivotFoldedOut = ()->{return currentPosition > constants.PlasmaPivot.minPositionToBeSafeFromStage1Crossbar ? true:false;};
  public BooleanSupplier IsPivotAwayFromReef = ()->{return currentPosition < constants.PlasmaPivot.maxPositionToBeSafeFromSmashingintoReef ? true:false;};

  public BooleanSupplier IsPivotFoldedFarOut = ()->{return currentPosition < constants.PlasmaPivot.maxPositionToBeSafeFromSmashingintoSelf ? true:false;};
  public BooleanSupplier IsPivotinTravelPosition = ()->{return IsPivotFoldedOut.getAsBoolean() & IsPivotAwayFromReef.getAsBoolean();};
  public BooleanSupplier IsPivotParked = ()->{return frc.robot.AlphaBots.Tools.isPosAtSetpoint(getPosition(),constants.PlasmaPivot.ParkPosition,constants.PlasmaPivot.MoveTolerance);};

  //IsSafeToGoDown TODO: this needs a linear interpolation map because at 0 elevator we can only be 90. at mid height we can point down a bit. 
  //also extension will change this number but maybe just assume always extened (ie worst case scenario)

  //public BooleanSupplier IsSafeToGoDown = ()->{return getPosition() < constants.PlasmaPivot.maxPositionToBeSafeFromSmashingintoSelf ? true:false;};  
  
  //public Trigger IsPivotOutPastStage1 = new Trigger(IsOutPastPastStage1);

  public double getPosition()
  {
    return currentPosition;//m_PivotMotor.getPosition().getValueAsDouble(); // / gearRatio;
  }
  public Command C_GotoPositon(double positon) {
    return new C_PivotToPosition(this,positon);
      // return new InstantCommand(()->{
      //   GotoPosition(positon);
      // });
  }
  public InstantCommand C_Stop() {
    return new InstantCommand(()->{
      BRAKE();
    });
  }

  public void HoldPosition(){ 
    currentPosition = m_PivotMotor.getPosition().getValueAsDouble();
      GotoPosition(getPosition()-(m_PivotMotor.getVelocity().getValueAsDouble()/constants.CanBus.canBusUpdateFrequency));
  }
  public void RequestPosition(double wantedposition)
    {
      requestedPosition = wantedposition;
    }

  public void GotoPosition(double wantedposition){ 
    NT_BrakeEnabled.set(false);
    setPointPosition = wantedposition;
    NT_SetpointPosition.set(setPointPosition);

    // m_PivotMotor.setControl( 
    //   new MotionMagicTorqueCurrentFOC(wantedposition)
    //   .withSlot(1)
  
    // );
    m_PivotMotor.setControl( 
      PivotRequest.withPosition(wantedposition)
  
    );
  }

  public void BRAKE(){
    NT_BrakeEnabled.set(true);
    m_PivotMotor.setControl(new StaticBrake());
  }  
  //Kraken x60 FOC kT = 19.81;  From https://ctre.download/files/datasheet/Motor%20Performance%20Analysis%20Report.pdf
  //private MotionMagicTorqueCurrentFOC PivotRequest = new MotionMagicTorqueCurrentFOC(0).withSlot(1).withFeedForward(0);
  
  private MotionMagicVoltage PivotRequest = new MotionMagicVoltage(0).withSlot(1).withFeedForward(0);
  //private MotionMagicExpoTorqueCurrentFOC



  ///////
  private final TorqueCurrentFOC m_torqueCurrentReq = new TorqueCurrentFOC(0.0);
  
  private final SysIdRoutine m_pivotSysID =
   new SysIdRoutine(
      new SysIdRoutine.Config(
         null,        // Use default ramp rate (1 V/s)
         Volts.of(20), // Reduce dynamic step voltage to 4 to prevent brownout
         null,        // Use default timeout (10 s)
                      // Log state with Phoenix SignalLogger class
         (state) -> SignalLogger.writeString("state", state.toString())
      ),
      new SysIdRoutine.Mechanism(
         (volts) -> m_PivotMotor.setControl(m_torqueCurrentReq.withOutput(volts.in(Volts))),
         null,
         this
      )
   );
   public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_pivotSysID.quasistatic(direction);
 }
 
 public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_pivotSysID.dynamic(direction);
 }

 //////////////
}