package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants;
import frc.robot.AlphaBots.NT;

public class MantaRay extends SubsystemBase {
 
    //This will make smartdashboardPuts goto the classes subfolder in the network tables. the / does the subfoldering.
  String className = this.getClass().getSimpleName();
  
  public final TalonFX m_TridentMotor = new TalonFX(constants.CanBus.MantaRayMotorCanID, constants.CanBus.RioCANBusName);

  // private final com.ctre.phoenix6.controls.PositionDutyCycle m_positionDC = new PositionDutyCycle(0);
  // private final com.ctre.phoenix6.controls.DutyCycleOut m_DutyCycle = new DutyCycleOut(constants.MantaRay.IntakeDutyCycle);

  private final StaticBrake m_s_Brake = new StaticBrake();
  private final NeutralOut m_s_Neutral = new NeutralOut();
  
  TalonFXConfiguration configuration;


  DoubleEntry NT_Rpm =  NT.getDoubleEntry(className ,"RPM",0);
  DoubleEntry NT_MotorTemp =  NT.getDoubleEntry(className,"MotorTemp",0);
  DoubleEntry NT_position = NT.getDoubleEntry(className, "position",0);
  DoubleEntry NT_StatorCurrent = NT.getDoubleEntry(className, "StatorCurrent", 0);
  
  DoubleEntry NT_PGain = NT.getDoubleEntry(className , "P Gain",0);
  DoubleEntry NT_IGain = NT.getDoubleEntry(className, "I Gain",0);
  DoubleEntry NT_DGain = NT.getDoubleEntry(className , "D Gain",0);

  DoubleEntry NT_SetpointPosition = NT.getDoubleEntry(className , "SetpointPosition",0.0);
  BooleanEntry NT_BrakeEnabled = NT.getBooleanEntry(className , "BrakeOn",false);
  
  public MantaRay() {
    System.out.println("Creating " + className + " object"); 
    setMotorConfig();
    NT_PGain.set(kP);
    NT_IGain.set(kI);
    NT_DGain.set(kD);
    NT_SetpointPosition.set(0);
    MantaState.NT_IsLoaded.set(_isloaded);
  }


  public double LastPosition = 0;
  public double kP = 6.0;
  public double kI = 10.0;
  public double kD = 0.0;

  private void setMotorConfig(){
    configuration = new TalonFXConfiguration();

    configuration.Slot0.kP = kP;
    configuration.Slot0.kI = kI;
    configuration.Slot0.kD = kD;

    configuration.CurrentLimits.StatorCurrentLimitEnable = true;
    configuration.CurrentLimits.StatorCurrentLimit = constants.MantaRay.intakeAmpLimit;
    

    SetConfigToMotor();
  }

  public void SetConfigToMotor()
  {
    frc.robot.AlphaBots.Tools.SetConfigToTalonFX(m_TridentMotor,configuration,className);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    NT_Rpm.set(m_TridentMotor.getVelocity().getValueAsDouble() * 60);
    NT_MotorTemp.set(m_TridentMotor.getDeviceTemp().getValueAsDouble());
    NT_position.set(m_TridentMotor.getPosition().getValueAsDouble());
    NT_StatorCurrent.set(m_TridentMotor.getStatorCurrent().getValueAsDouble());
    
    double p = NT_PGain.getAsDouble();
    double i = NT_IGain.getAsDouble();
    double d = NT_DGain.getAsDouble();
          
    if((p != kP)) { configuration.Slot0.kP = p; kP = p; frc.robot.AlphaBots.Tools.SetConfigToTalonFX(m_TridentMotor,configuration,className); }
    if((i != kI)) { configuration.Slot0.kI = i; kI = i; frc.robot.AlphaBots.Tools.SetConfigToTalonFX(m_TridentMotor,configuration,className); }
    if((d != kD)) { configuration.Slot0.kD = d; kD = d; frc.robot.AlphaBots.Tools.SetConfigToTalonFX(m_TridentMotor,configuration,className); }

    //double currentRotorposition = m_TridentMotor.getPosition().getValueAsDouble();
    // if (LastPosition != currentRotorposition) {
    //     LastPosition = currentRotorposition;
    // }
    //

  }


    public InstantCommand SpinUp(double rpmGoal) {
        double rpmgoal = rpmGoal;
        return new InstantCommand(()->{
            setDutyCycle(rpmgoal);
        });

    }
    public void setUnloaded()
    {
      _isloaded = false;
      MantaState.NT_IsLoaded.set(_isloaded);
    }

    private boolean _isloaded = false;
    public BooleanSupplier getisloaded = ()->{return _isloaded;};
    public final Timer intakedebounceTimer = new Timer();
    //Alert alert = new Alert("IsLoaded", AlertType.kInfo);
    
    public boolean isLoaded()
    {
      if(getisloaded.getAsBoolean()){return getisloaded.getAsBoolean();}
        //check if amps are high
        //check is rotor is locked
        if(m_TridentMotor.getStatorCurrent().getValueAsDouble() > constants.MantaRay.intakeAmpCutoffThreshold)
        {   
            //if the timer hasnt been started start it
            if(!intakedebounceTimer.isRunning())
            {
                intakedebounceTimer.restart();
                _isloaded = false;
                MantaState.NT_IsLoaded.set(getisloaded.getAsBoolean());
                return false;
            }

            //if timer running and the 
            if(intakedebounceTimer.isRunning() && intakedebounceTimer.get() >= constants.MantaRay.intakeAmpLimittime)
            {
                System.out.println("ISLOADED NOW");
                //alert.set(true);
                intakedebounceTimer.stop();
                intakedebounceTimer.reset();
                _isloaded = true;
                MantaState.NT_IsLoaded.set(_isloaded);
                return true;
            }
            return false;
           
        }
        //alert.set(false);
        intakedebounceTimer.restart();
        _isloaded = false;
        MantaState.NT_IsLoaded.set(_isloaded);
        //if its true then return true;
        return false;
    }
    public double canBusUpdateFrequency = 45;
    public void setDutyCycle(double DutyPercent) {
      //m_TridentMotor.setControl(m_torqueVelocity.withVelocity(rpmgoal/60));
      m_TridentMotor.setControl(new DutyCycleOut(DutyPercent));
    }

    public InstantCommand Stop() {
      return new InstantCommand(()->{
        BRAKE();
      });
    }
    public double PostRollAmount = 10;
    public void PostRollPosition()
    {
      double currentRotorposition = m_TridentMotor.getPosition(true).getValueAsDouble();
      LastPosition = currentRotorposition;
      //BRAKE();
      GotoPosition(currentRotorposition+PostRollAmount);//(m_TridentMotor.getVelocity().getValueAsDouble()/canBusUpdateFrequency));
    }
    public void HoldPosition(){ 
       
        holdPositionThroughVelocity();
    }
    public void GotoPosition(double wantedposition){ 
      NT_SetpointPosition.set(wantedposition);
        m_TridentMotor.setControl(
            new PositionDutyCycle(wantedposition)
            .withEnableFOC(true)
            .withSlot(0)
        );
    }
    public void holdPositionThroughVelocity(){
      m_TridentMotor.setControl(
            new VelocityTorqueCurrentFOC(0)
            .withSlot(0)
        );
    }
    public Command bumpout(double dutycycle)
    {
        return new InstantCommand(()->{m_TridentMotor.setControl(new DutyCycleOut(dutycycle));});
        
    }

    public Command bumpout()
    {
        return new InstantCommand(()->{m_TridentMotor.setControl(new DutyCycleOut(-1.0));});
        
    }
    public Command LooseGrip()
    {
        return new InstantCommand(()->{m_TridentMotor.setControl(new DutyCycleOut(-.03));});
        
    }
    public Command LooseBump()
    {
        return new InstantCommand(()->{m_TridentMotor.setControl(new DutyCycleOut(-0.5));});
        
    }

    public void BRAKE(){
      m_TridentMotor.setControl(m_s_Brake);
    }
    public void Neutral(){
      m_TridentMotor.setControl(m_s_Neutral);
    }
    
}