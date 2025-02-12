package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;

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
import frc.robot.constants;
import frc.robot.AlphaBots.NT;

public class MantaRay extends SubsystemBase {
 
    //This will make smartdashboardPuts goto the classes subfolder in the network tables. the / does the subfoldering.
  String className = this.getClass().getSimpleName()+"/";
  
  public final TalonFX m_TridentMotor = new TalonFX(constants.CanBus.MantaRayMotorCanID, "rio");

  private final com.ctre.phoenix6.controls.PositionDutyCycle m_positionDC = new PositionDutyCycle(0);
  private final com.ctre.phoenix6.controls.DutyCycleOut m_DutyCycle = new DutyCycleOut(constants.MantaRay.IntakeDutyCycle);

  private final StaticBrake m_s_Brake = new StaticBrake();
  
  TalonFXConfiguration configuration;

  DoubleTopic RpmTopic = NT.table.getDoubleTopic(className + " rpm");
  DoublePublisher RpmPub =  RpmTopic.publish();
  DoublePublisher MotorTemp =  NT.table.getDoubleTopic(className + "MotorTemp").publish();

  public MantaRay() {
    System.out.println("Creating " + className + " object"); 
    setMotorConfig();
    SmartDashboard.putNumber(className +" P Gain", kP);
    SmartDashboard.putNumber(className +" I Gain", kI);
    SmartDashboard.putNumber(className +" D Gain", kD);
  }


  public double LastPosition = 0;
  public double kP = 0.013;
  public double kI = 0.0;
  public double kD = 0.0;

  private void setMotorConfig(){
    configuration = new TalonFXConfiguration();

    configuration.Slot1.kP = kP;
    configuration.Slot1.kI = kI;
    configuration.Slot1.kD = kD;

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
    RpmPub.set(m_TridentMotor.getVelocity().getValueAsDouble() * 60);
    MotorTemp.set(m_TridentMotor.getDeviceTemp().getValueAsDouble());
    //SmartDashboard.putNumber(className + " rpm", (m_TridentMotor.getVelocity().getValueAsDouble() * 60));
    SmartDashboard.putNumber(className + " MotorTemp", m_TridentMotor.getDeviceTemp().getValueAsDouble());
    SmartDashboard.putNumber(className + " StatorCurrent", m_TridentMotor.getStatorCurrent().getValueAsDouble());

    double p = SmartDashboard.getNumber(className +" P Gain", kP);
    double i = SmartDashboard.getNumber(className +" I Gain", kI);
    double d = SmartDashboard.getNumber(className +" D Gain", kD);
          
    if((p != kP)) { configuration.Slot1.kP = p; kP = p; frc.robot.AlphaBots.Tools.SetConfigToTalonFX(m_TridentMotor,configuration,className); }
    if((i != kI)) { configuration.Slot1.kI = i; kI = i; frc.robot.AlphaBots.Tools.SetConfigToTalonFX(m_TridentMotor,configuration,className); }
    if((d != kD)) { configuration.Slot1.kD = d; kD = d; frc.robot.AlphaBots.Tools.SetConfigToTalonFX(m_TridentMotor,configuration,className); }

    double currentRotorposition = m_TridentMotor.getPosition().getValueAsDouble();
    SmartDashboard.putNumber(className + "CurrentPosition", currentRotorposition);
    // if (LastPosition != currentRotorposition) {
    //     LastPosition = currentRotorposition;
    // }
    //

  }


    public InstantCommand SpinUp(double rpmGoal) {
        double rpmgoal = rpmGoal;
        return new InstantCommand(()->{
            setMotorRPM(rpmgoal);
        });

    }

    public final Timer intakedebounceTimer = new Timer();
    Alert alert = new Alert("IsLoaded", AlertType.kInfo);
    public boolean isLoaded()
    {
        //check if amps are high
        //check is rotor is locked
        if(m_TridentMotor.getStatorCurrent().getValueAsDouble() > constants.MantaRay.intakeAmpCutoffThreshold)
        {   
            //if the timer hasnt been started start it
            if(!intakedebounceTimer.isRunning())
            {
                intakedebounceTimer.restart();
                SmartDashboard.putBoolean(className + " IsLoaded", false);
                return false;
            }

            //if timer running and the 
            if(intakedebounceTimer.isRunning() && intakedebounceTimer.get() >= constants.MantaRay.intakeAmpLimittime)
            {
                alert.set(true);
                intakedebounceTimer.stop();
                intakedebounceTimer.reset();
                SmartDashboard.putBoolean(className + " IsLoaded", true);
                return true;
            }
            return false;
           
        }
        alert.set(false);
        intakedebounceTimer.reset();
        SmartDashboard.putBoolean(className + " IsLoaded", false);
        //if its true then return true;
        return false;
    }
    public double canBusUpdateFrequency = 50;
    public void setMotorRPM(double rpmgoal) {
      //m_TridentMotor.setControl(m_torqueVelocity.withVelocity(rpmgoal/60));
      m_TridentMotor.setControl(new DutyCycleOut(rpmgoal));
    }

    public InstantCommand Stop() {
      return new InstantCommand(()->{
        BRAKE();
      });
    }
    public void HoldPosition(){ 
        double currentRotorposition = m_TridentMotor.getPosition().getValueAsDouble();
        LastPosition = currentRotorposition;
        GotoPosition(currentRotorposition-(m_TridentMotor.getVelocity().getValueAsDouble()/canBusUpdateFrequency));
    }
    public void GotoPosition(double wantedposition){ 
        SmartDashboard.putNumber(className + "SetpointPosition", LastPosition);
        m_TridentMotor.setControl(
            new PositionDutyCycle(wantedposition)
            .withEnableFOC(true)
            .withSlot(1)
        );
    }

    public Command bumpout()
    {
        return new InstantCommand(()->{m_TridentMotor.setControl(new DutyCycleOut(-1.0));});
    }

    public void BRAKE(){
      m_TridentMotor.setControl(m_s_Brake);
    }
    
}