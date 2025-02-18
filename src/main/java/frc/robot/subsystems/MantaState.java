package frc.robot.subsystems;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.function.BooleanSupplier;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.TunerConstants;

public class MantaState extends SubsystemBase {
    private static MantaState instance;

    @Override
    public void periodic() {

      //hoping this works, stolen from Elastic Documentation
      // SmartDashboard.putData("Swerve Drive", new Sendable() {
      //   @Override
      //   public void initSendable(SendableBuilder builder) {
      //     builder.setSmartDashboardType("SwerveDrive");
      
      //     builder.addDoubleProperty("Front Left Angle", () -> DriveTrain.getModule(0).getEncoder().getPosition().getValueAsDouble(), null);
      //     builder.addDoubleProperty("Front Left Velocity", () -> DriveTrain.getModule(0).getDriveMotor().getVelocity().getValueAsDouble(), null);
      
      //     builder.addDoubleProperty("Front Right Angle", () -> DriveTrain.getModule(1).getEncoder().getPosition().getValueAsDouble(), null);
      //     builder.addDoubleProperty("Front Right Velocity", () -> DriveTrain.getModule(1).getDriveMotor().getVelocity().getValueAsDouble(), null);
      
      //     builder.addDoubleProperty("Back Left Angle", () -> DriveTrain.getModule(2).getEncoder().getPosition().getValueAsDouble(), null);
      //     builder.addDoubleProperty("Back Left Velocity", () -> DriveTrain.getModule(2).getDriveMotor().getVelocity().getValueAsDouble(), null);
      
      //     builder.addDoubleProperty("Back Right Angle", () -> DriveTrain.getModule(3).getEncoder().getPosition().getValueAsDouble(), null);
      //     builder.addDoubleProperty("Back Right Velocity", () -> DriveTrain.getModule(3).getDriveMotor().getVelocity().getValueAsDouble(), null);
      
      //     builder.addDoubleProperty("Robot Angle", () -> DriveTrain.getRotation3d().getX(), null);
      //   }
      // });
    }
    
    public static MantaState getInstance() throws Throwable
    {
        if (instance == null){
          throw new Throwable("Calling MantaState Before object was created");//instance = new MantaState();
        }
        return instance;
    }
    public static Elevator ss_Elevator;
    public static Pivot ss_Pivot;

    public MantaState(CommandSwerveDrivetrain incDriveTrain,Elevator ssElevator, Pivot ssPivot)
    {
      DriveTrain = incDriveTrain;
      ss_Elevator = ssElevator;

      ss_Pivot = ssPivot;
      IsPivotFoldedOut = ss_Pivot.IsPivotFoldedOut;
      IsPivotFoldedFarOut = ss_Pivot.IsPivotFoldedFarOut;
      IsPivotinTravelPosition = ss_Pivot.IsPivotinTravelPosition;
      instance = this;
    }

    public BooleanSupplier IsPivotFoldedOut;
    public BooleanSupplier IsPivotFoldedFarOut;
    public BooleanSupplier IsPivotinTravelPosition;
    
    public static double getmaxspeed()
    {
      return TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    }
    public static double getmaxAngularRate()
    {
      return RotationsPerSecond.of(0.75).in(RadiansPerSecond);
    }
    public static  CommandSwerveDrivetrain DriveTrain;  
    //fields
    private static boolean AltControlModeEnabled = false;
    private static boolean LimeLightBypassed = false;

    //getters
    public static BooleanSupplier getAltControlModeEnabled = ()->{return AltControlModeEnabled;};
    public static BooleanSupplier getLimeLightBypassed = ()->{return LimeLightBypassed;};

    //setters
    public static boolean setLimeLightBypassed(boolean setTo)
    {
        LimeLightBypassed = setTo;
      return LimeLightBypassed;
    }
    public static boolean setAltControlModeEnabled(boolean setTo)
    {
        AltControlModeEnabled = setTo;
      return AltControlModeEnabled;
    }
}
