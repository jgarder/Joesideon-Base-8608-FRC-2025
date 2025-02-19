package frc.robot.subsystems;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.StructEntry;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants;
import frc.robot.AlphaBots.NT;
import frc.robot.generated.TunerConstants;

public class MantaState extends SubsystemBase {
    private static MantaState instance;

    public static StructEntry<Pose2d> NT_AlignSetpoint = NT.getStructEntry_Pose2D("Poses","AlignSetpoint",new Pose2d());
    
    public static DoubleEntry NT_XPGain = NT.getDoubleEntry("pidAlignment" , "XP Gain",constants.drivetrainThings.k_PoseX_P);
    public static DoubleEntry NT_XIGain = NT.getDoubleEntry("pidAlignment", "XI Gain",constants.drivetrainThings.k_PoseX_I);
    public static DoubleEntry NT_XDGain = NT.getDoubleEntry("pidAlignment" , "XD Gain",constants.drivetrainThings.k_PoseX_D);

    public static DoubleEntry NT_ZPGain = NT.getDoubleEntry("pidAlignment" , "ZP Gain",constants.drivetrainThings.k_RZ_P);
    public static DoubleEntry NT_ZIGain = NT.getDoubleEntry("pidAlignment", "ZI Gain",constants.drivetrainThings.k_RZ_I);
    public static DoubleEntry NT_ZDGain = NT.getDoubleEntry("pidAlignment" , "ZD Gain",constants.drivetrainThings.k_RZ_D);
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

      NT_AlignSetpoint.set(new Pose2d());//send out a default;
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
