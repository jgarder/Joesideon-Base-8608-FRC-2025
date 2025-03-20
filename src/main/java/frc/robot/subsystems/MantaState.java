package frc.robot.subsystems;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.StringEntry;
import edu.wpi.first.networktables.StructEntry;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants;
import frc.robot.AlphaBots.NT;
import frc.robot.AlphaBots.AprilTagSystem.AprilTag;
import frc.robot.AlphaBots.AprilTagSystem.AprilTag.TagType;
import frc.robot.AlphaBots.AprilTagSystem.PoseFinder;
import frc.robot.generated.TunerConstants;

public class MantaState extends SubsystemBase {
  //This will make smartdashboardPuts goto the classes subfolder in the network tables. the / does the subfoldering.
    public static String className = "MantaState";
    public static final String PidAlignmentClassname = "pidAlignment";
    private static MantaState instance;

    
    
    public static BooleanEntry NT_IsLoaded = NT.getBooleanEntry(className , "IsLoaded",false);
    
    public static StructEntry<Pose2d> NT_AlignSetpoint = NT.getStructEntry_Pose2D("Poses","AlignSetpoint",new Pose2d());
    public static DoubleEntry NT_XPGain = NT.getDoubleEntry(PidAlignmentClassname, "XP Gain",constants.drivetrainThings.k_PoseX_P);
    public static DoubleEntry NT_XIGain = NT.getDoubleEntry(PidAlignmentClassname, "XI Gain",constants.drivetrainThings.k_PoseX_I);
    public static DoubleEntry NT_XDGain = NT.getDoubleEntry(PidAlignmentClassname, "XD Gain",constants.drivetrainThings.k_PoseX_D);

    public static DoubleEntry NT_ZPGain = NT.getDoubleEntry(PidAlignmentClassname, "ZP Gain",constants.drivetrainThings.k_RZ_P);
    public static DoubleEntry NT_ZIGain = NT.getDoubleEntry(PidAlignmentClassname, "ZI Gain",constants.drivetrainThings.k_RZ_I);
    public static DoubleEntry NT_ZDGain = NT.getDoubleEntry(PidAlignmentClassname, "ZD Gain",constants.drivetrainThings.k_RZ_D);
    public static BooleanEntry NT_Xok = NT.getBooleanEntry(PidAlignmentClassname, "Xok", false);
    public static BooleanEntry NT_Yok = NT.getBooleanEntry(PidAlignmentClassname, "Yok", false);
    public static BooleanEntry NT_Zok = NT.getBooleanEntry(PidAlignmentClassname, "Zok", false);

    public static BooleanEntry NT_LLDisable = NT.getBooleanEntry(className, "LLDisabled", false);
    public static BooleanEntry NT_AltControls = NT.getBooleanEntry(className, "AltControlsEnable", false);
    public static BooleanEntry NT_UpperAlgae = NT.getBooleanEntry(className, "ClosestIsUpperAlgae", false);

    public static BooleanEntry NT_PivotPosOk = NT.getBooleanEntry(className, "PivotPosOk", false);
    public static BooleanEntry NT_ElevatorPosOk = NT.getBooleanEntry(className, "ElevatorPosOk", false);
    public static BooleanEntry NT_ExtensionPosOk = NT.getBooleanEntry(className, "ExtensionPosOk", false);


    DoubleEntry NT_ExtensionLiveOffset = NT.getDoubleEntry(className , "ExtensionLiveOffset",0.0);
    DoubleEntry NT_RearIntakeLiveOffset = NT.getDoubleEntry(className , "RearIntakeLiveOffset",constants.RearMotorizedIntake.IntakeRps);

    public static StringEntry NT_AlignedUsing = NT.getStringEntry(className, "AlignedUsing", "none");
    public static DoubleEntry NT_TimeToAlign = NT.getDoubleEntry(PidAlignmentClassname, "TimeToAlign",0.0);
    public static LoggedNetworkBoolean NT_Mt1FrontdoRejectUpdate = new LoggedNetworkBoolean("/AlphaBots/Mt1FrontdoRejectUpdate",false);
    public static LoggedNetworkBoolean NT_Mt1BackdoRejectUpdate = new LoggedNetworkBoolean("/AlphaBots/Mt1BackdoRejectUpdate",false);

    public static LoggedNetworkBoolean NT_Mt2FrontdoRejectUpdate = new LoggedNetworkBoolean("/AlphaBots/Mt2FrontdoRejectUpdate",false);
    public static LoggedNetworkBoolean NT_Mt2BackdoRejectUpdate = new LoggedNetworkBoolean("/AlphaBots/Mt2BackdoRejectUpdate",false);

    public static DoubleEntry NT_MaxSpeed = NT.getDoubleEntry(className, "Max Speed",TunerConstants.kSpeedAt12Volts.in(MetersPerSecond));

    @Override
    public void periodic() {
      double Liveoffset = NT_ExtensionLiveOffset.getAsDouble();
      if(Liveoffset != constants.PlasmaExtension.LiveOffset){constants.PlasmaExtension.LiveOffset = Liveoffset;}

      double LiveIntakeoffset = NT_RearIntakeLiveOffset.getAsDouble();
      if(LiveIntakeoffset != constants.RearMotorizedIntake.IntakeRps){constants.RearMotorizedIntake.IntakeRps = LiveIntakeoffset;}

      NT_UpperAlgae.set(NearestTagIsUpperAlgae.getAsBoolean());
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

      getmaxspeed();
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


      NT_Xok.set(false);
      NT_Yok.set(false);
      NT_Zok.set(false);
      NT_AlignSetpoint.set(new Pose2d());//send out a default;
      setLimeLightBypassed(false);
      setAltControlModeEnabled(false);
      NT_ExtensionLiveOffset.set(constants.PlasmaExtension.LiveOffset);
      NT_AlignedUsing.set("none");
      NT_RearIntakeLiveOffset.set(constants.RearMotorizedIntake.IntakeRps);
    }


    //Robot State Triggers (really only used for lights because why not)
    Trigger LimelightBypass = new Trigger(getLimeLightBypassed)
      .whileTrue(new InstantCommand(()->{CANdleSubsystem.limelightBypassLights();}))
      .onFalse(new InstantCommand(()->{CANdleSubsystem.clearAnimations();}));

    Trigger climbTimeLEDs = new Trigger(isClimbTime)
      .whileTrue(new InstantCommand(()->{CANdleSubsystem.timeToClimbLights();}))
      .whileFalse(new InstantCommand(()->{CANdleSubsystem.clearAnimations();}));

    Trigger climbingLED = new Trigger(getAltControlModeEnabled)
      .whileTrue(new InstantCommand(()->{CANdleSubsystem.climbLights();}))
      .onFalse(new InstantCommand(()->{CANdleSubsystem.clearAnimations();}));

    public static  CommandSwerveDrivetrain DriveTrain;  
    
    //fields
    private static boolean AltControlModeEnabled = false;
    private static boolean LimeLightBypassed = false;

    private static boolean climbTime = false;

    //getters
    public static BooleanSupplier getAltControlModeEnabled = ()->{return AltControlModeEnabled;};
    public static BooleanSupplier getAltControlModeDisabled = ()->{return !AltControlModeEnabled;};
    public static BooleanSupplier getLimeLightBypassed = ()->{return LimeLightBypassed;};
    public BooleanSupplier IsPivotFoldedOut;
    public BooleanSupplier IsPivotFoldedFarOut;
    public BooleanSupplier IsPivotinTravelPosition;
    public static BooleanSupplier NearestTagIsUpperAlgae = ()->{AprilTag targetTag = PoseFinder.getClosestTagofTypeToRobotCenterForAlliance(DriveTrain.getState().Pose, TagType.Reef); return targetTag.algaeOnUpper;};

    public static BooleanSupplier isClimbTime = ()->{return climbTime;};
    //setters
    public static boolean setLimeLightBypassed(boolean setTo)
    {
        LimeLightBypassed = setTo;
        NT_LLDisable.set(LimeLightBypassed);
      return LimeLightBypassed;
    }
    public static boolean setToClimbTime(){
        climbTime = true;
        return climbTime;
    }
    public static boolean setAltControlModeEnabled(boolean setTo)
    {
      
        AltControlModeEnabled = setTo;
        NT_AltControls.set(AltControlModeEnabled);
      return AltControlModeEnabled;
    }
    public static double additionalSpeedOffset = 1.10;
    public static double getmaxspeed()
    {
      double percentofMaxheight = ss_Elevator.getPosition() / constants.Elevator.maxElevatorheight * additionalSpeedOffset;
      double reductionOfSpeedAmount = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * percentofMaxheight;
      double maxSpeedAtheight = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) - reductionOfSpeedAmount;
      NT_MaxSpeed.set(maxSpeedAtheight);

      return TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    }
    public static double getmaxAngularRate()
    {
      return RotationsPerSecond.of(0.75).in(RadiansPerSecond);
    }
}
