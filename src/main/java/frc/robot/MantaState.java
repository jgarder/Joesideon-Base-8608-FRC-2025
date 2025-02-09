package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.function.BooleanSupplier;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;

public class MantaState {
    // private MantaState(){}
    // private MantaState instance;

    
    // public MantaState getInstance()
    // {
    //     if (instance == null){instance = new MantaState();}
    //     return instance;
    // }
    private static Elevator ss_Elevator;
    public static void BuildMantaState(CommandSwerveDrivetrain incDriveTrain,Elevator ssElevator)
    {
      DriveTrain = incDriveTrain;
      ss_Elevator = ssElevator;
    }
    
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
