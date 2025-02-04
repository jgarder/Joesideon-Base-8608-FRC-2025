package frc.robot;

import java.util.function.BooleanSupplier;

public class MantaState {
    private MantaState(){}
    private MantaState instance;
    public MantaState getInstance()
    {
        if (instance == null){instance = new MantaState();}
        return instance;
    }
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
