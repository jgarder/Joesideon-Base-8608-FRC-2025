package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants;
import frc.robot.subsystems.MantaRay;

public class C_TridentIntake extends Command {
    
    MantaRay ss_Trident;
    boolean isloaded = false;
    public final Timer startupdebounceTimer = new Timer();
    private double debounceSecondsNeeded = .30;
    private double dutycycleSpeed = constants.MantaRay.IntakeDutyCycle;
    public C_TridentIntake(MantaRay incomingss_Trident,double _dutycycleSpeed)
    {
        dutycycleSpeed =_dutycycleSpeed;
        ss_Trident = incomingss_Trident;
        addRequirements(incomingss_Trident);
    }
    public C_TridentIntake(MantaRay incomingss_Trident){
        ss_Trident = incomingss_Trident;
        addRequirements(incomingss_Trident);
    }

    @Override
    public void initialize() {
        startupdebounceTimer.restart();
        ss_Trident.setUnloaded();
        ss_Trident.setDutyCycle(dutycycleSpeed);
    }
    
    @Override
    public void execute() {}

    //is called once per periodic and will run the end command when true is returned. 
    @Override
    public boolean isFinished(){
        if(startupdebounceTimer.get()<debounceSecondsNeeded){return false;}

        boolean isloaded = ss_Trident.isLoaded();
        if(isloaded)
        {
            ss_Trident.HoldPosition();
        }
        return isloaded;
    }

    // If "isfinished" end true OR if we cancel this command for some reason. 
    // we need some actions to happen no matter what. 
    @Override
    public void end(boolean interrupted) {
        ss_Trident.HoldPosition();
    }

}
