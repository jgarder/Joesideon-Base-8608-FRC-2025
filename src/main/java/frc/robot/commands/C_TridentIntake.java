package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants;
import frc.robot.subsystems.MantaRay;
import frc.robot.subsystems.MantaState;
import frc.robot.subsystems.RearIntake;

public class C_TridentIntake extends Command {
    
    MantaRay ss_Trident;
    RearIntake ss_RearIntake;
    boolean isloaded = false;
    public final Timer startupdebounceTimer = new Timer();
    private double debounceSecondsNeeded = .50;
    private double dutycycleSpeed = constants.MantaRay.IntakeDutyCycle;
    //private double RearIntakeDutyCyle = constants.RearMotorizedIntake.dutyCyclePercent;
    public C_TridentIntake(MantaRay incomingss_Trident, RearIntake _ss_RearIntake,double _dutycycleSpeed)
    {
        dutycycleSpeed =_dutycycleSpeed;
        ss_Trident = incomingss_Trident;
        ss_RearIntake = _ss_RearIntake;
        addRequirements(incomingss_Trident);
        addRequirements(ss_RearIntake);
    }
    public C_TridentIntake(MantaRay incomingss_Trident, RearIntake _ss_RearIntake){
        ss_Trident = incomingss_Trident;
        ss_RearIntake = _ss_RearIntake;
        addRequirements(incomingss_Trident);
        addRequirements(ss_RearIntake);
    }

    @Override
    public void initialize() {
        //RearIntakeDutyCyle = constants.RearMotorizedIntake.dutyCyclePercent;
        startupdebounceTimer.restart();
        
        ss_Trident.setUnloaded();
        ss_Trident.setDutyCycle(dutycycleSpeed);
        //ss_RearIntake.GotoDutyCycle(RearIntakeDutyCyle);
        ss_RearIntake.GotoVelocity(constants.RearMotorizedIntake.IntakeRps);
        ss_Trident.intakedebounceTimer.restart();
    }
    
    @Override
    public void execute() {}

    //is called once per periodic and will run the end command when true is returned. 
    @Override
    public boolean isFinished(){
        boolean isloaded = ss_Trident.isLoaded();
        if(startupdebounceTimer.get()<debounceSecondsNeeded){return false;}
        
        
        // if(isloaded)
        // {
        //     ss_Trident.HoldPosition();     
        // }
        MantaState.NT_IsLoaded.set(isloaded);
        return isloaded;
    }

    // If "isfinished" end true OR if we cancel this command for some reason. 
    // we need some actions to happen no matter what. 
    @Override
    public void end(boolean interrupted) {
        ss_Trident.HoldPosition();
        // if (interrupted) {
        //     //just hold here if manually stopped
        //     ss_Trident.HoldPosition();
        // }
        // else{
        //     //roll in another 10 if ended cleanly (postroll)
        //     ss_Trident.PostRollPosition();
        // }
        
        
        ss_RearIntake.COAST(); //hold position does not work here because of such a large feedforward. so we coast out when nuetral. 
    }

}
