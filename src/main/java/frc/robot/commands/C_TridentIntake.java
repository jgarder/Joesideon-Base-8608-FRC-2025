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
    private double debounceSecondsNeeded = .40;
    private double dutycycleSpeed = constants.MantaRay.IntakeDutyCycle;
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
        startupdebounceTimer.restart();
        ss_Trident.setUnloaded();
        ss_Trident.setDutyCycle(dutycycleSpeed);
        ss_RearIntake.GotoDutyCycle(constants.RearMotorizedIntake.dutyCyclePercent);
        //ss_RearIntake.GotoVelocity(constants.RearMotorizedIntake.IntakeRps);
    }
    
    @Override
    public void execute() {}

    //is called once per periodic and will run the end command when true is returned. 
    @Override
    public boolean isFinished(){
        if(startupdebounceTimer.get()<debounceSecondsNeeded){return false;}

        boolean isloaded = ss_Trident.isLoaded();
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
        ss_RearIntake.COAST(); //hold position does not work here because of such a large feedforward. so we coast out when nuetral. 
    }

}
