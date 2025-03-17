package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants;
import frc.robot.subsystems.MantaRay;
import frc.robot.subsystems.MantaState;
import frc.robot.subsystems.RearIntake;

public class C_ClearRearIntake extends Command {
    
    RearIntake ss_RearIntake;
    public final Timer startupdebounceTimer = new Timer();
    private double debounceSecondsNeeded = .250;
    public C_ClearRearIntake(RearIntake _ss_RearIntake,double _dutycycleSpeed)
    {
        ss_RearIntake = _ss_RearIntake;
        addRequirements(ss_RearIntake);
    }
    public C_ClearRearIntake(RearIntake _ss_RearIntake){
        ss_RearIntake = _ss_RearIntake;
        addRequirements(ss_RearIntake);
    }

    @Override
    public void initialize() {
        startupdebounceTimer.restart();
        ss_RearIntake.GotoVelocity(-constants.RearMotorizedIntake.IntakeRps);
    }
    
    @Override
    public void execute() {}

    //is called once per periodic and will run the end command when true is returned. 
    @Override
    public boolean isFinished(){
        if(startupdebounceTimer.get()<debounceSecondsNeeded){return false;}
        return true;
    }

    // If "isfinished" end true OR if we cancel this command for some reason. 
    // we need some actions to happen no matter what. 
    @Override
    public void end(boolean interrupted) {
        ss_RearIntake.COAST(); //hold position does not work here because of such a large feedforward. so we coast out when nuetral. 
    }

}
