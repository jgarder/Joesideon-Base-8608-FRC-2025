package frc.robot.commands;

import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants;
import frc.robot.AlphaBots.NT;
import frc.robot.constants.groundIntake;
import frc.robot.subsystems.MantaState;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.groundIntake.groundPivot;
import frc.robot.subsystems.groundIntake.groundPivot.ArmState;

public class C_PivotGroundIntake extends Command{
    
    groundPivot SubSystem;
    double wantedPosition;
    //double Tolerance = constants.groundIntake.PIDtolerance;
    public final Timer SettleDebounceTimer = new Timer();
    private double debounceSecondsNeeded = constants.groundIntake.MovementDebounceTime;
    public C_PivotGroundIntake(groundPivot subSys, double wantedposition){
        SubSystem = subSys;
        wantedPosition = wantedposition;
        addRequirements(subSys);
    }

    @Override
    public void initialize() {
        SubSystem.setArmRequest(wantedPosition);
        SettleDebounceTimer.restart();
        //MantaState.NT_PivotPosOk.set(false);
    }

    @Override
    public void execute(){
        SubSystem.setArmRequest(wantedPosition);
    }

    @Override
    public boolean isFinished() {
        if(SubSystem.getArmState() == ArmState.inPosition)
        {
            if(SettleDebounceTimer.get() > debounceSecondsNeeded)
            {
                MantaState.NT_PivotPosOk.set(true);
                return true;
                
            } 
        }
        else
        {
            SettleDebounceTimer.restart();
        }
        //MantaState.NT_PivotPosOk.set(false);
        return false;
    }

   

}
