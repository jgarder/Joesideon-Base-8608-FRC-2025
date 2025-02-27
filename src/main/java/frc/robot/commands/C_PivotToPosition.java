package frc.robot.commands;

import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants;
import frc.robot.AlphaBots.NT;
import frc.robot.subsystems.MantaState;
import frc.robot.subsystems.Pivot;

public class C_PivotToPosition extends Command{
    
    Pivot SubSystem;
    double wantedPosition;
    double Tolerance = constants.PlasmaPivot.MoveTolerance;
    public final Timer SettleDebounceTimer = new Timer();
    private double debounceSecondsNeeded = constants.PlasmaPivot.MovementDebounceTime;
    public C_PivotToPosition(Pivot subSys, double wantedposition){
        SubSystem = subSys;
        wantedPosition = wantedposition;
        addRequirements(subSys);
    }

    @Override
    public void initialize() {
        SubSystem.RequestPosition(wantedPosition);
        SettleDebounceTimer.restart();
        MantaState.NT_PivotPosOk.set(false);
    }

    @Override
    public boolean isFinished() {
        boolean isatSetpos = frc.robot.AlphaBots.Tools.isPosAtSetpoint(SubSystem.getPosition(), wantedPosition, Tolerance);
        if(isatSetpos)
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
        MantaState.NT_PivotPosOk.set(false);
        return false;
    }

   

}
