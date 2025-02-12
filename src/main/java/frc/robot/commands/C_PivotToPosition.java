package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants;
import frc.robot.subsystems.Pivot;

public class C_PivotToPosition extends Command{
    Pivot SubSystem;
    double wantedPosition;
    double Tolerance = constants.PlasmaPivot.MoveTolerance;
    
    public C_PivotToPosition(Pivot subSys, double wantedposition){
        SubSystem = subSys;
        wantedPosition = wantedposition;
        addRequirements(subSys);
    }

    @Override
    public void initialize() {
        SubSystem.GotoPosition(wantedPosition);
    }

    @Override
    public boolean isFinished() {
        return frc.robot.AlphaBots.Tools.isPosAtSetpoint(SubSystem.getPosition(), wantedPosition, Tolerance);
    }

   

}
