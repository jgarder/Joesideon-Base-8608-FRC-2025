package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants;
import frc.robot.subsystems.Elevator;

public class C_ElevateToPosition extends Command{
    Elevator SubSystem;
    double wantedPosition;
    double Tolerance = constants.Elevator.MoveTolerance;
    
    public C_ElevateToPosition(Elevator subSys, double wantedposition){
        SubSystem = subSys;
        wantedPosition = wantedposition;
    }

    @Override
    public void initialize() {
        SubSystem.RequestPosition(wantedPosition);
    }

    @Override
    public boolean isFinished() {
        return frc.robot.AlphaBots.Tools.isPosAtSetpoint(SubSystem.getPosition(), wantedPosition, Tolerance);
    }

   

}
