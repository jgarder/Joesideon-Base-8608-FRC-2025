package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.MantaState;

public class C_ElevateToPosition extends Command{
    Elevator SubSystem;
    double wantedPosition;
    double Tolerance = constants.Elevator.MoveTolerance;
    
    public C_ElevateToPosition(Elevator subSys, double wantedposition){
        SubSystem = subSys;
        wantedPosition = wantedposition;
        addRequirements(subSys);
    }

    @Override
    public void initialize() {
        SubSystem.RequestPosition(wantedPosition);
        MantaState.NT_ElevatorPosOk.set(false);
    }

    @Override
    public boolean isFinished() {
        boolean isatPosition = frc.robot.AlphaBots.Tools.isPosAtSetpoint(SubSystem.getPosition(), wantedPosition, Tolerance);
        MantaState.NT_ElevatorPosOk.set(isatPosition);
        return isatPosition;
    }

   

}
