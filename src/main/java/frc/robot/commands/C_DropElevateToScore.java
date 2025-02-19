package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants;
import frc.robot.subsystems.Elevator;

public class C_DropElevateToScore extends Command{
    Elevator SubSystem;
    double wantedPosition;
    double Tolerance = constants.Elevator.MoveTolerance;
    double positionAmountToDrop = 6;
    public C_DropElevateToScore(Elevator subSys){
        SubSystem = subSys;
        wantedPosition = 0;
        addRequirements(subSys);
    }

    @Override
    public void initialize() {
        wantedPosition =  SubSystem.currentHeight.getAsDouble() - positionAmountToDrop;
        SubSystem.RequestPosition(wantedPosition);
    }

    @Override
    public boolean isFinished() {
        return frc.robot.AlphaBots.Tools.isPosAtSetpoint(SubSystem.getPosition(), wantedPosition, Tolerance);
    }

   

}
