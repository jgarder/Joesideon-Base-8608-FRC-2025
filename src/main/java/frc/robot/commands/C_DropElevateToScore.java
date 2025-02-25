package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.MantaState;

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
        double newwantedPosition =  SubSystem.currentHeight.getAsDouble() - positionAmountToDrop;
        wantedPosition = MathUtil.clamp(newwantedPosition,constants.Elevator.minElevatorHeight,constants.Elevator.maxElevatorheight);
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
