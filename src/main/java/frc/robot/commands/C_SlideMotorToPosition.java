package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants;
import frc.robot.subsystems.josiahClimber;

public class C_SlideMotorToPosition extends Command{
    josiahClimber SubSystem;
    double wantedPosition;
    double Tolerance = constants.Elevator.MoveTolerance;
    
    public C_SlideMotorToPosition(josiahClimber subSys, double wantedposition){
        SubSystem = subSys;
        wantedPosition = wantedposition;
        addRequirements(subSys);
    }

    @Override
    public void initialize() {
        SubSystem.SlideGotoPosition(wantedPosition);
    }

    @Override
    public boolean isFinished() {
        return frc.robot.AlphaBots.Tools.isPosAtSetpoint(SubSystem.getSlidePosition(), wantedPosition, Tolerance);
    }

   

}
