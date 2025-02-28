package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants;
import frc.robot.subsystems.josiahClimber;

public class C_CatchMotorToPosition extends Command{
    josiahClimber SubSystem;
    double wantedPosition;
    double Tolerance = constants.Climber.MoveTolerance;
    
    public C_CatchMotorToPosition(josiahClimber subSys, double wantedposition){
        SubSystem = subSys;
        wantedPosition = wantedposition;
        addRequirements(subSys);
    }

    @Override
    public void initialize() {
        SubSystem.CatchGotoPosition(wantedPosition);
    }

    @Override
    public boolean isFinished() {
        return frc.robot.AlphaBots.Tools.isPosAtSetpoint(SubSystem.getCatchPosition(), wantedPosition, Tolerance);
    }

   

}
