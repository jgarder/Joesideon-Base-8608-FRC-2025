package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants;
import frc.robot.constants.PlasmaExtension;
import frc.robot.subsystems.ArmExtension;
import frc.robot.subsystems.Elevator;

public class C_ExtendToPosition extends Command{
    ArmExtension SubSystem;
    double wantedPosition;
    double Tolerance = constants.PlasmaExtension.MoveTolerance;
    
    public C_ExtendToPosition(ArmExtension subSys, double wantedposition){
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

    @Override
    public void end(boolean interrupted) {
        SubSystem.BRAKE();
    }

   

}
