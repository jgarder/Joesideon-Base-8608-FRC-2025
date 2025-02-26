package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants;
import frc.robot.constants.PlasmaExtension;
import frc.robot.subsystems.ArmExtension;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.MantaState;

public class C_ExtendToPosition extends Command{
    ArmExtension SubSystem;
    double wantedPosition;
    double Tolerance = constants.PlasmaExtension.MoveTolerance;
    double Liveoffset = 0;
    double wantedPositionWithOffset = 0;
    public C_ExtendToPosition(ArmExtension subSys, double wantedposition){
        SubSystem = subSys;
        wantedPosition = wantedposition;
        addRequirements(subSys);
    }

    @Override
    public void initialize() {
        Liveoffset = constants.PlasmaExtension.LiveOffset;
        double tempnewwantedPosition =  wantedPosition + Liveoffset;
        wantedPositionWithOffset = MathUtil.clamp(tempnewwantedPosition,constants.PlasmaExtension.minposition,constants.PlasmaExtension.maxposition);
        SubSystem.GotoPosition(wantedPositionWithOffset);
        MantaState.NT_ExtensionPosOk.set(false);
    }

    @Override
    public boolean isFinished() {
        
        boolean isatPosition = frc.robot.AlphaBots.Tools.isPosAtSetpoint(SubSystem.getPosition(), wantedPositionWithOffset, Tolerance);
        MantaState.NT_ExtensionPosOk.set(isatPosition);
        return isatPosition;
    }

    @Override
    public void end(boolean interrupted) {
        SubSystem.BRAKE();
    }

   

}
