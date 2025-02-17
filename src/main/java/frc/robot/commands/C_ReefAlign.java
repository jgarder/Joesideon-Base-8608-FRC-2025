package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.constants;
import frc.robot.AlphaBots.AprilTag;
import frc.robot.AlphaBots.AprilTag.TagType;
import frc.robot.subsystems.AprilTagManager;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class C_ReefAlign extends Command{
    
    AprilTagManager ATMan;
    CommandSwerveDrivetrain drivetrain;
    IntSupplier AlignOnLeft = ()->{return 1;}; //0 left, 1 center, 2 right
    public C_ReefAlign(CommandSwerveDrivetrain _drivetrain,AprilTagManager _ATMan,IntSupplier _AlignOnLeft ){
        ATMan = _ATMan;
        drivetrain =_drivetrain;
        AlignOnLeft = _AlignOnLeft;
    }
    Command m_command;
    @Override
    public void initialize() {
        AprilTag targetTag = ATMan.getClosestTagofTypeToRobotCenterForAlliance(drivetrain.getState().Pose, TagType.Reef);
        Pose2d locationToAlignTo = ATMan.getPose2DStraightLocTranslation(targetTag,AprilTagManager.robotmetersdistToCenter);
        switch (AlignOnLeft.getAsInt()) {
            case 0://left
            locationToAlignTo = ATMan.getOffSet90Loc(targetTag,AprilTagManager.robotmetersdistToCenter,constants.ReefWidthCenteronCenter,true);
                break;
            case 1://center is already the default. 
                break;
            case 2://right
            locationToAlignTo = ATMan.getOffSet90Loc(targetTag,AprilTagManager.robotmetersdistToCenter,constants.ReefWidthCenteronCenter,false);
                break;
            default:
                break;
        }
        
        m_command =  new C_Align(locationToAlignTo);
 
        if (m_command != null) {
            m_command.schedule();
            
        }
    }

    @Override
    public void end(boolean interrupted) {
        if (m_command != null) {
            m_command.cancel();
        }
        
    }
    @Override
    public boolean isFinished() {
        if (m_command != null) {
            if(m_command.isFinished()){
            return true;
            }
            else{
                return false;
            }
        }
        else{
            return true;

        }
        }
}

