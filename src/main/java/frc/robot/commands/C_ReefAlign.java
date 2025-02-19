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
import frc.robot.subsystems.MantaState;

public class C_ReefAlign extends Command{
    
  
    //AprilTagManager ATMan;
    CommandSwerveDrivetrain drivetrain;
    IntSupplier AlignOnLeft = ()->{return 1;}; //0 left, 1 center, 2 right
    public C_ReefAlign C_ReefAlign(CommandSwerveDrivetrain _drivetrain){IntSupplier jake = ()->{return 1;}; return new C_ReefAlign(_drivetrain,jake);}//this is for algae alignment. always lines up middle. 
    public C_ReefAlign(CommandSwerveDrivetrain _drivetrain,IntSupplier _AlignOnLeft ){
        //ATMan = _ATMan;
        drivetrain =_drivetrain;
        AlignOnLeft = _AlignOnLeft;
    }
    Command m_command;
    @Override
    public void initialize() {

        AprilTag targetTag = AprilTagManager.getClosestTagofTypeToRobotCenterForAlliance(drivetrain.getState().Pose, TagType.Reef);
        Pose2d locationToAlignTo = AprilTagManager.getPose2DStraightLocTranslation(targetTag,0.0);
        switch (AlignOnLeft.getAsInt()) {
            case 0://left
            locationToAlignTo = AprilTagManager.getOffSet90Loc(targetTag,0.0,constants.ReefWidthCenterOffset,true);
                break;
            case 1://center is already the default. 
                break;
            case 2://right
            locationToAlignTo = AprilTagManager.getOffSet90Loc(targetTag,0.0,constants.ReefWidthCenterOffset,false);
                break;
            default:
                break;
        }
        MantaState.NT_AlignSetpoint.set(locationToAlignTo);
        m_command =  new C_Align(locationToAlignTo);//align to pose2d provided above. 
 
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

