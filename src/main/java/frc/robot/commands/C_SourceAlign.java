package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.constants;
import frc.robot.AlphaBots.AprilTag;
import frc.robot.AlphaBots.AprilTag.TagType;
import frc.robot.subsystems.AprilTagManager;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.MantaState;

public class C_SourceAlign extends Command{
    
  
    //AprilTagManager ATMan;
    CommandSwerveDrivetrain drivetrain;
    IntSupplier AlignOnLeft = ()->{return 1;}; //0 left, 1 center, 2 right
    public C_SourceAlign(CommandSwerveDrivetrain _drivetrain){
        IntSupplier jake = ()->{return 1;};  
        drivetrain =_drivetrain;
        AlignOnLeft = jake;}//this is for auton alignment. always lines up middle. 
    public C_SourceAlign(CommandSwerveDrivetrain _drivetrain,IntSupplier _AlignOnLeft ){
        drivetrain =_drivetrain;
        AlignOnLeft = _AlignOnLeft;
    }
    C_Align m_command;
    @Override
    public void initialize() {

        AprilTag targetTag = AprilTagManager.getClosestTagofTypeToRobotCenterForAlliance(drivetrain.getState().Pose, TagType.Source);
        Pose2d locationToAlignTo = AprilTagManager.getPose2DStraightLocTranslation(targetTag,0.0);
        switch (AlignOnLeft.getAsInt()) {
            case 0://left
            locationToAlignTo = AprilTagManager.getOffSet90Loc(targetTag,0.0,constants.SourcePickupWidthCenterOffset,true);
                break;
            case 1://center is already set. should only happen during auton. 
                break;
            case 2://right
            locationToAlignTo = AprilTagManager.getOffSet90Loc(targetTag,0.0,constants.SourcePickupWidthCenterOffset,false);
                break;
            default:
                break;
        }
        //locationToAlignTo = locationToAlignTo.rotateBy(Rotation2d.fromDegrees(180)); rotates around origin? 0,0? wtf?
        locationToAlignTo = new Pose2d(locationToAlignTo.getX(), locationToAlignTo.getY(), Rotation2d.fromDegrees(locationToAlignTo.getRotation().getDegrees() +180));
        MantaState.NT_AlignSetpoint.set(locationToAlignTo);
        if(locationToAlignTo == null){System.out.println("NO LOCATION TO ALIGN TO!"); return;}
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
    public int PoseOffsetErrors = 0;
    @Override
    public boolean isFinished() {
        if (m_command != null) {
            if (m_command.PoseOffset == null){PoseOffsetErrors++; System.err.println("No pose OFFSET! Broken CODE? ERRRCOUNT: " + PoseOffsetErrors); return false;}
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

