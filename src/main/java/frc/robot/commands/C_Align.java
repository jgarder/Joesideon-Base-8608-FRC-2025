package frc.robot.commands;

import java.util.Optional;

import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.google.flatbuffers.Constants;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.MantaState;
import frc.robot.RobotContainer;
import frc.robot.constants;
//import frc.robot.AlphaBots.CommandSwerveDrivetrain;
import frc.robot.AlphaBots.AprilTag;
import frc.robot.AlphaBots.NT;

public class C_Align extends Command{
    //Get ClassName to help network tables auto sort by creating a sub Table with the same name.
    String className = this.getClass().getSimpleName();

    public final frc.robot.subsystems.CommandSwerveDrivetrain drivetrain = MantaState.DriveTrain;
    public final SwerveRequest.FieldCentric FCdriveAuton = new SwerveRequest.FieldCentric();

    private final PIDController AlignXPid = new PIDController(constants.drivetrainThings.k_PoseX_P,constants.drivetrainThings.k_PoseX_I,constants.drivetrainThings.k_PoseX_D);
    private final PIDController AlignYPid = new PIDController(constants.drivetrainThings.k_PoseY_P,constants.drivetrainThings.k_PoseY_I,constants.drivetrainThings.k_PoseY_D);
    private final PIDController AlignRZPid = new PIDController(constants.drivetrainThings.k_RZ_P,constants.drivetrainThings.k_RZ_I,constants.drivetrainThings.k_RZ_D);



    public double MaxSpeedPercent = 1.0;//9; //6; // 6 meters per second desired top speed
    public double MaxAngularRatePercent = 1.0; //2.5 // 3/4 of a rotation per second max angular velocity

    double maxYvelocity = constants.drivetrainThings.maxYvelocity;
    double maxXvelocity = constants.drivetrainThings.maxXvelocity;
    double maxRZvelocity = constants.drivetrainThings.maxRZvelocity;

    double minXposeErrorToCorrect = constants.drivetrainThings.minXposeErrorMetersToCorrect; //.03175 Meters error is 1.25"
    double minYposeErrorToCorrect = constants.drivetrainThings.minYposeErrorMetersToCorrect;
    double minRZErrorToCorrect = constants.drivetrainThings.minRZErrorToCorrect;

    Pose2d CurrentPose;//this is our latest position according to our chassis odometry
    Pose2d TargetPose;//this is where we wnt to go in field space coords X,y,Rotation
    Pose2d PoseOffset;//This is how far we are from where we want to be. this is CurrentPose minus TargetPose.

    public C_Align(AprilTag PosePositionGoal){new C_Align(PosePositionGoal.Pose);}
    public C_Align(Pose2d PosePositionGoal){
        TargetPose = PosePositionGoal;
        AlignXPid.setSetpoint(TargetPose.getX());
        AlignYPid.setSetpoint(TargetPose.getY());
        AlignRZPid.setSetpoint(0);


        //when we startup an alignment pull the latest numbers to try from the user.
        MantaState.NT_XPGain.set(MantaState.NT_XPGain.getAsDouble());
        MantaState.NT_XIGain.set(MantaState.NT_XIGain.getAsDouble());
        MantaState.NT_XDGain.set(MantaState.NT_XDGain.getAsDouble());
        MantaState.NT_ZPGain.set(MantaState.NT_ZPGain.getAsDouble());
        MantaState.NT_ZIGain.set(MantaState.NT_ZIGain.getAsDouble());
        MantaState.NT_ZDGain.set(MantaState.NT_ZDGain.getAsDouble());
    }
    Alliance allianceOnInit;//DriverStation.getAlliance().get();
    @Override
    public void initialize() {
        allianceOnInit = DriverStation.getAlliance().get();     
        setposeoffsets();
        AlignXPid.reset();
        AlignYPid.reset();
        AlignRZPid.reset();
    }

    @Override
    public void execute() {
        setposeoffsets();
        //PID
        double RZAdjust = AlignRZPid.calculate(PoseOffset.getRotation().getDegrees());
        double xpose_adjust = AlignXPid.calculate(CurrentPose.getX());//GetXPoseAdjust(XP_buffer, min_xpose_command);
        double Ypose_adjust = AlignYPid.calculate(CurrentPose.getY());//GetYPoseAdjust(YP_buffer, min_Ypose_command );    
        //drive drive drivetrain with PID clamped something to not go crazy or something
        //clamp all results to a max (and negative max) top speed
        Ypose_adjust = MathUtil.clamp(Ypose_adjust, -maxYvelocity, maxYvelocity);
        xpose_adjust = MathUtil.clamp(xpose_adjust, -maxXvelocity, maxXvelocity);
        RZAdjust = MathUtil.clamp(RZAdjust, -maxRZvelocity, maxRZvelocity);

        //MOVE!!!
        MoveRobotToTargetInFieldCoordinates(Ypose_adjust, xpose_adjust, RZAdjust);

        //tune:
        PidTune();
        PidTuneRotation();
    }

    private void setposeoffsets() {
      //get position
      CurrentPose = drivetrain.getState().Pose;
      //get offsets
      //SUBTRACT where we need to go, from where we are. this will give us the translations we need to make 
      double Xpose_Offset = CurrentPose.getX() - TargetPose.getX();
      double Ypose_Offset = CurrentPose.getY() - TargetPose.getY();             
      Rotation2d RZ_Offset2 = CurrentPose.getRotation().minus(TargetPose.getRotation());
      PoseOffset = new Pose2d(Xpose_Offset, Ypose_Offset, RZ_Offset2);
    }

    @Override
    public boolean isFinished(){
        //near the final positon x
        //near the final positon y
        //near the final positon z (rotation)
        //stop driving
     
    
        if(IsXInTarget() && IsYInTarget()  && isRotInTarget()){
            //timesgood = 0;
            //Stop movement if we are there.
            StopDriveTrain();
            //
            return true;

        }
        else{
            return false;

        }
        
    }

    public void MoveRobotToTargetInFieldCoordinates(double YposeAxis, double XposeAxis, double RZposeAxis) {
      
      var xyMirrorRed = (DriverStation.getAlliance().get() == Alliance.Blue) ? 1.0:-1.0; //our drivetrain auto flips itself when we are on red. so we have to aswell. 

        drivetrain.setControl(FCdriveAuton
            .withVelocityX(XposeAxis * MaxSpeedPercent * xyMirrorRed) // Drive forward with // negative Y (forward)
            .withVelocityY(YposeAxis * MaxSpeedPercent * xyMirrorRed) // Drive left with negative X (left)
            .withRotationalRate(RZposeAxis * MaxAngularRatePercent) // Drive counterclockwise with negative X (left)
        );
      }
      public void StopDriveTrain() {
        drivetrain.setControl(FCdriveAuton.withVelocityX(0 * MantaState.getmaxspeed()) // Drive forward with // negative Y (forward)
        .withVelocityY(0 * MantaState.getmaxspeed()) // Drive left with negative X (left)
        .withRotationalRate(0 * MantaState.getmaxAngularRate()) // Drive counterclockwise with negative X (left)
        );
      }

      private boolean IsXInTarget() {
        return Math.abs(PoseOffset.getX()) < minXposeErrorToCorrect;
      }
    
      private boolean IsYInTarget() {
        return Math.abs(PoseOffset.getY()) < minYposeErrorToCorrect;
      }
    
      private boolean isRotInTarget() {
        return Math.abs(PoseOffset.getRotation().getDegrees()) < minRZErrorToCorrect;
      }

      //This is used with the SmartDashboard to Tune the PID. Unneeded for competition.
  private void PidTune() {
    double p = MantaState.NT_XPGain.getAsDouble();
    double i = MantaState.NT_XIGain.getAsDouble();
    double d = MantaState.NT_XDGain.getAsDouble();
    
      
    if((p != AlignXPid.getP())) { AlignXPid.setP(p); constants.drivetrainThings.k_PoseX_P = p;}
    if((i != AlignXPid.getI())) { AlignXPid.setI(i); constants.drivetrainThings.k_PoseX_I = i;}
    if((d != AlignXPid.getD())) { AlignXPid.setD(d); constants.drivetrainThings.k_PoseX_D = d;}

    if((p != AlignYPid.getP())) { AlignYPid.setP(p); constants.drivetrainThings.k_PoseY_P = p;}
    if((i != AlignYPid.getI())) { AlignYPid.setI(i); constants.drivetrainThings.k_PoseY_I = i;}
    if((d != AlignYPid.getD())) { AlignYPid.setD(d); constants.drivetrainThings.k_PoseY_D = d;}
  }
  private void PidTuneRotation() {
    double p = MantaState.NT_ZPGain.getAsDouble();
    double i = MantaState.NT_ZIGain.getAsDouble();
    double d = MantaState.NT_ZDGain.getAsDouble();
    
      
    if((p != AlignRZPid.getP())) { AlignRZPid.setP(p); constants.drivetrainThings.k_RZ_P = p;}
    if((i != AlignRZPid.getI())) { AlignRZPid.setI(i); constants.drivetrainThings.k_RZ_I = i;}
    if((d != AlignRZPid.getD())) { AlignRZPid.setD(d); constants.drivetrainThings.k_RZ_D = d;}

  }
    
}
