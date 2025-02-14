package frc.robot.commands;

import java.util.Optional;

import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;



import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.MantaState;
import frc.robot.RobotContainer;
import frc.robot.constants;
//import frc.robot.AlphaBots.CommandSwerveDrivetrain;

public class C_Align extends Command{
    public final frc.robot.subsystems.CommandSwerveDrivetrain drivetrain = MantaState.DriveTrain;
    public final SwerveRequest.FieldCentric FCdriveAuton = new SwerveRequest.FieldCentric();

    private final PIDController AlignXController = new PIDController(constants.drivetrainThings.k_PoseX_P,constants.drivetrainThings.k_PoseX_I,constants.drivetrainThings.k_PoseX_D);
    private final PIDController AlignPoseYController = new PIDController(constants.drivetrainThings.k_PoseY_P,constants.drivetrainThings.k_PoseY_I,constants.drivetrainThings.k_PoseY_D);
    private final PIDController AlignRZController = new PIDController(constants.drivetrainThings.k_RZ_P,constants.drivetrainThings.k_RZ_I,constants.drivetrainThings.k_RZ_D);


    public double MaxSpeedPid = 9;//6; // 6 meters per second desired top speed
    public double MaxAngularRatePid = 2.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

    double maxYvelocity = constants.drivetrainThings.maxYvelocity;
    double maxXvelocity = constants.drivetrainThings.maxXvelocity;
    double maxRZvelocity = constants.drivetrainThings.maxRZvelocity;

    double minXposeErrorToCorrect = constants.drivetrainThings.minXposeErrorToCorrect; //.03175 Meters error is 1.25"
    double minYposeErrorToCorrect = constants.drivetrainThings.minYposeErrorToCorrect;
    double minRZErrorToCorrect = constants.drivetrainThings.minRZErrorToCorrect;

    Pose2d CurrentPose;//this is our latest position according to our chassis odometry
    Pose2d TargetPose;//this is where we wnt to go in field space coords X,y,Rotation
    Pose2d PoseOffset;//This is how far we are from where we want to be. this is CurrentPose minus TargetPose.


    public C_Align(Pose2d PosePositionGoal){
        TargetPose = PosePositionGoal;
        AlignXController.setSetpoint(TargetPose.getX());
        AlignPoseYController.setSetpoint(TargetPose.getY());
        AlignRZController.setSetpoint(0);
    }
    Alliance allianceOnInit;//DriverStation.getAlliance().get();
    @Override
    public void initialize() {
        allianceOnInit = DriverStation.getAlliance().get();     
        setposeoffsets();
        AlignXController.reset();
        AlignPoseYController.reset();
        AlignRZController.reset();
    }

    @Override
    public void execute() {
        setposeoffsets();
        //PID
        double RZAdjust = AlignRZController.calculate(PoseOffset.getRotation().getDegrees());
        double xpose_adjust = AlignXController.calculate(CurrentPose.getX());//GetXPoseAdjust(XP_buffer, min_xpose_command);
        double Ypose_adjust = AlignPoseYController.calculate(CurrentPose.getY());//GetYPoseAdjust(YP_buffer, min_Ypose_command );    
        //drive drive drivetrain with PID clamped something to not go crazy or something
        //clamp all results to a max (and negative max) top speed
        Ypose_adjust = MathUtil.clamp(Ypose_adjust, -maxYvelocity, maxYvelocity);
        xpose_adjust = MathUtil.clamp(xpose_adjust, -maxXvelocity, maxXvelocity);
        RZAdjust = MathUtil.clamp(RZAdjust, -maxRZvelocity, maxRZvelocity);

        //MOVE!!!
        MoveRobotToTargetInFieldCoordinates(Ypose_adjust, xpose_adjust, RZAdjust);
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
            .withVelocityX(XposeAxis * MaxSpeedPid * xyMirrorRed) // Drive forward with // negative Y (forward)
            .withVelocityY(YposeAxis * MaxSpeedPid * xyMirrorRed) // Drive left with negative X (left)
            .withRotationalRate(RZposeAxis * MaxAngularRatePid) // Drive counterclockwise with negative X (left)
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
    
}
