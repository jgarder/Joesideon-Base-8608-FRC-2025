package frc.robot.AlphaBots;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.networktables.LoggedDashboardBoolean;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.Elastic;
import frc.robot.LimelightHelpers;
import frc.robot.constants;
import frc.robot.Elastic.Notification.NotificationLevel;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.MantaState;

public class LimeLightPoseFilter {

    private static final boolean UseLimelight = true;
    
    private static final double MT1maxrotationalVelocityForLLUpdate = 100;//80;
    private static final double MT1maxSpeedMeterPerSecondForLLUpdate = 1.0;
    private static final double MT2maxrotationalVelocityForLLUpdate = 400;//150;//80;
    private static final double MT2maxSpeedMeterPerSecondForLLUpdate = 5.0;//4.0;
    private static final String FrontLLName = constants.CanBus.limelightFrontName;
    public static boolean BootupRobotOrientationSet = false;
    public static boolean bootupAprilTagError = false;
    
    @AutoLogOutput(key = "Odometry/Robot")
    public static double getTotalspeedvector() {
        double vx = MantaState.DriveTrain.getCurrentRobotChassisSpeeds().vxMetersPerSecond;
        double vy = MantaState.DriveTrain.getCurrentRobotChassisSpeeds().vyMetersPerSecond;
        double totalspeedvector = Math.hypot(Math.abs(vx),Math.abs(vy));
        return totalspeedvector;
    }

    public static LoggedNetworkBoolean Mt1doRejectUpdate = new LoggedNetworkBoolean("/AlphaBots/Mt1doRejectUpdate",false);
    public static LoggedNetworkBoolean Mt2doRejectUpdate = new LoggedNetworkBoolean("/AlphaBots/Mt2doRejectUpdate",false);
    
    public static void limelightupdateDrivetrain(String thislimelight) {
        boolean doRejectUpdate = false;
        if (UseLimelight) {
        double totalspeedvector = getTotalspeedvector();
        double rotationalvelocity = MantaState.DriveTrain.getPigeon2().getAngularVelocityZWorld().getValueAsDouble();  
        LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(thislimelight);
        //
        if(mt1 == null)
        {
            doRejectUpdate = true;
        }
        else
        {
            //these are all the reject update test here!
            if(mt1.tagCount == 1 && mt1.rawFiducials.length == 1)
            {
                //  if(mt1.rawFiducials[0].ambiguity > .7){
                //  doRejectUpdate = true;}
                //  if(mt1.rawFiducials[0].distToCamera > 3){
                //  doRejectUpdate = true;}
            }
            //
            if(mt1.tagCount == 0 || // if no tags are seen OR
            Math.abs(rotationalvelocity) > MT1maxrotationalVelocityForLLUpdate || // if our angular velocity is greater than 720 degrees per second, ignore vision updates OR
            totalspeedvector > MT1maxSpeedMeterPerSecondForLLUpdate // if our floor speed is greater than x meters per second, ignore vision updates, lower for global shutter cameras. 
            ){
                doRejectUpdate = true;
            }
            //
        } 
        
        
        LimeLightPoseFilter.Mt1doRejectUpdate.set(doRejectUpdate);
        if(!doRejectUpdate)
        {
            if(!BootupRobotOrientationSet)
            {
                    resetVision(mt1);
                    Elastic.sendNotification(notification.withDisplaySeconds(120).withDescription(thislimelight + "Tag Found Robot Orientated").withLevel(NotificationLevel.INFO));
            }

            if(DriverStation.isDisabled())
            {
                
                if(BootupRobotOrientationSet & bootupAprilTagError)
                {
                   // CANdleSystem.noAprilTagOnBoot_MightBeOkYellow();    
                }

            }
            LimelightHelpers.SetRobotOrientation(constants.CanBus.limelightFrontName,mt1.pose.getRotation().getDegrees() ,0 , 0, 0, 0, 0);//m_gyro.getRate()
            LimelightHelpers.SetRobotOrientation(constants.CanBus.limelightBackName,mt1.pose.getRotation().getDegrees() ,0 , 0, 0, 0, 0);//m_gyro.getRate()

            //m_robotContainer.drivetrainManager.drivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(.5,.5,9999999));
            MantaState.DriveTrain.addVisionMeasurement(
                mt1.pose,
                mt1.timestampSeconds);
                return;//only 1 sample per robot periodic
        }
        else{
            if(DriverStation.isDisabled() & !BootupRobotOrientationSet)
            {
                
               
                //System.out.println(thislimelight + " Cant see tags from this disabled location!!!");
                //LimelightHelpers.printPoseEstimate(mt1);
                //CANdleSystem.noAprilTagOnBoot_strobeRed();
                if(!bootupAprilTagError)
                {
                    Elastic.sendNotification(notification.withDisplaySeconds(120));
                    NoTagSeenOnBoot.setText(thislimelight + " Cant see tags from this disabled location!!!");
                }
                bootupAprilTagError = true;
  
            }
            
        }    
        }
    }
    public static Elastic.Notification notification = new Elastic.Notification(Elastic.Notification.NotificationLevel.ERROR, FrontLLName +"NoTag", FrontLLName +"NoTag");
    public static Alert NoTagSeenOnBoot = new Alert(FrontLLName +"NoTag", AlertType.kWarning);
    public static void DoResetVision()
    {
        BootupRobotOrientationSet = false;
        bootupAprilTagError = false;
        limelightupdateDrivetrain(FrontLLName);
    }

    public static void resetVision(PoseEstimate mt1)
    {
        if(mt1 == null){return;}
        //MantaState.DriveTrain.seedFieldCentric();
        var m_gyro = MantaState.DriveTrain.getPigeon2();
        m_gyro.setYaw(mt1.pose.getRotation().getDegrees(),5);
        MantaState.DriveTrain.resetPose(new Pose2d(mt1.pose.getX(),mt1.pose.getY(),mt1.pose.getRotation()));//MantaState.DriveTrain.seedFieldRelative(new Pose2d(mt1.pose.getX(),mt1.pose.getY(),mt1.pose.getRotation()));
        //LimelightHelpers.SetRobotOrientation("limelight",mt1.pose.getRotation().getDegrees() ,0 , 0, 0, 0, 0);//m_gyro.getRate()
        LimelightHelpers.SetRobotOrientation(constants.CanBus.limelightFrontName,mt1.pose.getRotation().getDegrees() ,0 , 0, 0, 0, 0);//m_gyro.getRate()
        LimelightHelpers.SetRobotOrientation(constants.CanBus.limelightBackName,mt1.pose.getRotation().getDegrees() ,0 , 0, 0, 0, 0);//m_gyro.getRate()
        
        BootupRobotOrientationSet = true;
    }
    public static void updateOdometry() {
        updateOdometryCamera(constants.CanBus.limelightFrontName);
        updateOdometryCamera(constants.CanBus.limelightBackName);
        //FOR CALIBRATING SECOND CAMERA (also comment line below) : LimelightHelpers.SetRobotOrientation(constants.motorCurrentsAndCanID.frontlimelightName,TunerConstants.DriveTrain.getState().Pose.getRotation().getDegrees() ,0 , 0, 0, 0, 0);//m_gyro.getRate()
        //if(!DriverStation.isAutonomousEnabled()){updateOdometryCamera(constants.motorCurrentsAndCanID.frontlimelightName);}
    }
    public static void updateOdometryCamera(String Thislimelight) {
         var m_poseEstimator = MantaState.DriveTrain;
         var m_gyro = MantaState.DriveTrain.getPigeon2();
         boolean doRejectUpdate = false;

        if (BootupRobotOrientationSet == true)
        {
            LimelightHelpers.SetRobotOrientation(Thislimelight,m_poseEstimator.getState().Pose.getRotation().getDegrees() ,0 , 0, 0, 0, 0);//m_gyro.getRate()
        LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(Thislimelight);
        double totalspeedvector = getTotalspeedvector();
        double rotationSpeed = Math.abs(m_gyro.getAngularVelocityZWorld().getValueAsDouble());// m_gyro.getrate();deprecated in 2026
            if(mt2!=null)
            {
                if(
                    mt2.tagCount == 0 ||
                //mt2.avgTagDist > 3 ||
                rotationSpeed > MT2maxrotationalVelocityForLLUpdate ||
                totalspeedvector > MT2maxSpeedMeterPerSecondForLLUpdate ){ // if our angular velocity is greater than X degrees per second, ignore vision updates
                    doRejectUpdate = true;
                }

                if(!doRejectUpdate)
                {
                    m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999));
                    m_poseEstimator.addVisionMeasurement(
                        mt2.pose,
                        mt2.timestampSeconds);
                }
                LimeLightPoseFilter.Mt2doRejectUpdate.set(doRejectUpdate);
            }
        }
        else
        {
            limelightupdateDrivetrain(FrontLLName);
        }
  }

}
