package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale.IsoCountryCode;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.constants;
import frc.robot.LimelightHelpers.RawDetection;
import frc.robot.constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class gamepiecePoseEstimator extends SubsystemBase {
    //Get ClassName to help network tables auto sort by creating a sub Table with the same name.
    String className = this.getClass().getSimpleName();

    //@AutoLogOutput
    private ArrayList<gamePiece> gamePiecePositions = new ArrayList<>();

    private int amountOfTargets;

    private CommandSwerveDrivetrain drive;
    private Pose2d robotPoseMeters;

    // distance from the center of the Limelight lens to the floor
    private double limelightLensHeightInches = 11.614; 

    // distance from the target to the floor
    //im assuming the height should be the distance between the center of the object and the floor
    private double coralHeightInches = 2.0; 
    //these need to be seperate since there is no mechanism to determine the height of the game piece
    //assuming coral for right now
    private double algaeHeightInches = 8.0;

    // how many degrees back is your limelight rotated from perfectly vertical?
    private double limelightMountAngleDegrees = -5.3;
    
    private double angleToGamePieceRadians;
    double distanceFromLimelightToGamePieceInches;

    private double inchesToMeters = 0.0254;

    //in Inches because of Distance to limelight
    private double randomToleranceInches = .5;

    //depends on how the limelight is mounted, but this should equal the yaw set in the limelight website settings thing
    private double limelightRotationDegrees = 14.385;

    private double limelightFOV = 80;

    /**
     * Sets up the Pose estimator for estimating the 2d Pose of an object
     * @param CommandSwerveDrivetrain CTRE Generated Swerve
     */
    public gamepiecePoseEstimator(CommandSwerveDrivetrain drivetrain){
        drive = drivetrain; 
    }

    private class gamePiece{
        public int typeOfObject;
        public Translation2d gamePiecePose;
        //public double limelightDistance;

        /**
         * Stores the game piece data
         * @param Translation2d the game Piece's Position on the field
         * @param Double the distance from the limelight before the pose calculation
         */
        private gamePiece(int typeOfObject, Translation2d gamePiecePose){
            this.typeOfObject = typeOfObject;
            this.gamePiecePose = gamePiecePose;
           // this.limelightDistance = distanceToLimelight;
        }
    }

    @Override
    public void periodic(){
         /*Another thing i thought about that i need to fix, for a future creation of 3d pose estimation, it has a chicken and egg issue where the height/ "altitude" of the 
          * game piece is needed to calculate where it is on a 2d plane, which is fine for 2d pose estimation since it's assumed to be on the floor, 
          * but if you dont have the height of where it is, you need to find it. ok, so just calculate the height first, 
          * but you need the distance to make sure the height that you calculated is accurate, and you can't get the distance without knowing the height of the piece!
          */
        /*
         * you can also calculate the rotation of a piece because the limelight provides "corners" of the detected box, 
         * and you can determine it's rotation based on how the box is affected 
         * (only to ~180 degrees though, but that should be all you need for something like coral, and is completely unnecessary for something like a sphere)
         */

        //first it clears all visible piece positions
        for(int i = 0; i <= gamePiecePositions.size(); i++){
            if(gamePiecePositions.size() <= 1){
                break;
            }
            if(isPoseVisible(gamePiecePositions.get(i).gamePiecePose) == true){
                gamePiecePositions.remove(i);
            }
        }

        robotPoseMeters = drive.getState().Pose;
        RawDetection[] detectedGamePieceData = LimelightHelpers.getRawDetections(constants.CanBus.limelightFrontName);
        amountOfTargets = detectedGamePieceData.length;

        /*then adds the pieces the limelight detects
        I'd rather not see a gamepiece, then hallucinate a gamepiece that's not there
        this prevents that from happening, by just clearing all pieces and just re-adding the ones it can see for certain.
        it's also a very simple, reliable way of doing it
        */
        for(int i = 0; i < amountOfTargets; i++){
            if(amountOfTargets == 0){break;}
            if(i > 0){
                gamePiece currentPiece = new gamePiece(detectedGamePieceData[i].classId, calculate2dPose(detectedGamePieceData[i].txnc, detectedGamePieceData[i].tync));
                Logger.recordOutput(className + "/" + "detectedPiece " + i, currentPiece.gamePiecePose);
                gamePiecePositions.add(currentPiece);
            }
        }
    }

    

    public boolean isPoseVisible(Translation2d checkedPose){
        /*the camera's FOV is in degrees, so you can half use polar coordinates to decide if a coral's position is visible to the camera or not
         * to decide whether to overwrite a coral's position or to leave it alone.
         * For example, if a known coral position is x:1 y:1 and the robot can't see that position, it doesn't overwrite the coral that has that position in the array
         * but if a known coral position is x:0 y:2 and it CAN see that position and it doesn't detect a coral it will delete the coral object in the array, 
         * or if there's a coral that it sees nearby instead, it will just replace the pose with that of the coral it sees nearby
         * the final implementation looks a little different
         */
        //takes the robot's rotation and adds how rotated the limelight is compared to the front of the robot to find the angle at which the tolerance is applied to
        double visibleAngle = robotPoseMeters.getRotation().getDegrees() + limelightRotationDegrees;
        //uses the formula to calculate theta for polar coordinates to determine what angle the checked pose is compared to the robot
        //this requires the robot pose to be thrown in the equation as well since otherwise you get the angle relative to the field instead of the robot
        double checkedPoseAngle = Math.atan2(checkedPose.getY() - robotPoseMeters.getY(), checkedPose.getX() - robotPoseMeters.getX());

        //limelight fov needs to be halved, hard to describe, easy to visualize
        //this takes the robot's angle and the angle of the checked pose and applies a tolerance 
        //which is the FOV in degrees of the camera to determine if the checkedpose would be considered visible
        return MathUtil.isNear(visibleAngle, checkedPoseAngle, limelightFOV/2);
    }
    
    /**
     * checks if a Pose is within a set Tolerance of another pose
     *
     * @return True or False: if Checkedpose is close to checkingPose
     */
    public boolean comparePositions(Translation2d checkingPose, Translation2d checkedPose, double Tolerance){
        double checkingPoseX = checkingPose.getX();
        double checkingPoseY = checkingPose.getY();

        double checkedPoseX = checkedPose.getX();
        double checkedPoseY = checkedPose.getY();


        boolean isXClose = MathUtil.isNear(checkingPoseX, checkedPoseX, Tolerance);
        boolean isYClose = MathUtil.isNear(checkingPoseY, checkedPoseY, Tolerance);

        if(isXClose && isYClose){return true;}else{return false;}
    }


    /**
     * calculates the Pose of the Game Piece by taking the Pose and Rotation of the robot, and the position of the game piece on the camera's screen and returns a 2d Pose of the game piece
     * @param txnc / tx
     * @param tync / ty
     * @return Translation2d
     */
    private Translation2d calculate2dPose(double txnc, double tync){
        //the difference in angle from where the camera is pointing to where the piece is on the screen
        angleToGamePieceRadians = Math.toRadians(limelightMountAngleDegrees + tync);

        //calculate distance from the piece to the limelight
        distanceFromLimelightToGamePieceInches = (coralHeightInches - limelightLensHeightInches) / Math.tan(angleToGamePieceRadians);

        /*tx affects the rotation, for example a limelight is facing at a 45 degree angle towards the ground and is perfectly centered on the axis of rotation, 
        (it works fine if the limelight isn't centered its just easier to understand if you assume it is (because if it isn't centered ty SHOULD change since it actually is changing distance as it rotates))
        now imagine your looking at the camera stream, if you rotate the limelight, where the object is vertically (ty) wont change, but it will horizontally (tx)*/
        Double calculatedTargetRotation = Math.toRadians(robotPoseMeters.getRotation().getDegrees() + txnc);

        //seperates the X position and Y position of the object depending on the rotation of the robot
        //basically it converts the distance from the limelight to field coordinates using the robot's pose as reference
        double calculatedTargetPoseX = (robotPoseMeters.getX() + (inchesToMeters * distanceFromLimelightToGamePieceInches * Math.cos(calculatedTargetRotation)));
        double calculatedTargetPoseY = (robotPoseMeters.getY() + (inchesToMeters * distanceFromLimelightToGamePieceInches * Math.sin(calculatedTargetRotation)));

        //adds together all of the previous calculations into a single Translation2d
        Translation2d calculatedTargetPose = new Translation2d(calculatedTargetPoseX, calculatedTargetPoseY);
        return calculatedTargetPose;
    }


    /**
     * This needs to change to be based on the piece's pose and not limelightdistance
     *
     * @return Returns Translation2d
     */
    // public Translation2d getClosestGamePiecePose(int typeOfObject){
    //     double currentCheckedDistance;
    //     //shortestDistance works fine if it's zero, since the if statement that checks if the currentCheckedDistance is closer or not 
    //     //has a case where it can increase the shortest distance because of the "|| i = 0" part
    //     double shortestDistance = 0;
    //     int closestGamePiece = 0;

    //     for(int i = 0; i < gamePiecePositions.size(); i++){
    //         if(gamePiecePositions.get(i).typeOfObject == typeOfObject){
    //             currentCheckedDistance = gamePiecePositions.get(i).limelightDistance;
            
    //             if(currentCheckedDistance < shortestDistance || i == 0 ){
    //                 shortestDistance = currentCheckedDistance;
    //                 closestGamePiece = i;
    //             }
    //         }  
    //     }

    //     return gamePiecePositions.get(closestGamePiece).gamePiecePose;
    // }

}