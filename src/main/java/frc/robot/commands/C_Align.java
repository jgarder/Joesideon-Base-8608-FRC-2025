package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.MantaState;
import frc.robot.LimelightHelpers;
import frc.robot.constants;
import frc.robot.LimelightHelpers.PoseEstimate;

public class C_Align extends Command{
  //Get ClassName to help network tables auto sort by creating a sub Table with the same name.
  String className = this.getClass().getSimpleName();
  public final Timer TimeToAlignTimer = new Timer();
  public final Timer SettleDebounceTimer = new Timer();
  private final double debounceSecondsNeeded = .02;
  public final frc.robot.subsystems.CommandSwerveDrivetrain drivetrain = MantaState.DriveTrain;
  

  private final PIDController AlignXPid = new PIDController(constants.drivetrainThings.k_PoseX_P,constants.drivetrainThings.k_PoseX_I,constants.drivetrainThings.k_PoseX_D);
  private final PIDController AlignYPid = new PIDController(constants.drivetrainThings.k_PoseY_P,constants.drivetrainThings.k_PoseY_I,constants.drivetrainThings.k_PoseY_D);
  private final PIDController AlignRZPid = new PIDController(constants.drivetrainThings.k_RZ_P,constants.drivetrainThings.k_RZ_I,constants.drivetrainThings.k_RZ_D);

  private DoubleSupplier yAlignOverride;
  Alliance allianceOnInit;//DriverStation.getAlliance().get();

  public double MaxSpeedPercent = 1.0;//9; //6; // 6 meters per second desired top speed
  public double MaxAngularRatePercent = 1.0; //2.5 // 3/4 of a rotation per second max angular velocity

  double maxYvelocity = constants.drivetrainThings.maxYvelocity;
  double maxXvelocity = constants.drivetrainThings.maxXvelocity;
  double maxRZvelocity = constants.drivetrainThings.maxRZvelocity;

  double minXposeErrorToCorrect = constants.drivetrainThings.minXposeErrorMetersToCorrect; //.03175 Meters error is 1.25"
  double minYposeErrorToCorrect = constants.drivetrainThings.minYposeErrorMetersToCorrect;
  double minRZDegreesErrorToCorrect = constants.drivetrainThings.minRZErrorToCorrect;

  Pose2d CurrentPose;//this is our latest position according to our chassis odometry
  Pose2d TargetPose;//this is where we wnt to go in field space coords X,y,Rotation
  Pose2d PoseOffset;//This is how far we are from where we want to be. this is CurrentPose minus TargetPose.
  //double xyMirrorRed; //our drivetrain auto flips itself when we are on red. so we have to aswell. 
  public C_Align(double _minXposeErrorToCorrect,double _minYposeErrorToCorrect,double _minRZDegreesErrorToCorrect,Pose2d PosePositionGoal, DoubleSupplier _yAlignOverride){
    minXposeErrorToCorrect = _minXposeErrorToCorrect;
    minYposeErrorToCorrect = _minYposeErrorToCorrect;
    minRZDegreesErrorToCorrect = _minRZDegreesErrorToCorrect;
    yAlignOverride = _yAlignOverride;
    SetupAlign(PosePositionGoal);
  }
  public C_Align(Pose2d PosePositionGoal, DoubleSupplier _yAlignOverride){
    yAlignOverride = _yAlignOverride;
    SetupAlign(PosePositionGoal);
  }
  public C_Align(Pose2d PosePositionGoal){
      SetupAlign(PosePositionGoal);
  }

  private void SetupAlign(Pose2d PosePositionGoal) {
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
    addRequirements(drivetrain);
  }
  
  @Override
  public void initialize() {
      allianceOnInit = DriverStation.getAlliance().get();
      //xyMirrorRed = (allianceOnInit == Alliance.Blue) ? 1.0:-1.0;
      CurrentPose = drivetrain.getState().Pose;    
      MantaState.NT_AlignSetpoint.set(TargetPose); 
      //setposeoffsets();
      AlignXPid.reset();
      AlignYPid.reset();
      AlignRZPid.reset();
      TimeToAlignTimer.restart();
  }

  @Override
  public void execute() {
      setposeoffsets();
      //PID
      double RZAdjust = AlignRZPid.calculate(PoseOffset.getRotation().getDegrees());
      double xpose_adjust = AlignXPid.calculate(CurrentPose.getX());//GetXPoseAdjust(XP_buffer, min_xpose_command);
      double Ypose_adjust = AlignYPid.calculate(CurrentPose.getY());
      //if we have a y Override, feed that override In, instead of the Pose. this removes all Y adjustment. 
      if(yAlignOverride != null)
      {
        Ypose_adjust = yAlignOverride.getAsDouble();
      }
      //GetYPoseAdjust(YP_buffer, min_Ypose_command );    
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

  // If "isfinished" end true OR if we cancel this command for some reason. 
  // end runs we need some actions to happen no matter how this command is cancelled. 
  @Override
  public void end(boolean interrupted) {
    StopDriveTrain();
    MantaState.NT_AlignedUsing.set("End Align");
    MantaState.NT_TimeToAlign.set(TimeToAlignTimer.get());  
  }

  @Override
  public boolean isFinished(){
    if(PoseOffset == null){System.err.println("No pose OFFSET! Broken CODE?"); return false;}
      
    boolean isAtSetPos = getisAtSetPos();

    if(isAtSetPos){
      if(SettleDebounceTimer.get() > debounceSecondsNeeded)
        {
        //Stop movement if we are there.
        StopDriveTrain();
        //
        return true;
        }             
    }
    else{
        SettleDebounceTimer.restart();
    }
    return false;
      
  }

  public void MoveRobotToTargetInFieldCoordinates(double YposeAxis, double XposeAxis, double RZposeAxis) {
    //var xyMirrorRed = (DriverStation.getAlliance().get() == Alliance.Blue) ? 1.0:-1.0; //our drivetrain auto flips itself when we are on red. so we have to aswell. 
    drivetrain.setControl(constants.drivetrainThings.FCdriveAuton
        .withVelocityX(XposeAxis * MaxSpeedPercent ) // * xyMirrorRed Drive forward with // negative Y (forward)
        .withVelocityY(YposeAxis * MaxSpeedPercent ) // * xyMirrorRed Drive left with negative X (left)
        .withRotationalRate(RZposeAxis * MaxAngularRatePercent) // Drive counterclockwise with negative X (left)
    );
  }
  private void setposeoffsets() {
    if(CurrentPose == null){System.err.println("CurrentPose missing"); return;}
    if(CurrentPose == null){System.err.println("TargetPose missing");  return;}
    //System.out.println("Running setposeoffsets");
    //get position

    PoseEstimate LimelightMt1 = null;
    LimelightMt1 = SelectBestLLPose();

    //double TagdistMaxMeters = 6;
    boolean shoulduseLLMT1Pose = LimelightMt1 !=null && LimelightMt1.tagCount > 0;// & LimelightMt1.avgTagDist < TagdistMaxMeters;
    if(shoulduseLLMT1Pose){
      CurrentPose = LimelightMt1.pose;
    }
    else
    {
      CurrentPose = drivetrain.getState().Pose;
      MantaState.NT_AlignedUsing.set("chassisPose");
    }
    
    //get offsets
    //SUBTRACT where we need to go, from where we are. this will give us the translations we need to make 
    double Xpose_Offset = CurrentPose.getX() - TargetPose.getX();
    double Ypose_Offset = CurrentPose.getY() - TargetPose.getY();             
    Rotation2d RZ_Offset2 = CurrentPose.getRotation().minus(TargetPose.getRotation());
    
    PoseOffset = new Pose2d(Xpose_Offset, Ypose_Offset, RZ_Offset2);
    if(PoseOffset == null){System.err.println("No pose OFFSET created");}
  }

  private PoseEstimate SelectBestLLPose() {
    PoseEstimate frontLimelightMt1 =  LimelightHelpers.getBotPoseEstimate_wpiBlue(constants.CanBus.limelightFrontName);
    PoseEstimate backLimelightMt1 =  LimelightHelpers.getBotPoseEstimate_wpiBlue(constants.CanBus.limelightBackName);
    PoseEstimate LimelightMt1 = null;
    if(backLimelightMt1 != null && backLimelightMt1.tagCount > 0)//if we have a back shot
    {
      if(frontLimelightMt1 !=null && frontLimelightMt1.tagCount > 0)//if we also have a front shot
      {
        if (Math.abs(frontLimelightMt1.avgTagDist) > Math.abs(backLimelightMt1.avgTagDist)) { //if our back shots are closer than the front just use the back instead of defautl front. 
          LimelightMt1 = backLimelightMt1;
          MantaState.NT_AlignedUsing.set("BackLimelightCloser");
        }
      }
      else//no front shot? just use back shot. 
      {
        LimelightMt1 = backLimelightMt1;
        MantaState.NT_AlignedUsing.set("BackLimelightNoFront");
      }
    }
    else{
      MantaState.NT_AlignedUsing.set("FrontLimelightNoBack");
      LimelightMt1 = frontLimelightMt1;
    }
    return LimelightMt1;
  }
  
  public void StopDriveTrain() {drivetrain.setControl(constants.drivetrainThings.StopDrivetrain);}

  private boolean IsXInTarget() {
    return Math.abs(PoseOffset.getX()) < minXposeErrorToCorrect;
  }

  private boolean IsYInTarget() {
    if(yAlignOverride != null)
    {
        return true;
    }
    return Math.abs(PoseOffset.getY()) < minYposeErrorToCorrect;
  }

  private boolean isRotInTarget() {
    return Math.abs(PoseOffset.getRotation().getDegrees()) < minRZDegreesErrorToCorrect;
  }

  private boolean getisAtSetPos() {
    boolean Xok = IsXInTarget();
    boolean Yok = IsYInTarget();
    boolean Zok = isRotInTarget();

    MantaState.NT_Xok.set(Xok);
    MantaState.NT_Yok.set(Yok);
    MantaState.NT_Zok.set(Zok);

    boolean isatSetpos = Xok && Yok  && Zok;
    return isatSetpos;
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
