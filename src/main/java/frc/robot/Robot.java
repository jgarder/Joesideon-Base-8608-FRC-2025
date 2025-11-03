// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LoggedRobot;

import com.ctre.phoenix6.SignalLogger;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.AlphaBots.LimeLightPoseFilter;
import frc.robot.AlphaBots.NT;
import frc.robot.AlphaBots.advantageKitBootstrap;
import frc.robot.subsystems.MantaState;

public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;
  private final NT OurNT = new NT();
  private final advantageKitBootstrap akit = new advantageKitBootstrap(this);
  private final RobotContainer m_robotContainer;

  public Robot() {
    //Pathfinding.setPathfinder(new LocalADStarAK());
    RobotController.setBrownoutVoltage(6.0);//trade battery life for performance, im sure it will be fine.
    akit.startAdvantageKitLogger();//before robot container even boots we log.
    m_robotContainer = new RobotContainer();
    SetupPathplannerLog();
    //PathfindingCommand.warmupCommand().schedule();
    SignalLogger.setPath("/home/lvuser/logs/");
  }

  @Override
  public void robotPeriodic() {
        // Switch thread to high priority to improve loop timing
    Threads.setCurrentThreadPriority(true, 99);

    //copied from last year to be used on the Elastic dashboard
    SmartDashboard.putNumber("MatchTime", DriverStation.getMatchTime());
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled commands, running already-scheduled commands, removing
    // finished or interrupted commands, and running subsystem periodic() methods.
    // This must be called from the robot's periodic block in order for anything in
    // the Command-based framework to work.
    CommandScheduler.getInstance().run();

    // Return to normal thread priority
    Threads.setCurrentThreadPriority(false, 10);
    
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {
    //we need to detect if limelight is enabled. 
     MantaState.NT_Mt1FrontdoRejectUpdate.set(LimeLightPoseFilter.limelightupdateDrivetrain(constants.CanBus.limelightFrontName));
     //MantaState.NT_Mt1BackdoRejectUpdate.set(LimeLightPoseFilter.limelightupdateDrivetrain(constants.CanBus.limelightBackName));
  }

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {LimeLightPoseFilter.updateOdometry();}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {LimeLightPoseFilter.updateOdometry();
    
   if(DriverStation.getMatchTime() < constants.Climber.secondsToClimb){
      MantaState.setToClimbTime();
   }else{}
  }

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}

  /////////////////////////////
  private final Field2d field = new Field2d();
  public void SetupPathplannerLog(){

    SmartDashboard.putData("PPField", field);

    // Logging callback for current robot pose
    PathPlannerLogging.setLogCurrentPoseCallback((pose) -> {
        // Do whatever you want with the pose here
        field.setRobotPose(pose);
    });

    // Logging callback for target robot pose
    PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
        // Do whatever you want with the pose here
        field.getObject("target pose").setPose(pose);
    });

    // Logging callback for the active path, this is sent as a list of poses
    PathPlannerLogging.setLogActivePathCallback((poses) -> {
        // Do whatever you want with the poses here
        field.getObject("path").setPoses(poses);
    });
  }
}
