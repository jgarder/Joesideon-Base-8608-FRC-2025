// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//import static edu.wpi.first.units.Units.*;

import java.util.function.IntSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.C_ClearRearIntake;
import frc.robot.commands.C_ElevateToPosition;
import frc.robot.commands.C_TridentIntake;
import frc.robot.AlphaBots.Tools;
import frc.robot.AlphaBots.AprilTagSystem.SelectCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ChassisTelemetryUpdater;
import frc.robot.subsystems.ArmExtension;
import frc.robot.subsystems.CANdleSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.MantaRay;
import frc.robot.subsystems.MantaState;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.josiahClimber;
import frc.robot.subsystems.superStruture.SuperStructure;
import frc.robot.subsystems.RearIntake;

public class RobotContainer {
    //fields
    
    

    //Subsystem bootup Zone - Order matters.
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final ChassisTelemetryUpdater ATMan = new ChassisTelemetryUpdater(drivetrain); 

    public final MantaRay ss_Trident = new MantaRay();
    public final Elevator ss_Elevator = new Elevator();
    public final Pivot ss_Pivot = new Pivot(ss_Elevator.currentHeight);
    public final ArmExtension ss_ArmExtension = new ArmExtension(ss_Elevator.currentHeight);
    public final josiahClimber ss_Climber = new josiahClimber();
    
    public final CANdleSubsystem Candle = new CANdleSubsystem();
    public final RearIntake ss_RearIntake = new RearIntake();

    public final MantaState MS = new MantaState(drivetrain, ss_Elevator, ss_Pivot, ss_ArmExtension,ss_RearIntake);
    public final SuperStructure SuperS = new SuperStructure(ss_Trident, ss_Elevator, ss_Pivot, ss_ArmExtension, ss_Climber, Candle, ss_RearIntake);
    
   

    /* Setting up bindings for necessary control of the swerve drive platform */




    private final Telemetry logger = new Telemetry(constants.drivetrainThings.MaxSpeed);

    public static final CommandXboxController joystick = new CommandXboxController(0);
    //private final CommandXboxController Testjoystick = new CommandXboxController(1);

    public static final IntSupplier OptionalButtonSupplier = ()-> {
        if(joystick.x().getAsBoolean())
        {
            return 2;//2 is the right side option on the reef
        }
            //default option is that x is not pressed and we score left side.
        else return 0;//0 is the left side option on reef
        
    };
    
    
    
    /* Path follower */
    //edu.wpi.first.networktables.NetworkTableEntry NT_AutoChooser = NT.getStringArrayEntry("Auto" , "Auto Mode",new String[]{});
    private final SendableChooser<Command> autoChooser; 

    public RobotContainer() {
        bindNamedCommands();
        autoChooser = AutoBuilder.buildAutoChooser("");
        SmartDashboard.putData("Auto Mode", autoChooser);
        configureBindings();
        
    }

    
    private void configureBindings() {
        
        joystick.start().and(joystick.x().negate())
        .onTrue( SuperS.Btn_ResetVision());
        //barge mode no scoring though, usually for debug
        joystick.start().and(joystick.x())
        .onTrue(SuperS.Btn_GotoL4DebugMode(this));
        
        joystick.back().and(joystick.x().negate())
        .onTrue(SuperS.Btn_EnableClimbMode(this));

        joystick.back().and(joystick.x())
        .onTrue(SuperS.Btn_DisableClimbMode(this));
        
        //on the fly align test
        Pose2d test = new Pose2d(3.8, 5.17, Rotation2d.fromDegrees(-60));
        // joystick.rightStick()
        // .whileTrue(new C_Align(test, true));

        // joystick.rightStick()
        //     .onTrue(new C_PivotToPosition(ss_Pivot, constants.PlasmaPivot.ParkPosition))
        //     .onFalse(new C_PivotToPosition(ss_Pivot, constants.PlasmaPivot.GroundPickupPosition));

        joystick.a().and(joystick.x().negate())
        .whileTrue(SuperS.DebugIntake());
        //Algae 
        joystick.a().and(joystick.x())
        .whileTrue(SuperS.Btn_ManualBumpOut(this));

        //limelight bypass
        joystick.b()
        .onTrue(SuperS.Btn_BypassLimelight())
        .onFalse(SuperS.Btn_EnableLimelight());

        joystick.b().and(joystick.x())
        .onTrue(SuperS.gotoL2Travel());
        
        joystick.x();//X button is the alt button dont assign it anything more. unless its a combo
        //test button please diable for comp!
        if(!DriverStation.isFMSAttached())
        {
            //joystick.x().whileTrue(AprilTagManager.C_OnTheFlyWaypointAlign(new Pose2d(1.19,6.93,Rotation2d.fromDegrees(-53))));
        }
        
        joystick.y().onTrue(SuperS.Btn_ScoreBarge());


        joystick.rightTrigger().and(joystick.x().negate())
        .whileTrue(SuperS.Btn_GetAlgaeFromReef());
            //.onFalse(gotoMinTravel());
        
        //processor score
        joystick.rightTrigger().and(joystick.x())
        .onTrue(SuperS.Btn_GotoProcessorPos(this));

        joystick.rightBumper()
        .whileTrue(SuperS.Btn_RearIntake());
        
        //pick up algae (and technically coral too)
        joystick.leftTrigger().and(joystick.x())
        .whileTrue(SuperS.Btn_GroundIntake());

        //score algae in amp
        joystick.leftTrigger().and(joystick.x().negate())
        .toggleOnTrue(SuperS.Btn_GroundIntakeAngled());

        joystick.leftBumper().and(()->!MantaState.getAltControlModeEnabled.getAsBoolean())
            .onTrue(SuperS.Btn_Park());
    

        //Faster
        joystick.povUp().and(()->!MantaState.getAltControlModeEnabled.getAsBoolean())
            .onTrue(SuperS.Btn_ScoreL4(this));
            
            //.onFalse(ParkElevatorAndHead());

        joystick.povRight().and(()->!MantaState.getAltControlModeEnabled.getAsBoolean())
            .onTrue(SuperS.Btn_ScoreL3());
            
        joystick.povDown().and(()->!MantaState.getAltControlModeEnabled.getAsBoolean())
            .onTrue(SuperS.Btn_ScoreL2());

        joystick.povLeft().and(()->!MantaState.getAltControlModeEnabled.getAsBoolean())
            .onTrue(SuperS.Btn_ScoreL1());


        // Note that X is defined as forward according to WPILib convention in robot centric, but we use field centric where x is x and y is y.
        // and Y is defined as to the left according to WPILib convention in robot centric, but we use field centric where x is x and y is y.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                constants.drivetrainThings.TeleOpDrive.withVelocityX(Tools.getExpoJoystickInput(-joystick.getLeftY(),constants.drivetrainThings.MaxSpeed)) // Drive forward with negative Y (forward)
                    .withVelocityY(Tools.getExpoJoystickInput(-joystick.getLeftX(),constants.drivetrainThings.MaxSpeed)) // Drive left with negative X (left)
                    .withRotationalRate(Tools.getExpoJoystickInput(-joystick.getRightX(), constants.drivetrainThings.MaxAngularRate)) // Drive counterclockwise with negative X (left)
            )
        );

        drivetrain.registerTelemetry(logger::telemeterize);
        configAltCommands();
    }

    public void configAltCommands()
    {
        
        joystick.povUp().and(MantaState.getAltControlModeEnabled).onTrue(
            SuperS.Btn_ClimbHookReady()
        );
        joystick.povRight().and(MantaState.getAltControlModeEnabled).onTrue(
            SuperS.Btn_ClimbHookStartFlat()
        );
        joystick.povDown().and(MantaState.getAltControlModeEnabled).onTrue(
            SuperS.Btn_ClimbNow(this));

         
        joystick.povLeft().and(MantaState.getAltControlModeEnabled).onTrue(
            ss_Climber.C_Stop()
        );
    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }
    public void bindNamedCommands()
    {
        // Register Named Commands
        NamedCommands.registerCommand("DoclosestSourceIntake", SuperS.Control_AutoRearIntake());
        NamedCommands.registerCommand("DoclosestScoreL4", SuperS.Control_AlignClosestScoreL4());
        NamedCommands.registerCommand("DoclosestLeftScoreL4", SuperS.Control_AutonAlignClosestLeftScoreL4());
        NamedCommands.registerCommand("DoclosestRightScoreL4", SuperS.Control_AutonAlignClosestRightScoreL4());
        NamedCommands.registerCommand("ClearRearIntake", new C_ClearRearIntake(ss_RearIntake));
        NamedCommands.registerCommand("ParkElevatorAndHead", SuperS.ParkElevatorAndHead().withTimeout(3));
        NamedCommands.registerCommand("PivotPullIntake", SuperS.control_PivotPullIntakeCommand().withTimeout(1));
        NamedCommands.registerCommand("GrabClosestAlgae", SuperS.Auto_GetClosestAlgae());
        NamedCommands.registerCommand("ShootClosestBarge", SuperS.control_AutoBarge());
        NamedCommands.registerCommand("GotoAlgaeTravel", SuperS.GotoTravelPostion().alongWith(new C_ElevateToPosition(ss_Elevator, constants.Elevator.minElevatorHeight)).withTimeout(1));
        
        NamedCommands.registerCommand("Spinintake", new C_TridentIntake(1,ss_Trident,ss_RearIntake).withTimeout(2));
        //unused below lol
        NamedCommands.registerCommand("Test", new InstantCommand(()->{System.out.println("running test command");}));
        NamedCommands.registerCommand("AlignprocSource",  SelectCommands.C_SourceSelectCommand().until(ss_Trident.getisloaded));
        NamedCommands.registerCommand("RearIntake", SuperS.RearIntake()); //new C_ClearRearIntake(RearIntake).asProxy()
       
        NamedCommands.registerCommand("AlignReefLeft",  SelectCommands.C_ReefLeftSelectCommand().withTimeout(5));
        NamedCommands.registerCommand("AlignReefRight",  SelectCommands.C_ReefRightSelectCommand().withTimeout(5));
        NamedCommands.registerCommand("gotoL4Travel", SuperS.gotoL4Travel());
        NamedCommands.registerCommand("PivotIntoReefL4", SuperS.PivotIntoReefL4());
        NamedCommands.registerCommand("ScoreL4", new SequentialCommandGroup(SuperS.PivotIntoReefL4(),SuperS.CoralDropScoreL4(),SuperS.ParkElevatorAndHead()));


        NamedCommands.registerCommand("GroundIntakeCoral", SuperS.AutoGroundIntake());//.until(ss_Trident.getisloaded).andThen(ParkElevatorAndHead()));
    }

}
