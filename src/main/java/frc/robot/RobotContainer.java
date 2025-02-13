// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.C_intake;
import frc.robot.constants.Climber;
import frc.robot.AlphaBots.NT;
import frc.robot.AlphaBots.Tools;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.AprilTagManager;
import frc.robot.subsystems.ArmExtension;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.MantaRay;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.josiahClimber;
import frc.robot.subsystems.Elevator.POSITION;

public class RobotContainer {
    //Subsystem bootup Zone - Order matters.
   
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final AprilTagManager ATMan = new AprilTagManager(drivetrain); 
    public final MantaRay ss_Trident = new MantaRay();
    public final Elevator ss_Elevator = new Elevator();
    public final Pivot ss_Pivot = new Pivot(ss_Elevator.currentHeight);
    public final ArmExtension ss_ArmExtension = new ArmExtension(ss_Elevator.currentHeight);
    public final josiahClimber ss_Climber = new josiahClimber();
    public final MantaState MS = new MantaState(drivetrain, ss_Elevator, ss_Pivot);

    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.2) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.Velocity); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.Velocity);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    
    
    /* Path follower */
    //edu.wpi.first.networktables.NetworkTableEntry NT_AutoChooser = NT.getStringArrayEntry("Auto" , "Auto Mode",new String[]{});
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        autoChooser = AutoBuilder.buildAutoChooser("Tests");
        SmartDashboard.putData("Auto Mode", autoChooser);
        configureBindings();
    }
    BooleanSupplier elevatorisparked = ()->{return ss_Elevator.currentState.equals(Elevator.POSITION.parked);};
    double TridentEjectMovement = 20;
    ////////////// movement commands
    public Command gotoL1Travel()
    {
        return ss_Elevator.GotoPositonCommand(constants.Elevator.l1Position).alongWith(GotoTravelPostion());
    }
    public Command gotoL2Travel()
    {
        return ss_Elevator.GotoPositonCommand(constants.Elevator.l2Position).alongWith(GotoTravelPostion());
    }
    public Command gotoL3Travel()
    {
        return ss_Elevator.GotoPositonCommand(constants.Elevator.l3Position).alongWith(GotoTravelPostion());
    }
    public Command gotoL4Travel()
    {
        return ss_Elevator.GotoPositonCommand(constants.Elevator.l4Position).alongWith(GotoTravelPostion());
    }
    public Command GotoTravelPostion()
    {
        return ss_Pivot.C_GotoPositon(constants.PlasmaPivot.TravelPosition);
    }
    public Command ParkElevatorAndHead()
    {
        return ss_Elevator.GotoPositonCommand(constants.Elevator.minElevatorHeight).alongWith(
            ss_Pivot.C_GotoPositon(constants.PlasmaPivot.TravelPosition).unless(ss_Pivot.IsPivotinTravelPosition)
            ).andThen(
                ss_Pivot.C_GotoPositon(constants.PlasmaPivot.ParkPosition));
    }
    ///////////////
    private void configureBindings() {

        
        joystick.rightBumper().onTrue(ss_Trident.bumpout()
            .andThen(new WaitCommand(0.2))
            .andThen(ss_Trident.Stop()));
        joystick.a().onTrue(new C_intake(ss_Trident).withTimeout(20));
        //joystick.b().onTrue(ss_Trident.bumpout()).onFalse(ss_Trident.Stop());

        //joystick.y().onTrue(ss_Elevator.GotoPositonCommand(7));
        //joystick.x().onTrue(ss_Elevator.GotoPositonCommand(2));

        //joystick.start().onTrue(new InstantCommand(()->{ss_Elevator.setMotorConfig();}));
        joystick.x().onTrue(ss_Pivot.C_GotoPositon(constants.PlasmaPivot.TravelPosition));
        joystick.y().onTrue(ss_Pivot.C_GotoPositon(constants.PlasmaPivot.ParkPosition));
        joystick.b().onTrue(ss_Pivot.C_GotoPositon(constants.PlasmaPivot.GroundPickupPosition));

        joystick.povUp().and(()->!MantaState.getAltControlModeEnabled.getAsBoolean())
        .onTrue(gotoL4Travel());
        joystick.povLeft().and(()->!MantaState.getAltControlModeEnabled.getAsBoolean())
        .onTrue(gotoL3Travel());
        joystick.povRight().and(()->!MantaState.getAltControlModeEnabled.getAsBoolean())
        .onTrue(gotoL2Travel());
        joystick.povDown().and(()->!MantaState.getAltControlModeEnabled.getAsBoolean())
        .onTrue(gotoL1Travel());


   
        joystick.leftBumper().and(()->!MantaState.getAltControlModeEnabled.getAsBoolean())
        .onTrue(ParkElevatorAndHead());
        //joystick.b().onTrue(new InstantCommand(()->{ss_Trident.GotoPosition(ss_Trident.LastPosition-TridentEjectMovement);}));
        
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(Tools.getExpoJoystickInput(-joystick.getLeftY(),MaxSpeed)) // Drive forward with negative Y (forward)
                    .withVelocityY(Tools.getExpoJoystickInput(-joystick.getLeftX(),MaxSpeed)) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        //joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        // joystick.b().whileTrue(drivetrain.applyRequest(() ->
        //     point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        // ));

        // joystick.pov(0).whileTrue(drivetrain.applyRequest(() ->
        //     forwardStraight.withVelocityX(0.5).withVelocityY(0))
        // );
        // joystick.pov(180).whileTrue(drivetrain.applyRequest(() ->
        //     forwardStraight.withVelocityX(-0.5).withVelocityY(0))
        // );


        // reset the field-centric heading on start button press
        joystick.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);
        configAltCommands();
    }
    public void configAltCommands()
    {
        joystick.back().onTrue(new InstantCommand(()->{MantaState.setAltControlModeEnabled(!MantaState.getAltControlModeEnabled.getAsBoolean());}));
        joystick.povUp().and(MantaState.getAltControlModeEnabled).onTrue(
            ss_Climber.C_CatchGotoPositon(constants.Climber.CatchSide.maxPostion).alongWith(
            ss_Climber.C_SlideGotoPositon(constants.Climber.SlideSide.maxPostion)
        ));
        joystick.povRight().and(MantaState.getAltControlModeEnabled).onTrue(
            ss_Climber.C_CatchGotoPositon(constants.Climber.CatchSide.startPos).alongWith(
            ss_Climber.C_SlideGotoPositon(constants.Climber.SlideSide.startPos)
        ));
        joystick.povDown().and(MantaState.getAltControlModeEnabled).onTrue(
            ss_Climber.C_CatchGotoPositon(constants.Climber.CatchSide.minPostion).alongWith(
            ss_Climber.C_SlideGotoPositon(constants.Climber.SlideSide.minPostion)
        ));
        joystick.povLeft().and(MantaState.getAltControlModeEnabled).onTrue(
            ss_Climber.C_Stop()
        );
    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }
}
