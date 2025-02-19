// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.C_DropElevateToScore;
import frc.robot.commands.C_ElevateToPosition;
import frc.robot.commands.C_ExtendToPosition;
import frc.robot.commands.C_PivotToPosition;
import frc.robot.commands.C_ReefAlign;
import frc.robot.commands.C_TridentIntake;
import frc.robot.constants.Climber;
import frc.robot.AlphaBots.NT;
import frc.robot.AlphaBots.Tools;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.AprilTagManager;
import frc.robot.subsystems.ArmExtension;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.MantaRay;
import frc.robot.subsystems.MantaState;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.josiahClimber;
import frc.robot.subsystems.Elevator.POSITION;

public class RobotContainer {
    //fields
    
    double TridentEjectMovement = 20;

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
    private double MaxAngularRate = RotationsPerSecond.of(0.5).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.05).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.Velocity); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.Velocity);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);
    private final IntSupplier OptionalButtonSupplier = ()-> {
        if(joystick.x().getAsBoolean())
        {
            return 2;//2 is the rightsideoption on the reef
        }
            //default option is that x is not pressed and we score left side.
        else return 0;//0 is the left side option on reef
        
    };
    private final CommandXboxController Testjoystick = new CommandXboxController(1);
    
    
    /* Path follower */
    //edu.wpi.first.networktables.NetworkTableEntry NT_AutoChooser = NT.getStringArrayEntry("Auto" , "Auto Mode",new String[]{});
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        autoChooser = AutoBuilder.buildAutoChooser("Tests");
        SmartDashboard.putData("Auto Mode", autoChooser);
        configureBindings();
    }

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
        return new C_PivotToPosition(ss_Pivot,constants.PlasmaPivot.TravelPosition);//ss_Pivot.C_GotoPositon(constants.PlasmaPivot.TravelPosition);
    }
    BooleanSupplier jake = ()->{return ss_Elevator.currentHeight.getAsDouble() < constants.Elevator.l1Position;};
    BooleanSupplier jake2 = ()->{return ss_Elevator.m_ElevatorMotor1.getVelocity().getValueAsDouble() < 100;};

    // public Command elevator2StepPark()
    // {   //& jake2.getAsBoolean()
    //    return new C_ElevateToPosition(ss_Elevator,constants.Elevator.l1Position)
    //     .until(()->{return jake.getAsBoolean();})
    //     .andThen(new C_ElevateToPosition(ss_Elevator,constants.Elevator.minElevatorHeight));//C_ElevateToPosition(ss_Elevator,constants.Elevator.minElevatorHeight);
    // }
    public Command elevator1StepPark(){
        return new C_ElevateToPosition(ss_Elevator, constants.Elevator.minElevatorHeight);
    }
    public Command ParkElevatorAndHead()
    {
        return elevator1StepPark()//new C_ElevateToPosition(ss_Elevator,constants.Elevator.minElevatorHeight)
        .alongWith(new C_ExtendToPosition(ss_ArmExtension,constants.PlasmaExtension.minposition))
        .alongWith(new C_PivotToPosition(ss_Pivot,constants.PlasmaPivot.TravelPosition)
        //.unless(ss_Pivot.IsPivotinTravelPosition) Why is this here? it seems redundant?
            ).andThen(new C_PivotToPosition(ss_Pivot,constants.PlasmaPivot.ParkPosition));
    }
    public Command PivotIntoReef()
    {
        return new C_PivotToPosition(ss_Pivot, constants.PlasmaPivot.l1ReadyPosition)
                .alongWith(new C_ExtendToPosition(ss_ArmExtension,constants.PlasmaExtension.l1ReadyPosition));
    }
    public Command PivotIntoReefL4()
    {
        return new C_PivotToPosition(ss_Pivot, constants.PlasmaPivot.l1ReadyPosition)
                .alongWith(new C_ExtendToPosition(ss_ArmExtension,constants.PlasmaExtension.l4ScorePosition));
    }
    public Command CoralDropScoreL2()
    {
        return new C_DropElevateToScore(ss_Elevator).alongWith(TridentCoralBumpOut());
        //
        // return new C_PivotToPosition(ss_Pivot, constants.PlasmaPivot.l2ScorePosition)
        //         .alongWith(new C_ExtendToPosition(ss_ArmExtension,constants.PlasmaExtension.l2ScorePosition))
        //         .alongWith(TridentCoralBumpOut());
    }
    public Command TridentCoralBumpOut()
    {
        return ss_Trident.LooseGrip()
        .andThen(new WaitCommand(2))
        .andThen(ss_Trident.Stop());
    }
    public Command TridentAlgaeBumpOut()
    {
        return ss_Trident.bumpout()
        .andThen(new WaitCommand(2))
        .andThen(ss_Trident.Stop());
    }

    //started working on this, not done yet
    public Command Intake(){
        return new ParallelCommandGroup(
            new C_TridentIntake(ss_Trident).withTimeout(20),
            new C_PivotToPosition(ss_Pivot, constants.PlasmaPivot.GroundPickupPosition),
            new C_ElevateToPosition(ss_Elevator, constants.Elevator.minElevatorHeight)
            );
    }
    ///////////////
    private double testchoice = 0;
    DoubleSupplier gettestchoice = ()->{return testchoice;};
    private void configureBindings() {

        //TEST CONFIGURATIONS
        Testjoystick.leftTrigger().onTrue(CoralDropScoreL2().andThen(ParkElevatorAndHead()));
        Testjoystick.b().onTrue(new C_ExtendToPosition(ss_ArmExtension, constants.PlasmaExtension.minposition));
        Testjoystick.x().onTrue(new C_ExtendToPosition(ss_ArmExtension, constants.PlasmaExtension.maxposition));
        // reset the field-centric heading on start button press
        //joystick.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
        
        joystick.a().onTrue(new C_TridentIntake(ss_Trident).withTimeout(20));

        joystick.y().onTrue(gotoL4Travel()
        .andThen(new C_PivotToPosition(ss_Pivot, -0.2))
        .andThen(new C_ExtendToPosition(ss_ArmExtension, constants.PlasmaExtension.maxposition)));



        joystick.leftTrigger().whileTrue(new C_ReefAlign(drivetrain,OptionalButtonSupplier));

        //joystick.b().onTrue(ss_Trident.bumpout()).onFalse(ss_Trident.Stop());

        //joystick.y().onTrue(ss_ArmExtension.C_GotoPositon(constants.PlasmaExtension.maxposition));
        //joystick.b().onTrue(ss_ArmExtension.C_GotoPositon(constants.PlasmaExtension.minposition));
        //joystick.x().onTrue(ss_Elevator.GotoPositonCommand(2));

        //joystick.start().onTrue(new InstantCommand(()->{ss_Elevator.setMotorConfig();}));
        // joystick.x().onTrue(new InstantCommand(()->{testchoice= testchoice +1;ss_ArmExtension.GotoPosition(gettestchoice.getAsDouble());}));
        // joystick.y().onTrue(new InstantCommand(()->{testchoice= testchoice -1;ss_ArmExtension.GotoPosition(gettestchoice.getAsDouble());}));


        joystick.povUp().and(()->!MantaState.getAltControlModeEnabled.getAsBoolean())
        .onTrue(gotoL4Travel().andThen(PivotIntoReefL4()));
        joystick.povLeft().and(()->!MantaState.getAltControlModeEnabled.getAsBoolean())
        .onTrue(gotoL3Travel().andThen(PivotIntoReef()));
        joystick.povRight().and(()->!MantaState.getAltControlModeEnabled.getAsBoolean())
        .onTrue(gotoL2Travel().andThen(PivotIntoReef()));
        joystick.povDown().and(()->!MantaState.getAltControlModeEnabled.getAsBoolean())
        .onTrue(gotoL1Travel().andThen(PivotIntoReef()));

        //joystick.leftTrigger().onTrue(new C_PivotToPosition(ss_Pivot,constants.PlasmaPivot.GroundPickupPosition));
        joystick.rightTrigger().onTrue(GotoTravelPostion());
        joystick.rightBumper().onTrue(TridentCoralBumpOut());

        
        joystick.leftBumper().and(()->!MantaState.getAltControlModeEnabled.getAsBoolean())
        .onTrue(ParkElevatorAndHead());

        joystick.start().whileTrue(Intake());

        
        //joystick.b().onTrue(new InstantCommand(()->{ss_Trident.GotoPosition(ss_Trident.LastPosition-TridentEjectMovement);}));
        
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(Tools.getExpoJoystickInput(-joystick.getLeftY(),MaxSpeed)) // Drive forward with negative Y (forward)
                    .withVelocityY(Tools.getExpoJoystickInput(-joystick.getLeftX(),MaxSpeed)) // Drive left with negative X (left)
                    .withRotationalRate(Tools.getExpoJoystickInput(-joystick.getRightX(), MaxAngularRate)) // Drive counterclockwise with negative X (left)
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
