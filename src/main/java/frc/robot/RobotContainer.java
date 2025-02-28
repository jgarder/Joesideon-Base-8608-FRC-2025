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
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.C_DropElevateToScore;
import frc.robot.commands.C_ElevateToPosition;
import frc.robot.commands.C_ExtendToPosition;
import frc.robot.commands.C_PivotToPosition;
import frc.robot.commands.C_TridentIntake;
import frc.robot.AlphaBots.LimeLightPoseFilter;
import frc.robot.AlphaBots.Tools;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.AprilTagManager;
import frc.robot.subsystems.ArmExtension;
import frc.robot.subsystems.CANdleSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.MantaRay;
import frc.robot.subsystems.MantaState;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.josiahClimber;

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
    public final CANdleSubsystem Candle = new CANdleSubsystem();

    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.5).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.025).withRotationalDeadband(MaxAngularRate * 0.05) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.Velocity); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.Velocity);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);
    //private final CommandXboxController Testjoystick = new CommandXboxController(1);

    private final IntSupplier OptionalButtonSupplier = ()-> {
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
        autoChooser = AutoBuilder.buildAutoChooser("Tests");
        SmartDashboard.putData("Auto Mode", autoChooser);
        configureBindings();
        
    }

    ////////////// movement commands
    public Command gotoMinTravel()
    {
        return ss_Elevator.GotoPositonCommand(constants.Elevator.minElevatorHeight).alongWith(GotoTravelPostion());
    }
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
    public Command gotoUpperAlgaeTravel()
    {
        return ss_Elevator.GotoPositonCommand(constants.Elevator.l3algae).alongWith(GotoTravelPostion());
    }
    public Command gotoLowerAlgaeTravel()
    {
        return ss_Elevator.GotoPositonCommand(constants.Elevator.l2Algae).alongWith(GotoTravelPostion());
    }
    public Command AlgaeReefIntake()
    {
        return
            new C_PivotToPosition(ss_Pivot, constants.PlasmaPivot.AlgaeReefPickup)
            .andThen(new C_ExtendToPosition(ss_ArmExtension,constants.PlasmaExtension.ReefAlgaePickupExtension))
            .alongWith(new C_TridentIntake(ss_Trident).withTimeout(5))
            .andThen(new C_ExtendToPosition(ss_ArmExtension,constants.PlasmaExtension.minposition)
            .andThen(new C_PivotToPosition(ss_Pivot, constants.PlasmaPivot.TravelPosition))
            );
    }
    public Command GotoTravelPostion()
    {
        return new C_PivotToPosition(ss_Pivot,constants.PlasmaPivot.TravelPosition)
            .alongWith(new C_ExtendToPosition(ss_ArmExtension, constants.PlasmaExtension.minposition));//ss_Pivot.C_GotoPositon(constants.PlasmaPivot.TravelPosition);
    }
    // BooleanSupplier jake = ()->{return ss_Elevator.currentHeight.getAsDouble() < constants.Elevator.l1Position;};
    // BooleanSupplier jake2 = ()->{return ss_Elevator.m_ElevatorMotor1.getVelocity().getValueAsDouble() < 100;};

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
        return elevator1StepPark().unless(ss_Elevator.elevatorisparked)//new C_ElevateToPosition(ss_Elevator,constants.Elevator.minElevatorHeight)
        .alongWith(new C_ExtendToPosition(ss_ArmExtension,constants.PlasmaExtension.minposition))
        .alongWith(new C_PivotToPosition(ss_Pivot,constants.PlasmaPivot.TravelPosition)
            .unless(()->{return ss_Pivot.IsPivotParked.getAsBoolean() && ss_Elevator.elevatorisparked.getAsBoolean();}) //Why is this here? it seems redundant?
            ).andThen(new C_PivotToPosition(ss_Pivot,constants.PlasmaPivot.ParkPosition));
    }
    public Command PivotIntoReefl3(){
        return new C_PivotToPosition(ss_Pivot, constants.PlasmaPivot.l3ReadyPosition)
                .alongWith(new C_ExtendToPosition(ss_ArmExtension,constants.PlasmaExtension.l3ReadyPosition));
    }
    public Command PivotIntoReefL1(){
        return new C_PivotToPosition(ss_Pivot, constants.PlasmaPivot.l1ReadyPosition)
                .alongWith(new C_ExtendToPosition(ss_ArmExtension,constants.PlasmaExtension.l1ReadyPosition));
    }
    public Command PivotIntoReefL2(){
        return new C_PivotToPosition(ss_Pivot, constants.PlasmaPivot.l2ReadyPosition)
                .alongWith(new C_ExtendToPosition(ss_ArmExtension,constants.PlasmaExtension.l2ReadyPosition));
    }
    public Command PivotIntoReefL4()
    {
        return new C_PivotToPosition(ss_Pivot, constants.PlasmaPivot.l3ReadyPosition)
                .alongWith(new C_ExtendToPosition(ss_ArmExtension,constants.PlasmaExtension.l4ScorePosition));
    }
    public Command CoralDropScoreL4()
    {
        return new C_DropElevateToScore(ss_Elevator).deadlineFor(TridentCoralBumpOut().alongWith(new C_PivotToPosition(ss_Pivot, constants.PlasmaPivot.l3ReadyPosition +.04)).finallyDo(()->{ss_Trident.setDutyCycle(0);}));
        //
        // return new C_PivotToPosition(ss_Pivot, constants.PlasmaPivot.l2ScorePosition)
        //         .alongWith(new C_ExtendToPosition(ss_ArmExtension,constants.PlasmaExtension.l2ScorePosition))
        //         .alongWith(TridentCoralBumpOut());
    }

    public Command CoralDropScoreL2()
    {
        return new C_DropElevateToScore(ss_Elevator)
            .alongWith(new C_PivotToPosition(ss_Pivot,constants.PlasmaPivot.SideScore))//.withTimeout(1)
            .deadlineFor(TridentCoralBumpOut()//.withTimeout(2)
            .finallyDo(()->{ss_Trident.setDutyCycle(0);}));
                                    //.setDutyCycle(0)
    
    }

    // public Command coralScoreL2(){
    //     return new C_PivotToPosition(ss_Pivot, constants.PlasmaPivot.l2ReadyPosition)
    //         .alongWith(new C_ExtendToPosition(ss_ArmExtension,constants.PlasmaExtension.l2ReadyPosition))
    //     .andThen(new C_DropElevateToScore(ss_Elevator))
    //         .alongWith(new C_PivotToPosition(ss_Pivot,constants.PlasmaPivot.SideScore))
    //         .deadlineFor(TridentCoralBumpOut()
    //         .finallyDo(()->{ss_Trident.BRAKE();}));
    //                             //.setDutyCycle(0));
    // }

    public Command TridentCoralBumpOut()
    {
        return ss_Trident.LooseGrip()
        .andThen(new WaitCommand(2))
        .andThen(ss_Trident.Stop());
    }
    public Command TridentCoralShootOut()
    {
        return ss_Trident.LooseBump()
        .andThen(new WaitCommand(.5))
        .andThen(ss_Trident.Stop());
    }
    public Command TridentAlgaeBumpOut()
    {
        return ss_Trident.bumpout()
        .andThen(new WaitCommand(2))
        .andThen(ss_Trident.Stop());
    }

    //started working on this, not done yet
    double intaketimeout = 20;
    public ParallelCommandGroup RearIntake(){
        return new ParallelCommandGroup(
            new C_TridentIntake(ss_Trident).withTimeout(intaketimeout),
            new C_PivotToPosition(ss_Pivot, constants.PlasmaPivot.rearintakePos),
            new C_ElevateToPosition(ss_Elevator, constants.Elevator.minElevatorHeight),
            new C_ExtendToPosition(ss_ArmExtension,constants.PlasmaExtension.rearintakePos)
            );
    }
    public Command DebugIntake(){
        return new ParallelCommandGroup(
            new C_TridentIntake(ss_Trident,.75).withTimeout(intaketimeout)
            );
    }

    double groundintakeTimeout = 20; //auton this command will run until finished or this timeout.
    double groundintakedutycycle = 1.0; 

    
    //fixed the robot spitting out coral, ground intake should work well for both now
    public Command GroundIntake(){
        return new ParallelCommandGroup(
            new C_ElevateToPosition(ss_Elevator, constants.Elevator.minElevatorHeight),
            new C_TridentIntake(ss_Trident,groundintakedutycycle).withTimeout(groundintakeTimeout),
            new C_PivotToPosition(ss_Pivot, constants.PlasmaPivot.GroundPickupPosition),
            new C_ExtendToPosition(ss_ArmExtension,constants.PlasmaExtension.GroundPickupExtension))
        .finallyDo(groundIntakeReset());
    }

    public Runnable groundIntakeReset(){
        return ()->{
            new C_PivotToPosition(ss_Pivot, constants.PlasmaPivot.TravelPosition).andThen(GotoTravelPostion()).schedule();
            new C_ElevateToPosition(ss_Elevator, constants.Elevator.minElevatorHeight).schedule();};
    }

    public Runnable traveltopark()
    {
        return ()->{ParkElevatorAndHead().schedule();};
    }


    ///////////////
    private double testchoice = 0;
    DoubleSupplier gettestchoice = ()->{return testchoice;};
    private void configureBindings() {


        //TEST CONFIGURATIONS
        // Testjoystick.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));// reset the field-centric heading on start button press
        
        // Testjoystick.b().onTrue(new C_ExtendToPosition(ss_ArmExtension, constants.PlasmaExtension.minposition));
        // Testjoystick.x().onTrue(new C_ExtendToPosition(ss_ArmExtension, constants.PlasmaExtension.maxposition));
        // Testjoystick.rightBumper().onTrue(TridentCoralBumpOut());
        // Testjoystick.leftTrigger().onTrue(CoralDropScoreL4().andThen(ParkElevatorAndHead()));
        //joystick.start().onTrue(new InstantCommand(()->{ss_Elevator.setMotorConfig();}));
        // joystick.x().onTrue(new InstantCommand(()->{testchoice= testchoice +1;ss_ArmExtension.GotoPosition(gettestchoice.getAsDouble());}));
        // joystick.y().onTrue(new InstantCommand(()->{testchoice= testchoice -1;ss_ArmExtension.GotoPosition(gettestchoice.getAsDouble());}));

        
        joystick.start().onTrue(
            new InstantCommand(()->{LimeLightPoseFilter.DoResetVision();})
        .andThen(
            new WaitCommand(.01),
            new InstantCommand(()->{LimeLightPoseFilter.DoResetVision();}),
            new WaitCommand(.01),
            new InstantCommand(()->{LimeLightPoseFilter.DoResetVision();})
            ));
        
        joystick.back().onTrue(
            new InstantCommand(()->{MantaState.setAltControlModeEnabled(!MantaState.getAltControlModeEnabled.getAsBoolean());})
        .alongWith(
            new C_PivotToPosition(ss_Pivot, constants.PlasmaPivot.ParkPosition)
            ,new C_ExtendToPosition(ss_ArmExtension, constants.PlasmaExtension.climbExtension)
            ));

        joystick.a().and(joystick.x().negate()).whileTrue(DebugIntake());
        //Algae 
        joystick.a().and(joystick.x()).whileTrue(TridentAlgaeBumpOut().finallyDo(()->{ss_Trident.HoldPosition();}));

        //limelight bypass
        joystick.b().onTrue(new InstantCommand(()->{MantaState.setLimeLightBypassed(true);})).onFalse(new InstantCommand(()->{MantaState.setLimeLightBypassed(false);}));
        
        joystick.x();//X button is the alt button dont assign it anything more. unless its a combo
        joystick.y().toggleOnTrue(gotoL4Travel()
            //.andThen(new C_PivotToPosition(ss_Pivot, constants.PlasmaPivot.TravelPosition))//-0.2
            .andThen(new C_ExtendToPosition(ss_ArmExtension, constants.PlasmaExtension.maxposition))
            .andThen(new WaitCommand(30))
            .finallyDo(traveltopark()));


        joystick.rightTrigger().whileTrue(
            ATMan.C_ReefCenterAlgaeSelectCommand().asProxy().until(MantaState.getLimeLightBypassed)
            .alongWith(new ConditionalCommand(gotoUpperAlgaeTravel(),gotoLowerAlgaeTravel(),MantaState.NearestTagIsUpperAlgae))
            .andThen(AlgaeReefIntake(),gotoMinTravel()))
            .onFalse(gotoMinTravel());
        
        //processor score
        joystick.rightTrigger().and(joystick.x()).onTrue(
            ATMan.C_ProcessorSelectCommand().until(MantaState.getLimeLightBypassed)
            .alongWith(
                new C_ElevateToPosition(ss_Elevator, constants.Elevator.minElevatorHeight),
                new C_PivotToPosition(ss_Pivot, constants.PlasmaPivot.processorPivot),
                new C_ExtendToPosition(ss_ArmExtension, constants.PlasmaExtension.processorExtension)
                )
            .andThen(TridentAlgaeBumpOut().withTimeout(.5).finallyDo(()->{ss_Trident.HoldPosition(); traveltopark();}))
            
        );

        joystick.rightBumper().whileTrue(Control_RearIntake());
        
        //pick up algae (and technically coral too)
        joystick.leftTrigger().and(joystick.x().negate()).whileTrue(GroundIntake());

        //score algae in amp
        joystick.leftTrigger().and(joystick.x()).whileTrue(new InstantCommand(()->{}));

        joystick.leftBumper().and(()->!MantaState.getAltControlModeEnabled.getAsBoolean())
            .onTrue(ParkElevatorAndHead());
    

        //Faster
        joystick.povUp().and(()->!MantaState.getAltControlModeEnabled.getAsBoolean())
            .onTrue(Control_AlignClosestScoreL4().andThen(ParkElevatorAndHead()).finallyDo(traveltopark()));
            
            //.onFalse(ParkElevatorAndHead());

        joystick.povRight().and(()->!MantaState.getAltControlModeEnabled.getAsBoolean())
            .onTrue(alignReefForCoral()
            .alongWith(gotoL3Travel())
            .andThen(PivotIntoReefl3(),CoralDropScoreL2(),ParkElevatorAndHead()).finallyDo(traveltopark()));
            
        joystick.povDown().and(()->!MantaState.getAltControlModeEnabled.getAsBoolean())
            .onTrue(alignReefForCoral()
            .alongWith(gotoL2Travel())
            .andThen(PivotIntoReefL2(),CoralDropScoreL2(),ParkElevatorAndHead()).finallyDo(traveltopark()));

        joystick.povLeft().and(()->!MantaState.getAltControlModeEnabled.getAsBoolean())
            .onTrue(ATMan.C_ReefL1CenterSelectCommand().until(MantaState.getLimeLightBypassed)
            .alongWith(gotoL1Travel())
            .andThen(PivotIntoReefL1(),TridentCoralShootOut(),ParkElevatorAndHead()).finallyDo(traveltopark()));


 

        
        

       

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

   

        drivetrain.registerTelemetry(logger::telemeterize);
        configAltCommands();
    }


    public void configAltCommands()
    {
        
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
            ss_Climber.C_SlideGotoPositon(constants.Climber.SlideSide.minPostion).andThen(ss_Climber.C_Stop())
        ));
        joystick.povLeft().and(MantaState.getAltControlModeEnabled).onTrue(
            ss_Climber.C_Stop()
        );
    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }
    public Command alignReefForCoral()
    {
        return new ConditionalCommand( ATMan.C_ReefLeftSelectCommand(), ATMan.C_ReefRightSelectCommand(),()->{return OptionalButtonSupplier.getAsInt() == 0;}).asProxy().until(MantaState.getLimeLightBypassed);
    }
    public Command Control_AlignClosestScoreL4() {
        return alignReefForCoral().until(MantaState.getLimeLightBypassed)
        .alongWith(ScoreL4());
    }
    public Command Control_AutonAlignClosestLeftScoreL4() {
        return ATMan.C_ReefLeftSelectCommand().withTimeout(4)
        .alongWith(ScoreL4());
    }
    public Command Control_AutonAlignClosestRightScoreL4() {
        return ATMan.C_ReefRightSelectCommand().withTimeout(4)
        .alongWith(ScoreL4());
    }
    public Command ScoreL4()
    {
        return gotoL4Travel()
        .andThen(PivotIntoReefL4(),CoralDropScoreL4());
    }
    public Command Control_RearIntake()
    {
        return ATMan.C_SourceSelectCommand().until(ss_Trident.getisloaded).until(MantaState.getLimeLightBypassed).withTimeout(5)
            .alongWith(RearIntake());
    }
    public void bindNamedCommands()
    {
        // Register Named Commands
        NamedCommands.registerCommand("DoclosestSourceIntake", Control_RearIntake());
        NamedCommands.registerCommand("DoclosestScoreL4", Control_AlignClosestScoreL4());
        NamedCommands.registerCommand("DoclosestLeftScoreL4", Control_AutonAlignClosestLeftScoreL4());
        NamedCommands.registerCommand("DoclosestRightScoreL4", Control_AutonAlignClosestRightScoreL4());
        
        //unused below lol
        NamedCommands.registerCommand("Test", new InstantCommand(()->{System.out.println("running test command");}));
        NamedCommands.registerCommand("AlignprocSource",  ATMan.C_SourceSelectCommand().until(ss_Trident.getisloaded));
        NamedCommands.registerCommand("RearIntake", RearIntake());
        NamedCommands.registerCommand("Spinintake", new C_TridentIntake(ss_Trident).withTimeout(intaketimeout));
        NamedCommands.registerCommand("AlignReefLeft",  ATMan.C_ReefLeftSelectCommand().withTimeout(5));
        NamedCommands.registerCommand("AlignReefRight",  ATMan.C_ReefRightSelectCommand().withTimeout(5));
        NamedCommands.registerCommand("gotoL4Travel", gotoL4Travel());
        NamedCommands.registerCommand("PivotIntoReefL4", PivotIntoReefL4());
        NamedCommands.registerCommand("ScoreL4", new SequentialCommandGroup(PivotIntoReefL4(),CoralDropScoreL4(),ParkElevatorAndHead()));
        NamedCommands.registerCommand("ParkElevatorAndHead", ParkElevatorAndHead());

        NamedCommands.registerCommand("GroundIntakeCoral", GroundIntake());
    }

    

    
}
