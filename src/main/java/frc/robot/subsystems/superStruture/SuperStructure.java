package frc.robot.subsystems.superStruture;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WrapperCommand;
import frc.robot.RobotContainer;
import frc.robot.constants;
import frc.robot.constants.Climber.CatchSide;
import frc.robot.constants.Climber.SlideSide;
import frc.robot.constants.PlasmaExtension;
import frc.robot.constants.PlasmaPivot;
import frc.robot.constants.RearMotorizedIntake;
import frc.robot.AlphaBots.LimeLightPoseFilter;
import frc.robot.AlphaBots.Tools;
import frc.robot.AlphaBots.AprilTagSystem.SelectCommands;
import frc.robot.commands.C_CatchMotorToPosition;
import frc.robot.commands.C_ClearRearIntake;
import frc.robot.commands.C_DropElevateToScore;
import frc.robot.commands.C_ElevateToPosition;
import frc.robot.commands.C_ExtendToPosition;
import frc.robot.commands.C_PivotToPosition;
import frc.robot.commands.C_SlideMotorToPosition;
import frc.robot.commands.C_TridentIntake;
import frc.robot.subsystems.ArmExtension;
import frc.robot.subsystems.CANdleSubsystem;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.MantaRay;
import frc.robot.subsystems.MantaState;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.RearIntake;
import frc.robot.subsystems.josiahClimber;

public class SuperStructure extends SubsystemBase{
    
    public final MantaRay ss_Trident;
    public final Elevator ss_Elevator;
    public final Pivot ss_Pivot;
    public final ArmExtension ss_ArmExtension;
    public final josiahClimber ss_Climber;
    
    public final CANdleSubsystem Candle;
    public final RearIntake ss_RearIntake;

    public double processorAlgaeScoringDutyCycle = -.35;
    public double BargeAlgaeScoringDutyCycle = -1.0;//-.75;
    //started working on this, not done yet
    public double intaketimeout = 20;
    public double groundintakeTimeout = 20; //auton this command will run until finished or this timeout.
    public double groundintakedutycycle = 1.0;
    public boolean isbargeing = false; 
    public DoubleSupplier getYAxis = ()->{ return RobotContainer.joystick.getLeftX();};

    public SuperStructure(
        MantaRay _ss_Trident,Elevator _ss_Elevator,Pivot _ss_Pivot,ArmExtension _ss_ArmExtension,
        josiahClimber _ss_Climber,CANdleSubsystem _Candle,RearIntake _ss_RearIntake
    )
    {
        ss_Trident = _ss_Trident;
        ss_Elevator = _ss_Elevator;
        ss_Pivot = _ss_Pivot;
        ss_ArmExtension = _ss_ArmExtension;
        ss_Climber = _ss_Climber;
        Candle = _Candle;
        ss_RearIntake = _ss_RearIntake;
    }

@Override
public void periodic() {
    //TODO: put the elevator slam protection here
    doTravelIfInCorrectPosition();
}

public void doTravelIfInCorrectPosition()
  {
    double _requestedPosition = ss_Elevator.requestedHeight.getAsDouble();
    
    if(MantaState.ss_RearIntake.LaserDetectsCoral())
    {
      return; 
    }
    if (ss_Elevator.setpointHeight.getAsDouble() != _requestedPosition) {
      
      //if we are above the CannotFoldBelow position
      if(ss_Elevator.currentHeight.getAsDouble() > constants.Elevator.CannotPivotParkBelowElevatorPosition)
      {
        //if we are going below the cannot fold position
        if(_requestedPosition <= constants.Elevator.CannotPivotParkBelowElevatorPosition)
        {
          //check if pivot is in a safe travel position
          if(MantaState.ss_Pivot.IsPivotFoldedOut.getAsBoolean()) //IsPivotinTravelPosition
          {
             //if/when we are folded out, set position to requested position
            //safe to goto requestion position
            ss_Elevator.GotoPosition(_requestedPosition);
          }
          else{
            //if not IsPivotinTravelPosition, set position to "cannotfoldbelowPosition"
            //ONLY safe to goto CannotFoldBelowPosition
            ss_Elevator. GotoPosition(constants.Elevator.CannotPivotParkBelowElevatorPosition);
          }
        }
        else{
          //if we are above the safe zone and staying above the safe zone then request the new position. 
          ss_Elevator.GotoPosition(_requestedPosition);
        }
      }
      //else if we are in the void zone only allow travel mode
      else if(ss_Elevator.currentHeight.getAsDouble() > constants.Elevator.CannotPivotParkAboveElevatorPosition && ss_Elevator.currentHeight.getAsDouble() < constants.Elevator.CannotPivotParkBelowElevatorPosition)
      {
        //if we are above the CannotFoldBelow position
      if(ss_Elevator.currentHeight.getAsDouble() > constants.Elevator.CannotPivotParkAboveElevatorPosition)
      {
        //if we are going below the cannot fold position
        if(_requestedPosition <= constants.Elevator.CannotPivotParkAboveElevatorPosition)
        {
          //check if pivot is in a safe travel position
          if(MantaState.ss_Pivot.IsPivotFoldedOut.getAsBoolean()) //IsPivotinTravelPosition
          {
             //if/when we are folded out, set position to requested position
            //safe to goto requestion position
            ss_Elevator.GotoPosition(_requestedPosition);
          }
          else{
            //if not IsPivotinTravelPosition, set position to "cannotfoldbelowPosition"
            //ONLY safe to goto CannotFoldBelowPosition
            ss_Elevator.GotoPosition(constants.Elevator.CannotPivotParkBelowElevatorPosition);
          }
        }
        else{
          //if we are above the safe zone and staying above the safe zone then request the new position. 
          ss_Elevator.GotoPosition(_requestedPosition);
        }
      }
      }   
      //if we are below the CannotFoldabove position
      else if(ss_Elevator.currentHeight.getAsDouble() < constants.Elevator.CannotPivotParkAboveElevatorPosition)
      {
        //if we are going above the cannot fold position
        if(_requestedPosition > constants.Elevator.CannotPivotParkAboveElevatorPosition)
        {
          //check if pivot is in a safe travel position
          if(MantaState.ss_Pivot.IsPivotinTravelPosition.getAsBoolean())
          {
              //if/when we are folded out, set position to requested position
            //safe to goto requestion position
            ss_Elevator.GotoPosition(_requestedPosition);
          }
          else{
            //if not IsPivotinTravelPosition, set position to "cannotfoldbelowPosition"
            //ONLY safe to goto CannotFoldBelowPosition
            ss_Elevator.GotoPosition(constants.Elevator.CannotPivotParkAboveElevatorPosition);
          }
        }
        else{
          //if we are below the safe zone and going below the safe zone then request the new position. 
          ss_Elevator.GotoPosition(_requestedPosition);
        }
      }//if we are not above the nogo and we are not below the nogo we are in the nogo. make sure we are in travel position and goto the called position
      else 
      {
        //check if pivot is in a safe travel position
        if(MantaState.ss_Pivot.IsPivotinTravelPosition.getAsBoolean())
        {
            //if/when we are folded out, set position to requested position
          //safe to goto requestion position
          ss_Elevator.GotoPosition(_requestedPosition);
        }
        else{
          //if not IsPivotinTravelPosition, dont move we are in the No-go zone already. 
        }
      }
    }// else if we are close to parked and we are requesting a park. then just brake mode. 
    else if ((ss_Elevator.setpointHeight.getAsDouble() < constants.Elevator.ElevatorBrakeParkTolerance) 
          & (_requestedPosition < constants.Elevator.ElevatorBrakeParkTolerance)
          &  ss_Elevator.elevatorVelocity.getAsDouble() < 100
          & Tools.isPosAtSetpoint(ss_Elevator.currentHeight.getAsDouble(), constants.Elevator.minElevatorHeight, constants.Elevator.ElevatorBrakeParkTolerance))
    {
      //System.out.println("elevator Braking");
      ss_Elevator.BRAKE();
    }
  }
  

    public Command gotoMinTravel()
    {
        return ss_Elevator.GotoPositonCommand(frc.robot.constants.Elevator.minElevatorHeight).alongWith(GotoTravelPostion());
    }

    public Command gotoL1Travel()
    {
        return ss_Elevator.GotoPositonCommand(frc.robot.constants.Elevator.l1Position).alongWith(GotoTravelPostion());
    }

    public Command gotoL2Travel()
    {
        return ss_Elevator.GotoPositonCommand(frc.robot.constants.Elevator.l2Position).alongWith(GotoTravelPostion());
    }

    public Command gotoL3Travel()
    {
        return ss_Elevator.GotoPositonCommand(frc.robot.constants.Elevator.l3Position).alongWith(GotoTravelPostion());
    }

    public Command gotoL4Travel()
    {
        return ss_Elevator.GotoPositonCommand(frc.robot.constants.Elevator.l4Position).alongWith(GotoTravelPostion());
    }

    public Command gotoUpperAlgaeTravel()
    {
        return ss_Elevator.GotoPositonCommand(frc.robot.constants.Elevator.l3algae).alongWith(GotoTravelPostion());
    }

    public Command gotoLowerAlgaeTravel()
    {
        return ss_Elevator.GotoPositonCommand(frc.robot.constants.Elevator.l2Algae).alongWith(GotoTravelPostion());
    }

    public Command AlgaeReefIntake()
    {
        return
            new C_PivotToPosition(ss_Pivot, PlasmaPivot.AlgaeReefPickup)
            .andThen(new C_ExtendToPosition(ss_ArmExtension,PlasmaExtension.ReefAlgaePickupExtension))
            .alongWith(new C_TridentIntake(ss_Trident,ss_RearIntake).withTimeout(5))
            .andThen(new C_ExtendToPosition(ss_ArmExtension,PlasmaExtension.parkPostion,true)
            .alongWith(new C_TridentIntake(ss_Trident,ss_RearIntake).asProxy().withTimeout(.4),
            new C_PivotToPosition(ss_Pivot, PlasmaPivot.TravelPosition))
            );
    }

    public Command Auto_AlgaeReefIntake()
    {
        return
            new C_PivotToPosition(ss_Pivot, PlasmaPivot.AlgaeReefPickup)
            .andThen(new C_ExtendToPosition(ss_ArmExtension,PlasmaExtension.ReefAlgaePickupExtension))
            .alongWith(new C_TridentIntake(ss_Trident,ss_RearIntake).withTimeout(5))
            .andThen(new C_ExtendToPosition(ss_ArmExtension,PlasmaExtension.parkPostion,true)
            .alongWith(new C_TridentIntake(ss_Trident,ss_RearIntake).withTimeout(.4),
            new C_PivotToPosition(ss_Pivot, PlasmaPivot.TravelPosition))
            );
    }

    public Command GotoTravelPostion()
    {
        return new C_PivotToPosition(ss_Pivot,PlasmaPivot.TravelPosition)
            .alongWith(new C_ExtendToPosition(ss_ArmExtension, PlasmaExtension.parkPostion,true));//ss_Pivot.C_GotoPositon(constants.PlasmaPivot.TravelPosition);
    }

    public SequentialCommandGroup GotoBargePosition() {
        return ss_Elevator.GotoPositonCommand(frc.robot.constants.Elevator.l4Position)
        .alongWith(new C_PivotToPosition(ss_Pivot,PlasmaPivot.TravelPosition).withTimeout(.25),
        new InstantCommand(()->{isbargeing = true;})
        )
        .andThen(
            new C_ExtendToPosition(ss_ArmExtension, PlasmaExtension.maxposition).alongWith(new C_PivotToPosition(ss_Pivot,PlasmaPivot.BargePosition))     
            );
    }

    public Command elevator1StepPark(){
        return new C_ElevateToPosition(ss_Elevator, frc.robot.constants.Elevator.minElevatorHeight);
    }

    public Command ParkElevatorAndHead()
    {
        return elevator1StepPark().unless(ss_Elevator.elevatorisparked)//new C_ElevateToPosition(ss_Elevator,constants.Elevator.minElevatorHeight)
        .alongWith(new C_ExtendToPosition(ss_ArmExtension,PlasmaExtension.parkPostion,true))
        .alongWith(new C_PivotToPosition(ss_Pivot,PlasmaPivot.TravelPosition)
            .unless(()->{return ss_Pivot.IsPivotParked.getAsBoolean() && ss_Elevator.elevatorisparked.getAsBoolean();}) //Why is this here? it seems redundant?
            ).andThen(new C_PivotToPosition(ss_Pivot,PlasmaPivot.ParkPosition));
    }

    public Command PivotIntoReefl3(){
        return new C_PivotToPosition(ss_Pivot, PlasmaPivot.l3ReadyPosition)
                .alongWith(new C_ExtendToPosition(ss_ArmExtension,PlasmaExtension.l3ReadyPosition));
    }

    public Command PivotIntoReefL1(){
        return new C_PivotToPosition(ss_Pivot, PlasmaPivot.l1ReadyPosition)
                .alongWith(new C_ExtendToPosition(ss_ArmExtension,PlasmaExtension.l1ReadyPosition));
    }

    public Command PivotIntoReefL2(){
        return new C_PivotToPosition(ss_Pivot, PlasmaPivot.l2ReadyPosition)
                .alongWith(new C_ExtendToPosition(ss_ArmExtension,PlasmaExtension.l2ReadyPosition));
    }

    public Command PivotIntoReefL4()
    {
        return new C_PivotToPosition(ss_Pivot, PlasmaPivot.l4ReadyPosition)
                .alongWith(new C_ExtendToPosition(ss_ArmExtension,PlasmaExtension.l4ScorePosition));
    }

    public Command CoralDropScoreL4()
    {
        return new C_DropElevateToScore(ss_Elevator)
        .deadlineFor(TridentCoralBumpOut()
        .alongWith(
            new C_PivotToPosition(ss_Pivot, PlasmaPivot.l4ReadyPosition + PlasmaPivot.L4CoralDropPivotAmount)
            )
            .finallyDo(()->{ss_Trident.setDutyCycle(0);}));
        //
        // return new C_PivotToPosition(ss_Pivot, constants.PlasmaPivot.l2ScorePosition)
        //         .alongWith(new C_ExtendToPosition(ss_ArmExtension,constants.PlasmaExtension.l2ScorePosition))
        //         .alongWith(TridentCoralBumpOut());
    }

    public Command CoralDropScoreL2()
    {
        return new C_DropElevateToScore(ss_Elevator)
            .alongWith(new C_PivotToPosition(ss_Pivot,PlasmaPivot.SideScorePosition))//.withTimeout(1)
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

    public Command TridentRunReverse()
    {
        return ss_Trident.bumpout()
        .andThen(new WaitCommand(2))
        .andThen(ss_Trident.Stop());
    }

    public Command TridentAlgaeBumpOut()
    {
        return ss_Trident.bumpout(processorAlgaeScoringDutyCycle)
        .andThen(new WaitCommand(1))
        .andThen(ss_Trident.Stop());
    }

    public Command TridentBargeAlgaeBumpOut()
    {
        return ss_Trident.bumpout(BargeAlgaeScoringDutyCycle)
        .andThen(new WaitCommand(.5))
        .finallyDo(()->{ss_Trident.HoldPosition();});
        //.andThen(ss_Trident.Stop());
    }

    public ParallelDeadlineGroup RearIntake(){
        return new ParallelDeadlineGroup(
            new C_TridentIntake(ss_Trident,ss_RearIntake).withTimeout(intaketimeout),
            new C_PivotToPosition(ss_Pivot, constants.PlasmaPivot.rearintakePos),
            new C_ElevateToPosition(ss_Elevator, constants.Elevator.minElevatorHeight),
            new C_ExtendToPosition(ss_ArmExtension,constants.PlasmaExtension.rearintakePos,true)
            );
    }

    public ParallelDeadlineGroup Auto_RearIntake(){
        return new ParallelDeadlineGroup(
            new C_TridentIntake(.1,ss_Trident,ss_RearIntake).withTimeout(intaketimeout),
            new C_PivotToPosition(ss_Pivot, constants.PlasmaPivot.rearintakePos),
            new C_ElevateToPosition(ss_Elevator, constants.Elevator.minElevatorHeight),
            new C_ExtendToPosition(ss_ArmExtension,constants.PlasmaExtension.rearintakePos,true)
            );
    }

    public Command DebugIntake(){
        return new ParallelCommandGroup(
            new C_TridentIntake(ss_Trident,ss_RearIntake,.75).withTimeout(intaketimeout)
            );
    }

    public Command GroundIntake(){
        return new ParallelCommandGroup(
            new C_ElevateToPosition(ss_Elevator, constants.Elevator.groundPickup),
            new C_PivotToPosition(ss_Pivot, constants.PlasmaPivot.GroundPickupPosition),
            new SequentialCommandGroup(
                new C_ExtendToPosition(ss_ArmExtension,constants.PlasmaExtension.climbExtension),
            new WaitCommand(.1),//small delay to stop motor from smasshing into rear intake. might not be needed when motor is 90 in future. 
            new C_ExtendToPosition(ss_ArmExtension,constants.PlasmaExtension.GroundPickupExtension,true),
            //new WaitCommand(.1),//small delay to debounce the head moving and causing high amps. 
            new C_TridentIntake(3.0,ss_Trident,ss_RearIntake,groundintakedutycycle).withTimeout(groundintakeTimeout)
            )
            )
        .finallyDo(groundIntakeReset());
    }

    public Command AutoGroundIntake(){
        return new ParallelCommandGroup(
            new C_ElevateToPosition(ss_Elevator, constants.Elevator.groundPickup),
            new C_PivotToPosition(ss_Pivot, constants.PlasmaPivot.GroundPickupPosition),
            new SequentialCommandGroup(
                new C_ExtendToPosition(ss_ArmExtension,constants.PlasmaExtension.climbExtension),
                new WaitCommand(.1),//small delay to stop motor from smasshing into rear intake. might not be needed when motor is 90 in future. 
                new C_ExtendToPosition(ss_ArmExtension,constants.PlasmaExtension.GroundPickupExtension,true),
                new WaitCommand(.1),//small delay to debounce the head moving and causing high amps. 
                new C_TridentIntake(ss_Trident,ss_RearIntake,groundintakedutycycle).withTimeout(groundintakeTimeout))
            );
    }

    public Command GroundIntakeAngled(){
        return new ParallelCommandGroup(
            new C_ElevateToPosition(ss_Elevator, constants.Elevator.AngledgroundPickup),
            new C_PivotToPosition(ss_Pivot, constants.PlasmaPivot.AngledGroundPickupPos),
            new SequentialCommandGroup(
                new C_ExtendToPosition(ss_ArmExtension,constants.PlasmaExtension.climbExtension),
            new WaitCommand(.1),//small delay to stop motor from smasshing into rear intake. might not be needed when motor is 90 in future. 
            new C_ExtendToPosition(ss_ArmExtension,constants.PlasmaExtension.maxposition,true),
            new WaitCommand(.1),//small delay to debounce the head moving and causing high amps. 
            new C_TridentIntake(ss_Trident,ss_RearIntake,groundintakedutycycle).withTimeout(groundintakeTimeout)
            )
            )
        .finallyDo(groundIntakeReset());
    }

    public Runnable groundIntakeReset(){
        return ()->{
            GotoTravelPostion().alongWith(new C_ElevateToPosition(ss_Elevator, frc.robot.constants.Elevator.minElevatorHeight)).schedule();
        };
    }

    public Runnable traveltopark()
    {
        return ()->{ParkElevatorAndHead().schedule();};
    }

    public SequentialCommandGroup GetClosestAlgae() {
        return SelectCommands.C_ReefCenterAlgaeSelectCommand().asProxy().until(MantaState.getLimeLightBypassed)
        .alongWith(new ConditionalCommand(gotoUpperAlgaeTravel(),gotoLowerAlgaeTravel(),MantaState.NearestTagIsUpperAlgae))//.withTimeout(2)
        .andThen(AlgaeReefIntake(),gotoMinTravel());
    }

    public SequentialCommandGroup Auto_GetClosestAlgae() {
        return SelectCommands.C_ReefCenterAlgaeSelectCommand().withTimeout(1.5)
        .alongWith(new ConditionalCommand(gotoUpperAlgaeTravel(),gotoLowerAlgaeTravel(),MantaState.NearestTagIsUpperAlgae))//.withTimeout(2)
        .andThen(Auto_AlgaeReefIntake(),gotoMinTravel());
    }

    public ParallelCommandGroup ClimbHookReady() {
        return ss_Climber.C_CatchGotoPositon(CatchSide.maxPostion).alongWith(
        ss_Climber.C_SlideGotoPositon(SlideSide.maxPostion)
      );
    }

    public ParallelCommandGroup ClimbHookStartFlat() {
        return  ss_Climber.C_CatchGotoPositon(CatchSide.startPos).alongWith(
            ss_Climber.C_SlideGotoPositon(SlideSide.startPos)
      );
    }

    public Command C_ClimbHookStartFlat() {
        return  new C_CatchMotorToPosition(ss_Climber,CatchSide.startPos).alongWith(
            new C_SlideMotorToPosition(ss_Climber,SlideSide.startPos)
      );
    }

    public Command alignReefForCoral()
    {
        return new ConditionalCommand( SelectCommands.C_ReefLeftSelectCommand(), SelectCommands.C_ReefRightSelectCommand(),()->{return RobotContainer.OptionalButtonSupplier.getAsInt() == 0;}).asProxy().until(MantaState.getLimeLightBypassed);
    }

    public Command Control_AlignClosestScoreL4() {
        return alignReefForCoral().until(MantaState.getLimeLightBypassed)
        .alongWith(gotoL4Travel().andThen(new C_ExtendToPosition(ss_ArmExtension,PlasmaExtension.l4ScorePosition)))
        .andThen(PivotIntoReefL4(),CoralDropScoreL4().withTimeout(.25));
    }

    public Command Control_AutonAlignClosestLeftScoreL4() {
        return SelectCommands.C_ReefLeftSelectCommand().withTimeout(1.5)
        .alongWith(ScoreL4());
    }

    public Command Control_AutonAlignClosestRightScoreL4() {
        return SelectCommands.C_ReefRightSelectCommand().withTimeout(1.5)
        .alongWith(ScoreL4());
    }

    public Command ScoreL4()
    {
        return gotoL4Travel()
        .andThen(PivotIntoReefL4(),CoralDropScoreL4().withTimeout(.25));//timeout incase we get stuck then just auto reset 
    }

    public Command Control_RearIntake()
    {
        return SelectCommands.C_SourceSelectCommand().asProxy().until(ss_Trident.getisloaded).until(MantaState.getLimeLightBypassed).withTimeout(6)
            .alongWith(RearIntake().andThen(new ParallelCommandGroup(new C_PivotToPosition(ss_Pivot, constants.PlasmaPivot.TravelPosition),
            new C_TridentIntake(ss_Trident,ss_RearIntake).withTimeout(intaketimeout))
            )
            .andThen(new ScheduleCommand(new C_ClearRearIntake(ss_RearIntake)))
            );
    }

    public Command Control_AutoRearIntake()
    {
        return SelectCommands.C_SourceSelectCommand().until(ss_Trident.getisloaded).until(MantaState.getLimeLightBypassed).withTimeout(3)
        .alongWith(
            RearIntake()//All post commands are in pathplanner!
        );
    }

    public Command control_PivotPullIntakeCommand()
    {
        return new ParallelCommandGroup(new C_PivotToPosition(ss_Pivot, constants.PlasmaPivot.TravelPosition),
        new C_TridentIntake(ss_Trident,ss_RearIntake).withTimeout(intaketimeout));
    }

    public Command control_AutoBarge()
    {
        return SelectCommands.C_BargeSelectCommand(getYAxis).until(MantaState.getLimeLightBypassed).withTimeout(3)
            .alongWith(GotoBargePosition())
            .andThen(TridentBargeAlgaeBumpOut());
    }

        public Command ScoreBarge()
    {
        return 
        SelectCommands.C_BargeSelectCommand(getYAxis).asProxy().until(MantaState.getLimeLightBypassed).withTimeout(3)
        .alongWith(
            GotoBargePosition()
            )
        .andThen(TridentBargeAlgaeBumpOut())
        .finallyDo(traveltopark());
    }

        public WrapperCommand ScoreL1() {
            return SelectCommands.C_ReefL1CenterSelectCommand().asProxy().until(MantaState.getLimeLightBypassed)
            .alongWith(gotoL1Travel())
            .andThen(PivotIntoReefL1(),TridentCoralShootOut(),ParkElevatorAndHead()).finallyDo(traveltopark());
        }

        public WrapperCommand ScoreL2() {
            return alignReefForCoral()
            .alongWith(gotoL2Travel())
            .andThen(PivotIntoReefL2(),CoralDropScoreL2(),ParkElevatorAndHead()).finallyDo(traveltopark());
        }

        public WrapperCommand ScoreL3() {
            return alignReefForCoral()
            .alongWith(gotoL3Travel())
            .andThen(PivotIntoReefl3(),CoralDropScoreL2(),ParkElevatorAndHead()).finallyDo(traveltopark());
        }

        public WrapperCommand ScoreL4(RobotContainer robotContainer) {
            return Control_AlignClosestScoreL4()
            .andThen(GetClosestAlgae().unless(()->{return !robotContainer.joystick.x().getAsBoolean();}))
            .andThen(ParkElevatorAndHead()).finallyDo(traveltopark());
        }

        public Command Btn_Park()
        {
            return ParkElevatorAndHead().alongWith(new InstantCommand(()->{isbargeing = false;}));
        }

        public ParallelCommandGroup GotoProcessorPos(RobotContainer robotContainer) {
            return new InstantCommand(()->{})//SelectCommands.C_ProcessorSelectCommand().asProxy().until(MantaState.getLimeLightBypassed)
            .alongWith(
                new C_ElevateToPosition(robotContainer.ss_Elevator, frc.robot.constants.Elevator.minElevatorHeight),
                new C_PivotToPosition(robotContainer.ss_Pivot, PlasmaPivot.processorPivot),
                new C_ExtendToPosition(robotContainer.ss_ArmExtension, PlasmaExtension.processorExtension,true)
                )
            //.andThen(TridentAlgaeBumpOut().withTimeout(.5).finallyDo(()->{ss_Trident.HoldPosition(); traveltopark();}))
        ;
        }

        public WrapperCommand GetAlgaeFromReef() {
            return //
            GetClosestAlgae().finallyDo(traveltopark());
        }

        public WrapperCommand Btn_ManualBumpOut(RobotContainer robotContainer) {
            return TridentBargeAlgaeBumpOut().alongWith(new InstantCommand(()->{robotContainer.ss_RearIntake.GotoDutyCycle(-RearMotorizedIntake.ReversingdutyCyclePercent);})).finallyDo(()->{robotContainer.ss_Trident.HoldPosition(); robotContainer.ss_RearIntake.COAST();});
        }

        public InstantCommand Btn_BypassLimelight() {
            return new InstantCommand(()->{MantaState.setLimeLightBypassed(true);});
        }

        public InstantCommand Btn_EnableLimelight() {
            return new InstantCommand(()->{MantaState.setLimeLightBypassed(false);});
        }

        public ParallelCommandGroup Btn_DisableClimbMode(RobotContainer robotContainer) {
            return new InstantCommand(()->{MantaState.setAltControlModeEnabled(false);})
          .alongWith(
            ClimbHookStartFlat(),
            new C_ExtendToPosition(robotContainer.ss_ArmExtension, PlasmaExtension.minposition)
            );
        }

        public ParallelCommandGroup Btn_EnableClimbMode(RobotContainer robotContainer) {
            return new InstantCommand(()->{MantaState.setAltControlModeEnabled(true);})
          .alongWith(
            new C_PivotToPosition(robotContainer.ss_Pivot, PlasmaPivot.ParkPosition)
            ,new C_ExtendToPosition(robotContainer.ss_ArmExtension, PlasmaExtension.climbExtension)
            ,ClimbHookReady()
            );
        }

        public SequentialCommandGroup Btn_GotoL4DebugMode(RobotContainer robotContainer) {
            return robotContainer.ss_Elevator.GotoPositonCommand(frc.robot.constants.Elevator.l4Position)
            .alongWith(GotoTravelPostion().withTimeout(1))
            .andThen(new C_ExtendToPosition(robotContainer.ss_ArmExtension, PlasmaExtension.maxposition));
        }

        public SequentialCommandGroup Btn_ResetVision() {
            return new InstantCommand(()->{LimeLightPoseFilter.DoResetVision();})
          .andThen(
            new WaitCommand(.01),
            new InstantCommand(()->{LimeLightPoseFilter.DoResetVision();}),
            new WaitCommand(.01),
            new InstantCommand(()->{LimeLightPoseFilter.DoResetVision();})
            );
        }

        public SequentialCommandGroup Btn_ClimbNow(RobotContainer robotContainer) {
            return C_ClimbHookStartFlat()//.withTimeout(1).unless(()->{return ss_Climber.getCatchPosition() < constants.Climber.CatchSide.startPos;})
            .andThen(
                robotContainer.ss_Climber.C_CatchGotoPositon(CatchSide.minPostion),
                robotContainer.ss_Climber.C_SlideGotoPositon(SlideSide.minPostion))
                //wait command acts as timeout since if the match ends the motor stops anyway
            .andThen(new WaitCommand(3.0),robotContainer.ss_Climber.C_Stop());
        }

}
