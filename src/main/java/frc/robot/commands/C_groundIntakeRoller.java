package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants;
import frc.robot.subsystems.groundIntake.intakeRoller;

public class C_groundIntakeRoller extends Command{
    intakeRoller subSystem;
    double speed;
    public C_groundIntakeRoller(intakeRoller subsystem, double RollerSpeed){
        subSystem = subsystem;
        speed = RollerSpeed;
        addRequirements(subsystem);
    }
    @Override
    public void initialize(){
        subSystem.setRollerSpeed(speed);
    }

    @Override
    public boolean isFinished(){
        return subSystem.getHasCoral();
    }
    
    @Override
    public void end(boolean interrupted) {
        subSystem.holdPosition();
    }
}
