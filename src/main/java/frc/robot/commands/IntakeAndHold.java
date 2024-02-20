package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Intake;

public class IntakeAndHold extends Command {
    private Intake s_Intake;
    private Timer s_Timer = new Timer();

    private final double intakeSpeed = 0.5;
    private final double intakeReverseSpeed = -0.2;
    private final double reverseTime = 2; //To be tested
    private final double stopTime = 2.5; //To be tested
    private boolean isDone = false;

    public IntakeAndHold(Intake s_Intake) {
        this.s_Intake = s_Intake;
        addRequirements(s_Intake);
    }
    
    @Override
    public void initialize() {
        s_Timer.reset();
        s_Timer.start();
    }

    @Override
    public void execute() {
        s_Intake.setPower(intakeSpeed);
        s_Shooter.setPower(shooterReverseSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        s_Intake.stop();
        s_Shooter.stop();
        
        new InstantCommand(() -> s_Intake.setPower(intakeReverseSpeed))
            .andThen(new WaitCommand(0.25), new InstantCommand(() -> s_Intake.stop()))
            .execute();
    }
}