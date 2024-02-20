package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Intake;

public class IntakeAndHold extends Command {
    private Shooter s_Shooter;
    private Intake s_Intake;
    private final double intakeSpeed = 0.5;
    private final double shooterReverseSpeed = -0.3;
    private final double intakeReverseSpeed = -0.1;
    public IntakeAndHold(Intake s_Intake, Shooter s_Shooter) {
        this.s_Intake = s_Intake;
        this.s_Shooter = s_Shooter;
        addRequirements(s_Intake, s_Shooter);
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

        /* change to andThen... */
        new InstantCommand(() -> s_Intake.setPower(intakeReverseSpeed))
            .andThen(new WaitCommand(0.25), new InstantCommand(() -> s_Intake.stop()))
            .execute();
    }
}
