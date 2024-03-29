package frc.robot.commands.intake;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Intake;

public class IntakeForAuto extends SequentialCommandGroup {
    private final double intakeSpeed = 0.9;
    private final double shooterReverseSpeed = -0.2;
    private final double intakeReverseSpeed = -0.1;

    private final double intakeDeadlineSeconds = 1;    
    private final double reverseTimeSeconds = 0.5;
    private BooleanSupplier deadline;

    public IntakeForAuto(Intake s_Intake, Shooter s_Shooter) {
        System.out.println("Running IntakeForAuto");
        Command intake = new IntakeReverseShooterTimed(s_Intake, s_Shooter, () -> intakeSpeed, () -> shooterReverseSpeed, intakeDeadlineSeconds);
        Command reverseIntake = new IntakeTimed(s_Intake, () -> intakeReverseSpeed, reverseTimeSeconds);

        addCommands(
            intake.andThen(reverseIntake)
        );
    }
}
