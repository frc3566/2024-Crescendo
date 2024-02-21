package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Intake;
import frc.robot.commands.*;

public class IntakeAndHold extends SequentialCommandGroup {

    public IntakeAndHold(Intake s_Intake, Shooter s_Shooter) {

        double intakeReverseSpeed = 0.7;

        Command intake = new TakeIn(s_Shooter, s_Intake);
        Command buffer = new IntakeTest(s_Intake, () -> intakeReverseSpeed, 0.1);

        addCommands(
            intake, 
            andThen(buffer)
        );
    }
}
