package frc.robot.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.commands.Drive;
import frc.robot.commands.IntakeTest;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Swerve;

public class TwoPiece extends SequentialCommandGroup {
    public TwoPiece(Swerve s_Swerve, Intake s_Intake) {
        Command drive = new Drive(s_Swerve, new Pose2d());
        Command intake = new IntakeTest(s_Intake, () -> 0.7);

        Command driveAndIntake = new ParallelDeadlineGroup(drive, intake);

        addCommands(
            driveAndIntake
        );
    }
}