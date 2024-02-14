package frc.robot.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.commands.Drive;
import frc.robot.commands.IntakeTest;
import frc.robot.commands.PrimeAndShoot;
import frc.robot.commands.Shoot;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;

public class TwoPiece extends SequentialCommandGroup {
    public TwoPiece(Swerve s_Swerve, Intake s_Intake, Shooter s_Shooter) {
        Command driveToFirstPosition = new Drive(s_Swerve, new Pose2d());
        Command shoot = new PrimeAndShoot(s_Shooter, 0.7);
        Command driveAndIntake = new DriveAndIntake(s_Swerve, s_Intake, new Pose2d());
        

        addCommands(
            driveToFirstPosition,
            shoot,
            driveAndIntake
        );
    }
}

class DriveAndIntake extends SequentialCommandGroup {
    public DriveAndIntake(Swerve s_Swerve, Intake s_Intake, Pose2d pose) {
        Command drive = new Drive(s_Swerve, pose);
        Command intake = new IntakeTest(s_Intake, () -> 0.7);

        Command driveAndIntake = new ParallelDeadlineGroup(drive, intake);

        addCommands(
            driveAndIntake
        );
    }
}