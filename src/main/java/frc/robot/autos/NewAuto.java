package frc.robot.autos;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.commands.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

//Middle starting position

public class NewAuto extends SequentialCommandGroup {
    public NewAuto(Swerve s_Swerve, Intake s_Intake) {
        addCommands(
            // new InstantCommand(() -> s_Shooter.setPower(1)),
            new RunIntakeWithTime(s_Intake, 0.5, 2),
            // new InstantCommand(() -> s_Shooter.stop()),
            new MoveToPose(s_Swerve, new Pose2d(-1, 0 , Rotation2d.fromDegrees(0))),
            // new InstantCommand(() -> s_Shooter.setPower(1)),
            new WaitCommand(5),
            new RunIntakeWithTime(s_Intake, 0.5, 2),
            new WaitCommand(10),
            new RunIntakeWithTime(s_Intake, 0.5, 2)
            // new InstantCommand(() -> s_Shooter.stop())
        );
    }
}
