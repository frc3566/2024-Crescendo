package frc.robot.autos;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;

//Middle starting position

public class FirstInTeam extends SequentialCommandGroup{
    public FirstInTeam(Swerve s_Swerve, Shooter s_Shooter, Intake s_Intake) {

        TrajectoryConfig config =
            new TrajectoryConfig(
                    Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                    Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                .setKinematics(Constants.Swerve.swerveKinematics);

        Trajectory Trajectory1 =
            TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(0, 0, new Rotation2d(0)),
                // Pass through these two interior waypoints, making an 's' curve path
                List.of(new Translation2d(-0.5, 0), new Translation2d(-1, 0)),
                // End 3 meters straight ahead of where we started, facing forward
                new Pose2d(-1, 0, new Rotation2d(0)),
                config);
    
        var thetaController =
            new ProfiledPIDController(
                Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
    
        SwerveControllerCommand swerveControllerCommand1 =
            new SwerveControllerCommand(
                Trajectory1,
                s_Swerve::getPose,
                Constants.Swerve.swerveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                thetaController,
                s_Swerve::setModuleStates,
                s_Swerve);

        addCommands(
            new InstantCommand(() -> s_Swerve.resetOdometry(Trajectory1.getInitialPose())),
            new InstantCommand(() -> s_Shooter.setPower(1)),
            new WaitCommand(3),
            new InstantCommand(() -> s_Intake.setPower(1)),
            new WaitCommand(1),
            new InstantCommand(() -> s_Intake.stop()),
            new InstantCommand(() -> s_Shooter.stop()),
            new InstantCommand(() -> s_Intake.setPower(1)),
            swerveControllerCommand1,
            new InstantCommand(() -> s_Intake.stop()),
            new InstantCommand(() -> s_Shooter.setPower(1)),
            new WaitCommand(3),
            new InstantCommand(() -> s_Intake.setPower(1)),
            new WaitCommand(1),
            new InstantCommand(() -> s_Intake.stop()),
            new InstantCommand(() -> s_Shooter.stop())
        );

    }
}
