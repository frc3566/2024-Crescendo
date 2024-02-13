package frc.robot.autos;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.commands.*;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;

//Middle starting position

public class NewAuto extends SequentialCommandGroup {
    public NewAuto(Swerve s_Swerve, Intake s_Intake) {
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
                List.of(new Translation2d(0.25, 0), new Translation2d(0.5, 0)),
                // End 3 meters straight ahead of where we started, facing forward
                new Pose2d(0.5, 0, new Rotation2d(0)),
                config);
    
        var thetaController =
            new ProfiledPIDController(
                s_Swerve.getPID(Math.sqrt(Math.pow(1, 2) + Math.pow(0, 2))), 0.5 , 0.08, Constants.AutoConstants.kThetaControllerConstraints);
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
            // new InstantCommand(() -> s_Shooter.setPower(1)),
            new RunIntakeWithTime(s_Intake, 0.5, 2),
            // new InstantCommand(() -> s_Shooter.stop()),
            // new MoveToPose(s_Swerve, new Pose2d(-1, 0 , Rotation2d.fromDegrees(0))),
            // new InstantCommand(() -> s_Shooter.setPower(1)),
            swerveControllerCommand1,
            new RunIntakeWithTime(s_Intake, 0.5, 2),
            new WaitCommand(10),
            new RunIntakeWithTime(s_Intake, 0.5, 2)
            // new InstantCommand(() -> s_Shooter.stop())
        );
    }
}
