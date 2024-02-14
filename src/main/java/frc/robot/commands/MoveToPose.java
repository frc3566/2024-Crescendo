package frc.robot.commands;

import java.util.List;
import java.util.Optional;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

public class MoveToPose extends Command {
    /** Moves the robot to the desired position */

    // Main defines;
    public static final double coefficient = 1.2;
    private boolean cancelCommand;
    private SwerveControllerCommand swerveControllerCommand;
    private Pose2d pose;
    Swerve s_Swerve;
    Vision vision;

    public MoveToPose(Swerve s_Swerve, Pose2d pose) {
        this.pose = pose;
        this.s_Swerve = s_Swerve;
        // this.vision = vision;
        // Use addRequirements() here to declare subsystem dependencies.
        // addRequirements(s_Swerve, vision);
        addRequirements(s_Swerve);
    }

    @Override
    public void initialize() {
        System.out.println("Waiting for trajectory...");

        // vision.getAprilTag().ifPresentOrElse(target -> {
            System.out.println("Running vision trajectory");
            // var trajectory = vision.getTrajectory(target);

            TrajectoryConfig config =
            new TrajectoryConfig(
                    Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                    Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                .setKinematics(Constants.Swerve.swerveKinematics);
            if (pose.getX() < 0) {
                config.setReversed(true);
            }
            Trajectory trajectory =
            TrajectoryGenerator.generateTrajectory(
                new Pose2d(0, 0, new Rotation2d(0)),
                List.of(),
                new Pose2d(-1, 0, new Rotation2d(0)),
                config);
            
            var thetaController = new ProfiledPIDController(
                s_Swerve.getPID(Math.sqrt(Math.pow(pose.getX(), 2) + Math.pow(pose.getY(), 2))), 0.5 , 0.08, Constants.AutoConstants.kThetaControllerConstraints);
                thetaController.enableContinuousInput(-Math.PI, Math.PI);

            swerveControllerCommand = new SwerveControllerCommand(
                trajectory,
                s_Swerve::getPose,
                Constants.Swerve.swerveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                thetaController,
                s_Swerve::setModuleStates,
                s_Swerve);
            
            s_Swerve.resetOdometry(trajectory.getInitialPose());
            // cancelCommand = false;
            swerveControllerCommand.schedule();
        // }, () -> {
        //     cancelCommand = true;
        //     DriverStation.reportWarning("No vision target", false);
        // });
    }
    
    @Override
    public void execute() {
        // if (swerveControllerCommand.isFinished())
        //     cancelCommand = true;
    }

    public void end(boolean interrupted) {
        // if (swerveControllerCommand == null || !swerveControllerCommand.isFinished())
        //     swerveControllerCommand.cancel();
        System.out.println("ended");
    }
    public boolean isFinished() {
        return true;
    }
}