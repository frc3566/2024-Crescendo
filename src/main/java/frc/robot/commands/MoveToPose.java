// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.math.trajectory.constraint.TrajectoryConstraint;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.List;
import java.util.function.Supplier;
import frc.robot.Constants;
import frc.robot.subsystems.*;
import frc.lib.CustomTrajectoryGenerator;
import frc.lib.RotationSequence;
import frc.lib.Waypoint;
import frc.lib.CustomHolonomicDriveController;
import frc.lib.util.AllianceFlipUtil;

public class MoveToPose extends Command {

  private static boolean supportedRobot = true;
  private static double maxVelocityMetersPerSec;
  private static double maxAccelerationMetersPerSec2;
  private static double maxCentripetalAccelerationMetersPerSec2;

  private final PIDController xController = new PIDController(0.0, 0.0, 0.0);
  private final PIDController yController = new PIDController(0.0, 0.0, 0.0);
  private final PIDController thetaController = new PIDController(0.0, 0.0, 0.0);

  private final CustomHolonomicDriveController customHolonomicDriveController =
      new CustomHolonomicDriveController(xController, yController, thetaController);

  private Swerve s_Swerve = new Swerve(null, null, null, null);
  private final Timer timer = new Timer();

  private List<Waypoint> waypoints;
  private Supplier<List<Waypoint>> waypointsSupplier = null;
  private Supplier<List<TrajectoryConstraint>> constraintsSupplier = null;
  private Supplier<Double> startVelocitySupplier = null;
  private CustomTrajectoryGenerator customGenerator = new CustomTrajectoryGenerator();

  private final double driveKp = Constants.Swerve.driveKP;
  private final double driveKi = Constants.Swerve.driveKI;
  private final double driveKd = Constants.Swerve.driveKD;

  private final double turnKp = Constants.Swerve.angleKP;
  private final double turnKi = Constants.Swerve.angleKI;
  private final double turnKd = Constants.Swerve.angleKD;

  /** Creates a DriveTrajectory command with a dynamic set of waypoints. */
  public MoveToPose(Swerve s_Swerve, Supplier<List<Waypoint>> waypointsSupplier) {
    this.s_Swerve = s_Swerve;
    this.waypointsSupplier = waypointsSupplier;
  }

  /** Creates a DriveTrajectory command with a dynamic set of waypoints and constraints. */
  public MoveToPose(
      Swerve swerve,
      Supplier<List<Waypoint>> waypointsSupplier,
      Supplier<List<TrajectoryConstraint>> constraintsSupplier,
      Supplier<Double> startVelocitySupplier) {
    this.s_Swerve = s_Swerve;
    addRequirements(s_Swerve);
    this.waypointsSupplier = waypointsSupplier;
    this.constraintsSupplier = constraintsSupplier;
    this.startVelocitySupplier = startVelocitySupplier;
  }

  /** Creates a DriveTrajectory command with a static set of waypoints. */
  public MoveToPose(Swerve s_Swerve, List<Waypoint> waypoints) {
    this.s_Swerve = s_Swerve;
    this.waypoints = waypoints;
  }

  /** Creates a DriveTrajectory command with a static set of waypoints and constraints. */
  public MoveToPose(
      Swerve s_Swerve,
      List<Waypoint> waypoints,
      List<TrajectoryConstraint> constraints,
      double startVelocity) {
    this.s_Swerve = s_Swerve;
    addRequirements(s_Swerve);
    generate(waypoints, constraints, startVelocity, true);
  }

  /** Generates the trajectory. */
  private void generate(
      List<Waypoint> waypoints,
      List<TrajectoryConstraint> constraints,
      double startVelocity,
      boolean alertOnFail) {
    // Set up trajectory configuration
    TrajectoryConfig config =
        new TrajectoryConfig(maxVelocityMetersPerSec, maxAccelerationMetersPerSec2)
            .setKinematics(new SwerveDriveKinematics(s_Swerve.getModuleTranslations()))
            .setStartVelocity(startVelocity)
            .setEndVelocity(0.0)
            .addConstraint(
                new CentripetalAccelerationConstraint(maxCentripetalAccelerationMetersPerSec2))
            .addConstraints(constraints);

    // Generate trajectory
    customGenerator = new CustomTrajectoryGenerator(); // Reset generator
    try {
      customGenerator.generate(config, waypoints);
    } catch (Exception exception) {
      if (supportedRobot && alertOnFail) {
        exception.printStackTrace();
      }
    }
  }

  @Override
  public void initialize() {
    // Generate trajectory if supplied
    if (waypointsSupplier != null || constraintsSupplier != null) {
      generate(
          waypointsSupplier.get(), constraintsSupplier.get(), startVelocitySupplier.get(), false);
    }

    // Log trajectory

    // Reset all controllers
    timer.reset();
    timer.start();
    xController.reset();
    yController.reset();
    thetaController.reset();

    // Reset PID gains
    xController.setP(driveKp);
    xController.setD(driveKd);
    yController.setP(driveKp);
    yController.setD(driveKd);
    thetaController.setP(turnKp);
    thetaController.setD(turnKd);
  }

  @Override
  public void execute() {
    
    xController.setP(driveKp);
    xController.setD(driveKd);
    yController.setP(driveKp);
    yController.setD(driveKd);
    thetaController.setP(turnKp);
    thetaController.setD(turnKd);
    

    // Exit if trajectory generation failed
    if (customGenerator.getDriveTrajectory().getStates().size() <= 1) {
      return;
    }

    // Get setpoint
    Trajectory.State driveState =
        AllianceFlipUtil.apply(customGenerator.getDriveTrajectory().sample(timer.get()));
    RotationSequence.State holonomicRotationState =
        AllianceFlipUtil.apply(customGenerator.getHolonomicRotationSequence().sample(timer.get()));

    // Calculate velocity
    ChassisSpeeds nextDriveState =
        customHolonomicDriveController.calculate(
            s_Swerve.getPose(), driveState, holonomicRotationState);
    s_Swerve.runVelocity(nextDriveState);
  }

  @Override
  public void end(boolean interrupted) {
    s_Swerve.off();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.hasElapsed(customGenerator.getDriveTrajectory().getTotalTimeSeconds());
  }
}