package frc.robot.subsystems;

// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

public class Module {
  private final ModuleIO io;
  private final int index;

  private static final double wheelRadius =
      Constants.Swerve.wheelDiameter / 2;
  private static final double driveKp =
      Constants.Swerve.driveKP;
  private static final double driveKd =
      Constants.Swerve.driveKD;
  private static final double driveKs =
      Constants.Swerve.driveKS;
  private static final double driveKv =
      Constants.Swerve.driveKV;
  private static final double turnKp = Constants.Swerve.angleKP;
  private static final double turnKd = Constants.Swerve.angleKD;

  private SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(0.0, 0.0);
  private final PIDController driveFeedback =
      new PIDController(0.0, 0.0, 0.0, Constants.Swerve.loopPeriodSeconds);
  private final PIDController turnFeedback =
      new PIDController(0.0, 0.0, 0.0, Constants.Swerve.loopPeriodSeconds);
  
  public Module(ModuleIO io, int index) {
    System.out.println("[Init] Creating Module " + Integer.toString(index));
    this.io = io;
    this.index = index;

    turnFeedback.enableContinuousInput(-Math.PI, Math.PI);
  }

  /** Updates inputs and checks tunable numbers. */
  public void periodic() {
    io.updateInputs(inputs);

    // Update controllers if tunable numbers have changed
    if (driveKp.hasChanged(hashCode()) || driveKd.hasChanged(hashCode())) {
      driveFeedback.setPID(driveKp.get(), 0.0, driveKd.get());
    }
    if (turnKp.hasChanged(hashCode()) || turnKd.hasChanged(hashCode())) {
      turnFeedback.setPID(turnKp.get(), 0.0, turnKd.get());
    }
    if (driveKs.hasChanged(hashCode()) || driveKv.hasChanged(hashCode())) {
      driveFeedforward = new SimpleMotorFeedforward(driveKs.get(), driveKv.get());
    }
  }

  /**
   * Runs the module with the specified setpoint state. Must be called periodically. Returns the
   * optimized state.
   */
  public SwerveModuleState runSetpoint(SwerveModuleState state) {
    // Optimize state based on current angle
    var optimizedState = SwerveModuleState.optimize(state, getAngle());

    // Run turn controller
    io.setTurnVoltage(
        turnFeedback.calculate(getAngle().getRadians(), optimizedState.angle.getRadians()));

    // Update velocity based on turn error
    optimizedState.speedMetersPerSecond *= Math.cos(turnFeedback.getPositionError());

    // Run drive controller
    double velocityRadPerSec = optimizedState.speedMetersPerSecond / (Constants.Swerve.wheelDiameter / 2);
    io.setDriveVoltage(
        driveFeedforward.calculate(velocityRadPerSec)
            + driveFeedback.calculate(inputs.driveVelocityRadPerSec, velocityRadPerSec));

    return optimizedState;
  }

  /**
   * Runs the module with the specified voltage while controlling to zero degrees. Must be called
   * periodically.
   */
  public void runCharacterization(double volts) {
    io.setTurnVoltage(turnFeedback.calculate(getAngle().getRadians(), 0.0));
    io.setDriveVoltage(volts);
  }

  /** Disables all outputs to motors. */
  public void stop() {
    io.setTurnVoltage(0.0);
    io.setDriveVoltage(0.0);
  }

  /** Sets whether brake mode is enabled. */
  public void setBrakeMode(boolean enabled) {
    io.setDriveBrakeMode(enabled);
    io.setTurnBrakeMode(enabled);
  }

  /** Returns the current turn angle of the module. */
  public Rotation2d getAngle() {
    return new Rotation2d(MathUtil.angleModulus(inputs.turnAbsolutePositionRad));
  }

  /** Returns the current drive position of the module in meters. */
  public double getPositionMeters() {
    return inputs.drivePositionRad * wheelRadius;
  }

  /** Returns the current drive velocity of the module in meters per second. */
  public double getVelocityMetersPerSec() {
    return inputs.driveVelocityRadPerSec * wheelRadius;
  }

  /** Returns the module position (turn angle and drive position). */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getPositionMeters(), getAngle());
  }

  /** Returns the module state (turn angle and drive velocity). */
  public SwerveModuleState getState() {
    return new SwerveModuleState(getVelocityMetersPerSec(), getAngle());
  }

  /** Returns the drive velocity in radians/sec. */
  public double getCharacterizationVelocity() {
    return inputs.driveVelocityRadPerSec;
  }
}

  /** Returns the drive wheel radius. */