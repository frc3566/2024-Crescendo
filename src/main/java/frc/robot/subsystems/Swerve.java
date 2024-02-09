package frc.robot.subsystems;

import frc.robot.SwerveModule;
import frc.robot.Constants;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Swerve extends SubsystemBase {
    public SwerveDriveOdometry swerveOdometry;
    public SwerveModule[] mSwerveMods;
    public AHRS gyro;
    public double facing;

    private boolean isCharacterizing = false;
    private ChassisSpeeds setpoint = new ChassisSpeeds();
    private SwerveModuleState[] lastSetpointStates =
        new SwerveModuleState[] {
          new SwerveModuleState(),
          new SwerveModuleState(),
          new SwerveModuleState(),
          new SwerveModuleState()
        };

    public Swerve() {
        gyro = new AHRS(Constants.Swerve.navXPort);
        // gyro.calibrate(); // goofy aah deprecation
        zeroGyro();

        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, Constants.Swerve.Mod0.constants),
            new SwerveModule(1, Constants.Swerve.Mod1.constants),
            new SwerveModule(2, Constants.Swerve.Mod2.constants),
            new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };

        /* By pausing init for a second before setting module offsets, we avoid a bug with inverting motors.
         * See https://github.com/Team364/BaseFalconSwerve/issues/8 for more info.
         */
        Timer.delay(3.0);
        resetModulesToAbsolute();

        swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getYaw(), getModulePositions());
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates =
            Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation, 
                                    getYaw()
                                )
                                : new ChassisSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation)
                                );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }    

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);
        
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }    

    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        swerveOdometry.resetPosition(getYaw(), getModulePositions(), pose);
    }

    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : mSwerveMods){
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : mSwerveMods){
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public void zeroGyro(){
        gyro.zeroYaw();
        facing = 0;
    }

    public void off(){
        facing = getYaw().getRadians();
        drive(new Translation2d(), 0, false, true);
    }

    public Rotation2d getYaw() {
        return (Constants.Swerve.invertGyro) ? Rotation2d.fromDegrees(360 - gyro.getYaw()) : Rotation2d.fromDegrees(gyro.getYaw());
    }

    public Rotation2d getRoll() {
        return Rotation2d.fromDegrees(gyro.getRoll());
    }

    public Rotation2d getPitch() {
        return Rotation2d.fromDegrees(gyro.getPitch());
    }

    public void resetModulesToAbsolute() {
        for(SwerveModule mod : mSwerveMods){
            mod.resetToAbsolute();
        }
    }

    public Translation2d[] getModuleTranslations() {
        return new Translation2d[] {
          new Translation2d(Constants.Swerve.trackWidth / 2.0, Constants.Swerve.trackWidth / 2.0),
          new Translation2d(Constants.Swerve.trackWidth / 2.0, -Constants.Swerve.trackWidth / 2.0),
          new Translation2d(-Constants.Swerve.trackWidth / 2.0, Constants.Swerve.trackWidth / 2.0),
          new Translation2d(-Constants.Swerve.trackWidth / 2.0, -Constants.Swerve.trackWidth / 2.0)
        };
      }

    @Override
    public void periodic(){
        swerveOdometry.update(getYaw(), getModulePositions());  

        for(SwerveModule mod : mSwerveMods){
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Integrated", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);    
        }

        var setpointTwist =
          new Pose2d()
              .log(
                  new Pose2d(
                      setpoint.vxMetersPerSecond * Constants.Swerve.loopPeriodSeconds,
                      setpoint.vyMetersPerSecond * Constants.Swerve.loopPeriodSeconds,
                      new Rotation2d(setpoint.omegaRadiansPerSecond * Constants.Swerve.loopPeriodSeconds)));
        var adjustedSpeeds =
            new ChassisSpeeds(
              setpointTwist.dx / Constants.Swerve.loopPeriodSeconds,
              setpointTwist.dy / Constants.Swerve.loopPeriodSeconds,
              setpointTwist.dtheta / Constants.Swerve.loopPeriodSeconds);
        SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(adjustedSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, Constants.Swerve.maxLinearSpeed);

      // Set to last angles if zero
        if (adjustedSpeeds.vxMetersPerSecond == 0.0
            && adjustedSpeeds.vyMetersPerSecond == 0.0
            && adjustedSpeeds.omegaRadiansPerSecond == 0) {
            for (int i = 0; i < 4; i++) {
              setpointStates[i] = new SwerveModuleState(0.0, lastSetpointStates[i].angle);
            }
        }
        lastSetpointStates = setpointStates;

        // Send setpoints to modules
        SwerveModuleState[] optimizedStates = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
            optimizedStates[i] = mSwerveMods[i].runSetpoint(setpointStates[i]);
        }
    }

    public void runVelocity(ChassisSpeeds speeds) {
        isCharacterizing = false;
        setpoint = speeds;
      }
}