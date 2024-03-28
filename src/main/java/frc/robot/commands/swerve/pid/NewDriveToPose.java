package frc.robot.commands.swerve.pid;

import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.commands.WithStatus;
import frc.robot.subsystems.Swerve;

/* TODO: tune pid values */
public class NewDriveToPose extends Command implements WithStatus {
    private Swerve s_Swerve;
    private Pose2d targetPose;

    private PIDController xController;
    private PIDController yController;
    private ProfiledPIDController driveController;
    private ProfiledPIDController thetaController;

    public PIDConstants translationPIDConstants;
    public PIDConstants rotationPIDConstants;

    private boolean isRunning;

    public NewDriveToPose(Swerve s_Swerve, Pose2d targetPose, PIDConstants translationPIDConstants, PIDConstants rotationPIDConstants) {
        this.s_Swerve = s_Swerve;
        this.targetPose = targetPose;

        isRunning = false;

        this.translationPIDConstants = translationPIDConstants = new PIDConstants(4.0, 0.0, 0.0);
        this.rotationPIDConstants = rotationPIDConstants = new PIDConstants(6.0, 0.0, 0.0);

        var bezierPoints = PathPlannerPath.bezierFromPoses();
        var path = new PathPlannerPath(
            bezierPoints, 
            new PathConstraints(0, 0, 0, 0),
            null
        );

        var config = new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                /* TODO: tune pid constants */
                new PIDConstants(4.0, 0.0, 0.0), // Translation PID constants
                new PIDConstants(6.0, 0.0, 0.0), // Rotation PID constants
                Constants.AutoConstants.kMaxSpeedMetersPerSecond, // Max module speed, in m/s
                Constants.Swerve.wheelBase * Math.sqrt(2) / 2, // Drive base radius in meters. Distance from robot center to furthest module.
                new ReplanningConfig() // Default path replanning config. See the API for the options here
        );
        var followPath = new FollowPathHolonomic(
            path,
            null,
            null,
            null,
            // s_Swerve::getPose, // Robot pose supplier
            // s_Swerve::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            // s_Swerve::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            null,
            // () -> {
            //   return DriverStation.getAlliance().orElse(null) == DriverStation.Alliance.Red;
            // },
            null,
            null
        );

        addRequirements(s_Swerve);
    }

    @Override
    public void initialize() {
        isRunning = true;

        this.driveController = new ProfiledPIDController(translationPIDConstants.kP, translationPIDConstants.kI, translationPIDConstants.kD, new TrapezoidProfile.Constraints(
            Constants.AutoConstants.kMaxSpeedMetersPerSecond, Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared
        ));

        this.thetaController = new ProfiledPIDController(
            rotationPIDConstants.kP, rotationPIDConstants.kI, rotationPIDConstants.kD, Constants.AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        s_Swerve.zeroGyro();
        s_Swerve.resetOdometry(new Pose2d());
        driveController.reset(new Translation2d().getDistance(targetPose.getTranslation()));
        thetaController.reset(new Pose2d().getRotation().getRadians());
    }

    @Override
    public void execute() {
        Pose2d currentPose = s_Swerve.getPose();
        System.out.println("DriveToPose: " + currentPose);
        
        /* reduce current distance (error) to 0 */
        double currentDistance = currentPose.getTranslation().getDistance(targetPose.getTranslation());
        double driveVelocityScalar = driveController.atGoal() ? 0.0 : 
            driveController.calculate(currentDistance, 0.0);

        Translation2d driveVelocity = new Translation2d(
            driveVelocityScalar, 
            currentPose.getTranslation().minus(targetPose.getTranslation()).getAngle()
        );

        double thetaVelocity = thetaController.atGoal() ? 0.0 : 
            thetaController.calculate(currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());

        s_Swerve.drive(driveVelocity, thetaVelocity, true, true);
    }
    
    @Override
    public void end(boolean interrupted) {
        isRunning = false;
        s_Swerve.drive(new Translation2d(), 0, true, true);
        s_Swerve.zeroGyro();
    }

    @Override
    public boolean isFinished() {
        return atGoal();
    }

    public boolean atGoal() {
        return isRunning && driveController.atGoal() && thetaController.atGoal();
    }

    public boolean isRunning() {
        return isRunning;
    }
}
