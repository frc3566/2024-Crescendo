package frc.robot.commands;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class DriveToPose extends Command {
    private Swerve s_Swerve;
    private Pose2d targetPose;

    private PIDController xController;
    private PIDController yController;
    private ProfiledPIDController driveController;
    private ProfiledPIDController thetaController;

    private boolean isRunning;

    public DriveToPose(Swerve s_Swerve, Pose2d targetPose) {
        this.s_Swerve = s_Swerve;
        this.targetPose = targetPose;

        isRunning = false;

        addRequirements(s_Swerve);   
    }

    @Override
    public void initialize() {
        isRunning = true;

        this.xController = new PIDController(Constants.AutoConstants.kPXController, 0, 0);
        this.yController = new PIDController(Constants.AutoConstants.kPYController, 0, 0);

        // this.driveController = new ProfiledPIDController(Constants.AutoConstants.kPXController, 0, 0, new TrapezoidProfile.Constraints(
        //     Constants.AutoConstants.kMaxSpeedMetersPerSecond, Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared
        // ));

        this.thetaController = new ProfiledPIDController(
            Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        Pose2d currentPose = s_Swerve.getPose();

        s_Swerve.resetOdometry(currentPose);

        // driveController.reset(
        //     new TrapezoidProfile.State(currentPose.getTranslation().getDistance(targetPose.getTranslation()),
        //         -new Translation2d()
        //             .rotateBy(targetPose
        //                 .getTranslation()
        //                 .minus(s_Swerve.getPose().getTranslation())
        //                 .getAngle()
        //                 .unaryMinus()
        //             ).getX()
        //     )
        // );

        thetaController.reset(currentPose.getRotation().getRadians());
    }

    @Override
    public void execute() {
        Pose2d currentPose = s_Swerve.getPose();
        
        double currentDistance = currentPose.getTranslation().getDistance(targetPose.getTranslation());
        double driveErrorAbs = currentDistance;
        double driveVelocityScalar = driveController.calculate(driveErrorAbs, 0.0);
        if (driveController.atGoal())
            driveVelocityScalar = 0.0;

        Translation2d driveVelocity = new Pose2d(
                new Translation2d(),
                currentPose.getTranslation().minus(targetPose.getTranslation()).getAngle()
            ).transformBy(new Transform2d(driveVelocityScalar, 0.0, new Rotation2d())).getTranslation();

        // double thetaVelocity = thetaController.atGoal() ? 0.0 : 
        //     thetaController.calculate(currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());

        s_Swerve.drive(driveVelocity, 0, true, true);
    }
    
    @Override
    public void end(boolean interrupted) {
        isRunning = false;
        s_Swerve.drive(new Translation2d(), 0, true, true);
    }

    public boolean atGoal() {
        return isRunning && driveController.atGoal() && thetaController.atGoal();
    }

    public boolean isRunning() {
        return isRunning;
    }
}
