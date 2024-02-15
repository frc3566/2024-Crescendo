package frc.robot.commands.swerve.pid;


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

public class Drive extends Command {
    private Swerve s_Swerve;
    private Pose2d targetPose;

    private PIDController xController;
    private PIDController yController;
    private ProfiledPIDController driveController;

    private boolean isRunning;

    public Drive(Swerve s_Swerve, Pose2d targetPose) {
        this.s_Swerve = s_Swerve;
        this.targetPose = targetPose;

        isRunning = false;

        addRequirements(s_Swerve);   
    }

    @Override
    public void initialize() {
        isRunning = true;

        this.xController = new PIDController(Constants.AutoConstants.kPXController, 0.5, 0);
        this.yController = new PIDController(Constants.AutoConstants.kPYController, 0.5, 0);
        this.driveController = new ProfiledPIDController(Constants.AutoConstants.kPXController, 0, 0, new TrapezoidProfile.Constraints(
            Constants.AutoConstants.kMaxSpeedMetersPerSecond, Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared
        ));

        Pose2d currentPose = s_Swerve.getPose();
        s_Swerve.resetOdometry(new Pose2d());

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
    }

    @Override
    public void execute() {
        Pose2d currentPose = s_Swerve.getPose();
        System.out.println("Drive: " + currentPose);
        
        double currentDistance = currentPose.getTranslation().getDistance(targetPose.getTranslation());
        double driveErrorAbs = currentDistance;
        double driveVelocityScalar = driveController.calculate(driveErrorAbs, 0.0);
        if (driveController.atGoal())
            driveVelocityScalar = 0.0;

        Translation2d driveVelocity = new Pose2d(
                new Translation2d(),
                currentPose.getTranslation().minus(targetPose.getTranslation()).getAngle()
            ).transformBy(new Transform2d(driveVelocityScalar, 0.0, new Rotation2d())).getTranslation();

        System.out.println("Combined controller: " + driveVelocityScalar + ", " + driveVelocity);

        double xVelocity = xController.calculate(currentPose.getX(), targetPose.getX());
        double yVelocity = yController.calculate(currentPose.getY(), targetPose.getY());

        System.out.println("Before: " + xVelocity + ", " + yVelocity);

        if (xController.atSetpoint()) { xVelocity = 0; }
        if (yController.atSetpoint()) { yVelocity = 0; }

        System.out.println("After: " + xVelocity + ", " + yVelocity);

        // s_Swerve.drive(driveVelocity, 0, true, true);
        s_Swerve.drive(new Translation2d(xVelocity, yVelocity), 0, true, true);
    }
    
    @Override
    public void end(boolean interrupted) {
        isRunning = false;
        s_Swerve.drive(new Translation2d(), 0, true, true);
    }

    public boolean atGoal() {
        return isRunning && driveController.atGoal();
    }

    public boolean isRunning() {
        return isRunning;
    }
}
