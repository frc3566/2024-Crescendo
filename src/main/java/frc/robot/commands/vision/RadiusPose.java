package frc.robot.commands.vision;

import java.util.function.Consumer;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.WithStatus;
import frc.robot.commands.swerve.pid.Drive;
import frc.robot.commands.swerve.pid.Spin;
import frc.robot.subsystems.Vision;

public class RadiusPose extends Command implements WithStatus {
    private Vision s_Vision;
    
    private Consumer<Pose2d> setTargetPose;

    private int counter = 0;
    private boolean targetPoseComputed = false;
    private boolean isRunning = false;

    private static final double
        cameraToRobotFront = 0.5,
        speakerAprilTagGap = 1,
        additionalGapForGoodMeasure = 0.775;

    private static final double radius = cameraToRobotFront + speakerAprilTagGap + additionalGapForGoodMeasure;
    
    private static final double angleLimit = 45;

    public RadiusPose(Vision s_Vision, Pose2d currentPose, Consumer<Pose2d> setTargetPose) {
        this.s_Vision = s_Vision;
        this.setTargetPose = setTargetPose;
        addRequirements(s_Vision);
    }

    @Override
    public void initialize() {
        targetPoseComputed = false;
        isRunning = true;
        counter = 0;
    }

    @Override
    public void execute() {
        if (targetPoseComputed) { return; }

        if (counter > 10) { this.cancel(); }

        var result = s_Vision.getAprilTag();
        
        if (result.isEmpty()) {
            System.out.println("Cycle: " + ++counter);
            return;
        }

        System.out.println("Cycle: " + counter);

        s_Vision.printAllResults();

        Pose2d poseToAprilTag = s_Vision.getPoseTo(result.get());
        System.out.println("> April Tag: " + poseToAprilTag);

        Rotation2d facingAngle = poseToAprilTag.getRotation();
        Rotation2d translationAngle = poseToAprilTag.getTranslation().getAngle();

        Rotation2d extraAngle = facingAngle.minus(Vision.limitRange(facingAngle, -angleLimit, angleLimit));
        Translation2d gap = new Translation2d(radius, translationAngle.minus(extraAngle));
        Pose2d poseToAprilTagMinusGap = new Pose2d(
            poseToAprilTag.getTranslation().minus(gap),
            facingAngle.minus(extraAngle)
        );
        
        Rotation2d difference = poseToAprilTag.getRotation().minus(Vision.limitRange(poseToAprilTag.getRotation(), -angleLimit, angleLimit));
        Rotation2d kaienFinalTranslationAngle = poseToAprilTag.getRotation().minus(difference);
        Rotation2d kaienFinalFacingAngle = poseToAprilTag.getTranslation().getAngle().minus(difference);
        // Pose2d poseToAprilTagMinusGap = new Pose2d(
        //     poseToAprilTag.getTranslation().minus(new Translation2d(radius, kaienFinalTranslationAngle)),
        //     kaienFinalFacingAngle
        // );

        System.out.println("> April Tag minus gap: " + poseToAprilTagMinusGap);

        Pose2d singleDimensionTranslation = new Pose2d(
            poseToAprilTagMinusGap.getTranslation().rotateBy(poseToAprilTag.getRotation().unaryMinus()),
            new Rotation2d()
        );

        System.out.println("> Translation component: " + singleDimensionTranslation);

        setTargetPose.accept(poseToAprilTagMinusGap);
        targetPoseComputed = true;
    }

    @Override
    public void end(boolean interrupted) {
        isRunning = false;
    }

    @Override
    public boolean isFinished() {
        return targetPoseComputed;
    }

    @Override
    public boolean isRunning() {
        return isRunning;
    }
}
