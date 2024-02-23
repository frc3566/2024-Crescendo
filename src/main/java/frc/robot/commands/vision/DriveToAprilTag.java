package frc.robot.commands.vision;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.swerve.pid.Drive;
import frc.robot.commands.swerve.pid.Spin;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;

public class DriveToAprilTag extends Command {
    private Swerve s_Swerve;
    private Vision s_Vision;

    private Pose2d poseToAprilTag = new Pose2d();

    private SequentialCommandGroup commandGroup = new SequentialCommandGroup();

    private static final double
        cameraToRobotFront = 0.5,
        speakerAprilTagGap = 1,
        additionalGapForGoodMeasure = 0.5;

    public DriveToAprilTag(Swerve s_Swerve, Vision s_Vision) {
        this.s_Swerve = s_Swerve;
        this.s_Vision = s_Vision;
        addRequirements(s_Swerve, s_Vision);
    }

    @Override
    public void initialize() {
        System.out.println("Running DriveToAprilTag:");
        s_Vision.printAllResults();

        var result = s_Vision.getAprilTag();
        if (result.isEmpty()) {
            System.out.println("> No April Tag.");
            return;
        } 

        poseToAprilTag = s_Vision.getPoseTo(result.get());
        System.out.println("> April Tag: " + poseToAprilTag);
        
        Pose2d poseToAprilTagMinusGap = new Pose2d(
            poseToAprilTag.getTranslation().minus(new Translation2d(
                cameraToRobotFront + speakerAprilTagGap + additionalGapForGoodMeasure, 0)),
            poseToAprilTag.getRotation()
        );

        System.out.println("> April Tag minus gap: " + poseToAprilTagMinusGap);

        Pose2d singleDimensionTranslation = new Pose2d(
            poseToAprilTagMinusGap.getTranslation().rotateBy(poseToAprilTag.getRotation().unaryMinus()),
            new Rotation2d()
        );

        System.out.println("> Translation component: " + singleDimensionTranslation);

        commandGroup = new Drive(s_Swerve, poseToAprilTagMinusGap).andThen(new Spin(s_Swerve, poseToAprilTagMinusGap));
        // commandGroup = new Spin(s_Swerve, poseToAprilTagMinusGap).andThen(new Drive(s_Swerve, singleDimensionTranslation));
        commandGroup.schedule();
    }

    @Override
    public void end(boolean interrupted) {
        commandGroup.cancel();
    }

    @Override
    public boolean isFinished() {
        return commandGroup.isFinished();
    }
}
