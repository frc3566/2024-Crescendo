
import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilderImpl;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.swerve.pid.Drive;
import frc.robot.commands.swerve.pid.Spin;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;

public class GetAprilTagPose extends Command {
    private Vision s_Vision;

    private Pose2d poseToAprilTag = new Pose2d();

    private Command commandGroup = new SequentialCommandGroup();

    private int counter = 0;
    private boolean poseFound = false;
    private boolean ended = false;

    private static final double
        cameraToRobotFront = 0.5,
        speakerAprilTagGap = 1,
        additionalGapForGoodMeasure = 1.0;

    public GetAprilTagPose( Vision s_Vision) {
        this.s_Vision = s_Vision;
        addRequirements(s_Vision);
    }

    @Override
    public void initialize() {
        System.out.println("Running DriveToAprilTag:");
        ended = false;
        poseFound = false;
        counter = 0;
        poseToAprilTag = new Pose2d();
        s_Vision.resetPose();
    }

    @Override
    public void execute() {
        if (poseFound) return;

        var result = s_Vision.getAprilTag();
        if (result.isEmpty()) {
            System.out.println("Cycle: " + ++counter);
            return;
        } 

        System.out.println("Cycle: " + counter);

        s_Vision.printAllResults();

        poseToAprilTag = s_Vision.getPoseTo(result.get());
        System.out.println("> April Tag: " + poseToAprilTag);
        
        Pose2d poseToAprilTagMinusGap = new Pose2d(
            poseToAprilTag.getTranslation().minus(new Translation2d(
                cameraToRobotFront + speakerAprilTagGap + additionalGapForGoodMeasure, poseToAprilTag.getRotation())),
            poseToAprilTag.getRotation()
        );

        System.out.println("> April Tag minus gap: " + poseToAprilTagMinusGap);

        Pose2d singleDimensionTranslation = new Pose2d(
            poseToAprilTagMinusGap.getTranslation().rotateBy(poseToAprilTag.getRotation().unaryMinus()),
            new Rotation2d()
        );

        System.out.println("> Translation component: " + singleDimensionTranslation);

        s_Vision.writePose(poseToAprilTagMinusGap);
        poseFound = true;
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("DriveToAprilTag finished");
        ended = true;
    }

    @Override
    public boolean isFinished() {
        return poseFound || ended;
    }
}
