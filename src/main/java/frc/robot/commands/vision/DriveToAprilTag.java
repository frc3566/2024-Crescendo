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

    private Command commandGroup = new SequentialCommandGroup();

    private int counter = 0;
    private boolean interrupted = false;

    private static final double
        cameraToRobotFront = 0.5,
        speakerAprilTagGap = 1,
        additionalGapForGoodMeasure = 1.0;

    public DriveToAprilTag(Swerve s_Swerve, Vision s_Vision) {
        this.s_Swerve = s_Swerve;
        this.s_Vision = s_Vision;
        addRequirements(s_Swerve, s_Vision);
    }

    @Override
    public void initialize() {
        System.out.println("Running DriveToAprilTag:");
        interrupted = false;
        counter = 0;
    }

    @Override
    public void execute() {
        if (!poseToAprilTag.equals(new Pose2d()) || commandGroup.isScheduled() || commandGroup.isFinished()) return;

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

        commandGroup = new Drive(s_Swerve, poseToAprilTagMinusGap).andThen(new Spin(s_Swerve, poseToAprilTagMinusGap));
        // commandGroup = new Spin(s_Swerve, poseToAprilTagMinusGap).andThen(new Drive(s_Swerve, singleDimensionTranslation));
        // commandGroup = new Drive(s_Swerve, poseToAprilTagMinusGap);
        commandGroup.schedule();
    }

    @Override
    public void end(boolean interrupted) {
        poseToAprilTag = new Pose2d();
        commandGroup = new SequentialCommandGroup();
        this.interrupted = true;
        commandGroup.cancel();
    }

    @Override
    public boolean isFinished() {
        return commandGroup.isFinished() || interrupted;
    }
}
