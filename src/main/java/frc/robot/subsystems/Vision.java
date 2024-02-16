package frc.robot.subsystems;

import java.io.File;
import java.io.IOException;
import java.nio.file.Path;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class Vision extends SubsystemBase {
    private PhotonCamera apriltagCamera;
    private PhotonPoseEstimator poseEstimator;

    public Vision() throws IOException {
        apriltagCamera = new PhotonCamera(Constants.Vision.APRIL_TAG_CAMERA_NAME);
        poseEstimator = new PhotonPoseEstimator(
            Constants.Vision.APRIL_TAG_FIELD_LAYOUT,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            apriltagCamera,
            Constants.Vision.ROBOT_TO_CAMERA
        );
    }

    public Optional<PhotonTrackedTarget> getAprilTag() {
        var result = apriltagCamera.getLatestResult();

        if (!result.hasTargets())
            return Optional.empty();

        List<PhotonTrackedTarget> targets = result.getTargets();
        PhotonTrackedTarget target = result.getBestTarget();

        if (targets.size() == 1) 
            return Optional.of(target);

        for (PhotonTrackedTarget potentialTarget: targets) {
            int id = potentialTarget.getFiducialId();
            potentialTarget.getPoseAmbiguity();
            if (id == 4 || id == 7) {
                target = potentialTarget;
                break;
            }
        }

        return Optional.of(target);

    }

    public Pose2d getPoseTo(PhotonTrackedTarget target) {
        final double metersInFrontOfTarget = 0.5;

        Transform3d transform = target.getBestCameraToTarget();
        Translation2d end = transform.getTranslation().toTranslation2d()
            .minus(new Translation2d(metersInFrontOfTarget, 0));

        return new Pose2d(end, Rotation2d.fromDegrees(target.getYaw()));
    }

    // public Transform3d getMultiAprilTag() {
    //     var result = apriltagCamera.getLatestResult().getMultiTagResult();
    //     if (result.estimatedPose.isPresent) {
    //         Transform3d fieldToCamera = result.estimatedPose.best;
    //         return fieldToCamera;
    //     }

    //     return null;
    // }

    public Optional<Pose3d> estimatePose() {
        return poseEstimator.update().map(e -> e.estimatedPose);
    }
}