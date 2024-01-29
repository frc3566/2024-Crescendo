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

    private static enum Pipeline {
        AprilTag,
        ReflectiveTape
    };

    public Vision() throws IOException {
        apriltagCamera = new PhotonCamera(Constants.Vision.APRIL_TAG_CAMERA_NAME);
        poseEstimator = new PhotonPoseEstimator(
            Constants.Vision.APRIL_TAG_FIELD_LAYOUT,
            PoseStrategy.LOWEST_AMBIGUITY, 
            apriltagCamera, 
            Constants.Vision.ROBOT_TO_CAMERA
        );
    }

    public Optional<PhotonTrackedTarget> getAprilTag() {
        var result = apriltagCamera.getLatestResult();
        if (!result.hasTargets()) {
            System.out.println("No targets in range\n");
            return Optional.empty();
        }

        return Optional.of(result.getBestTarget());

        // /* Traditional distance using formula */
        // double dist = PhotonUtils.calculateDistanceToTargetMeters(
        //     VisionConstants.CAMERA_HEIGHT_METERS, 
        //     VisionConstants.APRILTAG_HEIGHT_METERS, 
        //     VisionConstants.CAMERA_PITCH_RADIANS, 
        //     Units.degreesToRadians(target.getPitch())
        // );
    }

    public Optional<Trajectory> getTrajectory(PhotonTrackedTarget target) {
        TrajectoryConfig config = Constants.Trajectory.CONFIG;
        double coefficient = Constants.Trajectory.COEFFICIENT;
        Transform3d transform = target.getBestCameraToTarget().plus(Constants.Vision.ROBOT_TO_CAMERA.inverse());
        Translation2d end = transform.getTranslation().toTranslation2d().minus(new Translation2d(0.5, 0)).times(coefficient);

        /* Pose2d start, List<Translation2D> pathPoints, Pose2d end, config */
        return Optional.of(TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(0)),
            List.of(end.div(2)),
            new Pose2d(end, transform.getRotation().toRotation2d()),
            config
        ));
    }
}