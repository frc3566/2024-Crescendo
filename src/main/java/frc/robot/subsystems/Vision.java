package frc.robot.subsystems;

import java.io.IOException;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
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
        for (int i = 0; i < 20; i++) {
            var result = apriltagCamera.getLatestResult();

            if (!result.hasTargets()) { continue; }

            List<PhotonTrackedTarget> targets = result.getTargets();
            PhotonTrackedTarget target = result.getBestTarget();

            for (PhotonTrackedTarget potentialTarget: targets) {
                int id = potentialTarget.getFiducialId();
                if (id == 4 || id == 7) {
                    target = potentialTarget;
                    break;
                }
            }

            if (target.getPoseAmbiguity() <= 0.4) { return Optional.of(target); }
        }

        return Optional.empty();
    }

    public Pose2d getPoseTo(PhotonTrackedTarget target) {
        Transform3d transform = target.getBestCameraToTarget();
        Translation2d end = transform.getTranslation().toTranslation2d();

        double zAngleTheta = transform.getRotation().getZ();
        Rotation2d yaw = Rotation2d.fromRadians(Math.signum(zAngleTheta) * (Math.PI - Math.abs(zAngleTheta))).unaryMinus();

        return new Pose2d(new Translation2d(end.getX(), -end.getY()), yaw);
    }

    public Optional<Transform3d> getMultiAprilTag() {
        var result = apriltagCamera.getLatestResult().getMultiTagResult();
        if (!result.estimatedPose.isPresent) { return Optional.empty(); }

        Transform3d fieldToCamera = result.estimatedPose.best;
        return Optional.of(fieldToCamera);
    }

    public Optional<Pose3d> estimatePose() {
        return poseEstimator.update().map(e -> e.estimatedPose);
    }

    public void printAllResults() {
        System.out.println("Vision log:");
        var result = apriltagCamera.getLatestResult();

        if (!result.hasTargets()) {
            System.out.println("> No targets found.");
            return;
        }

        System.out.println("> Single AprilTag: " + getAprilTag());
        System.out.println("> Multi AprilTag: " + getMultiAprilTag());
        System.out.println("> Estimated pose: " + estimatePose());
    }
}