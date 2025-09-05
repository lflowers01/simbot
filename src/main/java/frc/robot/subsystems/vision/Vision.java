package frc.robot.subsystems.vision;

import java.lang.StackWalker.Option;
import java.util.List;
import java.util.Optional;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.constVision;
import frc.robot.Robot;
import frc.robot.subsystems.drive.Drive;

public class Vision extends SubsystemBase {
    @AutoLog
    public static class VisionInputs {
        // Left camera data
        public boolean leftCameraConnected = false;
        public boolean leftCameraHasTargets = false;
        public int leftCameraTargetCount = 0;
        public double leftCameraBestTargetYaw = 0.0;
        public double leftCameraBestTargetPitch = 0.0;
        public double leftCameraBestTargetArea = 0.0;
        public double leftCameraBestTargetDistance = 0.0;
        public int leftCameraBestTargetID = -1;

        // Right camera data
        public boolean rightCameraConnected = false;
        public boolean rightCameraHasTargets = false;
        public int rightCameraTargetCount = 0;
        public double rightCameraBestTargetYaw = 0.0;
        public double rightCameraBestTargetPitch = 0.0;
        public double rightCameraBestTargetArea = 0.0;
        public double rightCameraBestTargetDistance = 0.0;
        public int rightCameraBestTargetID = -1;

        // Pose estimation data
        public boolean hasVisionPoseEstimate = false;
        public Pose2d latestVisionPose = new Pose2d();
        public double latestVisionTimestamp = 0.0;
        public double visionStdDevsX = 0.0;
        public double visionStdDevsY = 0.0;
        public double visionStdDevsTheta = 0.0;
        public int totalTagsSeen = 0;
    }

    private final VisionInputsAutoLogged inputs = new VisionInputsAutoLogged();

    private Drive drivetrain;

    // Cameras
    private PhotonCamera leftCamera;
    private PhotonCamera rightCamera;

    // Camera transforms
    private Transform3d robotToLeftCamera;
    private Transform3d robotToRightCamera;

    // Pose estimation
    private PhotonPoseEstimator leftPoseEstimator;
    private PhotonPoseEstimator rightPoseEstimator;

    // Simulation components
    private VisionSystemSim visionSim;
    private PhotonCameraSim leftCamSim;
    private PhotonCameraSim rightCamSim;
    private AprilTagFieldLayout tagLayout;

    private EstimateConsumer estimatorConsumer;
    private Matrix<N3, N1> curStdDevs;
    private boolean visionEnabled = true; // Add this flag

    // Camera pose visualization variables
    private Pose2d leftCameraPose = new Pose2d();
    private Pose2d rightCameraPose = new Pose2d();

    public Vision(EstimateConsumer estimatorConsumer) {
        this.estimatorConsumer = estimatorConsumer;

        // Camera transforms - use constants
        robotToLeftCamera = constVision.kRobotToLeftCam;
        robotToRightCamera = constVision.kRobotToRightCam;

        // Get tag layout from constants
        tagLayout = constVision.kTagLayout;

        leftCamera = new PhotonCamera(constVision.kLeftCameraName);
        rightCamera = new PhotonCamera(constVision.kRightCameraName);

        leftPoseEstimator = new PhotonPoseEstimator(tagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                robotToLeftCamera);
        leftPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        rightPoseEstimator = new PhotonPoseEstimator(tagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                robotToRightCamera);
        rightPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        if (Robot.isSimulation()) {
            visionSim = new VisionSystemSim("main");
            visionSim.addAprilTags(tagLayout);

            SimCameraProperties cameraProps = new SimCameraProperties();
            cameraProps.setCalibration(1280, 800, Rotation2d.fromDegrees(82));
            cameraProps.setCalibError(0.15, 0.04);
            cameraProps.setFPS(60);
            cameraProps.setAvgLatencyMs(5);
            cameraProps.setLatencyStdDevMs(5);

            leftCamSim = new PhotonCameraSim(leftCamera, cameraProps);
            rightCamSim = new PhotonCameraSim(rightCamera, cameraProps);

            visionSim.addCamera(leftCamSim, robotToLeftCamera);
            visionSim.addCamera(rightCamSim, robotToRightCamera);
        }
    }

    @Override
    public void periodic() {
        // Update vision inputs for logging
        updateVisionInputs();
        Logger.processInputs("Vision", inputs);

        // Only process vision if enabled
        if (visionEnabled) {
            // Process all camera results together for better pose estimation
            processAllCameras();
        }
    }

    public void simulationPeriodic(Pose2d robotPose) {
        // Update simulation if running
        if (Robot.isSimulation() && visionSim != null) {
            visionSim.update(robotPose);

            // Calculate and publish camera poses for visualization
            updateCameraPoses(robotPose);
            SmartDashboard.putString("Vision/LeftCameraPose", leftCameraPose.toString());
            SmartDashboard.putString("Vision/RightCameraPose", rightCameraPose.toString());
        }
    }

    // Add methods to enable/disable vision
    public void enableVision() {
        visionEnabled = true;
    }

    public void disableVision() {
        visionEnabled = false;
    }

    public boolean isVisionEnabled() {
        return visionEnabled;
    }

    private void updateVisionInputs() {
        // Left camera inputs - use getAllUnreadResults() and get the latest
        var leftResults = leftCamera.getAllUnreadResults();
        inputs.leftCameraConnected = leftCamera.isConnected();

        if (!leftResults.isEmpty()) {
            var leftResult = leftResults.get(leftResults.size() - 1); // Get latest result
            inputs.leftCameraHasTargets = leftResult.hasTargets();
            inputs.leftCameraTargetCount = leftResult.getTargets().size();

            if (leftResult.hasTargets()) {
                var bestTarget = leftResult.getBestTarget();
                inputs.leftCameraBestTargetYaw = bestTarget.getYaw();
                inputs.leftCameraBestTargetPitch = bestTarget.getPitch();
                inputs.leftCameraBestTargetArea = bestTarget.getArea();
                inputs.leftCameraBestTargetDistance = bestTarget.getBestCameraToTarget().getTranslation().getNorm();
                inputs.leftCameraBestTargetID = bestTarget.getFiducialId();
            } else {
                inputs.leftCameraBestTargetYaw = 0.0;
                inputs.leftCameraBestTargetPitch = 0.0;
                inputs.leftCameraBestTargetArea = 0.0;
                inputs.leftCameraBestTargetDistance = 0.0;
                inputs.leftCameraBestTargetID = -1;
            }
        } else {
            inputs.leftCameraHasTargets = false;
            inputs.leftCameraTargetCount = 0;
            inputs.leftCameraBestTargetYaw = 0.0;
            inputs.leftCameraBestTargetPitch = 0.0;
            inputs.leftCameraBestTargetArea = 0.0;
            inputs.leftCameraBestTargetDistance = 0.0;
            inputs.leftCameraBestTargetID = -1;
        }

        // Right camera inputs - use getAllUnreadResults() and get the latest
        var rightResults = rightCamera.getAllUnreadResults();
        inputs.rightCameraConnected = rightCamera.isConnected();

        if (!rightResults.isEmpty()) {
            var rightResult = rightResults.get(rightResults.size() - 1); // Get latest result
            inputs.rightCameraHasTargets = rightResult.hasTargets();
            inputs.rightCameraTargetCount = rightResult.getTargets().size();

            if (rightResult.hasTargets()) {
                var bestTarget = rightResult.getBestTarget();
                inputs.rightCameraBestTargetYaw = bestTarget.getYaw();
                inputs.rightCameraBestTargetPitch = bestTarget.getPitch();
                inputs.rightCameraBestTargetArea = bestTarget.getArea();
                inputs.rightCameraBestTargetDistance = bestTarget.getBestCameraToTarget().getTranslation().getNorm();
                inputs.rightCameraBestTargetID = bestTarget.getFiducialId();
            } else {
                inputs.rightCameraBestTargetYaw = 0.0;
                inputs.rightCameraBestTargetPitch = 0.0;
                inputs.rightCameraBestTargetArea = 0.0;
                inputs.rightCameraBestTargetDistance = 0.0;
                inputs.rightCameraBestTargetID = -1;
            }
        } else {
            inputs.rightCameraHasTargets = false;
            inputs.rightCameraTargetCount = 0;
            inputs.rightCameraBestTargetYaw = 0.0;
            inputs.rightCameraBestTargetPitch = 0.0;
            inputs.rightCameraBestTargetArea = 0.0;
            inputs.rightCameraBestTargetDistance = 0.0;
            inputs.rightCameraBestTargetID = -1;
        }

        // Total tags seen across both cameras
        inputs.totalTagsSeen = inputs.leftCameraTargetCount + inputs.rightCameraTargetCount;

        // Standard deviations
        if (curStdDevs != null) {
            inputs.visionStdDevsX = curStdDevs.get(0, 0);
            inputs.visionStdDevsY = curStdDevs.get(1, 0);
            inputs.visionStdDevsTheta = curStdDevs.get(2, 0);
        }
    }

    private void processAllCameras() {
        // Process left camera
        Optional<EstimatedRobotPose> visionEst = Optional.empty();

        for (var result : leftCamera.getAllUnreadResults()) {
            visionEst = leftPoseEstimator.update(result);
            updateEstimationStdDevs(visionEst, result.getTargets());

            if (Robot.isSimulation()) {
                visionEst.ifPresentOrElse(
                        est -> getSimDebugField()
                                .getObject("VisionEstimationLeft")
                                .setPose(est.estimatedPose.toPose2d()),
                        () -> {
                            getSimDebugField().getObject("VisionEstimationLeft").setPoses();
                        });
            }

            visionEst.ifPresent(
                    est -> {
                        // Update inputs for latest pose estimate
                        inputs.hasVisionPoseEstimate = true;
                        inputs.latestVisionPose = est.estimatedPose.toPose2d();
                        inputs.latestVisionTimestamp = est.timestampSeconds;

                        // Change our trust in the measurement based on the tags we can see
                        var estStdDevs = getEstimationStdDevs();

                        // Only send to drivetrain if vision is enabled
                        if (visionEnabled) {
                            estimatorConsumer.accept(est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
                        }
                    });
        }

        // Process right camera
        for (var result : rightCamera.getAllUnreadResults()) {
            visionEst = rightPoseEstimator.update(result);
            updateEstimationStdDevs(visionEst, result.getTargets());

            if (Robot.isSimulation()) {
                visionEst.ifPresentOrElse(
                        est -> getSimDebugField()
                                .getObject("VisionEstimationRight")
                                .setPose(est.estimatedPose.toPose2d()),
                        () -> {
                            getSimDebugField().getObject("VisionEstimationRight").setPoses();
                        });
            }

            visionEst.ifPresent(
                    est -> {
                        // Update inputs for latest pose estimate
                        inputs.hasVisionPoseEstimate = true;
                        inputs.latestVisionPose = est.estimatedPose.toPose2d();
                        inputs.latestVisionTimestamp = est.timestampSeconds;

                        // Change our trust in the measurement based on the tags we can see
                        var estStdDevs = getEstimationStdDevs();

                        // Only send to drivetrain if vision is enabled
                        if (visionEnabled) {
                            estimatorConsumer.accept(est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
                        }
                    });
        }
    }

    private Pose2d getPoseFromDrivetrain() {
        // You'll need to get this from your drivetrain somehow
        // For now, return a default pose - you can update this when you connect to
        // drivetrain
        return new Pose2d();
    }

    private void updateEstimationStdDevs(
            Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {

        if (estimatedPose.isEmpty()) {
            // No pose input. Default to single-tag std devs
            curStdDevs = constVision.kSingleTagStdDevs;
            return;
        }

        // Pose present. Start running Heuristic
        var estStdDevs = constVision.kSingleTagStdDevs;
        int numTags = 0;
        double avgDist = 0;

        // Precalculation - see how many tags we found, and calculate an
        // average-distance metric
        for (var tgt : targets) {
            var tagPose = tagLayout.getTagPose(tgt.getFiducialId());
            if (tagPose.isEmpty())
                continue;
            numTags++;
            avgDist += tagPose
                    .get()
                    .toPose2d()
                    .getTranslation()
                    .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
        }

        if (numTags == 0) {
            // No tags visible. Default to single-tag std devs
            curStdDevs = constVision.kSingleTagStdDevs;
        } else {
            // One or more tags visible, run the full heuristic.
            avgDist /= numTags;
            // Decrease std devs if multiple targets are visible
            if (numTags > 1) {
                estStdDevs = constVision.kMultiTagStdDevs;
            }
            // Increase std devs based on (average) distance
            // Adjusted distance threshold from 4 to 6 meters for better filtering
            if (numTags == 1 && avgDist > 6.0) {
                estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
            } else {
                estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
            }
            curStdDevs = estStdDevs;
        }
    }

    private void updateCameraPoses(Pose2d robotPose) {
        // Convert robot pose to 3D
        var robotPose3d = new edu.wpi.first.math.geometry.Pose3d(
                robotPose.getX(),
                robotPose.getY(),
                0.0,
                new Rotation3d(0, 0, robotPose.getRotation().getRadians()));

        // Calculate camera poses in field coordinates
        var leftCameraPose3d = robotPose3d.plus(robotToLeftCamera);
        var rightCameraPose3d = robotPose3d.plus(robotToRightCamera);

        // Convert back to 2D for dashboard
        leftCameraPose = leftCameraPose3d.toPose2d();
        rightCameraPose = rightCameraPose3d.toPose2d();
    }

    public Matrix<N3, N1> getEstimationStdDevs() {
        return curStdDevs;
    }

    public Field2d getSimDebugField() {
        if (!Robot.isSimulation())
            return null;
        return visionSim.getDebugField();
    }

    @FunctionalInterface
    public static interface EstimateConsumer {
        public void accept(Pose2d pose, double timestamp, Matrix<N3, N1> estimationStdDevs);
    }
}