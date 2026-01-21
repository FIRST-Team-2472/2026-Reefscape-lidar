package frc.robot.extras;

import static org.junit.jupiter.api.Assertions.assertTrue;

import java.lang.reflect.Field;

import org.junit.jupiter.api.Test;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.LidarConstants;
import frc.robot.extras.LidarMapComponents.LidarSimulator;
import frc.robot.extras.LidarMapComponents.MapPoint;

public class EstimatedClimbingAccTest {
    @Test
    public void testEstimatedClimbingAcc() {
        int iterations = 20000;// expecting much higher chance of seeing something with dual lidar
        int climbingImprovedIterations = 0;
        double totalClimbingImprovement = 0;
        double totalStartingInnacuracy = 0;

        System.out.println("Starting EstimatedClimbingAccTest...");

        for (int i = 0; i < iterations; i++) {
            double trueX = (Math.random() * 2 - 1) * .2;// 20 cm offset from origin max
            double trueY = (Math.random() * 2 - 1) * .2;

            double angleRadians = Math.atan2(-trueY, -trueX);
            double angleDegrees = Math.toDegrees(angleRadians);
            double trueRotation = angleDegrees + (Math.random() * 2 - 1) * 10; // 10 degrees max

            double maxTranslationalInaccuracy = 0.03; // 3 cm
            double maxAngleInaccuracy = 1; // 1 degree

            double innacurateX = trueX + (Math.random() * 2 - 1) * maxTranslationalInaccuracy;
            double innacurateY = trueY + (Math.random() * 2 - 1) * maxTranslationalInaccuracy;
            double innacurateRotation = trueRotation + (Math.random() * 2 - 1) * maxAngleInaccuracy;

            double[][] SimulatedDualScan = LidarSimulator.getDualLidarSimData(
                    new MapPoint(trueX, trueY),
                    trueRotation,
                    1, // messy with accurate angle
                    LidarConstants.kLidarHorizontalOffsetFromCenterMeters,
                    LidarConstants.kLidarAngleOffsetFromForwardDegrees);

            // Estimate pose using dual lidar data
            FieldPoint calculatedPose = PointCloudPositionEstimator.estimatePose(
                    new FieldPose2d(innacurateX, innacurateY, Rotation2d.fromDegrees(innacurateRotation)),
                    SimulatedDualScan);

            double lidarInnacuracy = Math.hypot(trueX - calculatedPose.getX(), trueY - calculatedPose.getY());
            double inputInnacuracy = Math.hypot(trueX - innacurateX, trueY - innacurateY);

            if (lidarInnacuracy != inputInnacuracy) {
                totalClimbingImprovement += (inputInnacuracy - lidarInnacuracy);
                climbingImprovedIterations++;
                totalStartingInnacuracy += inputInnacuracy;
            }

        }

        double averageClimbingImprovementWhileSeeing = totalClimbingImprovement / climbingImprovedIterations;
        System.out.println(
                "EstimatedClimbingAccTest complete. Average Climbing Improvement when seeing something: "
                        + averageClimbingImprovementWhileSeeing + " meters over " + climbingImprovedIterations
                        + " iterations out of " + iterations + " total.");

        System.out.println(
                "Average Starting Innacuracy when seeing something: "
                        + (totalStartingInnacuracy / climbingImprovedIterations) + " meters over "
                        + climbingImprovedIterations + " iterations out of " + iterations + " total.");
    }
}
