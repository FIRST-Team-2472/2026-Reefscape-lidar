package frc.robot.extras;

import static org.junit.jupiter.api.Assertions.assertTrue;

import java.lang.reflect.Field;

import org.junit.jupiter.api.Test;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.LidarConstants;
import frc.robot.extras.LidarMapComponents.LidarSimulator;
import frc.robot.extras.LidarMapComponents.MapPoint;
import frc.robot.extras.LidarMapComponents.Polygon;

public class EstimatedClimbingAccTest {
    @Test
    public void testEstimatedClimbingAcc() {
        int iterations = 1000;// expecting much higher chance of seeing something with dual lidar
        int climbingImprovedIterations = 0;
        double totalClimbingImprovement = 0;
        double totalStartingInnacuracy = 0;
        double totalICPImprovement = 0;
        double totalParticleFilterImprovement = 0;
        double totalFastPFImprovement = 0;
        double totalEvenFasterPFImprovement = 0;
        double sumFinalError = 0;
        double sumSqFinalError = 0;
        long totalTimeCombined = 0;
        long totalTimeICP = 0;
        long totalTimePF = 0;
        long totalTimeFastPF = 0;
        long totalTimeEvenFasterPF = 0;

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
                    LidarConstants.kLidarAngleOffsetFromForwardDegrees,
                    new Polygon[]{LidarMap.getMap()[2]});
            
            // Assume simulator map is correct from previous reasoning (we want map[2] for tower)

            // Estimate pose using dual lidar data
            long start = System.nanoTime();
            FieldPoint calculatedPose = PointCloudPositionEstimator.estimatePose(
                    new FieldPose2d(innacurateX, innacurateY, Rotation2d.fromDegrees(innacurateRotation)),
                    SimulatedDualScan, new Polygon[]{LidarMap.getMap()[2]});
            totalTimeCombined += (System.nanoTime() - start);

            start = System.nanoTime();
            FieldPoint ICPOnlyPose = PointCloudPositionEstimator.estimatePoseDualLidarICP(new FieldPose2d(innacurateX, innacurateY, Rotation2d.fromDegrees(innacurateRotation)),
                    SimulatedDualScan, new Polygon[]{LidarMap.getMap()[2]});
            totalTimeICP += (System.nanoTime() - start);

            start = System.nanoTime();
            FieldPoint ParticleFilterPose = PointCloudPositionEstimator.estimatePoseDualLidarParticleFilter(new FieldPose2d(innacurateX, innacurateY, Rotation2d.fromDegrees(innacurateRotation)),
                    SimulatedDualScan, new Polygon[]{LidarMap.getMap()[2]});
            totalTimePF += (System.nanoTime() - start);

            start = System.nanoTime();
            FieldPoint FastPFPose = PointCloudPositionEstimator.estimatePoseDualLidarParticleFilterFast(new FieldPose2d(innacurateX, innacurateY, Rotation2d.fromDegrees(innacurateRotation)),
                    SimulatedDualScan, new Polygon[]{LidarMap.getMap()[2]});
            totalTimeFastPF += (System.nanoTime() - start);

            start = System.nanoTime();
            FieldPoint EvenFasterPFPose = PointCloudPositionEstimator.estimatePoseDualLidarParticleFilterEvenFaster(new FieldPose2d(innacurateX, innacurateY, Rotation2d.fromDegrees(innacurateRotation)),
                    SimulatedDualScan, new Polygon[]{LidarMap.getMap()[2]});
            totalTimeEvenFasterPF += (System.nanoTime() - start);

            double lidarInnacuracy = Math.hypot(trueX - calculatedPose.getX(), trueY - calculatedPose.getY());
            double ICPOnlyInnacuracy = Math.hypot(trueX - ICPOnlyPose.getX(), trueY - ICPOnlyPose.getY());
            double ParticleFilterInnacuracy = Math.hypot(trueX - ParticleFilterPose.getX(), trueY - ParticleFilterPose.getY());
            double FastPFInnacuracy = Math.hypot(trueX - FastPFPose.getX(), trueY - FastPFPose.getY());
            double EvenFasterPFInnacuracy = Math.hypot(trueX - EvenFasterPFPose.getX(), trueY - EvenFasterPFPose.getY());
            double inputInnacuracy = Math.hypot(trueX - innacurateX, trueY - innacurateY);

            if (lidarInnacuracy != inputInnacuracy) {
                totalClimbingImprovement += (inputInnacuracy - lidarInnacuracy);
                totalICPImprovement += (inputInnacuracy - ICPOnlyInnacuracy);
                totalParticleFilterImprovement += (inputInnacuracy - ParticleFilterInnacuracy);
                totalFastPFImprovement += (inputInnacuracy - FastPFInnacuracy);
                totalEvenFasterPFImprovement += (inputInnacuracy - EvenFasterPFInnacuracy);
                climbingImprovedIterations++;
                totalStartingInnacuracy += inputInnacuracy;
                
                sumFinalError += lidarInnacuracy;
                sumSqFinalError += lidarInnacuracy * lidarInnacuracy;
            }

        }

        double averageClimbingImprovementWhileSeeing = totalClimbingImprovement / climbingImprovedIterations;
        double averageICPImprovementWhileSeeing = totalICPImprovement / climbingImprovedIterations;
        double averageParticleFilterImprovementWhileSeeing = totalParticleFilterImprovement / climbingImprovedIterations;
        double averageFastPFImprovementWhileSeeing = totalFastPFImprovement / climbingImprovedIterations;
        double averageEvenFasterPFImprovementWhileSeeing = totalEvenFasterPFImprovement / climbingImprovedIterations;

        System.out.println(
                "EstimatedClimbingAccTest complete. Average Climbing Improvement when seeing something: "
                        + averageClimbingImprovementWhileSeeing + " meters over " + climbingImprovedIterations
                        + " iterations out of " + iterations + " total.");
        System.out.println(
                "EstimatedClimbingAccTest complete. Average ICP Improvement when seeing something: "
                        + averageICPImprovementWhileSeeing + " meters over " + climbingImprovedIterations
                        + " iterations out of " + iterations + " total.");
        System.out.println(
                "EstimatedClimbingAccTest complete. Average Particle Filter Improvement when seeing something: "
                        + averageParticleFilterImprovementWhileSeeing + " meters over " + climbingImprovedIterations
                        + " iterations out of " + iterations + " total.");
        System.out.println(
                "EstimatedClimbingAccTest complete. Average Fast PF Improvement when seeing something: "
                        + averageFastPFImprovementWhileSeeing + " meters over " + climbingImprovedIterations
                        + " iterations out of " + iterations + " total.");
        System.out.println(
                "EstimatedClimbingAccTest complete. Average Even Faster PF Improvement when seeing something: "
                        + averageEvenFasterPFImprovementWhileSeeing + " meters over " + climbingImprovedIterations
                        + " iterations out of " + iterations + " total.");

        System.out.println(
                "Average Starting Innacuracy when seeing something: "
                        + (totalStartingInnacuracy / climbingImprovedIterations) + " meters over "
                        + climbingImprovedIterations + " iterations out of " + iterations + " total.");

        System.out.println("Average Time Combined: " + (totalTimeCombined / (double)iterations / 1_000_000.0) + " ms");
        System.out.println("Average Time ICP Only: " + (totalTimeICP / (double)iterations / 1_000_000.0) + " ms");
        System.out.println("Average Time Particle Filter: " + (totalTimePF / (double)iterations / 1_000_000.0) + " ms");
        System.out.println("Average Time Fast PF: " + (totalTimeFastPF / (double)iterations / 1_000_000.0) + " ms");
        System.out.println("Average Time Even Faster PF: " + (totalTimeEvenFasterPF / (double)iterations / 1_000_000.0) + " ms");

        double meanFinalError = sumFinalError / climbingImprovedIterations;
        double variance = (sumSqFinalError / climbingImprovedIterations) - (meanFinalError * meanFinalError);
        double stdDev = Math.sqrt(variance);
        
        System.out.println("Final Accuracy Mean: " + meanFinalError + " meters");
        System.out.println("Final Accuracy Std Dev: " + stdDev + " meters");
    }
}
