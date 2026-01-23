package frc.robot.extras;

import org.junit.jupiter.api.Test;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.LidarConstants;
import frc.robot.extras.LidarMapComponents.LidarSimulator;
import frc.robot.extras.LidarMapComponents.MapPoint;
import frc.robot.extras.LidarMapComponents.Polygon;

public class EstimatedClimbingAccTest {
    @Test
    public void testEstimatedClimbingAcc() {
        int iterations = 20000;
        int improvedIterations = 0;
        double totalImprovement = 0;
        double totalStartingInnacuracy = 0;
        double sumFinalError = 0;
        double sumSqFinalError = 0;
        long totalTime = 0;

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

            // Generate "Messy" data
            double[][] SimulatedDualScan = LidarSimulator.getDualLidarSimData(
                    new MapPoint(trueX, trueY),
                    trueRotation,
                    1, 
                    LidarConstants.kLidarHorizontalOffsetFromCenterMeters,
                    LidarConstants.kLidarAngleOffsetFromForwardDegrees,
                    new Polygon[]{LidarMap.getMap()[2]});
            
            // Estimate pose using ONLY the best method (EvenFaster)
            long start = System.nanoTime();
            FieldPoint bestPose = PointCloudPositionEstimator.estimatePoseDualLidar(
                    new FieldPose2d(innacurateX, innacurateY, Rotation2d.fromDegrees(innacurateRotation)),
                    SimulatedDualScan, new Polygon[]{LidarMap.getMap()[2]});
            totalTime += (System.nanoTime() - start);

            double finalInnacuracy = Math.hypot(trueX - bestPose.getX(), trueY - bestPose.getY());
            double inputInnacuracy = Math.hypot(trueX - innacurateX, trueY - innacurateY);

            if (finalInnacuracy != inputInnacuracy) {
                totalImprovement += (inputInnacuracy - finalInnacuracy);
                improvedIterations++;
                totalStartingInnacuracy += inputInnacuracy;
                
                sumFinalError += finalInnacuracy;
                sumSqFinalError += finalInnacuracy * finalInnacuracy;
            }

        }

        double averageImprovement = totalImprovement / improvedIterations;

        System.out.println(
                "EstimatedClimbingAccTest complete. Average Improvement: "
                        + averageImprovement + " meters over " + improvedIterations
                        + " iterations out of " + iterations + " total.");

        System.out.println(
                "Average Starting Innacuracy: "
                        + (totalStartingInnacuracy / improvedIterations) + " meters");

        double meanFinalError = sumFinalError / improvedIterations;
        double variance = (sumSqFinalError / improvedIterations) - (meanFinalError * meanFinalError);
        double stdDev = Math.sqrt(variance);
        
        System.out.println("Final Accuracy Mean: " + meanFinalError + " meters");
        System.out.println("Final Accuracy Std Dev: " + stdDev + " meters");
        System.out.println("Average Execution Time: " + (totalTime / (double)iterations / 1_000_000.0) + " ms");
    }
}
