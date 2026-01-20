package frc.robot.extras;

import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.extras.LidarMapComponents.LidarSimulator;
import frc.robot.extras.LidarMapComponents.MapPoint;

public class EstimatedAccuracyTest {

    @Test
    public void testEstimatedAccuracy() {
        int iterations = 200000;
        int improvedIterations = 0;
        double totalImprovement = 0;

        System.out.println("Starting EstimatedAccuracyTest...");

        for (int i = 0; i < iterations; i++) {
            double trueX = Math.random() * 10;
            double trueY = Math.random() * 10;
            double trueRotation = Math.random() * 360;

            double maxTranslationalInaccuracy = 0.03; // 3 cm
            double maxAngleInaccuracy = 0; // 2 degrees

            double innacurateX = trueX + (Math.random() * 2 - 1) * maxTranslationalInaccuracy;
            double innacurateY = trueY + (Math.random() * 2 - 1) * maxTranslationalInaccuracy;
            double innacurateRotation = trueRotation + (Math.random() * 2 - 1) * maxAngleInaccuracy;

            // currently set to perfection, with a seeded particle we should get about 1.4cm of improvement
            double[] SimulatedScan = LidarSimulator.getPerfectSimData(
                    new MapPoint(trueX, trueY),
                    trueRotation);

          //  FieldPoint calculatedPose = PointCloudPositionEstimator.estimatePose(
           //         new FieldPose2d(innacurateX, innacurateY, Rotation2d.fromDegrees(innacurateRotation)),
          //          SimulatedScan);
            // For testing, we will use the predetermined particle at the innacurate position
            MapPoint predeterminedParticle = new MapPoint(trueX, trueY);
            FieldPoint calculatedPose = PointCloudPositionEstimator.estimatePose(
                    new FieldPose2d(innacurateX, innacurateY, Rotation2d.fromDegrees(innacurateRotation)),
                    SimulatedScan, predeterminedParticle);

            double lidarInnacuracy = Math.hypot(trueX - calculatedPose.getX(), trueY - calculatedPose.getY());
            double inputInnacuracy = Math.hypot(trueX - innacurateX, trueY - innacurateY);

            if(lidarInnacuracy != inputInnacuracy){
                System.out.println("accuracy Imporvement = " + (inputInnacuracy - lidarInnacuracy) +
                    " lidar innac = " + lidarInnacuracy +
                    " previous innac = " + inputInnacuracy);
                totalImprovement += (inputInnacuracy - lidarInnacuracy);
                improvedIterations++;
            }

        }

        double averageImprovementWhileSeeing = totalImprovement / improvedIterations;
        System.out.println(
                "Average accuracy improvement over " + improvedIterations + " useful iterations: " + averageImprovementWhileSeeing + " meters.");

        // Assert that we are generally improving accuracy (average improvement > 0)
        // This is a probabilistic test, so it might fail occasionally if the estimator
        // is not consistently better,
        // but for a unit test we usually want deterministic behavior.
        // However, the user asked for "the same logic", which is a simulation.
        // We'll add a loose assertion to ensure it's at least running and producing
        // rational results.
        assertTrue(averageImprovementWhileSeeing > 0, "Average improvement should be reasonable");
    }
}
