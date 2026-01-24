package frc.robot.subsystems;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.extras.YDLidarGS2;
import frc.robot.extras.YDLidarGS2.LidarPoint;

public class LidarSubsystem extends SubsystemBase {
    private final YDLidarGS2 lidar;

    // Change this to kMXP if wired to the expansion port
    private static final SerialPort.Port kLidarPort = SerialPort.Port.kUSB;

    public LidarSubsystem() {
        lidar = new YDLidarGS2(kLidarPort);
        
        // Run setup in a separate thread so it doesn't block robot init
        new Thread(() -> {
            try {
                if (lidar.initialize()) {
                    lidar.startScanning();
                    SmartDashboard.putBoolean("Lidar/Connected", true);
                } else {
                    SmartDashboard.putBoolean("Lidar/Connected", false);
                }
            } catch (Exception e) {
                SmartDashboard.putBoolean("Lidar/Connected", false);
                SmartDashboard.putString("Lidar/Error", e.getMessage());
                System.err.println("Failed to start Lidar: " + e.getMessage());
            }
        }).start();
    }

    @Override
    public void periodic() {
        // Get the latest complete scan if available
        LidarPoint[] points = lidar.getLatestScan();

        if (points != null) {
            // Log diagnostics to SmartDashboard
            SmartDashboard.putNumber("Lidar/PointCount", points.length);

            if (points.length > 0) {
                // Log sample data from the center point
                LidarPoint sample = points[points.length / 2];
                SmartDashboard.putNumber("Lidar/Sample/Dist_mm", sample.distance);
                SmartDashboard.putNumber("Lidar/Sample/Angle_deg", sample.angle);
                
                // Calculate Cartesian coordinates for visualization
                double rads = Math.toRadians(sample.angle);
                double x = sample.distance * Math.cos(rads);
                double y = sample.distance * Math.sin(rads);

                SmartDashboard.putNumber("Lidar/Sample/X", x);
                SmartDashboard.putNumber("Lidar/Sample/Y", y);
                
                // Optional: Calculate and log average distance as a rough sanity check
                double totalDist = 0;
                for (LidarPoint p : points) {
                    totalDist += p.distance;
                }
                SmartDashboard.putNumber("Lidar/AvgDist_mm", totalDist / points.length);
            }
        }
    }
    
    /**
     * Stop the lidar when the subsystem is not in use or robot is disabled 
     * (though usually simpler to just leave it running).
     */
    public void stopLidar() {
        lidar.stopScanning();
    }
}
