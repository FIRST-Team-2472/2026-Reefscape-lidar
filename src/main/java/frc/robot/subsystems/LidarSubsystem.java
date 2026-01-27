package frc.robot.subsystems;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.extras.YDLidarGS2;
import frc.robot.extras.YDLidarGS2.LidarPoint;

public class LidarSubsystem extends SubsystemBase {
    private YDLidarGS2 leftLidar;
    private YDLidarGS2 rightLidar;

    // Change this to kMXP if wired to the expansion port
    private static final SerialPort.Port kLidar1Port = SerialPort.Port.kUSB1;
    private static final SerialPort.Port kLidar2Port = SerialPort.Port.kUSB2;

    public LidarSubsystem() {
        startLidarInit("Left", kLidar1Port);
        startLidarInit("Right", kLidar2Port);
    }

    private void startLidarInit(String name, SerialPort.Port port) {
         new Thread(() -> {
            int retryCount = 0;
            YDLidarGS2 lidar = null;

            // Give the roboRIO some time to boot and enumerate USB devices
            try { Thread.sleep(1000); } catch (InterruptedException e) {}

            while (retryCount < 20 && lidar == null) {
                try {
                    lidar = new YDLidarGS2(port);
                } catch (Exception e) {
                    SmartDashboard.putString("Lidar/" + name + "/Status", "Port Error (Entry " + retryCount + "): " + e.getMessage());
                    // Wait before retrying
                    try { Thread.sleep(1000); } catch (InterruptedException ie) {}
                }
                retryCount++;
            }

            if (lidar == null) {
                SmartDashboard.putString("Lidar/" + name + "/Status", "Gave up opening port after attempts.");
                return;
            }

            // Now try to initialize the sensor protocol
            retryCount = 0;
            boolean initialized = false;
            while(retryCount < 20 && !initialized) {
                 if (lidar.initialize()) {
                     initialized = true;
                     lidar.startScanning();
                     
                     // Assign to the class field safely
                     if (name.equals("Left")) this.leftLidar = lidar;
                     else this.rightLidar = lidar;

                     SmartDashboard.putBoolean("Lidar/" + name + "/Connected", true);
                     SmartDashboard.putString("Lidar/" + name + "/Status", "Running");
                 } else {
                     SmartDashboard.putString("Lidar/" + name + "/Status", "Proto Init Failed (Attempt " + retryCount + ")");
                     try { Thread.sleep(500); } catch (InterruptedException ie) {}
                 }
                 retryCount++;
            }
        }).start();
    }

    @Override
    public void periodic() {
        // Get the latest complete scan if available
        if (leftLidar != null) {
            LidarPoint[] leftLidarPoints = leftLidar.getLatestScan();

            if (leftLidarPoints != null) {
                // Log diagnostics to SmartDashboard
                SmartDashboard.putNumber("Lidar/Left/PointCount", leftLidarPoints.length);
                if (leftLidarPoints.length > 0) {
                    double sumDistances = 0;
                    for (LidarPoint p : leftLidarPoints) {
                        sumDistances += p.distance;
                    }
                    double avgDistance = sumDistances / leftLidarPoints.length;
                    SmartDashboard.putNumber("Lidar/Left/AvgDistanceMeters", avgDistance);
                }
            }
        }
        
        if (rightLidar != null) {
            LidarPoint[] rightLidarPoints = rightLidar.getLatestScan();
             if (rightLidarPoints != null) {
                SmartDashboard.putNumber("Lidar/Right/PointCount", rightLidarPoints.length);
                if (rightLidarPoints.length > 0) {
                    double sumDistances = 0;
                    for (LidarPoint p : rightLidarPoints) {
                        sumDistances += p.distance;
                    }
                    double avgDistance = sumDistances / rightLidarPoints.length;
                    SmartDashboard.putNumber("Lidar/Right/AvgDistanceMeters", avgDistance);
                }
             }
        }
    }
    
    /**
     * Stop the lidar when the subsystem is not in use or robot is disabled 
     * (though usually simpler to just leave it running).
     */
    public void stopLidar() {
        if (leftLidar != null) leftLidar.stopScanning();
        if (rightLidar != null) rightLidar.stopScanning();
    }
}
