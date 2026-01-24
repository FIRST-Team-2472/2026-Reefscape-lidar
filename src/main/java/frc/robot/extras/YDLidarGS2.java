package frc.robot.extras;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Timer;

public class YDLidarGS2 {
    private final SerialPort serial;
    private final int baudRate;
    private boolean isScanning = false;
    
    // Protocol Constants
    private static final byte[] HEADER = {(byte)0xA5, (byte)0xA5, (byte)0xA5, (byte)0xA5};
    // private static final byte CMD_GET_ADDRESS = 0x60;
    private static final byte CMD_GET_PARAMETERS = 0x61;
    // private static final byte CMD_GET_VERSION = 0x62;
    private static final byte CMD_START_SCAN = 0x63;
    private static final byte CMD_STOP_SCAN = 0x64;
    
    // Calibration parameters (fetched from device)
    private double k0 = 1.0, b0 = 0.0, k1 = 1.0, b1 = 0.0, bias = 0.0;
    
    // Math constants
    private static final double ANGLE_PX = 1.22;
    private static final double ANGLE_PY = 5.315;
    private static final double ANGLE_PANGLE = 22.5;

    // Buffer for reading
    private final byte[] buffer = new byte[1024];
    private int bufferIdx = 0;

    public static class LidarPoint {
        public double angle; // degrees
        public double distance; // mm
        public int intensity;
    }

    public YDLidarGS2(SerialPort.Port port) {
        this.baudRate = 921600; // GS2 Default
        this.serial = new SerialPort(baudRate, port);
        this.serial.setReadBufferSize(2048);
        this.serial.setTimeout(0.01);
    }

    public boolean initialize() {
        stopScanning();
        Timer.delay(0.1);
        
        // flush
        readAllBytes();
        
        // 1. Get Parameters (Calibration)
        if (!fetchParameters()) {
            System.out.println("YDLIDAR GS2: Failed to fetch parameters!");
            return false;
        }
        
        System.out.println(String.format("YDLIDAR GS2 Calibrated: K0=%.4f B0=%.4f K1=%.4f B1=%.4f Bias=%.4f", k0, b0, k1, b1, bias));
        return true;
    }

    public void startScanning() {
        if (isScanning) return;
        sendCommand(0x00, CMD_START_SCAN, new byte[0]);
        isScanning = true;
    }

    public void stopScanning() {
        sendCommand(0x00, CMD_STOP_SCAN, new byte[0]);
        sendCommand(0x00, CMD_STOP_SCAN, new byte[0]); // Send twice to be sure
        isScanning = false;
        Timer.delay(0.1);
        readAllBytes(); // Flush buffer
    }

    /**
     * Reads available data and parses complete packets. 
     * call this periodically (e.g. in robotPeriodic or a separate thread)
     * returns an array of points if a full scan is parsed, otherwise null.
     */
    public LidarPoint[] getLatestScan() {
        readDataToBuffer();
        
        // Look for header
        int headerIdx = findHeader();
        if (headerIdx == -1) {
            // No header found, keep last few bytes in case header is split, discard rest
            compactBuffer(bufferIdx > 3 ? bufferIdx - 3 : 0);
            return null;
        }

        // Check if we have enough bytes for header + basic fields (Header(4)+Addr(1)+Type(1)+Len(2))
        if (bufferIdx - headerIdx < 8) {
            return null; // Wait for more data
        }

        // Parse Length
        int lenLSB = buffer[headerIdx + 6] & 0xFF;
        int lenMSB = buffer[headerIdx + 7] & 0xFF;
        int dataLen = (lenMSB << 8) | lenLSB;
        
        // Full packet size = 8 (Head+Info) + dataLen + 1 (Checksum)
        int packetSize = 8 + dataLen + 1;
        
        if (bufferIdx - headerIdx < packetSize) {
            return null; // Wait for full packet
        }

        // Verify Checksum
        byte calcSum = 0;
        for (int i = headerIdx + 4; i < headerIdx + packetSize - 1; i++) {
            calcSum += buffer[i];
        }
        byte rxSum = buffer[headerIdx + packetSize - 1];
        
        if (calcSum != rxSum) {
            System.out.println("YDLIDAR Checksum Mismatch!");
            compactBuffer(headerIdx + 1); // Skip A5 and try again
            return null;
        }

        // Parse Data
        byte packetType = buffer[headerIdx + 5];
        LidarPoint[] points = null;

        if (packetType == (byte)0x81 || packetType == (byte)0x01) { // Measurement Packet
            points = parseMeasurement(buffer, headerIdx + 8, dataLen);
        }

        // Done with this packet
        compactBuffer(headerIdx + packetSize);
        return points;
    }

    private LidarPoint[] parseMeasurement(byte[] data, int offset, int length) {
        // Data format: Environment(2 bytes) + 160 points * 2 bytes
        // Each point (2 bytes): [Quality(7bit) | Dist(9bit)? Is that right? check C++]
        // C++: MSB_LSBtoUINT16(..,..) & 0x01ff -> Distance? No.
        
        // C++ getMeasurements logic:
        // captured is array of bytes.
        // i goes 0 to 320 (160 * 2)
        // distance/angle is calculated from "pixelU" (the index n)
        // WAIT. The C++ code implies the data doesn't contain distance directly?
        // "getMeasurements(..., n, ...)"
        // The data seems to be "captured[(i+1)+2]" and "captured[i+2]" ??
        // Ah, "captured" in iter_Scan starts after address/type/len?
        // Let's assume standard payload.
        
        // According to C++:
        // captured array contains: Env(2) + 160 * 2 bytes.
        // index 0,1 is Env.
        // loop i=0 to 319 step 2.
        // byte1 = data[2 + i + 1]
        // byte2 = data[2 + i]
        // raw = (byte1 << 8) | byte2
        // distance raw component? 
        // quality = byte1 >> 1
        
        // Wait, the C++ code says:
        // getMeasurements((MSB_LSBtoUINT16(captured[(i+1)+2], captured[i+2]) & 0x01ff), n, ...)
        
        int pointCount = 160;
        LidarPoint[] points = new LidarPoint[pointCount];
        
        // Environment bytes at offset, offset+1 (Total 2 bytes)
        // Points start at offset + 2
        
        for (int n = 0; n < pointCount; n++) {
            int i = n * 2;
            int bLSB = data[offset + 2 + i] & 0xFF;
            int bMSB = data[offset + 2 + i + 1] & 0xFF;
            
            int raw = (bMSB << 8) | bLSB;
            int pixelData = raw & 0x01FF; // Lower 9 bits?
            int quality = raw >> 9; // Upper 7 bits? check C++: (captured[...] >> 1) which is Byte2 >> 1?
            // C++: quality = (captured[(i+1)+2] >> 1); -> That is MSB >> 1. 
            // So quality is 7 bits. (MSB & 0xFE) >> 1 ?
            
            points[n] = calculatePoint(n, pixelData); // n is the pixel index (0-159)
            points[n].intensity = quality;
        }
        return points;
    }

    private LidarPoint calculatePoint(int n, int pixelData) {
        // This is complex triangulation math from the driver
        // pixelU = n ? C++ says "double pixelU = n"
        // But checking `getMeasurements`: `pixelU = n` passed in.
        // Wait, what is `pixelData` used for?
        // In C++ `getMeasurements`: `if (n < 80) ... pixelU = 80 - pixelU;`
        // It DOES NOT use the raw data (0x01ff) for angle calc?
        // The raw data (0x01ff) seems unused in the C++ math snippet I saw??
        // Wait. `getMeasurements(uint16_t dist, int n, ...)`
        // `dist` param is `raw & 0x01ff`.
        // Inside: `tempDist = (dist - ANGLE_PX) / cos(...)` 
        // So `pixelData` is "dist" in the formula.
        
        LidarPoint p = new LidarPoint();
        double pixelU = n;
        double dist = pixelData; // "dist" input
        double tempTheta, tempDist, tempX, tempY;

        if (n < 80) {
            pixelU = 80 - pixelU;
            if (b0 > 1) {
                tempTheta = k0 * pixelU - b0;
            } else {
                tempTheta = Math.toDegrees(Math.atan(k0 * pixelU - b0));
            }
            tempDist = (dist - ANGLE_PX) / Math.cos(Math.toRadians(ANGLE_PANGLE + bias - tempTheta));
            tempTheta = Math.toRadians(tempTheta);
            
            tempX = Math.cos(Math.toRadians(ANGLE_PANGLE + bias)) * tempDist * Math.cos(tempTheta) +
                    Math.sin(Math.toRadians(ANGLE_PANGLE + bias)) * (tempDist * Math.sin(tempTheta));
            
            tempY = -Math.sin(Math.toRadians(ANGLE_PANGLE + bias)) * tempDist * Math.cos(tempTheta) +
                    Math.cos(Math.toRadians(ANGLE_PANGLE + bias)) * (tempDist * Math.sin(tempTheta));
                    
            tempX = tempX + ANGLE_PX;
            tempY = tempY - ANGLE_PY;
            
            p.distance = Math.sqrt(tempX * tempX + tempY * tempY);
            p.angle = Math.toDegrees(Math.atan2(tempY, tempX));
        } else {
            pixelU = 160 - pixelU;
            if (b1 > 1) {
                tempTheta = k1 * pixelU - b1;
            } else {
                tempTheta = Math.toDegrees(Math.atan(k1 * pixelU - b1));
            }
            tempDist = (dist - ANGLE_PX) / Math.cos(Math.toRadians(ANGLE_PANGLE + bias + tempTheta));
            tempTheta = Math.toRadians(tempTheta);
            
            tempX = Math.cos(Math.toRadians(-(ANGLE_PANGLE + bias))) * tempDist * Math.cos(tempTheta) +
                    Math.sin(Math.toRadians(-(ANGLE_PANGLE + bias))) * (tempDist * Math.sin(tempTheta));
            
            tempY = -Math.sin(Math.toRadians(-(ANGLE_PANGLE + bias))) * tempDist * Math.cos(tempTheta) +
                    Math.cos(Math.toRadians(-(ANGLE_PANGLE + bias))) * (tempDist * Math.sin(tempTheta));

            tempX = tempX + ANGLE_PX;
            tempY = tempY + ANGLE_PY;
            
            p.distance = Math.sqrt(tempX * tempX + tempY * tempY);
            p.angle = Math.toDegrees(Math.atan2(tempY, tempX)); 
        }

        if (p.angle < 0) {
            p.angle += 360;
        }

        return p;
    }

    private boolean fetchParameters() {
        sendCommand(0x00, CMD_GET_PARAMETERS, new byte[0]);
        Timer.delay(0.1);
        
        byte[] resp = readPacket(CMD_GET_PARAMETERS);
        if (resp == null) return false;

        // resp is just the data payload (after header/addr/type/len)
        // C++: 
        // K0: bytes 9(MSB), 8(LSB) -> index 8,9 in capture (which includes header?)
        // C++ capture index 9 is K0_MSB, 8 is K0_LSB if offset=0.
        // Packet: Head(4)+Addr(1)+Type(1)+Len(2)+Data(N)+Check(1)
        // C++ loop: capture[8+offset] is V1... wait that's GetVersion.
        // For Parameters:
        // k0_LSB = captured[8], k0_MSB = captured[9]
        // k1_LSB = captured[12], k1_MSB = captured[13]
        // b0_LSB = captured[10], b0_MSB = captured[11]
        // b1...
        // bias_LSB = captured[16]
        
        // Note: My readPacket returns the "Data" segment.
        // So index 0 corresponds to captured[8]? No.
        // C++ reads (STD+PARAM_LEN) bytes. 
        // STD length = 8 (Head..Len).
        // PARAM_LEN = from define... likely 17?
        // C++: K0_LSB = captured[8...]. 
        // So yes, index 0 of the data payload is 'captured[8]'. 
        
        // Data payload map:
        // 0: K0 LSB
        // 1: K0 MSB
        // 2: B0 LSB
        // 3: B0 MSB
        // 4: K1 LSB
        // 5: K1 MSB
        // 6: B1 LSB
        // 7: B1 MSB
        // 8: Bias LSB
        
        if (resp.length < 9) return false;
        
        k0 = toUInt16(resp[1], resp[0]) / 10000.0;
        b0 = toUInt16(resp[3], resp[2]) / 10000.0;
        k1 = toUInt16(resp[5], resp[4]) / 10000.0;
        b1 = toUInt16(resp[7], resp[6]) / 10000.0;
        bias = (resp[8] & 0xFF) / 10.0;
        
        return true;
    }
    
    // Blocking read for specific packet type (with timeout)
    private byte[] readPacket(byte expectedType) {
        double start = Timer.getFPGATimestamp();
        while (Timer.getFPGATimestamp() - start < 1.0) {
            readDataToBuffer();
            int headerIdx = findHeader();
            if (headerIdx == -1) { 
                compactBuffer(0); 
                Timer.delay(0.01); 
                continue; 
            }
            
            if (bufferIdx - headerIdx < 8) continue;
            
            int len = (buffer[headerIdx + 7] & 0xFF) << 8 | (buffer[headerIdx + 6] & 0xFF);
            int packetSize = 8 + len + 1;
            
            if (bufferIdx - headerIdx < packetSize) continue;
            
            byte type = buffer[headerIdx + 5];
            if (type == expectedType) {
                byte[] data = new byte[len];
                System.arraycopy(buffer, headerIdx + 8, data, 0, len);
                compactBuffer(headerIdx + packetSize);
                return data;
            } else {
                compactBuffer(headerIdx + packetSize); // Wrong packet, skip
            }
        }
        return null;
    }

    private void sendCommand(int addr, byte cmd, byte[] data) {
        int len = data.length;
        byte[] frame = new byte[8 + len + 1];
        System.arraycopy(HEADER, 0, frame, 0, 4);
        frame[4] = (byte)addr;
        frame[5] = cmd;
        frame[6] = (byte)(len & 0xFF);
        frame[7] = (byte)((len >> 8) & 0xFF);
        
        byte checksum = 0;
        checksum += frame[4];
        checksum += frame[5];
        checksum += frame[6];
        checksum += frame[7];
        
        for (int i=0; i<len; i++) {
            frame[8+i] = data[i];
            checksum += data[i];
        }
        frame[8+len] = checksum;
        
        serial.write(frame, frame.length);
    }
    
    private void readDataToBuffer() {
        try {
            int bytes = serial.getBytesReceived();
            if (bytes > 0) {
                byte[] raw = serial.read(Math.min(bytes, buffer.length - bufferIdx));
                System.arraycopy(raw, 0, buffer, bufferIdx, raw.length);
                bufferIdx += raw.length;
            }
        } catch (Exception e) {
            System.out.println("Serial Read Error: " + e.getMessage());
        }
    }
    
    private int findHeader() {
        for (int i = 0; i < bufferIdx - 3; i++) {
            if (buffer[i] == HEADER[0] && buffer[i+1] == HEADER[1] &&
                buffer[i+2] == HEADER[2] && buffer[i+3] == HEADER[3]) {
                return i;
            }
        }
        return -1;
    }
    
    private void compactBuffer(int startOfUsefulData) {
        if (startOfUsefulData >= bufferIdx) {
            bufferIdx = 0;
            return;
        }
        int len = bufferIdx - startOfUsefulData;
        System.arraycopy(buffer, startOfUsefulData, buffer, 0, len);
        bufferIdx = len;
    }
    
    private void readAllBytes() {
        serial.read(serial.getBytesReceived());
        bufferIdx = 0;
    }
    
    private int toUInt16(byte msb, byte lsb) {
        return ((msb & 0xFF) << 8) | (lsb & 0xFF);
    }
}
