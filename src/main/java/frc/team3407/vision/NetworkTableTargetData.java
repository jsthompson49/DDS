package frc.team3407.vision;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class NetworkTableTargetData {

    private static final String VISION_TABLE_HATCH = "Vision_Hatch";
    private static final String OFFSET_NAME = "offset";
    private static final String COUNT_NAME = "count";
    private static final String AVERAGE_TIME_NAME = "timing";

    private NetworkTableEntry offsetEntry;
    private NetworkTableEntry countEntry;
    private NetworkTableEntry timingEntry;

    public NetworkTableTargetData() {
        NetworkTableInstance root = NetworkTableInstance.getDefault();
        NetworkTable table = root.getTable(VISION_TABLE_HATCH);
        offsetEntry = table.getEntry(OFFSET_NAME);
        countEntry = table.getEntry(COUNT_NAME);
        timingEntry = table.getEntry(AVERAGE_TIME_NAME);
    }

    public void update(double offset, long count, long duration) {
        offsetEntry.setNumber(offset);
        countEntry.setNumber(count);
        if (count > 0) {
            timingEntry.setNumber(duration / count);
        }
    }
}
