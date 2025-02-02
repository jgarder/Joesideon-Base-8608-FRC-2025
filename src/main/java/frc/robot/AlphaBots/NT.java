package frc.robot.AlphaBots;

import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class NT {
    public static NetworkTableInstance inst = NetworkTableInstance.getDefault();
    public static NetworkTable table = inst.getTable("AlphaBots");

    public static DoubleEntry getDoubleEntry(String SubTableName, String key, double defaultvalue)
    {
        return NT.table.getDoubleTopic(SubTableName + "/" + key).getEntry(defaultvalue);
    }
    public static BooleanEntry getBooleanEntry(String SubTableName,String key, Boolean defaultvalue)
    {
        return NT.table.getBooleanTopic(SubTableName + "/" + key).getEntry(defaultvalue);
    }
}
