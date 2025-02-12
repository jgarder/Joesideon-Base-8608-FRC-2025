package frc.robot.AlphaBots;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringArrayEntry;
import edu.wpi.first.networktables.StructEntry;
import edu.wpi.first.networktables.StructPublisher;

public class NT {
    public static final String TeamNetworkTableName = "AlphaBots";
    public static NetworkTableInstance inst = NetworkTableInstance.getDefault();
    public static NetworkTable table = inst.getTable(TeamNetworkTableName);

    public static DoubleEntry getDoubleEntry(String SubTableName, String key, double defaultvalue)
    {
        return NT.table.getDoubleTopic(SubTableName + "/" + key).getEntry(defaultvalue);
    }
    public static BooleanEntry getBooleanEntry(String SubTableName,String key, Boolean defaultvalue)
    {
        return NT.table.getBooleanTopic(SubTableName + "/" + key).getEntry(defaultvalue);
    }
    public static StringArrayEntry getStringArrayEntry(String SubTableName,String key, String[] defaultvalue)
    {
        return NT.table.getStringArrayTopic(SubTableName + "/" + key).getEntry(defaultvalue);
    }
    // public static NetworkTableEntry getNetworkTableEntry(String SubTableName,String key, String[] defaultvalue)
    // {
    //     return NT.table.getEntry(SubTableName + "/" + key).getEntry(defaultvalue);
    // }
    public static StructEntry<Pose2d> getStructEntry_Pose2D(String SubTableName,String key, Pose2d defaultvalue)
    {
        return NT.table.getStructTopic(SubTableName + "/" + key, Pose2d.struct).getEntry(new Pose2d());
    }

    
}
