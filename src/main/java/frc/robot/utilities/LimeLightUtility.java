package frc.robot.utilities;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.LimelightHelpers;

public class LimeLightUtility {

    private String m_LimeLightName;
    private NetworkTableInstance m_networkTable = NetworkTableInstance.getDefault();

    public LimeLightUtility(String LimeLightName) {
        m_LimeLightName = LimeLightName;
    }

    public double get_tx(String m_LimeLightName) {
        return 1.0;
        // return LimelightHelpers.getTX(m_LimeLightName);
    }

    public double get_ty(String m_LimeLightName) {
        return LimelightHelpers.getTY(m_LimeLightName);
    }

}
