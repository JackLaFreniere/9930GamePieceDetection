package frc.robot.utilities;
import org.littletonrobotics.junction.Logger;

import frc.robot.LimelightHelpers;

public class LimeLightUtility {

    public LimeLightUtility() {}

    public double get_tx(String m_LimeLightName) {
        Logger.getInstance().recordOutput("tx", LimelightHelpers.getTX(m_LimeLightName));
        return LimelightHelpers.getTX(m_LimeLightName);
    }

    public double get_ty(String m_LimeLightName) {
        return LimelightHelpers.getTY(m_LimeLightName);
    }

    public double get_ta(String m_LimeLightName) {
        return LimelightHelpers.getTA(m_LimeLightName);
    }

}





