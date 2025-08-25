package frc.robot.lib;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class VisionHelper {
    private final NetworkTableEntry tx;
    private final NetworkTableEntry ty;
    private final NetworkTableEntry tz;
    private final NetworkTableEntry rx;
    private final NetworkTableEntry ry;
    private final NetworkTableEntry rz;
    private final NetworkTableEntry id;
    private final NetworkTableEntry tv;

    public VisionHelper(String tableName) {
        this.tx = NetworkTableInstance.getDefault().getTable(tableName).getEntry("tx");
        this.ty = NetworkTableInstance.getDefault().getTable(tableName).getEntry("ty");
        this.tz = NetworkTableInstance.getDefault().getTable(tableName).getEntry("tz");
        this.rx = NetworkTableInstance.getDefault().getTable(tableName).getEntry("rx");
        this.ry = NetworkTableInstance.getDefault().getTable(tableName).getEntry("ry");
        this.rz = NetworkTableInstance.getDefault().getTable(tableName).getEntry("rz");
        this.id = NetworkTableInstance.getDefault().getTable(tableName).getEntry("tid");
        this.tv = NetworkTableInstance.getDefault().getTable(tableName).getEntry("tv"); 
    }

    public boolean hasTarget() {
        return tv.getDouble(0) == 1;
    }

    public double getTx() {
        return tx.getDouble(0);
    }

    public double getTy() {
        return ty.getDouble(0);
    }

    public double getTz() {
        return tz.getDouble(0);
    }

    public double getRx() {
        return rx.getDouble(0);
    }

    public double getRy() {
        return ry.getDouble(0);
    }
    public double getRz() {
        return rz.getDouble(0);
    }

    public double getId() {
        return id.getDouble(0);
    }
}