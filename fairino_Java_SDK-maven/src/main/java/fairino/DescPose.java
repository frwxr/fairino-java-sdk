package fairino;

/**
 * 笛卡尔位置xyz和位姿rpy
 */
public class DescPose
{
    public DescTran tran = new DescTran(0.0, 0.0, 0.0);      /* 笛卡尔空间位置  */
    public Rpy rpy = new Rpy(0.0, 0.0, 0.0);			/* 笛卡尔空间姿态  */

    public DescPose()
    {

    }

    public DescPose(DescTran descTran, Rpy rotateRpy)
    {
        tran = descTran;
        rpy = rotateRpy;
    }

    public DescPose(double tranX, double tranY, double tranZ, double rX, double ry, double rz)
    {
        tran.x = tranX;
        tran.y = tranY;
        tran.z = tranZ;
        rpy.rx = rX;
        rpy.ry = ry;
        rpy.rz = rz;
    }

    public String toString()
    {
        return String.valueOf(tran.x) + "," +  String.valueOf(tran.y) + "," +String.valueOf(tran.z) + "," +String.valueOf(rpy.rx) + "," +String.valueOf(rpy.ry) + "," +String.valueOf(rpy.rz);
    }
}
