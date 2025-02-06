package fairino;

/**
 * 力矩传感器
 */
public class ForceTorque
{
    public double fx;  /* 沿x轴受力分量，单位N  */
    public double fy;  /* 沿y轴受力分量，单位N  */
    public double fz;  /* 沿z轴受力分量，单位N  */
    public double tx;  /* 绕x轴力矩分量，单位Nm */
    public double ty;  /* 绕y轴力矩分量，单位Nm */
    public double tz;  /* 绕z轴力矩分量，单位Nm */
    public ForceTorque(double fX, double fY, double fZ, double tX, double tY, double tZ)
    {
        fx = fX;
        fy = fY;
        fz = fZ;
        tx = tX;
        ty = tY;
        tz = tZ;
    }
}
