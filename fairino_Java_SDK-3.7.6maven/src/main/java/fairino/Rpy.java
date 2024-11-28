package fairino;

/**
 * 笛卡尔空间位姿
 */
public class Rpy
{
    public double rx = 0.0;   /* 绕固定轴X旋转角度，单位：deg  */
    public double ry = 0.0;   /* 绕固定轴Y旋转角度，单位：deg  */
    public double rz = 0.0;   /* 绕固定轴Z旋转角度，单位：deg  */
    public Rpy(double rotateX, double rotateY, double rotateZ)
    {
        rx = rotateX;
        ry = rotateY;
        rz = rotateZ;
    }
}
