package fairino;

/**
 * 扩展轴位置
 */
public class ExaxisPos
{
    public double axis1 = 0.0;
    public double axis2 = 0.0;
    public double axis3 = 0.0;
    public double axis4 = 0.0;

    public ExaxisPos()
    {

    }
    public ExaxisPos(double[] exaxisPos)
    {
        axis1 = exaxisPos[0];
        axis2 = exaxisPos[1];
        axis3 = exaxisPos[2];
        axis4 = exaxisPos[3];
    }

    public ExaxisPos(double pos1, double pos2, double pos3, double pos4)
    {
        axis1 = pos1;
        axis2 = pos2;
        axis3 = pos3;
        axis4 = pos4;
    }
}
