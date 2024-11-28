package fairino;

/**
 * 笛卡尔空间位置
 */
public class DescTran
{
    public double x = 0.0;    /* x轴坐标，单位mm  */
    public double y = 0.0;    /* y轴坐标，单位mm  */
    public double z = 0.0;    /* z轴坐标，单位mm  */
    public DescTran(double posX, double posY, double posZ)
    {
        x = posX;
        y = posY;
        z = posZ;
    }

    public DescTran()
    {

    }

}
