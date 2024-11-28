package fairino;

public class EXT_AXIS_STATUS
{
    public double pos = 0;        //扩展轴位置
    public double vel = 0;        //扩展轴速度
    public int errorCode = 0;     //扩展轴故障码
    public int ready = 0;        //伺服准备好
    public int inPos = 0;        //伺服到位
    public int alarm = 0;        //伺服报警
    public int flerr = 0;        //跟随误差
    public int nlimit = 0;       //到负限位
    public int pLimit = 0;       //到正限位
    public int mdbsOffLine = 0;  //驱动器485总线掉线
    public int mdbsTimeout = 0;  //控制卡与控制箱485通信超时
    public int homingStatus = 0; //扩展轴回零状态
}
