package fairino;

public class ROBOT_AUX_STATE
{
    public int servoId = 0;           //伺服驱动器ID号
    public int servoErrCode = 0;       //伺服驱动器故障码
    public int servoState = 0;         //伺服驱动器状态
    public double servoPos = 0;        //伺服当前位置
    public float servoVel = 0;         //伺服当前速度
    public float servoTorque = 0;      //伺服当前转矩    25
}
