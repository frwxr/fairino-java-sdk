
package fairino;
public class Axis485Param
{
    int servoCompany;           // 伺服驱动器厂商，1-戴纳泰克
    int servoModel;             // 伺服驱动器型号，1-FD100-750C
    int servoSoftVersion;       // 伺服驱动器软件版本，1-V1.0
    int servoResolution;        // 编码器分辨率
    double axisMechTransRatio;  // 机械传动比

    public Axis485Param(int company, int model, int softVersion, int resolution, double mechTransRatio)
    {
        servoCompany = company;
        servoModel = model;
        servoSoftVersion = softVersion;
        servoResolution = resolution;
        axisMechTransRatio = mechTransRatio;
    }

    public Axis485Param()
    {

    }
}
