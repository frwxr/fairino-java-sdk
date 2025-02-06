package fairino;
public class WeldCurrentAORelation
{
    public WeldCurrentAORelation(double currentMin, double currentMax, double outputVoltageMin, double outputVoltageMax, int AOIndex)
    {
        this.currentMin = currentMin;
        this.currentMax = currentMax;
        this.outputVoltageMin = outputVoltageMin;
        this.outputVoltageMax = outputVoltageMax;
        this.AOIndex = AOIndex;
    }

    public WeldCurrentAORelation()
    {

    }

    double currentMin = 0.0;      // 焊接电流-模拟量输出线性关系左侧点电流值(A)
    double currentMax = 10;      // 焊接电流-模拟量输出线性关系右侧点电流值(A)
    double outputVoltageMin = 0.0;// 焊接电流-模拟量输出线性关系左侧点模拟量输出电压值(V)
    double outputVoltageMax = 0.01;// 焊接电流-模拟量输出线性关系右侧点模拟量输出电压值(V)
    int AOIndex = 0;              // 焊接电流模拟量输出端口
}
