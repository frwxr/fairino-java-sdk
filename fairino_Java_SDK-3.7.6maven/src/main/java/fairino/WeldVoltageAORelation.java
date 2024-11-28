
package fairino;
public class WeldVoltageAORelation
{
    public WeldVoltageAORelation(double weldVoltageMin, double weldVoltageMax, double outputVoltageMin, double outputVoltageMax, int AOIndex)
    {
        this.weldVoltageMin = weldVoltageMin;
        this.weldVoltageMax = weldVoltageMax;
        this.outputVoltageMin = outputVoltageMin;
        this.outputVoltageMax = outputVoltageMax;
        this.AOIndex = AOIndex;
    }

    public WeldVoltageAORelation()
    {

    }


    double weldVoltageMin;    // 焊接电压-模拟量输出线性关系左侧点焊接电压值(A)
    double weldVoltageMax;    // 焊接电压-模拟量输出线性关系右侧点焊接电压值(A)
    double outputVoltageMin;  // 焊接电压-模拟量输出线性关系左侧点模拟量输出电压值(V)
    double outputVoltageMax;  // 焊接电压-模拟量输出线性关系右侧点模拟量输出电压值(V)
    int AOIndex;              // 焊接电压模拟量输出端口
}
