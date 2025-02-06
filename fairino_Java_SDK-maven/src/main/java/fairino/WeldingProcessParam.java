package fairino;
public class WeldingProcessParam
{
     double  startCurrent = 0.0;   // 起弧电流(A)
     double  startVoltage = 0.0;   // 起弧电压(V)
     int  startTime = 0;         // 起弧时间(ms)
     double  weldCurrent = 0.0;    // 焊接电流(A)
     double  weldVoltage = 0.0;    // 焊接电压(V)
     double  endCurrent = 0.0;     // 收弧电流(A)
     double  endVoltage = 0.0;     // 收弧电压(V)
     int  endTime = 0;           // 收弧时间(ms)

     public WeldingProcessParam(  double startCurrent , double startVoltage , int startTime , double weldCurrent , double weldVoltage , double endCurrent , double endVoltage , int endTime  )
     {
          this.startCurrent = startCurrent;
          this.startVoltage = startVoltage;
          this.startTime = startTime;
          this.weldCurrent = weldCurrent;
          this.weldVoltage = weldVoltage;
          this.endCurrent = endCurrent;
          this.endVoltage = endVoltage;
          this.endTime = endTime;
     }

     public WeldingProcessParam()
     {

     }
}
