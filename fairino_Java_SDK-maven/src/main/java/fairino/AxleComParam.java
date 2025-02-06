package fairino;

public class AxleComParam
{
    public int baudRate;   //波特率：支持 1-9600，2-14400，3-19200，4-38400，5-56000，6-67600，7-115200，8-128000；
    public int dataBit;    //数据位：数据位支持（8,9），目前常用为 8
    public int stopBit;    //停止位：1-1，2-0.5，3-2，4-1.5，目前常用为 1
    public int verify;     //校验位：0-None，1-Odd，2-Even,目前常用为 0；
    public int timeout;    //超时时间：1~1000ms，此值需要结合外设搭配设置合理的时间参数
    public int timeoutTimes;  //超时次数：1~10，主要进行超时重发，减少偶发异常提高用户体验
    public int period;     //周期性指令时间间隔：1~1000ms，主要用于周期性指令每次下发的时间间隔

    public AxleComParam()
    {

    }

    public AxleComParam(int baudRate, int dataBit, int stopBit, int verify, int timeout, int timeoutTimes, int period)
    {
        this.baudRate = baudRate;
        this.dataBit = dataBit;
        this.stopBit = stopBit;
        this.verify = verify;
        this.timeout = timeout;
        this.timeoutTimes = timeoutTimes;
        this.period = period;
    }

    public String ToString()
    {
        return " " + baudRate + "," + dataBit + "," + stopBit + "," + verify + "," + timeout + "," + timeoutTimes + "," + period;
    }

}
