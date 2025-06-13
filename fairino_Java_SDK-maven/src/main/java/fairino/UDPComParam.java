package fairino;
public class UDPComParam
{
    public UDPComParam()
    {

    }

    public UDPComParam(String ip, int port, int period, int lossPkgTime, int lossPkgNum, int disconnectTime, int reconnectEnable, int reconnectPeriod, int reconnectNum,int selfConnect)
    {
        this.ip = ip;
        this.port = port;
        this.period = period;
        this.lossPkgTime = lossPkgTime;
        this.lossPkgNum = lossPkgNum;
        this.disconnectTime = disconnectTime;
        this.reconnectEnable = reconnectEnable;
        this.reconnectPeriod = reconnectPeriod;
        this.reconnectNum = reconnectNum;
        this.selfConnect =selfConnect;

    }

    public String ip = "192.168.58.88";
    public int port = 2021;
    public int period = 2;
    public int lossPkgTime = 50;
    public int lossPkgNum = 2;
    public int disconnectTime = 100;
    public int reconnectEnable = 0;
    public int reconnectPeriod = 100;
    public int reconnectNum = 3;
    public int selfConnect =0;
}
