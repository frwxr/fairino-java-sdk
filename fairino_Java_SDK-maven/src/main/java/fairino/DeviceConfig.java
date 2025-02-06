package fairino;
public class DeviceConfig
{
    int company = 0;          // 厂商
    int device = 0;           // 类型/设备号
    int softwareVersion = 0;  // 软件版本
    int bus = 0;              // 挂载位置

    public DeviceConfig()
    {

    }

    public DeviceConfig(int company, int device, int softwareVersion, int bus)
    {
        this.company = company;
        this.device = device;
        this.softwareVersion = softwareVersion;
        this.bus = bus;
    }
}
