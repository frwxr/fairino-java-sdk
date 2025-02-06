package fairino;
public class RobotInstCmdRecvRoutineThread   extends Thread
{
    public void run() {
        int i = 0;
        for(;i < 100;i++) {
            //当通过继承Thread类的方式实现多线程时，可以直接使用this获取当前执行的线程
            System.out.println(this.getName() + " "  + i);
        }
    }
}
