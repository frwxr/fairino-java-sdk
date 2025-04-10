package fairino;

import sun.security.krb5.internal.crypto.Des;

import java.util.Arrays;
import java.util.List;
import java.util.ArrayList;
import java.util.Scanner;


public class Main {
    public static void main(String[] args) throws InterruptedException {

        Robot robot = new Robot();
        robot.SetReconnectParam(true,100,200);//设置重连次数、间隔
        robot.LoggerInit(FrLogType.DIRECT, FrLogLevel.INFO, "D://log", 10, 10);
        int rtn = robot.RPC("192.168.58.2");
        if(rtn == 0)
        {
            System.out.println("rpc连接 success");
        }
        else
        {
            System.out.println("rpc连接 fail");
            return ;
        }

//*********************************日志功能 begin *******************************
//        String[] ip={""};
//        robot.GetControllerIP(ip);
//        System.out.println("ip: "+ip[0]);
//
//
//        for (int i=0;i<50;i++){
//            robot.GetControllerIP(ip);
//            robot.Sleep(500);
//        }
//        robot.Sleep(500);
//        String version = "";
//        version = robot.GetSDKVersion();
//*********************************日志功能 end *******************************

//        int rec=robot.StartJOG(0, 1, 0, 20, 100, 90);//关节点动
//        System.out.println("StartJOG: "+rec);
        //DescPose coord = new DescPose(0, 0 ,0, 1, 0, 0);
        //robot.FT_SetRCS(0, coord);//16、UDP扩展轴圆弧运动
        //DOReset(robot);//3、D0、A0测试
        //Standard(robot);//1、上下使能、手动自动状态切换  获取版本号
//        Moves(robot);//2、机器人运动测试，逆向运动学计算后做直线运动
        //IOTest(robot);//3、设置控制箱DO、AO停止/暂停后输出是否复位
//        CommonSets(robot);//4、机器人常用设置、标定
        //RobotSafety(robot);//5、机器人安全设置
        //RobotState(robot);//6、机器人状态查询
//        TPD(robot);//7、轨迹复现
//        TrajectoryJ(robot);//7、轨迹文件
//        Program(robot);//8、下载Lua文件
        //GripperTest(robot);//9、夹爪。11、传送带
        //ForceSensor(robot);//10、恒力控制
        //PointTableTest(robot);//12、点位表
//        Welding(robot);//14、焊接
        //SegmentWeld(robot);//14、段焊
        //TestAuxServo(robot);//15、485扩展轴
        //UDPAxis(robot);//16、UDP扩展轴
        //UDPAxisSyncMove(robot);//16、UDP扩展轴圆弧、直线运动
        //EndLuaUpload(robot);//17、Lua开发协议功能
        //TestEndLuaGripper(robot);//17、第二步 Lua夹爪
        //TestEndLuaForce(robot);//17
        //RobotStateTest(robot);//18、状态反馈

        //TestUDPWireSearch(robot);//焊丝寻位
        //TestTractorMove(robot);//测移动物体-小车的启停
        //TestWeldmechineMode(robot);//焊机控制模式切换
        //TestWeave(robot);
        //Stable(robot);

//        TestSingularAvoidSLin(robot);//肩部Line，测试奇异点保护
//        TestSingularAvoidSArc(robot);//肩部ARC，测试奇异点保护
        //TestSingularAvoidWLin(robot);//腕部Line，测试奇异点保护
        //TestSingularAvoidWArc(robot);//腕部ARC，测试奇异点保护
        //TestSingularAvoidEArc(robot);//肘部ARC，测试奇异点保护

        //上传轨迹J文件等功能测试-2024.12.16
////        SetAO(robot, 50);
//        MoveRotGripper(robot, 30, 2.2);
//
//        while (true){
//            ROBOT_STATE_PKG pkg = robot.GetRobotRealTimeState();
////            System.out.println("the robot AO0 "+pkg.cl_analog_output[0]/40.96+", AO1 "+pkg.cl_analog_output[1]/40.96+", tool AO0:  "+pkg.tl_analog_output/40.96);
//            System.out.println("gripper pos "+pkg.gripper_position+"- vel "+pkg.gripper_speed+" - torque "+pkg.gripper_current+" - rotPos "+pkg.gripperRotNum+" - rotvel "+pkg.gripperRotSpeed+" - rotTor "+ pkg.gripperRotTorque);
//            robot.Sleep(200);
//        }

        //奇异点测试
//        TestSingularAvoidSArc(robot);
//        TestSingularAvoidEArc(robot);
//        TestSingularAvoidSLin(robot);
//        TestSingularAvoidWArc(robot);
//        TestSingularAvoidWLin(robot);
        //end 奇异点测试

//        UploadTrajectoryJ(robot);//轨迹J文件删除、上传、运行
//        UploadTrajectoryB(robot);

        //稳定性测试
//        while (true)
//        {
//            MoveRotGripper(robot, 30, 0);
//            MoveRotGripper(robot, 90, 0);
//            UploadTrajectoryJ(robot);
////            MoveRotGripper(robot, 90, 2);
//            robot.Sleep(5000);
//            MoveRotGripper(robot, 30, 0);
//            robot.Sleep(1000);
//            MoveRotGripper(robot, 90, 0);
//            UploadTrajectoryB(robot);//轨迹J文件上传与删除
//            //MoveRotGripper(robot, 90, 0);
//            robot.Sleep(5000);
//            MoveRotGripper(robot, 30, 0);
//            ROBOT_STATE_PKG pkg=robot.GetRobotRealTimeState();
//            System.out.println("the robot AO0:"+ pkg.cl_analog_output[0]+", AO1: "+pkg.cl_analog_output[1]+"tool AO0:"+pkg.tl_analog_output);
//            System.out.println("gripper pos"+pkg.gripper_position+ "- vel "+pkg.gripper_speed+"- torque "+pkg.gripper_current+" - rotPos "+pkg.gripperRotNum+" - rotvel "+pkg.gripperRotSpeed+" - rotTor"+pkg.gripperRotTorque);
//        }
        //end 稳定性测试
        //end-上传轨迹J文件等功能测试-2024.12.16


        //测试FIR滤波
//         FIRPTP(robot, false);//PTP
//         FIRPTP(robot, true);
//
//        FIRLinL(robot, false);//Lin匀速
//        FIRLinL(robot, true);

//        FIRLin(robot,false);//Lin无匀速
//        FIRLin(robot,true);


//        FIRArc(robot, false);//arc匀速
//        FIRArc(robot, true);
        //end 测试FIR滤波


//        TestReWeld(robot);
//        TestTCP(robot);
//        TestTCP6(robot);
//        TestWObj(robot);
//        ExtAxisLaserTracking(robot);

//        TestArcWeldTraceChange(robot);//电弧跟踪控制
//        TestWeaveChange(robot);//摆动渐变测试

//        TestTrajectoryLA(robot);//测试轨迹预处理(轨迹前瞻)
//        CustomCollisionTest(robot);//自定义碰撞检测阈值功能开始

//        reconnect_test(robot);//测试重连
//        testSmooth(robot);

        //测试
//        ROBOT_STATE_PKG pkg=new ROBOT_STATE_PKG();//状态结构体
//        String err1=new String();
//        while (true)
//        {
//            auto start = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
//            rtn = robot.LuaUpload("D://zUP/27.lua");
//            auto end = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
//            cout << "file upload time is : " << (end - start) <<  "    rtn is " << rtn <<endl;

//            long start = System.currentTimeMillis();
//
//
//            rtn = robot.LuaUpload("D://zUP/27.lua",err1);
//            // 获取当前时间（毫秒）
//            long end = System.currentTimeMillis();
//
//            // 输出上传时间和返回值
//            System.out.println("file upload time is : " + (end - start) + "    rtn is " + rtn);
//
//            robot.Sleep(100);
//        }


//        while (true)
//        {
//            DescPose p1Desc=new DescPose(327.604, -104.449, 469.385, -170.284, 2.548, -75.248);
//            JointPos p1Joint=new JointPos(-1.953, -64.856, -112.831, -83.705, 95.191, -16.097);
//
//            DescPose p2Desc=new DescPose(305.959, 260.947, 408.152, 177.966, 0.257, -18.624);
//            JointPos p2Joint=new JointPos(55.276, -76.083, -111.563, -84.379, 89.683, -16.099);
//
//            ExaxisPos exaxisPos=new ExaxisPos(0.0, 0.0, 0.0, 0.0);
//            DescPose offdese=new DescPose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
//            rtn = robot.MoveL(p1Joint, p1Desc, 0, 0, 50, 100, 100, -1, exaxisPos, 0, 0, offdese,0,10);
//            if (rtn != 0)
//            {
//                System.out.println("moveL rtn is: "+ rtn);
//                break;
//            }
//            rtn = robot.MoveL(p2Joint, p2Desc, 0, 0, 50, 100, 100, -1, exaxisPos, 0, 0, offdese,0,10);
//            if (rtn != 0)
//            {
//                System.out.println("moveL rtn is: "+ rtn);
//                break;
//            }
//        }
//
//        int flag=0;
//        while (true)
//        {
//            if(flag<50){
//                flag++;
//            }
//            if(flag==50){
//                robot.ResetAllError();//清除错误
//                robot.Sleep(500);
//                flag=100;
//            }
//
//            pkg=robot.GetRobotRealTimeState();
//            System.out.println("main code is "+pkg.main_code+",  sub code is:  " +pkg.sub_code );
//
//            robot.Sleep(100);
//        }

        robot.CloseRPC();//关闭连接

//        while (true)
//        {
//            ROBOT_STATE_PKG pkg = robot.GetRobotRealTimeState();
//            System.out.println(pkg.jt_cur_pos[0]);
//            robot.Sleep(500);
//            System.out.println("robot Enable stare is " + pkg.rbtEnableState);
//        }
//        JointPos pos = new JointPos();
//        robot.GetActualJointPosDegree(1, pos);
        //System.out.println("J1: " + Double.toString(pos.J1) + "   J2: " + Double.toString(pos.J2) +"    J3: " + Double.toString(pos.J3) +"J4: "  + Double.toString(pos.J4) + "J5: " + Double.toString(pos.J5) +"J6: " + Double.toString(pos.J6));
    }

    //逆运动学测试
    public static void TestInverseKen(Robot robot)
    {
        DescPose dcs1=new DescPose(32.316, -232.029, 1063.415, 90.159, 18.376, 36.575);
        DescPose dcs2=new DescPose(105.25, -170.914, 1076.283, 87.032, 25.94, 54.644);
        DescPose dcs3=new DescPose(79.164, 81.645, 1045.609, 133.691, -73.265, 162.726);
        DescPose dcs4=new DescPose(298.779, -104.112, 298.242, 179.631, -0.628, -166.481);
        JointPos inverseRtn = new JointPos(){};

//        robot.GetInverseKin(0, dcs1, -1, inverseRtn);
//        System.out.println("dcs1 getinverse rtn is "+inverseRtn.J1+","+inverseRtn.J2+","+inverseRtn.J3+","+
//                inverseRtn.J4+","+inverseRtn.J5+","+inverseRtn.J6);
//        robot.GetInverseKin(0, dcs2, -1, inverseRtn);
//        System.out.println("dcs2 getinverse rtn is "+inverseRtn.J1+","+inverseRtn.J2+","+inverseRtn.J3+","+
//                inverseRtn.J4+","+inverseRtn.J5+","+inverseRtn.J6);
//        robot.GetInverseKin(0, dcs3, -1, inverseRtn);
//        System.out.println("dcs3 getinverse rtn is "+inverseRtn.J1+","+inverseRtn.J2+","+inverseRtn.J3+","+
//                inverseRtn.J4+","+inverseRtn.J5+","+inverseRtn.J6);
//        robot.GetInverseKin(0, dcs4, -1, inverseRtn);
//        System.out.println("dcs4 getinverse rtn is "+inverseRtn.J1+","+inverseRtn.J2+","+inverseRtn.J3+","+
//                inverseRtn.J4+","+inverseRtn.J5+","+inverseRtn.J6);

        JointPos jpos1=new JointPos(56.999, -59.002, 56.996, -96.552, 60.392, -90.005);
        DescPose forwordResult = new DescPose(){};
        robot.GetForwardKin(jpos1, forwordResult);
        System.out.println("jpos1 forwordResult rtn is "+forwordResult.tran.x+","+forwordResult.tran.y+","+forwordResult.tran.z+","+
                forwordResult.rpy.rx+","+forwordResult.rpy.ry+","+forwordResult.rpy.rz);
    }

    public static  void reconnect_test(Robot robot)//测试重连
    {
        int rtn=-1;
        DescPose p1Desc = new DescPose(-423.723, -145.123, 546.173, -161.851, -29.236, 150.755);
        JointPos p1Joint = new JointPos(6.001, -103.515, 102.462, -122.922, -90.77, -59.761);

        DescPose p2Desc = new DescPose(-458.433, -678.096, 290.075, -176.815, -6.699, -161.689);
        JointPos p2Joint = new JointPos(48.905, -43.486, 53.364, -107.265, -90.655, -59.635);


        ExaxisPos exaxisPos = new ExaxisPos(0, 0, 0, 0);
        DescPose offdese = new DescPose(0, 0, 0, 0, 0, 0);
        int i=0;
        while(i<1000){
            rtn=-1;
            boolean flag=robot.isConnected();//连接是否断开
            System.out.println("连接是否断开: "+flag);
            robot.Sleep(200);
//            while (flag==false)//没有断开
//            {
//                rtn=robot.MoveL(p1Joint, p1Desc, 0, 0, 100, 100, 100, -1, exaxisPos, 0, 0, offdese, 0, 10);
//                rtn=robot.MoveL(p2Joint, p2Desc, 0, 0, 100, 100, 100, -1, exaxisPos, 0, 0, offdese, 0, 10);
//                System.out.println("指令执行结果:"+rtn);
//            }
//            System.out.println("连接断开");
//            i++;
        }
    }

    private static void TestTrajectoryLA(Robot robot)
    {

        int rtn = 0;

        String nameA = "/fruser/traj/A.txt";
        String nameB = "/fruser/traj/B.txt";

        rtn = robot.LoadTrajectoryLA(nameA, 2, 0.0, 0, 1.0, 100.0, 200.0, 1000.0);//B样条
//        rtn = robot.LoadTrajectoryLA(nameA, 1, 2, 0, 2, 100.0, 200.0, 1000.0);

//        rtn = robot.LoadTrajectoryLA(nameB, 0, 0, 0, 1, 100.0, 100.0, 1000.0);    // 直线拟合
        System.out.println("LoadTrajectoryLA rtn is :"+ rtn);

        DescPose startPos = new DescPose(0, 0, 0, 0, 0, 0);
        robot.GetTrajectoryStartPose(nameA, startPos);

        // MoveCart方法调用，假设参数类型和顺序与C++版本一致
        robot.MoveCart(startPos, 1, 0, (float)100.0, (float)100.0, (float)100.0, -1, -1);

        rtn = robot.MoveTrajectoryLA();
        System.out.println("MoveTrajectoryLA rtn is: "+ rtn);
    }

    //自定义碰撞检测阈值功能开始
    public static void CustomCollisionTest(Robot robot)
    {
        int[] safety = { 5,5,5,5,5,5 };
        robot.SetCollisionStrategy(3, 1000, 150, 250, safety);
        double[] jointDetectionThreshould= { 0.3, 0.3, 0.3, 0.3, 0.3, 0.3};
        double[] tcpDetectionThreshould = { 60,60,60,60,60,60 };
        int rtn = robot.CustomCollisionDetectionStart(3, jointDetectionThreshould, tcpDetectionThreshould, 0);

        DescPose p1Desc=new DescPose(228.879, -503.594, 453.984, -175.580, 8.293, 171.267);
        JointPos p1Joint=new JointPos(102.700, -85.333, 90.518, -102.365, -83.932, 22.134);

        DescPose p2Desc=new DescPose(-333.302, -435.580, 449.866, -174.997, 2.017, 109.815);
        JointPos p2Joint=new JointPos(41.862, -85.333, 90.526, -100.587, -90.014, 22.135);

        ExaxisPos exaxisPos=new ExaxisPos(0.0, 0.0, 0.0, 0.0);
        DescPose offdese=new DescPose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        for(int i=0;i<10;++i) {
            robot.MoveL(p1Joint, p1Desc, 0, 0, 100, 100, 100, -1, exaxisPos, 0, 0, offdese, 0, 10);
            robot.MoveL(p2Joint, p2Desc, 0, 0, 100, 100, 100, -1, exaxisPos, 0, 0, offdese, 0, 10);
        }
        rtn = robot.CustomCollisionDetectionEnd();
    }

    private static void TestReWeld(Robot robot)
    {
        int rtn = -1;
        rtn = robot.WeldingSetCheckArcInterruptionParam(1, 200);
        System.out.println("WeldingSetCheckArcInterruptionParam: "+rtn);
        rtn = robot.WeldingSetReWeldAfterBreakOffParam(1, 5.7, 98.2, 0);
        System.out.println("WeldingSetReWeldAfterBreakOffParam: "+rtn);
        int enable = 0;
        double length = 0;
        double velocity = 0;
        int moveType = 0;
        int checkEnable = 0;
        int arcInterruptTimeLength = 0;
        List<Integer> rtnArray = new ArrayList<Integer>() {};
        List<Number> rtnArrayWeld = new ArrayList<Number>() {};
        rtnArray = robot.WeldingGetCheckArcInterruptionParam();
        checkEnable=rtnArray.get(1);
        arcInterruptTimeLength=rtnArray.get(2);
        System.out.println("WeldingGetCheckArcInterruptionParam  checkEnable:"+checkEnable +", arcInterruptTimeLength : "+ arcInterruptTimeLength);
        rtnArrayWeld = robot.WeldingGetReWeldAfterBreakOffParam();
        enable=(int) rtnArrayWeld.get(1);
        length=(double) rtnArrayWeld.get(2);
        velocity=(double) rtnArrayWeld.get(3);
        moveType=(int) rtnArrayWeld.get(4);
        System.out.println("WeldingGetReWeldAfterBreakOffParam :"+ enable +",length: "+length+",velocity :"+velocity+",moveType :"+moveType);
        //焊接中断恢复
        robot.ProgramLoad("/fruser/test.lua");
        robot.ProgramRun();

        robot.Sleep(5000);

        while (true)
        {
            ROBOT_STATE_PKG pkg=new ROBOT_STATE_PKG();
            pkg=robot.GetRobotRealTimeState();
            System.out.println("welding breakoff state is "+pkg.weldingBreakOffstate.breakOffState);
            if (pkg.weldingBreakOffstate.breakOffState == 1)
            {
                System.out.println("welding breakoff !");
                robot.Sleep(2000);
                rtn = robot.WeldingStartReWeldAfterBreakOff();
                System.out.println("WeldingStartReWeldAfterBreakOff: "+rtn);
                break;
            }
            robot.Sleep(100);
        }
    }

    public static void TestTCP(Robot robot)
    {
        DescPose p1Desc=new DescPose(-394.073, -276.405, 399.451, -133.692, 7.657, -139.047);
        JointPos p1Joint=new JointPos(15.234, -88.178, 96.583, -68.314, -52.303, -122.926);

        DescPose p2Desc=new DescPose( -187.141, -444.908, 432.425, 148.662, 15.483, -90.637);
        JointPos p2Joint=new JointPos(61.796, -91.959, 101.693, -102.417, -124.511, -122.767);

        DescPose p3Desc=new DescPose(-368.695, -485.023, 426.640, -162.588, 31.433, -97.036);
        JointPos p3Joint=new JointPos(43.896, -64.590, 60.087, -50.269, -94.663, -122.652);

        DescPose p4Desc=new DescPose(-291.069, -376.976, 467.560, -179.272, -2.326, -107.757);
        JointPos p4Joint=new JointPos(39.559, -94.731, 96.307, -93.141, -88.131, -122.673);

        DescPose p5Desc=new DescPose(-284.140, -488.041, 478.579, 179.785, -1.396, -98.030);
        JointPos p5Joint=new JointPos(49.283, -82.423, 81.993, -90.861, -89.427, -122.678);

        DescPose p6Desc=new DescPose(-296.307, -385.991, 484.492, -178.637, -0.057, -107.059);
        JointPos p6Joint=new JointPos(40.141, -92.742, 91.410, -87.978, -88.824, -122.808);

        ExaxisPos exaxisPos=new ExaxisPos(0, 0, 0, 0);
        DescPose offdese=new DescPose(0, 0, 0, 0, 0, 0);

        JointPos[] posJ ={p1Joint , p2Joint , p3Joint , p4Joint , p5Joint , p6Joint};
        DescPose coordRtn =new DescPose();
        int rtn = robot.ComputeToolCoordWithPoints(0, posJ, coordRtn);
        System.out.println("ComputeToolCoordWithPoints :"+rtn+",coord is:" +rtn+","+coordRtn.tran.x+","+coordRtn.tran.y+","+coordRtn.tran.z+","+coordRtn.rpy.rx+","+ coordRtn.rpy.ry+","+ coordRtn.rpy.rz);


        robot.MoveJ(p1Joint, p1Desc, 0, 0, 100, 100, 100, exaxisPos, -1, 0, offdese);
        robot.SetTcp4RefPoint(1);
        robot.MoveJ(p2Joint, p2Desc, 0, 0, 100, 100, 100, exaxisPos, -1, 0, offdese);
        robot.SetTcp4RefPoint(2);
        robot.MoveJ(p3Joint, p3Desc, 0, 0, 100, 100, 100, exaxisPos, -1, 0, offdese);
        robot.SetTcp4RefPoint(3);
        robot.MoveJ(p4Joint, p4Desc, 0, 0, 100, 100, 100, exaxisPos, -1, 0, offdese);
        robot.SetTcp4RefPoint(4);
        robot.ComputeTcp4(coordRtn);
        System.out.println("ComputeTcp4 : "+ rtn+", coord is: "+coordRtn.tran.x+","+coordRtn.tran.y+","+coordRtn.tran.z+","+coordRtn.rpy.rx+","+ coordRtn.rpy.ry+","+ coordRtn.rpy.rz);
        //robot->MoveJ(&p5Joint, &p5Desc, 0, 0, 100, 100, 100, &exaxisPos, -1, 0, &offdese);
        //robot->MoveJ(&p6Joint, &p6Desc, 0, 0, 100, 100, 100, &exaxisPos, -1, 0, &offdese);

    }

    //电弧跟踪控制,测试用例
    public static void TestArcWeldTraceChange(Robot robot)
    {
        DescPose p1Desc=new DescPose(-72.912, -587.664, 31.849, 43.283, -6.731, 15.068);
        JointPos p1Joint=new JointPos(74.620, -80.903, 94.608, -109.882, -90.436, -13.432);

        DescPose p2Desc=new DescPose( -104.915, -483.712, -25.231, 42.228, -6.572, 18.433);
        JointPos p2Joint=new JointPos(66.431, -92.875, 116.362, -120.516, -88.627, -24.731);

        DescPose p3Desc=new DescPose(-242.834, -498.697, -23.681, 46.576, -5.286, 8.318);
        JointPos p3Joint=new JointPos( 57.153, -82.046, 104.060, -116.659, -92.478, -24.735 );

        ExaxisPos exaxisPos=new ExaxisPos(0.0, 0.0, 0.0, 0.0);
        DescPose offdese=new DescPose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        robot.WeldingSetVoltage(1, 19, 0, 0);
        robot.WeldingSetCurrent(1, 190, 0, 0);
        robot.MoveJ(p1Joint, p1Desc, 1, 1, 100, 100, 100, exaxisPos, -1, 0, offdese);
        robot.MoveL(p2Joint, p2Desc, 1, 1, 100, 100, 50, -1, exaxisPos, 0, 0, offdese,0,10);
        robot.ARCStart(1, 0, 10000);
        robot.ArcWeldTraceControl(1, 0, 1, 0.06, 5, 5, 60, 1, 0.06, 5, 5, 60, 0, 0, 4, 1, 10, 2, 2);
        robot.WeaveStart(0);
        robot.MoveL(p3Joint, p3Desc, 1, 1, 100, 100, 1, -1, exaxisPos, 0, 0, offdese,0,10);
        robot.WeaveEnd(0);
        robot.ArcWeldTraceControl(0, 0, 1, 0.06, 5, 5, 60, 1, 0.06, 5, 5, 60, 0, 0, 4, 1, 10, 2, 2);
        robot.ARCEnd(1, 0, 10000);
    }

    //摆动渐变-测试用例
    public static void TestWeaveChange(Robot robot) {
        DescPose p1Desc = new DescPose(-72.912, -587.664, 31.849, 43.283, -6.731, 15.068);
        JointPos p1Joint = new JointPos(74.620, -80.903, 94.608, -109.882, -90.436, -13.432);

        DescPose p2Desc = new DescPose(-104.915, -483.712, -25.231, 42.228, -6.572, 18.433);
        JointPos p2Joint = new JointPos(66.431, -92.875, 116.362, -120.516, -88.627, -24.731);

        DescPose p3Desc = new DescPose(-240.651, -483.840, -7.161, 46.577, -5.286, 8.318);
        JointPos p3Joint = new JointPos(56.457, -84.796, 104.618, -114.497, -92.422, -25.430);

        ExaxisPos exaxisPos = new ExaxisPos(0.0, 0.0, 0.0, 0.0);
        DescPose offdese = new DescPose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        robot.WeldingSetVoltage(1, 19, 0, 0);
        robot.WeldingSetCurrent(1, 190, 0, 0);
        robot.MoveJ(p1Joint, p1Desc, 1, 1, 100, 100, 100, exaxisPos, -1, 0, offdese);
        robot.MoveL(p2Joint, p2Desc, 1, 1, 100, 100, 50, -1, exaxisPos, 0, 0, offdese, 0, 10);
        robot.ARCStart(1, 0, 10000);
        robot.ArcWeldTraceControl(1, 0, 1, 0.06, 5, 5, 60, 1, 0.06, 5, 5, 80, 0, 0, 4, 1, 10, 0, 0);
        robot.WeaveStart(0);
        robot.WeaveChangeStart(1);
        robot.MoveL(p3Joint, p3Desc, 1, 1, 100, 100, 1, -1, exaxisPos, 0, 0, offdese, 0, 10);
        robot.WeaveChangeEnd();
        robot.WeaveEnd(0);
        robot.ArcWeldTraceControl(0, 0, 1, 0.06, 5, 5, 60, 1, 0.06, 5, 5, 80, 0, 0, 4, 1, 10, 0, 0);
        robot.ARCEnd(1, 0, 10000);
    }

    public static void TestTCP6(Robot robot)
    {
        DescPose p1Desc=new DescPose(-394.073, -276.405, 399.451, -133.692, 7.657, -139.047);
        JointPos p1Joint=new JointPos(15.234, -88.178, 96.583, -68.314, -52.303, -122.926);

        DescPose p2Desc=new DescPose(-187.141, -444.908, 432.425, 148.662, 15.483, -90.637);
        JointPos p2Joint=new JointPos(61.796, -91.959, 101.693, -102.417, -124.511, -122.767);

        DescPose p3Desc=new DescPose(-368.695, -485.023, 426.640, -162.588, 31.433, -97.036);
        JointPos p3Joint=new JointPos(43.896, -64.590, 60.087, -50.269, -94.663, -122.652);

        DescPose p4Desc=new DescPose(-291.069, -376.976, 467.560, -179.272, -2.326, -107.757);
        JointPos p4Joint=new JointPos(39.559, -94.731, 96.307, -93.141, -88.131, -122.673);

        DescPose p5Desc=new DescPose(-284.140, -488.041, 478.579, 179.785, -1.396, -98.030);
        JointPos p5Joint=new JointPos(49.283, -82.423, 81.993, -90.861, -89.427, -122.678);

        DescPose p6Desc=new DescPose(-296.307, -385.991, 484.492, -178.637, -0.057, -107.059);
        JointPos p6Joint=new JointPos(40.141, -92.742, 91.410, -87.978, -88.824, -122.808);

        ExaxisPos exaxisPos=new ExaxisPos(0, 0, 0, 0);
        DescPose offdese=new DescPose(0, 0, 0, 0, 0, 0);

        JointPos[] posJ = { p1Joint , p2Joint , p3Joint , p4Joint , p5Joint , p6Joint };
        DescPose coordRtn = new DescPose();
        int rtn = robot.ComputeToolCoordWithPoints(1, posJ, coordRtn);
        System.out.println("ComputeToolCoordWithPoints: "+rtn+ ", coord is :"+ coordRtn.tran.x+","+coordRtn.tran.y+","+coordRtn.tran.z+","+ coordRtn.rpy.rx+","+ coordRtn.rpy.ry+","+coordRtn.rpy.rz);


        robot.MoveJ(p1Joint, p1Desc, 0, 0, 100, 100, 100, exaxisPos, -1, 0, offdese);
        robot.SetToolPoint(1);
        robot.MoveJ(p2Joint, p2Desc, 0, 0, 100, 100, 100, exaxisPos, -1, 0, offdese);
        robot.SetToolPoint(2);
        robot.MoveJ(p3Joint, p3Desc, 0, 0, 100, 100, 100, exaxisPos, -1, 0, offdese);
        robot.SetToolPoint(3);
        robot.MoveJ(p4Joint, p4Desc, 0, 0, 100, 100, 100, exaxisPos, -1, 0, offdese);
        robot.SetToolPoint(4);
        robot.MoveJ(p5Joint, p5Desc, 0, 0, 100, 100, 100, exaxisPos, -1, 0, offdese);
        robot.SetToolPoint(5);
        robot.MoveJ(p6Joint, p6Desc, 0, 0, 100, 100, 100, exaxisPos, -1, 0, offdese);
        robot.SetToolPoint(6);
        robot.ComputeTool(coordRtn);
        System.out.println("ComputeTool :"+rtn+",coord is :"+coordRtn.tran.x+","+ coordRtn.tran.y+","+ coordRtn.tran.z+","+ coordRtn.rpy.rx+","+ coordRtn.rpy.ry+","+ coordRtn.rpy.rz);
    }

    public static void TestWObj(Robot robot)
    {
        DescPose p1Desc=new DescPose(-275.046, -293.122, 28.747, 174.533, -1.301, -112.101);
        JointPos p1Joint=new JointPos(35.207, -95.350, 133.703, -132.403, -93.897, -122.768);

        DescPose p2Desc=new DescPose(-280.339, -396.053, 29.762, 174.621, -3.448, -102.901);
        JointPos p2Joint=new JointPos(44.304, -85.020, 123.889, -134.679, -92.658, -122.768);

        DescPose p3Desc=new DescPose(-270.597, -290.603, 83.034, 179.314, 0.808, -114.171);
        JointPos p3Joint=new JointPos(32.975, -99.175, 125.966, -116.484, -91.014, -122.857);



        ExaxisPos exaxisPos=new ExaxisPos(0, 0, 0, 0);
        DescPose offdese=new DescPose(0, 0, 0, 0, 0, 0);

        DescPose[] posTCP = { p1Desc , p2Desc , p3Desc };
        DescPose coordRtn = new DescPose();
        int rtn = robot.ComputeWObjCoordWithPoints(0, posTCP, 0, coordRtn);
        System.out.println("ComputeToolCoordWithPoints :"+rtn+",coord is :"+coordRtn.tran.x+","+coordRtn.tran.y+","+ coordRtn.tran.z+","+ coordRtn.rpy.rx+","+coordRtn.rpy.ry+","+ coordRtn.rpy.rz);


        robot.MoveJ(p1Joint, p1Desc, 1, 0, 100, 100, 100, exaxisPos, -1, 0, offdese);
        robot.SetWObjCoordPoint(1);
        robot.MoveJ(p2Joint, p2Desc, 1, 0, 100, 100, 100, exaxisPos, -1, 0, offdese);
        robot.SetWObjCoordPoint(2);
        robot.MoveJ(p3Joint, p3Desc, 1, 0, 100, 100, 100, exaxisPos, -1, 0, offdese);
        robot.SetWObjCoordPoint(3);
        robot.ComputeWObjCoord(0, 0, coordRtn);
        System.out.println("ComputeTool :"+rtn+  "coord is :"+coordRtn.tran.x+","+ coordRtn.tran.y+","+ coordRtn.tran.z+","+coordRtn.rpy.rx+","+ coordRtn.rpy.ry+","+ coordRtn.rpy.rz);
        //robot->MoveJ(&p5Joint, &p5Desc, 0, 0, 100, 100, 100, &exaxisPos, -1, 0, &offdese);
        //robot->MoveJ(&p6Joint, &p6Desc, 0, 0, 100, 100, 100, &exaxisPos, -1, 0, &offdese);


    }

    public static void ExtAxisLaserTracking(Robot robot)
    {
        DescPose p1Desc=new DescPose(381.070, -177.767, 227.851, 20.031, -2.455, -111.479);
        JointPos p1Joint=new JointPos(8.383, -44.801, -111.050, -97.707, 78.144, 27.709);

        DescPose p2Desc=new DescPose(381.077, -177.762, 217.865, 20.014, -0.131, -110.631);
        JointPos p2Joint=new JointPos(1.792, -44.574, -113.176, -93.687, 82.384, 21.154);

        DescPose p3Desc=new DescPose(381.070, -177.767, 227.851, 20.031, -2.455, -111.479);
        JointPos p3Joint=new JointPos(8.383, -44.801, -111.050, -97.707, 78.144, 27.709);

        ExaxisPos exaxisPos=new ExaxisPos(0.0, 0.0, 0.0, 0.0);
        DescPose offdese=new DescPose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

        ExaxisPos exaxisPosStart=new ExaxisPos(0.0, 0.0, 0.0, 0.0);
        robot.MoveJ(p1Joint, p1Desc, 8, 0, 100, 100, 100, exaxisPos, -1, 0, offdese);
        robot.ExtAxisMove(exaxisPosStart, 50.0);
        robot.MoveL(p2Joint, p2Desc, 8, 0, 100, 100, 100, -1, exaxisPos, 0, 0, offdese,0,10);
        robot.LaserSensorRecord(4, 1, 10, 2, 35, 0.1, 100);
        ExaxisPos exaxisPosTarget=new ExaxisPos(0.000, 400.015, 0.000, 0.000);
        robot.ExtAxisMove(exaxisPosTarget, 10.0);
        robot.LaserSensorRecord(0, 1, 10, 2, 35, 0.1, 100);
        robot.MoveJ(p3Joint, p3Desc, 8, 0, 100, 100, 100, exaxisPos, -1, 0, offdese);
        robot.ExtAxisMove(exaxisPosStart, 50.0);
    }

    private static void TestWeave(Robot robot)
    {
        DescPose  desc_p1, desc_p2;

        desc_p1 = new DescPose(-299.979,-399.974,74.979,0.009,0.001,-41.530);
        desc_p2 = new DescPose(-49.985,-399.956,74.978,0.009,0.005,-41.530);

        JointPos j1 = new JointPos(41.476,-77.300,118.714,-131.405,-90.002,-51.993);
        JointPos j2 = new JointPos(68.366,-89.562,133.018,-133.446,-90.002,-25.105);

        ExaxisPos epos = new ExaxisPos();
        DescPose offset_pos = new DescPose();

        robot.WeaveSetPara(0,4,2.000000,0,10.000000,0.000000,0.000000,0,0,0,0,0,60.000000);
        robot.MoveJ(j1, desc_p1,13, 0, 100, 100, 100, epos, -1, 0, offset_pos);
        robot.WeaveStart(0);
        robot.MoveL(j2, desc_p2,13, 0, 10, 100, 100, -1, epos, 0, 0, offset_pos, 0, 100);
        robot.WeaveEnd(0);

        robot.WeaveSetPara(0,0,1.000000,0,10.000000,0.000000,0.000000,0,0,0,0,0,30.000000);
        robot.MoveJ(j1, desc_p1,13, 0, 100, 100, 100, epos, -1, 0, offset_pos);
        robot.WeaveStart(0);
        robot.MoveL(j2, desc_p2,13, 0, 10, 100, 100, -1, epos, 0, 0, offset_pos, 0, 100);
        robot.WeaveEnd(0);
    }


    private static void TrajectoryJ(Robot robot)
    {
//********************************轨迹文件读取运动 begin *********************************88
        ForceTorque tor = new ForceTorque(10.0,10.0, 10.0, 10.0, 10.0, 10.0);
        robot.SetTrajectoryJForceTorque(tor);

        robot.SetTrajectoryJForceFx(2.0);
        robot.SetTrajectoryJForceFy(2.0);
        robot.SetTrajectoryJForceFz(2.0);
        robot.SetTrajectoryJTorqueTx(2.0);
        robot.SetTrajectoryJTorqueTy(2.0);
        robot.SetTrajectoryJTorqueTz(2.0);


        robot.LoadTrajectoryJ("/fruser/traj/test1011002.txt", 20, 1);
        DescPose startPos = new DescPose();
        robot.GetTrajectoryStartPose("/fruser/traj/test1011002.txt", startPos);
        robot.MoveCart(startPos, 0, 0, 40, 100.0, 100.0, -1.0, -1);

        ROBOT_STATE_PKG pkg = robot.GetRobotRealTimeState();
        System.out.println("Trajectory point num is " + pkg.trajectory_pnum);
        robot.SetTrajectoryJSpeed(40);
        robot.MoveTrajectoryJ();
//********************************轨迹文件读取运动 end *********************************88
    }


    private static void PointTableTest(Robot robot)
    {
//        robot.PointTableUpLoad("D://zUP/point_table_test1.db");//点位表上传
//        robot.PointTableDownLoad("point_table_test1.db", "D://zUP/");//点位表下载
//        String errStr = "";
//        robot.PointTableSwitch("point_table_test1.db", errStr);//切换点位表
//        //点位表更新LUA程序
//        robot.PointTableUpdateLua("point_table_test2.db", "1010Test.lua", errStr);
    }




    private static void RobotStateTest(Robot robot)
    {
        //robot.RobotEnable(0);//下使能
        //robot.Sleep(2000);
        ROBOT_STATE_PKG pkg = robot.GetRobotRealTimeState();//获取机械臂实时状态
        System.out.println("Tool pos " + pkg.tl_cur_pos[0] + "  " + pkg.tl_cur_pos[1] + "  " + pkg.tl_cur_pos[2] + "  " + pkg.tl_cur_pos[3] + "  " + pkg.tl_cur_pos[4] + "  " + pkg.tl_cur_pos[5] + "  ");
        System.out.println("Joint pos " + pkg.jt_cur_pos[0] + "  " + pkg.jt_cur_pos[1] + "  " + pkg.jt_cur_pos[2] + "  " + pkg.jt_cur_pos[3] + "  " + pkg.jt_cur_pos[4] + "  " + pkg.jt_cur_pos[5] + "  ");
        System.out.println("Flange pos " + pkg.flange_cur_pos[0] + "  " + pkg.flange_cur_pos[1] + "  " + pkg.flange_cur_pos[2] + "  " + pkg.flange_cur_pos[3] + "  " + pkg.flange_cur_pos[4] + "  " + pkg.flange_cur_pos[5] + "  ");
        System.out.println("Tool " + pkg.tool);//工具号
        System.out.println("WObj " + pkg.user);//工件号
        System.out.println("Mode " + pkg.robot_mode);//机器人模式
        System.out.println("Enable " + pkg.rbtEnableState);//使能状态
        System.out.println("CI " + pkg.cl_dgt_input_h);//数字输入
        System.out.println("AI " + Arrays.toString(pkg.cl_analog_input));//控制箱模拟量输入
    }

    private static void GripperTest(Robot robot)
    {
//***********************************夹爪配置获取 begin ***********************************
        int company = 3;
        int device = 0;
        int softversion = 0;
        int bus = 1;
        int deviceID = -1;

        DeviceConfig gripperConfig = new DeviceConfig(company, device, softversion, bus);

        robot.SetGripperConfig(gripperConfig);
        robot.Sleep(1000);

        DeviceConfig getConfig = new DeviceConfig();
        robot.GetGripperConfig(getConfig);
        System.out.println("gripper 厂商:" + getConfig.company + " , 类型: " + getConfig.device + " , 软件版本: " + getConfig.softwareVersion);
//***********************************夹爪配置获取 end ***********************************

//***********************************夹爪激活、运动、状态获取 begin ***********************************
//        int index = 1;
//        byte act = 0;
//        int max_time = 30000;
//        byte block = 0;
//        int status = -1, fault = -1;
//        int rtn = -1;
//
//        rtn = robot.ActGripper(index, act);//激活夹爪
//        System.out.println("ActGripper rtn : " + rtn);
//        //robot.Sleep(1000);
//        act = 1;
//        rtn = robot.ActGripper(index, act);
//        System.out.println("ActGripper rtn : " + rtn);
//        //robot.Sleep(4000);
//
//        rtn = robot.MoveGripper(index, 80, 20, 50, max_time, block);//移动夹爪
//        System.out.println("MoveGripper rtn : " + rtn);
//        robot.Sleep(2000);
//        robot.MoveGripper(index, 20, 20, 50, max_time, block);//移动夹爪
//
//        robot.Sleep(4000);
////        ROBOT_STATE_PKG pkg = robot.GetRobotRealTimeState();//获取状态
//        List<Integer> rtnArray = new ArrayList<Integer>() {};
//        rtnArray=robot.GetGripperMotionDone();
//        System.out.println("gripper motion done : " + rtnArray.get(2) +", " + rtnArray.get(1));
////        System.out.println("gripper motion done : " + pkg.gripper_motiondone +", " + pkg.gripper_fault);
//***********************************夹爪激活、运动、状态获取 end ***********************************

//***********************************正常进行抓取点和撤退点的计算 begin ***********************************
//        DescPose desc_pos1, desc_pos2;
//        desc_pos1 = new DescPose(-228.943, -584.228, 461.958,179.16, 5.559, 125.643);
//        desc_pos2 = new DescPose(-3.05, -627.474 , 461.967, 179.152, 5.565, 146.765);
//        robot.ComputePrePick(desc_pos1, 10, 0, desc_pos2);
//        System.out.println("ComputePrePick: " + desc_pos2.toString());
//
//        desc_pos2.tran.x = 0;
//        robot.ComputePostPick(desc_pos1, 10, 0, desc_pos2);
//        System.out.println("ComputePostPick: " + desc_pos2.toString());
//***********************************正常进行抓取点和撤退点的计算 end ***********************************

//        //传送带参考点
//***********************************传送带 begin ***********************************
        int rtn = -1;
        rtn = robot.ConveyorPointIORecord();//记录IO切入点
        System.out.println("ConveyorPointIORecord: rtn " + rtn);

        rtn = robot.ConveyorPointARecord();//记录A点
        System.out.println("ConveyorPointARecord: rtn " + rtn);

        rtn = robot.ConveyorRefPointRecord();//记录参考点
        System.out.println("ConveyorRefPointRecord: rtn  " + rtn);

        rtn = robot.ConveyorPointBRecord();//记录B点
        System.out.println("ConveyorPointBRecord: rtn " + rtn);

        //配置传送带
        robot.ConveyorSetParam(1, 10000, 2.0, 1, 1, 20);
        System.out.println("ConveyorSetParam: rtn  " + rtn);


//        传送带跟踪抓取
        DescPose pos1 = new DescPose(-351.549,87.914,354.176,-179.679,-0.134,2.468);
        DescPose pos2 = new DescPose(-351.203,-213.393,351.054,-179.932,-0.508,2.472);


        Object[] cmp = {0.0, 0.0, 0.0};
        rtn = robot.ConveyorCatchPointComp(cmp);//设置传动带抓取点补偿
        if(rtn != 0)
        {
            return;
        }
        System.out.println("ConveyorCatchPointComp: rtn  " + rtn);

        rtn = robot.MoveCart(pos1, 1, 0, 30.0, 180.0, 100.0, -1.0, -1);
        System.out.println("MoveCart: rtn  " + rtn);

        rtn = robot.ConveyorIODetect(10000);//传送带工件IO检测
        System.out.println("ConveyorIODetect: rtn   " + rtn);

        robot.ConveyorGetTrackData(1);//配置传送带跟踪抓取
        rtn = robot.ConveyorTrackStart(1);//跟踪开始
        System.out.println("ConveyorTrackStart: rtn  " + rtn);

        rtn = robot.ConveyorTrackMoveL("cvrCatchPoint", 1, 0, 100.0, 0.0, 100.0, -1.0);
        System.out.println("ConveyorTrackMoveL: rtn  " + rtn);

        rtn = robot.MoveGripper(1, 60, 60, 30, 30000, 0,0,0,0,0);
        System.out.println("MoveGripper: rtn  {rtn}");

        rtn = robot.ConveyorTrackMoveL("cvrRaisePoint", 1, 0, 100.0, 0.0, 100.0, -1.0);
        System.out.println("ConveyorTrackMoveL: rtn   " + rtn);

        rtn = robot.ConveyorTrackEnd();//传送带跟踪停止
        System.out.println("ConveyorTrackEnd: rtn  " + rtn);

        rtn = robot.MoveCart(pos2, 1, 0, 30.0, 180.0, 100.0, -1.0, -1);
        System.out.println("MoveCart: rtn  " + rtn);

        rtn = robot.MoveGripper(1, 100, 60, 30, 30000, 0,0,0,0,0);
        System.out.println("MoveGripper: rtn  " + rtn);
//
//***********************************传送带 end ***********************************

        //末端传感器配置参数、获取参数、激活和写入寄存器参数
//***********************************传感器配置与参数获取 begin ***********************************
//        DeviceConfig axleSensorConfig = new DeviceConfig(18, 0, 0, 1);
//        robot.AxleSensorConfig(axleSensorConfig);
//
//        DeviceConfig getConfig = new DeviceConfig();
//        robot.AxleSensorConfigGet(getConfig);
//        System.out.println("company is " + getConfig.company + ",  type is " + getConfig.device);
//
//        robot.AxleSensorActivate(1);
//
//        robot.AxleSensorRegWrite(1, 4, 6, 1, 0, 0, 0);
//***********************************传感器配置与参数获取 end ***********************************

    }


    private static void Stable(Robot robot)
    {
        int count = 0;
        while (true)
        {
            count ++;
            DescPose  desc_p1, desc_p2, desc_p3;
            desc_p1 = new DescPose(0, 0, 0, 0, 0, 0);
            desc_p2 = new DescPose(0, 0, 0, 0, 0, 0);
            desc_p3 = new DescPose(0, 0, 0, 0, 0, 0);

            desc_p1.tran.x = -104.846;
            desc_p1.tran.y = 309.573;
            desc_p1.tran.z = 336.647;
            desc_p1.rpy.rx = 179.681;
            desc_p1.rpy.ry = -0.419;
            desc_p1.rpy.rz = -92.692;

            desc_p2.tran.x = -318.287;
            desc_p2.tran.y = 158.502;
            desc_p2.tran.z = 346.184;
            desc_p2.rpy.rx = 179.602;
            desc_p2.rpy.ry = 1.081;
            desc_p2.rpy.rz = -46.342;

            desc_p3.tran.x = -67.635;
            desc_p3.tran.y = -387.487;
            desc_p3.tran.z = 283.656;
            desc_p3.rpy.rx = -175.639;
            desc_p3.rpy.ry = 0.886;
            desc_p3.rpy.rz = 93.416;

            JointPos j1 = new JointPos();
            JointPos j2 = new JointPos();
            JointPos j3 = new JointPos();


            robot.GetInverseKin(0, desc_p1, -1, j1);
            robot.GetInverseKin(0, desc_p2, -1, j2);
            robot.GetInverseKin(0, desc_p3, -1, j3);

            ExaxisPos epos = new ExaxisPos();
            DescPose offset_pos = new DescPose();

            robot.MoveJ(j1, desc_p1,0, 0, 100, 100, 100, epos, -1, 0, offset_pos);
            ROBOT_STATE_PKG pkg = robot.GetRobotRealTimeState();
            System.out.println("cur pos is " + Arrays.toString(pkg.jt_cur_pos) + "  motiondone state " + pkg.motion_done);
            robot.MoveJ(j2, desc_p2,0, 0, 100, 100, 100, epos, -1, 0, offset_pos);
            pkg = robot.GetRobotRealTimeState();
            System.out.println("cur pos is " + Arrays.toString(pkg.jt_cur_pos) + "  motiondone state " + pkg.motion_done + "  count is   " + count);
        }
    }


    private static void Welding(Robot robot)
    {
//        DescPose  desc_p1, desc_p2;
//        desc_p1 = new DescPose(0, 0, 0, 0, 0, 0);
//        desc_p2 = new DescPose(0, 0, 0, 0, 0, 0);
//
//        JointPos j1 = new JointPos(-28.529,-140.397,-81.08,-30.934,92.34,-5.629);
//        JointPos j2 = new JointPos(-11.045,-130.984,-104.495,-12.854,92.475,-5.547);
//
//        robot.GetForwardKin(j1,desc_p1);
//        robot.GetForwardKin(j2,desc_p2);
//
//
//        ExaxisPos epos = new ExaxisPos();
//        DescPose offset_pos = new DescPose();
//*********************************焊接起弧、收弧 begin **********************************
//        robot.MoveL(j1, desc_p1,0, 0, 20, 100, 100, -1, epos, 0, 0, offset_pos, 0, 100);
//        robot.ARCStart(0, 0, 10000);//焊接开始
//        robot.MoveL(j2, desc_p2,0, 0, 20, 100, 100, -1, epos, 0, 0, offset_pos, 0, 100);
//        robot.ARCEnd(0, 0, 10000);//焊接结束
//*********************************焊接起弧、收弧 end **********************************

//*********************************获取焊接电流电压、对应关系 begin **********************************
//        WeldCurrentAORelation currentRelation = new WeldCurrentAORelation(0, 1000, 0, 10, 0);
//        robot.WeldingSetCurrentRelation(currentRelation);
//        WeldVoltageAORelation voltageAORelation = new WeldVoltageAORelation(0, 100, 0, 10, 1);
//        robot.WeldingSetVoltageRelation(voltageAORelation);
//
//        WeldCurrentAORelation tmpCur = new WeldCurrentAORelation();
//        WeldVoltageAORelation tmpVol = new WeldVoltageAORelation();
//        robot.WeldingGetCurrentRelation(tmpCur);
//        robot.WeldingGetVoltageRelation(tmpVol);
//*********************************获取焊接电流电压、对应关系 end **********************************

//*********************************设置焊接电流电压值 begin **********************************
        robot.WeldingSetCurrent(0, 500, 0, 0);
        robot.WeldingSetVoltage(0, 60, 1, 0);
//*********************************设置焊接电流电压值 end **********************************

//*********************************设置摆动参数 begin **********************************
        robot.WeaveSetPara(0,0, 2.0, 0, 0.0, 0, 0, 0, 100, 100, 50, 50,1);
//*********************************设置摆动参数 end **********************************

//*********************************摆动焊接 begin **********************************
        DescPose desc_p1 = new DescPose(688.259,-566.358,-139.354,-158.206,0.324,-117.817);
        DescPose desc_p2 = new DescPose(700.078,-224.752,-149.191,-158.2,0.239,-94.978);


        JointPos j1 = new JointPos(0,0,0,0,0,0);
        JointPos j2 = new JointPos(0,0,0,0,0,0);

        robot.GetInverseKin(0, desc_p1, -1, j1);
        robot.GetInverseKin(0, desc_p2, -1, j2);

        ExaxisPos epos = new ExaxisPos();
        DescPose offset_pos = new DescPose();

        robot.MoveL(j1, desc_p1,3, 0, 30, 100, 100, -1, epos, 0, 0, offset_pos, 0, 100);
        robot.WeaveSetPara(0,0, 1.0, 0, 10.0, 0, 0, 0, 100, 100, 50, 50,1);
        robot.ARCStart(0, 0, 10000);
        robot.WeaveStart(0);
        robot.MoveL(j2, desc_p2,3, 0, 10, 100, 100, -1, epos, 0, 0, offset_pos, 0, 100);
        robot.ARCEnd(0, 0, 10000);
        robot.WeaveEnd(0);
//*********************************摆动焊接 end **********************************


//        robot.ArcWeldTraceControl(1, 0, 0, 0.06, 5, 5, 300, 1, -0.06, 5, 5, 300, 1, 0, 4, 1, 10);
//        robot.WeaveStart(0);
//        robot.WeaveStartSim(0);
//        //robot.WeaveInspectStart(0);
//        //robot.WeaveInspectEnd(0);
//        //robot.WeaveEndSim(0);
//        robot.ArcWeldTraceControl(0, 0, 0, 0.06, 5, 5, 300, 1, -0.06, 5, 5, 300, 1, 0, 4, 1, 10);
//        robot.WeaveEnd(0);

//        robot.WeldingSetCurrent(0, 500, 0, 0);
//        robot.WeldingSetVoltage(0, 60, 1, 0);
//        robot.Sleep(2000);

//*********************************送丝与送气 begin **********************************
//        robot.SetForwardWireFeed(0, 1);
//        robot.Sleep(2000);
//        robot.SetForwardWireFeed(0, 0);
//        robot.Sleep(2000);
//        robot.SetReverseWireFeed(0, 1);
//        robot.Sleep(2000);
//        robot.SetReverseWireFeed(0, 0);
//        robot.Sleep(2000);
//
//        robot.SetAspirated(0,1);
//        robot.Sleep(2000);
//        robot.SetAspirated(0,0);
//*********************************送丝与送气 end **********************************


        //焊丝寻位
//*********************************焊丝寻位 begin **********************************
//        ExaxisPos exaxisPos = new ExaxisPos(0, 0, 0, 0);
//        DescPose offdese = new DescPose(0, 0, 0, 0, 0, 0);
//
//        DescPose descStart = new DescPose(153.736,-715.249,-295.037,-179.829,2.613,-155.615);
//        JointPos jointStart = new JointPos(0,0,0,0,0,0);
//
//        DescPose descEnd = new DescPose(73.748,-645.825,-295.016,-179.828,2.608,-155.614);
//        JointPos jointEnd = new JointPos(0,0,0,0,0,0);
//
//        robot.GetInverseKin(0, descStart, -1, jointStart);
//        robot.GetInverseKin(0, descEnd, -1, jointEnd);
//
//        robot.MoveL(jointStart, descStart, 3, 0, 30, 100, 100, -1, exaxisPos, 0, 0, offdese, 0, 100);
//        robot.MoveL(jointEnd, descEnd, 3, 0, 30, 100, 100, -1, exaxisPos, 0, 0, offdese,0, 100);
//
//        DescPose descREF0A = new DescPose(273.716,-723.539,-295.075,-179.829,2.608,-155.614);
//        JointPos jointREF0A = new JointPos(0,0,0,0,0,0);
//
//        DescPose descREF0B = new DescPose(202.588,-723.543,-295.039,-179.829,2.609,-155.614);
//        JointPos jointREF0B = new JointPos(0,0,0,0,0,0);
//
//        DescPose descREF1A = new DescPose(75.265,-525.091,-295.059,-179.83,2.609,-155.616);
//        JointPos jointREF1A = new JointPos(0,0,0,0,0,0);
//
//        DescPose descREF1B = new DescPose(75.258,-601.157,-295.075,-179.834,2.609,-155.616);
//        JointPos jointREF1B = new JointPos(0,0,0,0,0,0);
//
//        robot.GetInverseKin(0, descREF0A, -1, jointREF0A);
//        robot.GetInverseKin(0, descREF0B, -1, jointREF0B);
//        robot.GetInverseKin(0, descREF1A, -1, jointREF1A);
//        robot.GetInverseKin(0, descREF1B, -1, jointREF1B);
//
//        robot.WireSearchStart(0, 10, 100, 0, 10, 100, 0);
//        robot.MoveL(jointREF0A, descREF0A, 3, 0, 30, 100, 100, -1, exaxisPos, 0, 0, offdese, 0, 100);  //起点
//        robot.MoveL(jointREF0B, descREF0B, 3, 0, 30, 100, 100, -1, exaxisPos, 1, 0, offdese, 0, 100);  //方向点
//        robot.WireSearchWait("REF0");
//        robot.WireSearchEnd(0, 10, 100, 0, 10, 100, 0);
//
//        robot.WireSearchStart(0, 10, 100, 0, 10, 100, 0);
//        robot.MoveL(jointREF1A, descREF1A, 3, 0, 30, 100, 100, -1, exaxisPos, 0, 0, offdese, 0, 100);  //起点
//        robot.MoveL(jointREF1B, descREF1B, 3, 0, 30, 100, 100, -1, exaxisPos, 1, 0, offdese, 0, 100);  //方向点
//        robot.WireSearchWait("REF1");
//        robot.WireSearchEnd(0, 10, 100, 0, 10, 100, 0);
//
//
//        robot.WireSearchStart(0, 10, 100, 0, 10, 100, 0);
//        robot.MoveL(jointREF0A, descREF0A, 3, 0, 30, 100, 100, -1, exaxisPos, 0, 0, offdese, 0, 100);  //起点
//        robot.MoveL(jointREF0B, descREF0B, 3, 0, 30, 100, 100, -1, exaxisPos, 1, 0, offdese, 0, 100);  //方向点
//        robot.WireSearchWait("RES0");
//        robot.WireSearchEnd(0, 10, 100, 0, 10, 100, 0);
//
//        robot.WireSearchStart(0, 10, 100, 0, 10, 100, 0);
//        robot.MoveL(jointREF1A, descREF1A, 3, 0, 30, 100, 100, -1, exaxisPos, 0, 0, offdese, 0, 100);  //起点
//        robot.MoveL(jointREF1B, descREF1B, 3, 0, 30, 100, 100, -1, exaxisPos, 1, 0, offdese, 0, 100);  //方向点
//        robot.WireSearchWait("RES1");
//        robot.WireSearchEnd(0, 10, 100, 0, 10, 100, 0);
//
//        String[] varNameRef = { "REF0", "REF1", "#", "#", "#", "#"};
//        String[] varNameRes = { "RES0", "RES1", "#", "#", "#", "#" };
//
//        DescOffset offectPos = new DescOffset();
//        robot.GetWireSearchOffset(0, 0, varNameRef, varNameRes, offectPos);
//        robot.PointsOffsetEnable(offectPos.offsetFlag, offectPos.offset);
//        robot.MoveL(jointStart, descStart, 3, 1, 30, 100, 100, -1, exaxisPos, 0, 0, offdese, 0, 100);
//        robot.MoveL(jointEnd, descEnd, 3, 1, 30, 100, 100, -1, exaxisPos, 1, 0, offdese, 0, 100);
//        robot.PointsOffsetDisable();
//*********************************焊丝寻位 end **********************************

//*********************************电弧跟踪 begin **********************************
//        robot.MoveJ(j1, desc_p1,1, 0, 10, 100, 100, epos, -1, 0, offset_pos);
//        robot.WeaveSetPara(0,0, 3.0, 0, 0.0, 0, 0, 0, 100, 100, 50, 50,1);
//        robot.ARCStart(0, 0, 10000);
//        robot.ArcWeldTraceControl(1, 0, 0, 0.06, 5, 5, 300, 1, -0.06, 5, 5, 300, 1, 0, 4, 1, 10);
//        robot.WeaveStart(0);
//        robot.MoveL(j2, desc_p2,1, 0, 10, 100, 100, -1, epos, 0, 0, offset_pos, 0, 100);
//        robot.ARCEnd(0, 0, 10000);
//        robot.ArcWeldTraceControl(0, 0, 0, 0.06, 5, 5, 300, 1, -0.06, 5, 5, 300, 1, 0, 4, 1, 10);
//        robot.WeaveEnd(0);
//*********************************电弧跟踪 end **********************************

//*********************************焊接工艺曲线参数配置与获取 begin **********************************
//        WeldingProcessParam param = new WeldingProcessParam(177.0,27.0,1000,178.0,28.0,176.0,26.0,1000);
//        robot.WeldingSetProcessParam(1, param);
//
//        WeldingProcessParam getParam = new WeldingProcessParam();
//        robot.WeldingGetProcessParam(1, getParam);
//*********************************焊接工艺曲线参数配置与获取 end **********************************

//*********************************焊接扩展IO功能配置 begin **********************************
//        robot.SetArcStartExtDoNum(1);
//        robot.SetAirControlExtDoNum(2);
//        robot.SetWireForwardFeedExtDoNum(3);
//        robot.SetWireReverseFeedExtDoNum(4);
//
//        robot.SetWeldReadyExtDiNum(5);
//        robot.SetArcDoneExtDiNum(6);
//        robot.SetExtDIWeldBreakOffRecover(7, 8);
//*********************************焊接扩展IO功能配置 end **********************************

//*********************************摆动预警 begin **********************************
//        DescPose desc_p1=new DescPose(-104.846,309.573,336.647,179.681,-0.419,-92.692);
//        DescPose desc_p2=new DescPose(-318.287,158.502,346.184,179.602,1.081,-46.342);
//        JointPos j1=new JointPos(0,0,0,0,0,0);
//        JointPos j2=new JointPos(0,0,0,0,0,0);
//
//        robot.GetInverseKin(0, desc_p1, -1, j1);
//        robot.GetInverseKin(0, desc_p2, -1, j2);
//
//        ExaxisPos epos = new ExaxisPos();
//        DescPose offset_pos = new DescPose();
//
//        robot.MoveL(j2, desc_p2,3, 0, 30, 100, 100, -1, epos, 0, 0, offset_pos, 0, 100);
//        robot.WeaveSetPara(0,0, 2.0, 0, 0.0, 0, 0, 0, 100, 100, 50, 50,1);
//        robot.WeaveInspectStart(0);
//        robot.MoveL(j2, desc_p2,3, 0, 30, 100, 100, -1, epos, 0, 0, offset_pos, 0, 100);
//        robot.WeaveInspectEnd(0);
//*********************************摆动预警 end **********************************


    }

    private static void TPD(Robot robot)
    {
//*************************************轨迹记录 begin ************************************
        int type = 1;
        String name = "tpd_2024";
        int period_ms = 2;
        int di_choose = 0;
        int do_choose = 0;

        robot.SetTPDDelete(name);

        robot.SetTPDParam(type, name, period_ms, di_choose, do_choose);

        robot.Mode(1);
        robot.Sleep(1000);
        robot.DragTeachSwitch(1);
        robot.SetTPDStart(type, name, period_ms, di_choose, do_choose);
        robot.Sleep(10000);
        robot.SetWebTPDStop();
        robot.DragTeachSwitch(0);
//*************************************轨迹记录 end ************************************

//*************************************轨迹复现 begin ************************************
        int tool = 0;
        int user = 0;
        double vel = 30.0;
        double acc = 100.0;
        double ovl = 100.0;
        double blendT = -1.0;
        int config = -1;
        byte blend = 1;

        DescPose desc_pose = new DescPose();
        robot.GetTPDStartPose(name,  desc_pose);
        robot.SetTrajectoryJSpeed(100.0);

        robot.LoadTPD(name);
        robot.MoveCart(desc_pose, tool, user, vel, acc, ovl, blendT, config);
        robot.MoveTPD(name, blend, 80.0);
//*************************************轨迹复现 begin ************************************

    }

    private static void RobotState(Robot robot)
    {
//***************************** 机械臂安装角度查询 begin *****************************
        robot.SetRobotInstallAngle(23.4, 56.7);
        List<Number> rtnArr =  robot.GetRobotInstallAngle();
        System.out.println("安装角度: " + rtnArr.get(1) + "  " + rtnArr.get(2));
        robot.SetRobotInstallAngle(0, 0);
//***************************** 机械臂安装角度查询 end *****************************


//        DescPose  desc_p1=new DescPose();
//
//        JointPos JP1=new JointPos(117.408,-86.777,81.499,-87.788,-92.964,92.959);
//        JointPos JP_test=new JointPos();
//        DescPose DP1 =new DescPose(327.359,-420.973,518.377,-177.199,3.209,114.449);
//        robot.GetInverseKin(0, DP1, -1, JP_test);
//        List<Integer> rtnArr =  robot.GetInverseKinHasSolution(0, DP1, JP1);//逆向是否有解
//        System.out.println("has Solution ? " + rtnArr.get(1));
//        robot.GetForwardKin(JP1, desc_p1);//正向运动学
//        JointPos j2 = new JointPos();
//        robot.GetInverseKinRef(0, DP1, JP1, JP_test);//逆向运动学

//*************************************获取固件、硬件版本 begin *********************************
//        String ctrlBoxBoardVersion = "";
//        String driver1Version = "";
//        String driver2Version = "";
//        String driver3Version = "";
//        String driver4Version = "";
//        String driver5Version = "";
//        String driver6Version = "";
//        String endBoardVersion = "";
//        robot.GetHardwareVersion(ctrlBoxBoardVersion ,driver1Version,  driver2Version,  driver3Version,
//                 driver4Version,  driver5Version,  driver6Version,  endBoardVersion);
//
//        robot.GetFirmwareVersion(ctrlBoxBoardVersion, driver1Version, driver2Version, driver3Version,
//                driver4Version, driver5Version, driver6Version, endBoardVersion);
//*************************************获取固件、硬件版本 end *********************************

//*************************************获取机械臂的SSH begin *********************************
//        String[] key = {""};
//        robot.GetSSHKeygen(key);
//        System.out.println("ssh key  " + key[0]);
//*************************************获取机械臂的SSH end *********************************

//*************************************获取负载重量和质心 begin *********************************
//        robot.SetLoadWeight(0);
//        robot.SetLoadCoord(new DescTran(0.0, 0.0, 0.0));
//
//        List<Number> rtnArr =  robot.GetTargetPayload(1);
//        DescTran cog = new DescTran();
//        robot.GetTargetPayloadCog(1, cog);
//
//        System.out.println("weight is " + rtnArr.get(1) + " cog is  " + cog.x + "  " + cog.y + "  " + cog.z);
//*************************************获取负载重量和质心 end *********************************

//*************************************获取机器人关节配置 begin *********************************
//        List<Integer> rtnArr = robot.GetRobotCurJointsConfig();
//        System.out.println("config is " + rtnArr.get(1));
//*************************************获取机器人关节配置 end *********************************

//*************************************获取机器人时钟 begin *********************************
//        List<Number> rtnArr = robot.GetSystemClock();
//        System.out.println("systom clock is  " + rtnArr.get(1));
//*************************************获取机器人时钟 end *********************************

//*************************************获取机器人当前速度 begin *********************************
//        List<Number> rtnArr = robot.GetDefaultTransVel();
//        System.out.println("机器人当前速度为: " + rtnArr.get(1));
//*************************************获取机器人当前速度 end *********************************

//*************************************获取机器人工具/工件坐标系 begin *********************************
//        DescPose offset = new DescPose();
//        robot.GetTCPOffset(1, offset);//工具
//        robot.GetWObjOffset(1, offset);//工件
//        System.out.println("offset is " + offset);
//*************************************获取机器人工具/工件坐标系 end *********************************

//*************************************获取机器人软限位 begin *********************************
//        Object[] neg_deg = new Object[]{0, 0 , 0, 0, 0, 0};
//        Object[] pos_deg = new Object[]{0, 0 , 0, 0, 0, 0};
//        robot.GetJointSoftLimitDeg(1,  neg_deg,  pos_deg);
//        System.out.println("neg is " + Arrays.toString(neg_deg) + " pos is " + Arrays.toString(pos_deg));
//*************************************获取机器人软限位 end *********************************

//*************************************获取机器人完成信号 begin *********************************
//        JointPos JP1=new JointPos(117.408,-86.777,81.499,-87.788,-92.964,92.959);
//        DescPose DP1 =new DescPose(327.359,-420.973,518.377,-177.199,3.209,114.449);
//        JointPos JP2=new JointPos(72.515,-86.774,81.525,-87.724,-91.964,92.958);
//        DescPose DP2=new DescPose(-63.512,-529.698,517.946,-178.192,3.070,69.554);
//
//        ExaxisPos epos = new ExaxisPos();
//        DescPose offset_pos = new DescPose();
//        robot.SetSpeed(10);
//        robot.MoveL(JP1, DP1,0, 0, 50, 100, 100, 0, epos, 0, 0, offset_pos, 0, 100);
//        robot.MoveL(JP2, DP2,0, 0, 50, 100, 100, 0, epos, 0, 0, offset_pos, 0, 100);
//        while (true)
//        {
//            ROBOT_STATE_PKG pkg = robot.GetRobotRealTimeState();
//            System.out.println("motion done is " + pkg.motion_done);
//            robot.Sleep(100);
//        }
//*************************************获取机器人完成信号 end *********************************

//*************************************获取机器人示教点 begin *********************************
//        List<Number> rtnArr = robot.GetRobotTeachingPoint("P1");
//        System.out.println("point data  " + rtnArr);
//*************************************获取机器人示教点 end *********************************

    }

    private static void RobotSafety(Robot robot)
    {
//****************************** 碰撞等级 begin***********************
//        Object[] config = {2.0, 2.0, 2.0, 2.0, 2.0, 2.0};
//        robot.SetAnticollision(0, config, 1);
//        int safetyMargin[]={10,10,10,10,10,10};
//        robot.SetCollisionStrategy(0,1000,10,safetyMargin);
//
//        robot.ProgramLoad("/fruser/test.lua");
//        robot.ProgramRun();
//****************************** 碰撞等级 end***********************

//****************************** 增加软限位 begin***********************
//        Object[] plimit = { 170.0, 80.0, 150.0, 80.0, 170.0, 160.0 };
//        robot.SetLimitPositive(plimit);
//
//        Object[] nlimit = { -170.0, -260.0, -150.0, -260.0, -170.0, -160.0 };
//        robot.SetLimitNegative(nlimit);
//****************************** 增加软限位 end***********************

//****************************** 清除所有错误 begin***********************
//        robot.SetLoadWeight(123.0);
//        robot.Sleep(3000);
//        robot.ResetAllError();
//****************************** 清除所有错误 end***********************

//****************************** 设置关节摩擦力补偿 begin ***********************
//        Object[] lcoeff = { 0.5, 0.5, 0.5, 0.5, 0.5, 0.5 };
//        Object[] wcoeff = { 0.5, 0.5, 0.5, 0.5, 0.5, 0.5 };
//        Object[] ccoeff = { 0.5, 0.5, 0.5, 0.5, 0.5, 0.5 };
//        Object[] fcoeff = { 0.5, 0.5, 0.5, 0.5, 0.5, 0.5 };
//
//        robot.FrictionCompensationOnOff(1);
//
//        robot.SetFrictionValue_level(lcoeff);//正装
//
//        robot.SetFrictionValue_wall(wcoeff);//侧装
//
//        robot.SetFrictionValue_ceiling(ccoeff);//倒装
//
//        robot.SetFrictionValue_freedom(fcoeff);//自由安装
//****************************** 设置关节摩擦力补偿 end ***********************


//        robot.SetCollisionDetectionMethod(1);//设置静态下碰撞检测
//        robot.SetStaticCollisionOnOff(1);

//****************************** 关节功率扭矩检测 begin ***********************
//        robot.DragTeachSwitch(1);
//        robot.SetPowerLimit(1, 2);
//        Object[] torques = { 0, 0, 0, 0, 0, 0 };
//        robot.GetJointTorques(1, torques);
//
//        int count = 20;
//        robot.ServoJTStart(); //   #servoJT开始
//        int error = 0;
//        while (count > 0)
//        {
//            //torques[0] = (double)torques[0] + 0.1;//  #每次1轴增加0.1NM，运动100次
//            error = robot.ServoJT(torques, 0.001);  //# 关节空间伺服模式运动
//            count = count - 1;
//            robot.Sleep(1);
//        }
//        error = robot.ServoJTEnd();  //#伺服运动结束

//****************************** 关节功率扭矩检测 end ***********************

    }


    private static void CommonSets(Robot robot)
    {
//**********************************设置速度 begin***************************
//        robot.Mode(1);
//        for(int i = 0; i < 90; i++)
//        {
//            robot.SetSpeed(i + 1);
//            robot.Sleep(50);
//        }
//
//        robot.Mode(0);
//**********************************设置速度 end***************************

//**********************************设置加速度 begin***************************
        JointPos j1 = new JointPos(-114.756,-109.423,87.751,-65.834,-52.210,92.992);
        JointPos j2 = new JointPos(-51.811,-91.530,74.859,-81.580,-121.913,92.990);
        JointPos j3 = new JointPos(-105.350,-65.747,32.346,-26.434,-60.784,92.992);
        JointPos j4 = new JointPos(-79.015,-81.540,47.786,-57.729,-88.577,92.991);
        JointPos j5 = new JointPos(-102.647,-99.334,68.842,-59.256,-88.407,92.937);
        JointPos j6 = new JointPos(-78.990,-78.608,38.428,-51.812,-88.310,92.939);

        DescPose desc_p1 = new DescPose(0, 0, 0, 0, 0, 0);
        DescPose desc_p2 = new DescPose(0, 0, 0, 0, 0, 0);
        DescPose desc_p3 = new DescPose(0, 0, 0, 0, 0, 0);
        DescPose desc_p4 = new DescPose(0, 0, 0, 0, 0, 0);
        DescPose desc_p5 = new DescPose(0, 0, 0, 0, 0, 0);
        DescPose desc_p6 = new DescPose(0, 0, 0, 0, 0, 0);

        robot.GetForwardKin(j1, desc_p1);
        robot.GetForwardKin(j2, desc_p2);
        robot.GetForwardKin(j3, desc_p3);
        robot.GetForwardKin(j4, desc_p4);
        robot.GetForwardKin(j5, desc_p5);
        robot.GetForwardKin(j6, desc_p6);
        ExaxisPos epos = new ExaxisPos();
        DescPose offset_pos = new DescPose();
//
//        robot.SetSpeed(60);
////
//        robot.SetOaccScale(10);
////        robot.MoveL(j1, desc_p1,0, 0, 100, 100, 100, -1, epos, 0, 0, offset_pos, 0, 100);
////        robot.MoveL(j2, desc_p2,0, 0, 100, 100, 100, -1, epos, 0, 0, offset_pos, 0, 100);
////
////        robot.SetOaccScale(90);
////        robot.MoveL(j1, desc_p1,0, 0, 100, 100, 100, -1, epos, 0, 0, offset_pos, 0, 100);
////        robot.MoveL(j2, desc_p2,0, 0, 100, 100, 100, -1, epos, 0, 0, offset_pos, 0, 100);

        //测试加速度
//        ExaxisPos epos = new ExaxisPos();
//        DescPose offset_pos = new DescPose();
//        JointPos JP1=new JointPos(117.408,-86.777,81.499,-87.788,-92.964,92.959);
//        DescPose DP1 =new DescPose(327.359,-420.973,518.377,-177.199,3.209,114.449);
//        JointPos JP2=new JointPos(72.515,-86.774,81.525,-87.724,-91.964,92.958);
//        DescPose DP2=new DescPose();
//        JointPos JP3=new JointPos(89.281,-102.959,81.527,-69.955,-86.755,92.958);
//        DescPose DP3=new DescPose();
//        robot.GetForwardKin(JP2,DP2);
//        robot.GetForwardKin(JP3,DP3);
//
//        robot.SetSpeed(60);
//
//        robot.SetOaccScale(10);
//        robot.MoveL(JP1, DP1,0, 0, 100, 100, 100, -1, epos, 0, 0, offset_pos, 0, 100);
//        robot.MoveL(JP2, DP2,0, 0, 100, 100, 100, -1, epos, 0, 0, offset_pos, 0, 100);
//
//        robot.SetOaccScale(90);
//        robot.MoveL(JP1, DP1,0, 0, 100, 100, 100, -1, epos, 0, 0, offset_pos, 0, 100);
//        robot.MoveL(JP2, DP2,0, 0, 100, 100, 100, -1, epos, 0, 0, offset_pos, 0, 100);
//**********************************设置加速度 end***************************

//**********************************设置和获取系统变量 begin***************************
//        for(int i = 1; i < 10; i++)
//        {
//            robot.SetSysVarValue(i, i * 10);
//        }
//        for(int i = 1; i < 10; i++)
//        {
//            List<Number> rtnArr = robot.GetSysVarValue(i);
//            System.out.println("SysVarValue " +  i  + " is " + rtnArr.get(1));
//        }
//**********************************设置和获取系统变量 end***************************

//**********************************六点法工具坐标系标定 begin ***************************
//        JointPos jp1=new JointPos(-89.407,-148.279,-83.169,-45.689,133.689,41.705);
//        JointPos jp2=new JointPos(-67.595,-143.7,-88.006,-48.514,57.073,56.189);
//        JointPos jp3=new JointPos(-88.229,-152.355,-67.815,-78.07,129.029,58.739);
//        JointPos jp4=new JointPos(-77.528,-141.519,-89.826,-37.184,90.274,41.769);
//        JointPos jp5=new JointPos(-76.744,-138.219,-97.714,-32.595,90.255,42.558);
//        JointPos jp6=new JointPos(-77.595,-138.454,-90.065,-40.014,90.275,41.709);
//        ExaxisPos epos = new ExaxisPos();
//        DescPose offset_pos = new DescPose();
//
//        DescPose desc_p1 = new DescPose(0, 0, 0, 0, 0, 0);
//        DescPose desc_p2 = new DescPose(0, 0, 0, 0, 0, 0);
//        DescPose desc_p3 = new DescPose(0, 0, 0, 0, 0, 0);
//        DescPose desc_p4 = new DescPose(0, 0, 0, 0, 0, 0);
//        DescPose desc_p5 = new DescPose(0, 0, 0, 0, 0, 0);
//        DescPose desc_p6 = new DescPose(0, 0, 0, 0, 0, 0);
//        robot.GetForwardKin(jp1, desc_p1);
//        robot.GetForwardKin(jp2, desc_p2);
//        robot.GetForwardKin(jp3, desc_p3);
//        robot.GetForwardKin(jp4, desc_p4);
//        robot.GetForwardKin(jp5, desc_p5);
//        robot.GetForwardKin(jp6, desc_p6);
//        robot.MoveJ(jp1, desc_p1,0, 0, 30, 100, 100, epos, -1, 0, offset_pos);
//        robot.SetToolPoint(1);
//
//        robot.MoveJ(jp2, desc_p2,0, 0, 30, 100, 100, epos, -1, 0, offset_pos);
//        robot.SetToolPoint(2);
//
//        robot.MoveJ(jp3, desc_p3,0, 0, 30, 100, 100, epos, -1, 0, offset_pos);
//        robot.SetToolPoint(3);
//
//        robot.MoveJ(jp4, desc_p4,0, 0, 30, 100, 100, epos, -1, 0, offset_pos);
//        robot.SetToolPoint(4);
//
//        robot.MoveJ(jp5, desc_p5,0, 0, 30, 100, 100, epos, -1, 0, offset_pos);
//        robot.SetToolPoint(5);
//
//        robot.MoveJ(jp6, desc_p6,0, 0, 30, 100, 100, epos, -1, 0, offset_pos);
//        robot.SetToolPoint(6);
//
//        DescPose coord = new DescPose();
//        robot.ComputeTool(coord);
//        System.out.println("result is " + coord.tran.x + "  " + coord.tran.y + "  " + coord.tran.z + "  " + coord.rpy.rx + "  " + coord.rpy.ry + "  " + coord.rpy.rz);
//**********************************六点法工具坐标系标定 end ***************************

//**********************************4点法标定 begin ***************************
//        JointPos jp1=new JointPos(-89.407,-148.279,-83.169,-45.689,133.689,41.705);
//        JointPos jp2=new JointPos(-67.595,-143.7,-88.006,-48.514,57.073,56.189);
//        JointPos jp3=new JointPos(-88.229,-152.355,-67.815,-78.07,129.029,58.739);
//        JointPos jp4=new JointPos(-77.528,-141.519,-89.826,-37.184,90.274,41.769);
//        ExaxisPos epos = new ExaxisPos();
//        DescPose offset_pos = new DescPose();
//        DescPose desc_p1 = new DescPose(0, 0, 0, 0, 0, 0);
//        DescPose desc_p2 = new DescPose(0, 0, 0, 0, 0, 0);
//        DescPose desc_p3 = new DescPose(0, 0, 0, 0, 0, 0);
//        DescPose desc_p4 = new DescPose(0, 0, 0, 0, 0, 0);
//        robot.GetForwardKin(jp1, desc_p1);
//        robot.GetForwardKin(jp2, desc_p2);
//        robot.GetForwardKin(jp3, desc_p3);
//        robot.GetForwardKin(jp4, desc_p4);
//
//        robot.MoveJ(jp1, desc_p1,0, 0, 30, 100, 100, epos, -1, 0, offset_pos);
//        robot.SetTcp4RefPoint(1);
//
//        robot.MoveJ(jp2, desc_p2,0, 0, 30, 100, 100, epos, -1, 0, offset_pos);
//        robot.SetTcp4RefPoint(2);
//
//        robot.MoveJ(jp3, desc_p3,0, 0, 30, 100, 100, epos, -1, 0, offset_pos);
//        robot.SetTcp4RefPoint(3);
//
//        robot.MoveJ(jp4, desc_p4,0, 0, 30, 100, 100, epos, -1, 0, offset_pos);
//        robot.SetTcp4RefPoint(4);
//
//        DescPose coord4 = new DescPose();
//        robot.ComputeTcp4(coord4);
//        System.out.println("result is " + coord4.tran.x + "  " + coord4.tran.y + "  " + coord4.tran.z + "  " + coord4.rpy.rx + "  " + coord4.rpy.ry + "  " + coord4.rpy.rz);
//**********************************4点法标定 end ***************************

//**********************************设置工具坐标系和列表 begin ***************************
//        DescPose coord = new DescPose(0,  0,  20,  0,  0,  0);
//        robot.SetToolCoord(5, coord, 0, 0);
//        robot.SetToolList(5, coord, 0, 0);
//**********************************设置工具坐标系和列表 end ***************************

//**********************************外部工具坐标系标定 begin ***************************
//        JointPos j1 = new JointPos(-84.787, -152.056,-75.689 , -37.899, 94.486,41.709);
//        JointPos j2 = new JointPos(-79.438,-152.139,-75.634,-37.469,94.065,47.058);
//        JointPos j3 = new JointPos(-84.788,-145.179,-77.119,-43.345,94.487,41.709);
//
//
//        DescPose desc_p1 = new DescPose(0, 0, 0, 0, 0, 0);
//        DescPose desc_p2 = new DescPose(0, 0, 0, 0, 0, 0);
//        DescPose desc_p3 = new DescPose(0, 0, 0, 0, 0, 0);
//
//        robot.GetForwardKin(j1, desc_p1);
//        robot.GetForwardKin(j2, desc_p2);
//        robot.GetForwardKin(j3, desc_p3);
//
//        ExaxisPos epos = new ExaxisPos();
//        DescPose offset_pos = new DescPose();
//
//        robot.MoveJ(j1, desc_p1,0, 0, 20, 100, 100, epos, -1, 0, offset_pos);
//        robot.SetExTCPPoint(1);
//
//        robot.MoveJ(j2, desc_p2,0, 0, 20, 100, 100, epos, -1, 0, offset_pos);
//        robot.SetExTCPPoint(2);
//
//        robot.MoveJ(j3, desc_p3,0, 0, 20, 100, 100, epos, -1, 0, offset_pos);
//        robot.SetExTCPPoint(3);
//
//        DescPose coordE = new DescPose();
//        robot.ComputeExTCF(coordE);
//        System.out.println("result is " + coordE.tran.x + "  " + coordE.tran.y + "  " + coordE.tran.z + "  " + coordE.rpy.rx + "  " + coordE.rpy.ry + "  " + coordE.rpy.rz);
//**********************************外部工具坐标系标定 end ***************************

//**********************************设置外部工具坐标系和列表 begin ***************************
//        DescPose coordE = new DescPose(100,  0,  0,  0,  0,  0);
//        robot.SetExToolCoord(5, coordE, coordE);
//        robot.SetExToolList(5,coordE, coordE);
//**********************************设置外部工具坐标系和列表 end ***************************

//**********************************工件坐标系的标定 begin ***************************
//        JointPos j1 = new JointPos(-84.787, -152.056,-75.689,-37.899,94.486,41.709);
//        JointPos j2 = new JointPos(-79.438,-152.139,-75.634,-37.469,94.065,47.058);
//        JointPos j3 = new JointPos(-84.788,-145.179,-77.119,-43.345,94.487,41.709);
//        DescPose desc_p1 = new DescPose(0, 0, 0, 0, 0, 0);
//        DescPose desc_p2 = new DescPose(0, 0, 0, 0, 0, 0);
//        DescPose desc_p3 = new DescPose(0, 0, 0, 0, 0, 0);
//
//        robot.GetForwardKin(j1, desc_p1);
//        robot.GetForwardKin(j2, desc_p2);
//        robot.GetForwardKin(j3, desc_p3);
//
//        ExaxisPos epos = new ExaxisPos();
//        DescPose offset_pos = new DescPose();
//
//        robot.MoveJ(j1, desc_p1,0, 0, 20, 100, 100, epos, -1, 0, offset_pos);
//        robot.SetWObjCoordPoint(1);
//
//        robot.MoveJ(j2, desc_p2,0, 0, 20, 100, 100, epos, -1, 0, offset_pos);
//        robot.SetWObjCoordPoint(2);
//
//        robot.MoveJ(j3, desc_p3,0, 0, 20, 100, 100, epos, -1, 0, offset_pos);
//        robot.SetWObjCoordPoint(3);
//
//        DescPose coordE = new DescPose();
//        robot.ComputeWObjCoord(0, coordE);
//        System.out.println("result is " + coordE.tran.x + "  " + coordE.tran.y + "  " + coordE.tran.z + "  " + coordE.rpy.rx + "  " + coordE.rpy.ry + "  " + coordE.rpy.rz);
//
//        robot.SetWObjCoord(5, coordE,0);
//        robot.SetWObjList(5,coordE,0);
//**********************************设置工件坐标系的标定 end ***************************

//**********************************设置末端负载 begin ***************************
//        robot.SetLoadWeight(2);
//        robot.SetLoadCoord(new DescTran(1.0, 2.0, 3.0));
//**********************************设置末端负载 end ***************************

//**********************************设置机器人安装方式 begin ***************************
//        robot.SetRobotInstallPos(0);
//**********************************设置机器人安装方式 begin ***************************

//**********************************设置机器人安装角度 begin ***************************
//        robot.SetRobotInstallAngle(0, 0);
//**********************************设置机器人安装角度 begin ***************************

    }

    private static void IOTest(Robot robot)
    {
//****************************** SetDO begin***********************
        robot.SetDO(8, 1, 0, 0);
        robot.Sleep(3000);
        robot.SetDO(8, 0, 0, 0);
//****************************** SetDO end***********************

//****************************** SetToolDO begin***********************
        robot.SetToolDO(0, 1, 0, 0);
        robot.Sleep(3000);
        robot.SetToolDO(0, 0, 0, 0);
//****************************** SetToolDO end***********************

//****************************** SetAO begin***********************
        for(int i = 0; i < 90; i++)
        {
            robot.SetAO(0, i+1, 0);
            robot.SetAO(1, i+1, 0);
            robot.Sleep(50);
        }
        robot.SetAO(0, 0.0, 0);
        robot.SetAO(1, 0.0, 0);
//****************************** SetAO end***********************

//****************************** SetToolAO begin***********************
        for(int i = 0; i < 99; i++)
        {
            robot.SetToolAO(0, i+1, 0);
            robot.Sleep(50);
        }
        robot.SetToolAO(0, 0.0, 0);
//****************************** SetToolAO end***********************

//****************************** Wait begin***********************
        System.out.println("wait  start ");
        robot.WaitDI(1, 1, 10000, 0);//WaitDI
        robot.WaitMultiDI(0, 6, 6, 10000, 0);//WaitMultiDI
        robot.WaitToolDI(0, 1, 5000, 0);//WaitToolDI
        robot.WaitAI(0, 0, 8.0, 5000, 0);//WaitAI
        robot.WaitToolAI(0, 0, 20, 5000, 0);//WaitToolAI
        System.out.println("wait  end ");
//****************************** Wait end***********************


//****************************** MoveAOStart/Stop  MoveToolAOStart/Stop begin***********************
//        robot.MoveToolAOStart(0, 100, 80, 1);//末端AO飞拍开始
//        robot.MoveAOStart(0, 100, 80, 1);//控制箱AO飞拍
//        DescPose  desc_p1, desc_p2;
//
//        desc_p1 = new DescPose(0, 0, 0, 0, 0, 0);
//        desc_p2 = new DescPose(0, 0, 0, 0, 0, 0);
//
//        JointPos j1 = new JointPos(-81.684,-106.159,-74.447,-86.33,94.725,41.639);
//        JointPos j2 = new JointPos(-102.804,-106.159,-74.449,-86.328,94.715,41.639);
//
//        robot.GetForwardKin(j1,desc_p1);
//        robot.GetForwardKin(j2,desc_p2);
//
//        ExaxisPos epos = new ExaxisPos();
//        DescPose offset_pos = new DescPose();
//        robot.MoveL(j1, desc_p1,0, 0, 30, 100, 100, -1, epos, 0, 0, offset_pos, 0, 100);
//        robot.MoveL(j2, desc_p2,0, 0, 30, 100, 100, -1, epos, 0, 0, offset_pos, 0, 100);
//        robot.MoveToolAOStop();
//        robot.MoveAOStop();
//****************************** MoveAOStart/Stop  MoveToolAOStart/Stop end***********************

//        robot.SetDO(1, 1, 0, 0);
//        robot.SetDO(2, 1, 0, 0);
//        robot.SetDO(3, 1, 0, 0);
//
//        robot.SetAO(0, 50, 0);//A0口电流/电压
//        robot.SetAO(1, 70, 0);
//
//        robot.SetToolDO(0, 1, 0, 0);//设置工具数字量输出
//        robot.SetToolDO(1, 1, 0, 0);
//
//        robot.SetToolAO(0, 50, 0);
//
//        robot.SetOutputResetCtlBoxDO(1);
//        robot.SetOutputResetCtlBoxAO(1);
//        robot.SetOutputResetAxleDO(1);
//        robot.SetOutputResetAxleAO(1);
//
//        robot.Sleep(3000);
//
//        robot.ProgramLoad("/fruser/test.lua");
//        robot.ProgramRun();
    }

    private static void Moves(Robot robot)
    {
        DescPose tcoord, desc_p1, desc_p2, desc_p3, desc_p4;//笛卡尔空间位置与姿态
        tcoord = new DescPose(0, 0, 0, 0, 0, 0);
        desc_p1 = new DescPose(0, 0, 0, 0, 0, 0);
        desc_p2 = new DescPose(0, 0, 0, 0, 0, 0);
        desc_p3 = new DescPose(0, 0, 0, 0, 0, 0);
        desc_p4 = new DescPose(0, 0, 0, 0, 0, 0);

        desc_p1.tran.x = -104.846;
        desc_p1.tran.y = 309.573;
        desc_p1.tran.z = 336.647;
        desc_p1.rpy.rx = 179.681;
        desc_p1.rpy.ry = -0.419;
        desc_p1.rpy.rz = -92.692;

        desc_p2.tran.x = -318.287;
        desc_p2.tran.y = 158.502;
        desc_p2.tran.z = 346.184;
        desc_p2.rpy.rx = 179.602;
        desc_p2.rpy.ry = 1.081;
        desc_p2.rpy.rz = -46.342;

        desc_p3.tran.x = -352.414;
        desc_p3.tran.y = 24.059;
        desc_p3.tran.z = 395.376;
        desc_p3.rpy.rx = 179.755;
        desc_p3.rpy.ry = -1.045;
        desc_p3.rpy.rz = -23.877;

        desc_p4.tran.x = 195.474;
        desc_p4.tran.y = 423.278;
        desc_p4.tran.z = 228.509;
        desc_p4.rpy.rx = -179.199;
        desc_p4.rpy.ry = -0.567;
        desc_p4.rpy.rz = -130.209;

        JointPos j1 = new JointPos();
        JointPos j2 = new JointPos();
        JointPos j3 = new JointPos();
        JointPos j4 = new JointPos();
        robot.GetInverseKin(0, desc_p1, -1, j1);//逆向运动学求解
        robot.GetInverseKin(0, desc_p2, -1, j2);
        robot.GetInverseKin(0, desc_p3, -1, j3);
        robot.GetInverseKin(0, desc_p4, -1, j4);
        ExaxisPos epos = new ExaxisPos();
        DescPose offset_pos = new DescPose();

        JointPos JP1=new JointPos(117.408,-86.777,81.499,-87.788,-92.964,92.959);
        DescPose DP1 =new DescPose(327.359,-420.973,518.377,-177.199,3.209,114.449);
        JointPos JP2=new JointPos(72.515,-86.774,81.525,-87.724,-91.964,92.958);
        DescPose DP2=new DescPose(-63.512,-529.698,517.946,-178.192,3.07,69.554);
//        JointPos JP3=new JointPos(89.281,-102.959,81.527,-69.955,-86.755,92.958);
//        DescPose DP3=new DescPose();
//        robot.GetForwardKin(JP3,DP3);

        robot.MoveJ(JP1, DP1,0, 0, 30, 30, 100, epos, -1, 0, offset_pos);//关节空间运动
        robot.MoveJ(JP2, DP2,0, 0, 30, 100, 100, epos, -1, 0, offset_pos);//关节空间运动
//        robot.MoveL(JP2, DP2,0, 0, 30, 100, 100, -1, epos, 0, 0, offset_pos, 0, 100);//直线运动
//        robot.MoveCart(DP2, 0, 0, 30.0, 100.0, 100.0, -1.0, -1);
//        robot.MoveC(JP3, DP3, 0, 0, 30, 100, epos, 0, offset_pos, JP1, DP1, 0, 0, 100, 100, epos, 0, offset_pos, 100, -1);
//        robot.Circle(JP3, DP3, 0, 0, 10, 100.0, epos, JP2, DP2, 0, 0, 100.0, 100.0, epos, 100.0, 0, offset_pos);



//        robot.MoveL(j2, desc_p2,0, 0, 100, 100, 100, -1, epos, 0, 0, offset_pos, 0, 100);//直线运动
//        robot.MoveC(j1, desc_p1, 0, 0, 100, 100, epos, 0, offset_pos, j2, desc_p2, 0, 0, 100, 100, epos, 0, offset_pos, 100, -1);
//        robot.Circle(j1, desc_p1, 0, 0, 100.0, 100.0, epos, j2, desc_p2, 0, 0, 100.0, 100.0, epos, 100.0, 0, offset_pos);
//        robot.MoveCart(desc_p3, 0, 0, 100.0, 100.0, 100.0, -1.0, -1);
//        robot.StartJOG(0, 1, 0, 30, 100, 90);//关节点动
        //robot.Sleep(3000);
//        robot.StopJOG(1);//点动停止
//        robot.ImmStopJOG();//点动立即停止
//        SpiralParam param = new SpiralParam(5,10.0,30.0,10.0,5.0,0);//螺旋线
//        robot.NewSpiral(JP1, DP1, 0, 0, 50, 100, epos, 100, 0, offset_pos, param);

//        JointPos j5 = new JointPos();
//        ExaxisPos ePos=new ExaxisPos();
//        int ret = robot.GetActualJointPosDegree(j5);
//        if (ret == 0)
//        {
//            int count = 200;
//            while (count > 0)
//            {
//                robot.ServoJ(j5, ePos,100, 100, 0.008, 0, 0);
//                j5.J1 += 0.2;//1关节位置增加
//                count -= 1;
//                robot.WaitMs((int)(8));
//            }
//        }

//*********************************ServoCart begin******************************
//        DescPose desc_pos_dt = new DescPose(0, 0, 0, 0, 0, 0);
//        desc_pos_dt.tran.z = -0.5;
//        Object[] pos_gain = { 0.0, 0.0, 1.0, 0.0, 0.0, 0.0 };//仅z轴增加
//        int mode = 2;//工具坐标系下增量运动
//        float vel = 0.0f;
//        float acc = 0.0f;
//        float cmdT = 0.008f;
//        float filterT = 0.0f;
//        float gain = 0.0f;
//        int count = 200;
//
//        robot.SetSpeed(20);
//
//        while (count > 0)
//        {
//            robot.ServoCart(mode, desc_pos_dt, pos_gain, acc, vel, cmdT, filterT, gain);
//            count -= 1;
//            robot.WaitMs((int)(cmdT * 1000));
//        }
        //*********************************ServoCart end******************************


//*********************************Spline begin******************************
//        robot.MoveJ(j1, desc_p1,4, 0, 100, 100, 100, epos, -1, 0, offset_pos);
//        robot.SplineStart();
//        robot.SplinePTP(j4, desc_p4, 0, 0, 100, 100, 100);
//        robot.SplinePTP(j1, desc_p1, 0, 0, 100, 100, 100);
//        robot.SplinePTP(j2, desc_p2, 0, 0, 100, 100, 100);
//        robot.SplinePTP(j3, desc_p3, 0, 0, 100, 100, 100);
//
//        robot.SplinePTP(JP1, DP1, 0, 0, 50, 100, 100);
//        robot.SplineEnd();
//*********************************Spline end******************************

//*********************************New Spline begin******************************
//        DescPose desc_p1 =new DescPose(-104.846, 309.573, 336.647, 179.681, -0.419, -92.692);
//        DescPose desc_p2=new DescPose(-194.846, 309.573, 336.647, 179.681,-0.419, -92.692;);
//        DescPose desc_p3=new DescPose(-254.846, 259.573,336.647, 179.681, -0.419, -92.692;);
//        DescPose desc_p4=new DescPose(-304.846,259.573, 336.647, 179.681, -0.419, -92.692;);
//        JointPos j1 = new JointPos();
//        JointPos j2 = new JointPos();
//        JointPos j3 = new JointPos();
//        JointPos j4 = new JointPos();
//        robot.GetInverseKin(0, desc_p1, -1, j1);//逆向运动学求解
//        robot.GetInverseKin(0, desc_p2, -1, j2);
//        robot.GetInverseKin(0, desc_p3, -1, j3);
//        robot.GetInverseKin(0, desc_p4, -1, j4);
//        robot.MoveCart(desc_p1, 0, 0, 100.0, 100.0, 100.0, -1.0, -1);
//        robot.NewSplineStart(0, 5000);//新样条开始
//        robot.NewSplinePoint(j1, desc_p1, 0, 0, 100, 100, 100, 50, 0);//新样条指令点
//        robot.NewSplinePoint(j2, desc_p2, 0, 0, 100, 100, 100, 50, 0);
//        robot.NewSplinePoint(j3, desc_p3, 0, 0, 100, 100, 100, 50, 0);
//        robot.NewSplinePoint(j4, desc_p4, 0, 0, 100, 100, 100, 50, 1);
//        robot.NewSplineEnd();//新样条结束
//*********************************New Spline end******************************

//*********************************点位偏移 begin******************************
//        robot.MoveL(j1, desc_p1,0, 0, 100, 100, 100, -1, epos, 0, 0, offset_pos, 0, 100);
//        robot.MoveL(j2, desc_p2,0, 0, 100, 100, 100, -1, epos, 0, 0, offset_pos, 0, 100);
//
//        DescPose off = new DescPose(0, 0, 100, 0, 0, 0);
//        robot.PointsOffsetEnable(0, off);
//        robot.MoveL(j1, desc_p1,0, 0, 100, 100, 100, -1, epos, 0, 0, offset_pos, 0, 100);
//        robot.MoveL(j2, desc_p2,0, 0, 100, 100, 100, -1, epos, 0, 0, offset_pos, 0, 100);
//        robot.PointsOffsetDisable();
//
//        robot.MoveL(JP1, DP1,0, 0, 100, 100, 100, -1, epos, 0, 0, offset_pos, 0, 100);
//        robot.MoveL(JP2, DP2,0, 0, 100, 100, 100, -1, epos, 0, 0, offset_pos, 0, 100);
//
//        DescPose off = new DescPose(0, 0, 50, 0, 0, 0);
//        robot.PointsOffsetEnable(0, off);
//        robot.MoveL(JP1, DP1,0, 0, 100, 100, 100, -1, epos, 0, 0, offset_pos, 0, 100);
//        robot.MoveL(JP2, DP2,0, 0, 100, 100, 100, -1, epos, 0, 0, offset_pos, 0, 100);
//        robot.PointsOffsetDisable();
//*********************************点位偏移 end******************************


//        desc_p1.tran.x = 279.039;
//        desc_p1.tran.y = 195.765;
//        desc_p1.tran.z = 274.245;
//        desc_p1.rpy.rx = 274.245;
//        desc_p1.rpy.ry = -1.997;
//        desc_p1.rpy.rz = -165.405;
//
//        desc_p2.tran.x = -316.395;
//        desc_p2.tran.y = 126.269;
//        desc_p2.tran.z = 299.333;
//        desc_p2.rpy.rx = 179.649;
//        desc_p2.rpy.ry = 2.389;
//        desc_p2.rpy.rz = -42.183;
//
//        robot.GetInverseKin(0, desc_p1, -1, j1);
//        robot.GetInverseKin(0, desc_p2, -1, j2);
//


//*********************************超速保护 begin******************************
        //robot.MoveL(j1, desc_p1,0, 0, 100, 100, 100, -1, epos, 0, 0, offset_pos, 3, 100);
        //robot.MoveL(j2, desc_p2,0, 0, 100, 100, 100, -1, epos, 0, 0, offset_pos, 3, 100);

        //测试
        //robot.MoveL(JP1, DP1,0, 0, 100, 100, 100, -1, epos, 0, 0, offset_pos, 3, 100);
        //robot.MoveL(JP2, DP2,0, 0, 100, 100, 100, -1, epos, 0, 0, offset_pos, 3, 100);
//*********************************超速保护 end******************************

    }

    private static void Standard(Robot robot)
    {
//******************************获取版本号和ip begin ***************************
        String[] ip={""};
        String version = "";
        byte state = 0;

        version=robot.GetSDKVersion();
        System.out.println("SDK version : " + version);
        int rtn = robot.GetControllerIP(ip);
        System.out.println("controller ip : " +  ip[0] + "  " + rtn);
//******************************获取版本号和ip end ***************************

        robot.Mode(1);//1-手动模式  0-自动模式
        robot.Sleep(1000);
        robot.DragTeachSwitch(1);//进入拖动模式
        robot.Sleep(1000);
        ROBOT_STATE_PKG pkg = robot.GetRobotRealTimeState();
        System.out.println("drag state : " + pkg.robot_state);
        robot.Sleep(1000);
        robot.DragTeachSwitch(0);//退出拖动模式
        robot.Sleep(1000);
//
//
        pkg = robot.GetRobotRealTimeState();
        System.out.println("drag state : " + pkg.robot_state);
       if (pkg.robot_state ==4){
           System.out.println("拖动模式");
       }else {
           System.out.println("非拖动模式");
       }

//使能和手动、自动切换
//        robot.RobotEnable(0);//下使能
//        robot.Sleep(1000);
//        robot.RobotEnable(1);//上使能
//        robot.Sleep(1000);
//        robot.RobotEnable(0);
//        robot.Sleep(1000);
//        robot.RobotEnable(1);
//        robot.Sleep(1000);
//
//        robot.Mode(0);//手动、自动状态切换 0-自动
//        robot.Sleep(1000);
//        robot.Mode(1);
//        robot.Sleep(1000);
//        robot.Mode(0);
//        robot.Sleep(1000);
//        robot.Mode(1);
//        robot.Sleep(1000);
    }

    private static void ForceSensor(Robot robot)
    {
//**********************************获取力传感器配置接口 begin ******************************
//        DeviceConfig config = new DeviceConfig();
//        config.company = 24;
//        config.device = 0;
//        config.softwareVersion = 0;
//        config.bus = 0;
//
//        robot.FT_SetConfig(config);
//        robot.Sleep(1000);
//        config.company = 0;
//        robot.FT_GetConfig(config);
//        System.out.println("FT config : " + config.company + ", " + config.device + ", " + config.softwareVersion + ", " + config.bus);
//**********************************获取力传感器配置接口 end ******************************

//**********************************力传感器激活与零点矫正 begin ******************************
//        robot.FT_Activate(0);  //复位
//        robot.Sleep(2000);
//
//        robot.FT_Activate(1);  //激活
//        robot.Sleep(2000);
//
//        robot.FT_SetZero(0);//0去除零点
//        robot.Sleep(2000);
//
//        robot.FT_SetZero(1);//1零点矫正
//**********************************力传感器激活与零点矫正 end ******************************

//**********************************力传感器负载辨识 begin ******************************
//        double weight = 0.1;
//        int rtn = -1;
//
//        DescPose tcoord, desc_p1, desc_p2, desc_p3;
//        tcoord = new DescPose(0, 0, 0, 0, 0, 0);
//        desc_p1 = new DescPose(-14.404,-455.283,319.847,-172.935,25.141,-68.097);
//        desc_p2 = new DescPose(-107.999,-599.174,285.939,153.472,12.686,-71.284);
//        desc_p3 = new DescPose(6.586,-704.897,309.638,178.909,-27.759,-70.479);
//
//        DescPose coord = new DescPose(0, 0 ,0, 1, 0, 0);
//        robot.FT_SetRCS(0, coord);
//        robot.Sleep(1000);
//
//        tcoord.tran.z = 35.0;
//        robot.SetToolCoord(8, tcoord, 1, 0);
//        robot.Sleep(1000);
//        robot.FT_PdIdenRecord(10);
//        robot.Sleep(1000);
//        List<Number> rtnArray =  robot.FT_PdIdenCompute();
//        System.out.println("payload weight : " + rtnArray.get(1));
//
//
//        rtn = robot.MoveCart(desc_p1, 0, 0, 20.0f, 100.0f, 100.0f, -1.0f, -1);
//        robot.Sleep(1000);
//        robot.FT_PdCogIdenRecord(2, 1);
//        robot.MoveCart(desc_p2, 0, 0, 20.0f, 100.0f, 100.0f, -1.0f, -1);
//        robot.Sleep(1000);
//        robot.FT_PdCogIdenRecord(2, 2);
//        robot.MoveCart(desc_p3, 0, 0, 20.0f, 100.0f, 100.0f, -1.0f, -1);
//        robot.Sleep(1000);
//        robot.FT_PdCogIdenRecord(2, 3);
//        robot.Sleep(1000);
//        //DescTran cog = new DescTran(0, 0, 0);
//
//        DescTran rtnCog = new DescTran();
//        robot.FT_PdCogIdenCompute(rtnCog);
//        System.out.println("cog : " + rtnCog.x + ", " + rtnCog.y + ", " + rtnCog.z);
//**********************************力传感器负载辨识 end ******************************


//**********************************恒力控制 begin ******************************
//        byte flag = 1;
//        byte sensor_id = 8;
//        Object[] select = { 0,0,1,0,0,0 };
//        Object[] ft_pid = { 0.0005, 0.0, 0.0, 0.0, 0.0, 0.0 };
//        byte adj_sign = 0;
//        byte ILC_sign = 0;
//        float max_dis = 100.0f;
//        float max_ang = 0.0f;
//        ForceTorque ft = new ForceTorque(0, 0, -10, 0 ,0 ,0);
//
//
//        JointPos j1=new JointPos(-21.724,-136.814,-59.518,-68.853,89.245,-66.35);
//        DescPose desc_p1 = new DescPose(703.996,-391.695,240.708,-178.756,-4.709,-45.447);
//
//        JointPos j2=new JointPos(0.079,-130.285,-71.029,-72.115,88.945,-62.736);
//        DescPose desc_p2 = new DescPose(738.755,-102.812,226.704,177.488,2.566,-27.209);
//
//        ExaxisPos epos = new ExaxisPos(0, 0, 0, 0);
//
//        DescPose offset_pos = new DescPose(0, 0, 0, 0, 0, 0);
//
//
//        //关节空间运动
//        robot.MoveL(j1, desc_p1, 0, 0, 40.0f, 180.0f, 20.0f, -1.0f, epos, 0, 0, offset_pos, 0, 100);
//        int rtn = robot.FT_Control(flag, sensor_id, select, ft, ft_pid, adj_sign, ILC_sign, max_dis, max_ang, 0, 0, 0);
//        System.out.println("FT_Control start rtn " + rtn);
//
//        robot.MoveL(j2, desc_p2, 0, 0, 10.0f, 180.0f, 20.0f, -1.0f, epos, 0, 0, offset_pos, 0, 100);
//        flag = 0;
//        rtn = robot.FT_Control(flag, sensor_id, select, ft, ft_pid, adj_sign, ILC_sign, max_dis, max_ang, 0, 0 ,0);
//        System.out.println("FT_Control end rtn " + rtn);
//**********************************恒力控制 end ******************************

//**********************************碰撞守护 begin ******************************
        byte flag = 1;
        byte sensor_id = 2;
        Object[] select = { 0, 0, 1, 0, 0, 0 };//只启用x轴碰撞守护
        Object[] max_threshold = { 0.01, 0.01, 5.01, 0.01, 0.01, 0.01 };
        Object[] min_threshold = { 0.01, 0.01, 5.01, 0.01, 0.01, 0.01 };

        ForceTorque ft = new ForceTorque(1.0, 0.0, 2.0, 0.0, 0.0, 0.0);
        DescPose  desc_p1, desc_p2, desc_p3;

        desc_p1 = new DescPose(-280.5,-474.534,320.677,177.986,1.498,-118.235);
        desc_p2 = new DescPose(-283.273,-468.668,172.905,177.986,1.498,-118.235);

        int[] safetyMargin={5,5,5,5,5,5};
        robot.SetCollisionStrategy(5,1000,150,150,safetyMargin);
        int rtn =  robot.FT_Guard(flag, sensor_id, select, ft, max_threshold, min_threshold);
        System.out.println("FT_Guard start rtn "+rtn);
        robot.MoveCart(desc_p1, 0, 0, 20, 100.0f, 100.0f, -1.0f, -1);
        robot.MoveCart(desc_p2, 0, 0, 20, 100.0f, 100.0f, -1.0f, -1);
        flag = 0;
        rtn = robot.FT_Guard(flag, sensor_id, select, ft, max_threshold, min_threshold);
        System.out.println("FT_Guard end rtn "+rtn);
//**********************************碰撞守护 end ******************************

//**********************************柔顺控制 begin ******************************
//        byte flag = 1;
//        int sensor_id = 8;
//        Object[] select = { 1, 1, 1, 0, 0, 0 };
//        Object[] ft_pid = { 0.0005, 0.0, 0.0, 0.0, 0.0, 0.0 };
//        int adj_sign = 0;
//        int ILC_sign = 0;
//        double max_dis = 100.0;
//        double max_ang = 0.0;
//
//        ForceTorque ft = new ForceTorque(-10.0, -10.0, -10.0, 0.0, 0.0, 0.0);
//        DescPose desc_p1, desc_p2, offset_pos;
//        JointPos j1, j2;
//        j1=new JointPos(-21.724, -136.814, -59.518, -68.853, 89.245, -66.359);
//        j2=new JointPos(0.079, -130.285, -71.029, -72.115, 88.945, -62.736);
//
//        ExaxisPos epos = new ExaxisPos(0, 0, 0, 0);
//        desc_p1 = new DescPose(703.996, -391.695, 240.708, -178.756, -4.709, -45.447);
//        desc_p2 = new DescPose(738.755, -102.812, 226.704, 177.488, 2.566, -27.209);
//        offset_pos = new DescPose(0, 0, 0, 0, 0, 0);
//
//
//        ft.fx = -10.0;
//        ft.fy = -10.0;
//        ft.fz = -10.0;
//        robot.FT_Control(flag, sensor_id, select, ft, ft_pid, adj_sign, ILC_sign, max_dis, max_ang,0,0,0);
//        float p = 0.00005f;
//        float force = 10.0f;
//        int rtn = robot.FT_ComplianceStart(p, force);
//        System.out.println("FT_ComplianceStart rtn " + rtn);
//
//        robot.MoveL(j1, desc_p1, 0, 0, 20.0, 180.0, 100.0, -1.0, epos, 0, 1, offset_pos, 0, 100);
//
//        rtn = robot.FT_ComplianceStop();
//        System.out.println("FT_ComplianceStop rtn " + rtn);
//        flag = 0;
//        robot.FT_Control(flag, sensor_id, select, ft, ft_pid, adj_sign, ILC_sign, max_dis, max_ang,0,0,0);
//**********************************柔顺控制 end ******************************

//**********************************力传感器辅助拖动、状态获取 begin ******************************
//        List<Integer> rtnArray = robot.GetForceAndTorqueDragState();
//        System.out.println("the drag state is  " + rtnArray.get(1) + "  ForceAndJointImpedance state  " + rtnArray.get(2));
//
//        robot.Sleep(1000);
//        Object[] M = { 15.0, 15.0, 15.0, 0.5, 0.5, 0.1 };
//        Object[] B = { 150.0, 150.0, 150.0, 5.0, 5.0, 1.0 };
//        Object[] K = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
//        Object[] F = { 10.0, 10.0, 10.0, 1.0, 1.0, 1.0 };
//        robot.EndForceDragControl(1, 0, 0, M, B, K, F, 50, 100);
//
//        rtnArray = robot.GetForceAndTorqueDragState();
//        System.out.println("the drag state is" + rtnArray.get(1) + "  ForceAndJointImpedance state  " + rtnArray.get(2));
//
//        robot.Sleep(1000 * 10);
//        robot.EndForceDragControl(0, 0, 0, M, B, K, F, 50, 100);
//
//        rtnArray = robot.GetForceAndTorqueDragState();
//        System.out.println("the drag state is" + rtnArray.get(1) + "  ForceAndJointImpedance state  " + rtnArray.get(2));
//        robot.Sleep(1000);
//**********************************力传感器辅助拖动、状态获取 end ******************************

//**********************************六维力混合拖动 begin ******************************
//        robot.DragTeachSwitch(1);
//        Object[] lamdeDain = { 3.0, 2.0, 2.0, 2.0, 2.0, 3.0 };
//        Object[] KGain = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
//        Object[] BGain = { 150.0, 150.0, 150.0, 5.0, 5.0, 1.0 };
//        robot.ForceAndJointImpedanceStartStop(1, 0, lamdeDain, KGain, BGain, 1000.0, 180.0);
//
//        List<Integer> rtnArray = robot.GetForceAndTorqueDragState();
//        System.out.println("the drag state is  " + rtnArray.get(1) + "  ForceAndJointImpedance state  " + rtnArray.get(2));
//
//        robot.Sleep(1000 * 10);
//        robot.DragTeachSwitch(0);
//
//        robot.ForceAndJointImpedanceStartStop(0, 0, lamdeDain, KGain, BGain, 1000.0, 180.0);
//
//        rtnArray = robot.GetForceAndTorqueDragState();
//        System.out.println("the drag state is  " + rtnArray.get(1) + "  ForceAndJointImpedance state  " + rtnArray.get(2));
//**********************************六维力混合拖动 end ******************************


//**********************************力传感器负载设置与获取 begin ******************************
//        robot.SetForceSensorPayLoad(1.34);
//        DescTran cog = new DescTran(0.778, 2.554, 48.765);
//        robot.SetForceSensorPayLoadCog(cog);
//        double weight = 0;
//
//        List<Number> rtnArrays = robot.GetForceSensorPayLoad();
//        DescTran getCog = new DescTran(0.0, 0.0, 0.0);
//        robot.GetForceSensorPayLoadCog(getCog);
//        System.out.println("the FT load is " +  rtnArrays.get(1) + "  cog is  " + getCog.x + "  " + getCog.y + "   " + getCog.z);
//**********************************力传感器负载设置与获取 end ******************************

//**********************************力传感器负载自动校零 begin ******************************
//        robot.SetForceSensorPayLoad(0.0);
//        DescTran zeroCog = new DescTran();
//        robot.SetForceSensorPayLoadCog(zeroCog);
//
//        MassCenter center = new MassCenter();
//        robot.ForceSensorAutoComputeLoad(center);
//        System.out.println("the result is weight " + center.weight + " pos is  " + center.cog.x + "  " + center.cog.y + "  " + center.cog.z);
//**********************************力传感器负载自动校零 end ******************************

//
//        robot.SetLoadWeight(0.0f);
//        robot.Sleep(1000);
//        DescTran coord = new DescTran(0, 0, 0);
//
//        robot.SetLoadCoord(coord);
//        robot.Sleep(1000);
//        robot.FT_SetZero(0);//0去除零点  1零点矫正
//        robot.Sleep(1000);
//
//        ForceTorque ft = new ForceTorque(0, 0, 0, 0, 0, 0);
//        int rtn = robot.FT_GetForceTorqueOrigin(1, ref ft);
//         System.out.println("ft origin : {ft.fx}, {ft.fy}, { ft.fz}, { ft.tx}, { ft.ty}, { ft.tz}    rtn   {rtn}");
//        rtn = robot.FT_SetZero(1);//零点矫正
//        // System.out.println("set zero rtn {rtn}");
//
//        robot.Sleep(2000);
//        rtn = robot.FT_GetForceTorqueOrigin(1,ft);
//         System.out.println("ft rcs : {ft.fx}, {ft.fy}, {ft.fz}, {ft.tx}, {ft.ty}, {ft.tz}  rtn  {rtn}");

//        robot.FT_GetForceTorqueRCS(1, ft);
//         System.out.println("FT_GetForceTorqueRCS rcs : {ft.fx}, {ft.fy}, {ft.fz}, {ft.tx}, {ft.ty}, {ft.tz}");

//        double weight = 0.1;
//        int rtn = -1;
//        DescPose tcoord, desc_p1, desc_p2, desc_p3;
//        tcoord = new DescPose(0, 0, 0, 0, 0, 0);
//        desc_p1 = new DescPose(0, 0, 0, 0, 0, 0);
//        desc_p2 = new DescPose(0, 0, 0, 0, 0, 0);
//        desc_p3 = new DescPose(0, 0, 0, 0, 0, 0);
//
//        robot.FT_SetRCS(0);
//        robot.Sleep(1000);
//
//        tcoord.tran.z = 35.0;
//        robot.SetToolCoord(10, tcoord, 1, 0);
//        robot.Sleep(1000);
//        robot.FT_PdIdenRecord(10);
//        robot.Sleep(1000);
//        robot.FT_PdIdenCompute();
//        System.out.println("payload weight : {weight}");
//
//        desc_p1.tran.x = -47.805;
//        desc_p1.tran.y = -362.266;
//        desc_p1.tran.z = 317.754;
//        desc_p1.rpy.rx = -179.496;
//        desc_p1.rpy.ry = -0.255;
//        desc_p1.rpy.rz = 34.948;
//
//        desc_p2.tran.x = -77.805;
//        desc_p2.tran.y = -312.266;
//        desc_p2.tran.z = 317.754;
//        desc_p2.rpy.rx = -179.496;
//        desc_p2.rpy.ry = -0.255;
//        desc_p2.rpy.rz = 34.948;
//
//        desc_p3.tran.x = -167.805;
//        desc_p3.tran.y = -312.266;
//        desc_p3.tran.z = 387.754;
//        desc_p3.rpy.rx = -179.496;
//        desc_p3.rpy.ry = -0.255;
//        desc_p3.rpy.rz = 34.948;
//
//        rtn = robot.MoveCart(desc_p1, 0, 0, 100.0f, 100.0f, 100.0f, -1.0f, -1);
//        System.out.println("MoveCart rtn  " + rtn);
//        robot.Sleep(1000);
//        robot.FT_PdCogIdenRecord(10, 1);
//        robot.MoveCart(desc_p2, 0, 0, 100.0f, 100.0f, 100.0f, -1.0f, -1);
//        robot.Sleep(1000);
//        robot.FT_PdCogIdenRecord(10, 2);
//        robot.MoveCart(desc_p3, 0, 0, 100.0f, 100.0f, 100.0f, -1.0f, -1);
//        robot.Sleep(1000);
//        robot.FT_PdCogIdenRecord(10, 3);
//        robot.Sleep(1000);
//
//        DescTran cog = new DescTran(0, 0, 0);
//        robot.FT_PdCogIdenCompute(cog);
//        System.out.println("cog : " + cog.x + ", " + cog.y + ", " + cog.z);
    }

    static void DOReset(Robot robot)
    {
//*********************************DO/AO输出口复位状态 begin ***********************************
//        robot.SetDO(0,1,0,0);
//        robot.SetToolDO(0,1,0,0);
        robot.SetAO(0,60,0);
        robot.SetToolAO(0,60,0);
        robot.Sleep(2000);

        robot.SetOutputResetCtlBoxDO(1);
        robot.SetOutputResetAxleDO(1);//工具
        robot.SetOutputResetCtlBoxAO(1);
        robot.SetOutputResetAxleAO(1);//工具

        JointPos j1 = new JointPos(-81.684,-106.159,-74.447,-86.33,94.725,41.639);
        JointPos j2 = new JointPos(-102.804,-106.159,-74.449,-86.328,94.715,41.639);
        DescPose  desc_p1, desc_p2;

        desc_p1 = new DescPose(0, 0, 0, 0, 0, 0);
        desc_p2 = new DescPose(0, 0, 0, 0, 0, 0);
        robot.GetForwardKin(j1,desc_p1);
        robot.GetForwardKin(j2,desc_p2);

        ExaxisPos epos = new ExaxisPos();
        DescPose offset_pos = new DescPose();
        robot.MoveL(j1, desc_p1,0, 0, 30, 100, 100, -1, epos, 0, 0, offset_pos, 0, 100);

//        robot.MoveToolAOStop();
//        robot.MoveAOStop();

//*********************************DO/AO输出口复位状态 end ***********************************


//********************************扩展AO/DO begin ***********************************
//        UDPComParam param = new UDPComParam("192.168.58.88", 2021, 2, 100, 3, 100, 1, 100, 10);
//        robot.ExtDevSetUDPComParam(param);
//
//        UDPComParam getParam = new UDPComParam();
//        robot.ExtDevGetUDPComParam(getParam);
//        System.out.println(" " + getParam.ip + " ,   " + getParam.port + " ,   " + getParam.period + " ,  " + getParam.lossPkgTime + " ,   " + getParam.lossPkgNum + " ,   " + getParam.disconnectTime + " ,   " + getParam.reconnectEnable + " ,   " + getParam.reconnectPeriod + " ,   " + getParam.reconnectNum);
//
//        robot.ExtDevUnloadUDPDriver();
//        robot.Sleep(1000);
//        robot.ExtDevLoadUDPDriver();
//        robot.Sleep(2000);
//
//        robot.SetAuxDO(0, true, false, false);
//        robot.SetAuxDO(1, true, false, false);
//        robot.SetAuxDO(2, true, false, false);
//        robot.SetAuxDO(3, true, false, false);
//
        robot.SetAuxAO(0, 1234, false);
        robot.SetAuxAO(1, 2345, false);
        robot.SetAuxAO(2, 3456, false);
        robot.SetAuxAO(3, 1111, false);
        robot.SetOutputResetExtDO(1);
        robot.SetOutputResetExtAO(1);
//
//        robot.Sleep(3000);
//        DescPose  desc_p1, desc_p2;
//
//        desc_p1 = new DescPose(0, 0, 0, 0, 0, 0);
//        desc_p2 = new DescPose(0, 0, 0, 0, 0, 0);
//
//        JointPos j1 = new JointPos(-81.684,-106.159,-74.447,-86.33,94.725,41.639);
//        JointPos j2 = new JointPos(-102.804,-106.159,-74.449,-86.328,94.715,41.639);
//
//        robot.GetForwardKin(j1,desc_p1);
//        robot.GetForwardKin(j2,desc_p2);
//
//        ExaxisPos epos = new ExaxisPos();
//        DescPose offset_pos = new DescPose();
//        robot.MoveL(j1, desc_p1,0, 0, 30, 100, 100, -1, epos, 0, 0, offset_pos, 0, 100);
//        robot.MoveL(j2, desc_p2,0, 0, 30, 100, 100, -1, epos, 0, 0, offset_pos, 0, 100);
//********************************扩展AO/DO end ***********************************

//        robot.ProgramLoad("/fruser/test.lua");
//        robot.ProgramRun();
    }


    private static void UDPAxisSyncMove(Robot robot)
    {
//        robot.Mode(0);
//        int tool = 1;
//        int user = 0;
//        double vel = 20.0;
//        double acc = 100.0;
//        double ovl = 100.0;
//        float blendT = -1;
//        float blendR = -1;
//        int flag = 0;
//        int type = 1;
//
//        ExaxisPos exaxisPos = new ExaxisPos( 0, 0, 0, 0 );
//
//        DescPose d0 = new DescPose(311.189, -309.688, 401.836, -174.375, -1.409, -82.354);
//        JointPos j0 =new JointPos(118.217, -99.669, 79.928, -73.559, -85.229, -69.359);
//
//        JointPos joint_pos0 =new JointPos(111.549,-99.821,108.707,-99.308,-85.305,-41.515);
//        DescPose desc_pos0 = new DescPose(273.499,-345.746,201.573,-176.566 ,3.235,-116.819);
//        ExaxisPos e_pos0=new ExaxisPos(0,0,0,0);
//
//        JointPos joint_pos1 = new JointPos(112.395,-65.118,67.815,-61.449,-88.669,-41.517);
//        DescPose desc_pos1 = new DescPose(291.393,-420.519,201.089,156.297,21.019,-120.919);
//        ExaxisPos e_pos1 = new ExaxisPos(-30, -30, 0, 0);
//
//
//        JointPos j2 = new JointPos(111.549,-98.369,108.036,-103.789,-95.203,-69.358);
//        DescPose desc_pos2 = new DescPose(234.544,-392.777,205.566,176.584,-5.694,-89.109);
//        ExaxisPos epos2 = new ExaxisPos(0.000,0.000,0.000,0.000);
//
//        JointPos j3 = new JointPos(113.908,-61.947,63.829,-64.478,-85.406,-69.256);
//        DescPose desc_pos3 = new DescPose(336.049,-444.969,192.799,173.776 ,27.104,-89.455);
//        ExaxisPos epos3 = new ExaxisPos(-30.000,-30.000, 0.000, 0.000);
//
//        //圆弧的起点
//        JointPos j4 = new JointPos(137.204,-98.475,106.624,-97.769,-90.634,-69.24);
//        DescPose desc_pos4 = new DescPose(381.269,-218.688,205.735,179.274,0.128,-63.556);
//
//        JointPos j5 = new JointPos(115.069,-92.709,97.285,-82.809,-90.455,-77.146);
//        DescPose desc_pos5 = new DescPose(264.049,-329.478 ,220.747,176.906,11.359,-78.044);
//        ExaxisPos  epos5 = new ExaxisPos(-15, 0, 0, 0);
//
//
//        JointPos j6 = new JointPos(102.409,-63.115,70.559,-70.156,-86.529,-77.148);
//        DescPose desc_pos6 = new DescPose(232.407,-494.228 ,158.115,176.803,27.319,-92.056);
//        ExaxisPos  epos6 = new ExaxisPos(-30, 0, 0, 0);
//
//        DescPose offset_pos = new DescPose(0, 0, 0, 0, 0, 0);
//
//        //同步关节运动
//        robot.MoveJ(j0, d0, 1, 0, vel, acc, ovl, exaxisPos, -1, 0, offset_pos);
//        robot.ExtAxisMove(exaxisPos,40);
//        robot.ExtAxisSyncMoveJ(joint_pos0, desc_pos0, 1, 0,20,100,100,e_pos0,-1,0,offset_pos);
//        robot.ExtAxisSyncMoveJ(joint_pos1, desc_pos1, 1, 0,20,100,100,e_pos1,-1,0,offset_pos);
//
//
//        //同步直线运动
//        robot.MoveJ(j0, d0, 1, 0, vel, acc, ovl, exaxisPos, -1, 0, offset_pos);
//        robot.ExtAxisMove(exaxisPos,40);
//        robot.ExtAxisSyncMoveL(j2, desc_pos2, tool, user, 40, 100, 100, -1, epos2, 0, offset_pos);
//        robot.ExtAxisSyncMoveL(j3, desc_pos3, tool, user, 40, 100, 100, -1, epos3, 0, offset_pos);
////
////        //同步圆弧运动
//        robot.MoveJ(j0, d0, 1, 0, vel, acc, ovl, exaxisPos, -1, 0, offset_pos);
//        robot.ExtAxisMove(exaxisPos,20);
//        robot.MoveJ(j4, desc_pos4, 1, 0, vel, acc, ovl, exaxisPos, -1, 0, offset_pos);
//
//        robot.ExtAxisSyncMoveC(j5, desc_pos5, tool, user, 40, 100, epos5, 0, offset_pos, j6, desc_pos6, tool, user, 40, 100, epos6, 0, offset_pos, 100, 0);
//
//        robot.Sleep(3000);
//        robot.MoveJ(j0, d0, 1, 0, vel, acc, ovl, exaxisPos, -1, 0, offset_pos);
//        robot.ExtAxisMove(exaxisPos,40);
//        robot.Mode(1);
    }

    private static void UDPAxis(Robot robot)
    {
//***********************************************UPD扩展轴参数配置与获取 begin**************************************
        UDPComParam param = new UDPComParam("192.168.58.88", 2021, 2, 100, 3, 100, 1, 100, 10);
        robot.ExtDevSetUDPComParam(param);//udp扩展轴通讯

        UDPComParam getParam = new UDPComParam();
        robot.ExtDevGetUDPComParam(getParam);
        System.out.println(" " + getParam.ip + " ,   " + getParam.port + " ,   " + getParam.period + " ,  " + getParam.lossPkgTime + " ,   " + getParam.lossPkgNum + " ,   " + getParam.disconnectTime + " ,   " + getParam.reconnectEnable + " ,   " + getParam.reconnectPeriod + " ,   " + getParam.reconnectNum);
//***********************************************UPD扩展轴参数配置与获取 end **************************************

//***********************************************UPD加载与卸载 begin **************************************
        robot.ExtDevUnloadUDPDriver();//卸载UDP通信
        robot.Sleep(1000);
        robot.ExtDevLoadUDPDriver();//加载UDP通信
        robot.Sleep(1000);
//***********************************************UPD加载与卸载 end **************************************

//***********************************************UPD扩展轴按照位置、DH参数、轴参数设置 begin **************************************
//        robot.ExtAxisServoOn(1, 1);//扩展轴1使能
//        robot.ExtAxisServoOn(2, 1);//扩展轴2使能
//        robot.Sleep(1000);
//        robot.ExtAxisSetHoming(1, 0, 10, 3);//1,2扩展轴都回零
//        robot.ExtAxisSetHoming(2, 0, 10, 3);
//        robot.Sleep(1000);
//
//        int rtn = 0;
//        rtn = robot.SetAxisDHParaConfig(1, 128.5, 206.4, 0, 0, 0, 0, 0, 0);
//        System.out.println("SetAxisDHParaConfig rtn is " + rtn);
//        rtn = robot.SetRobotPosToAxis(1);
//        System.out.println("SetRobotPosToAxis rtn is " + rtn);
//        rtn = robot.ExtAxisParamConfig(1,1, 0, 50, -50, 1000, 1000, 1.905, 262144, 200, 0, 0, 0);
//        System.out.println("ExtAxisParamConfig rtn is " + rtn);
//        rtn = robot.ExtAxisParamConfig(2,2, 0, 1000, -1000, 1000, 1000, 4.444, 262144, 200, 0, 0, 0);
//        System.out.println("ExtAxisParamConfig rtn is " + rtn);
//***********************************************UPD扩展轴按照位置、DH参数、轴参数设置 end **************************************

//***********************************************UPD扩展轴坐标系标定 begin *****a*********************************
//        int rtn = 0;
//        rtn = robot.SetAxisDHParaConfig(1, 128.5, 206.4, 0, 0, 0, 0, 0, 0);
//        System.out.println("SetAxisDHParaConfig rtn is " + rtn);
//        rtn = robot.SetRobotPosToAxis(1);
//        System.out.println("SetRobotPosToAxis rtn is " + rtn);
//        rtn = robot.ExtAxisParamConfig(1,1, 0, 1000, -1000, 100, 100, 1.905, 262144, 200, 0, 0, 0);
//        System.out.println("ExtAxisParamConfig rtn is " + rtn);
//        rtn = robot.ExtAxisParamConfig(2,2, 0, 1000, -1000, 100, 100, 4.444, 262144, 200, 0, 0, 0);
//        System.out.println("ExtAxisParamConfig rtn is " + rtn);
//
//        int tool = 1;
//        int user = 0;
//        double vel = 20;
//        double acc = 100;
//        double ovl = 100;
//        int offset_flag = 0;
//
//        ExaxisPos exaxisPos = new ExaxisPos( 0, 0, 0, 0 );
//        DescPose offdese = new DescPose( 0, 0, 0, 0, 0, 0 );
//
//
//        DescPose descPose0 = new DescPose(311.189,-309.688,401.836,-174.375,-1.409,-82.354);
//        JointPos jointPos0 = new JointPos(118.217,-99.669,79.928,-73.559,-85.229,-69.359);
//        robot.MoveJ(jointPos0, descPose0, 1, 0, vel, acc, ovl, exaxisPos, -1, offset_flag, offdese);
//
//        robot.ExtAxisMove(exaxisPos, 20);//UDP扩展轴运动,速度100
//
//        DescPose descPose1 = new DescPose(359.526,-516.038,194.469,-175.689,2.781,-87.609);
//        JointPos jointPos1 = new JointPos(113.015,-72.49,80.079,-96.505,-84.986,-69.309);
//
//        ExaxisPos pos1 = new ExaxisPos(0.000,0.000,0.000,0.000);
//        robot.ExtAxisMove(pos1, 20);//UDP扩展轴运动,速度100
//        robot.MoveJ(jointPos1, descPose1, 1, 0, vel, acc, ovl, exaxisPos, -1, offset_flag, offdese);
//        robot.PositionorSetRefPoint(1);
//
//        robot.MoveJ(jointPos0, descPose0, 1, 0, vel, acc, ovl, exaxisPos, -1, offset_flag, offdese);
//
//        ExaxisPos pos2 = new ExaxisPos(-10.000, -10.000, 0.000, 0.000);
//        robot.ExtAxisMove(pos2, 20);
//        DescPose descPose2 = new DescPose( 333.347,-541.958,164.894,-176.47,4.284,-90.725);
//        JointPos jointPos2 = new JointPos(109.989,-69.637,80.146,-97.755,-85.188,-69.269);
//        robot.MoveJ(jointPos2, descPose2, 1, 0, vel, acc, ovl, exaxisPos, -1, offset_flag, offdese);
//        robot.PositionorSetRefPoint(2);
//
//        robot.MoveJ(jointPos0, descPose0, 1, 0, vel, acc, ovl, exaxisPos, -1, offset_flag, offdese);
//
//        ExaxisPos pos3 = new ExaxisPos(-20.000, -20.000, 0.000, 0.00);
//        robot.ExtAxisMove(pos3, 20);
//        DescPose descPose3 = new DescPose(306.488,-559.238,135.948,-174.925,0.235,-93.517);
//        JointPos jointPos3 = new JointPos(107.137,-71.377,87.975,-108.167,-85.169,-69.269);
//        robot.MoveJ(jointPos3, descPose3, 1, 0, vel, acc, ovl, exaxisPos, -1, offset_flag, offdese);
//        robot.PositionorSetRefPoint(3);
//
//        robot.MoveJ(jointPos0, descPose0, 1, 0, vel, acc, ovl, exaxisPos, -1, offset_flag, offdese);
//
//        ExaxisPos pos4 = new ExaxisPos(-30.000, -30.000, 0.000, 0.000);
//        robot.ExtAxisMove(pos4, 20);
//        DescPose descPose4 = new DescPose( 285.528,-569.999,108.568,-174.367,-1.239,-95.643);
//        JointPos jointPos4 = new JointPos(105.016,-71.137,92.326,-114.339,-85.169,-69.269);
//        robot.MoveJ(jointPos4, descPose4, 1, 0, vel, acc, ovl, exaxisPos, -1, offset_flag, offdese);
//        robot.PositionorSetRefPoint(4);
//
//        DescPose axisCoord = new DescPose();
        //robot.PositionorComputeECoordSys(axisCoord);
        //robot.ExtAxisActiveECoordSys(3,1,axisCoord,1);
        //System.out.println("axis coord is " + axisCoord);

//        robot.MoveJ(jointPos0, descPose0, 1, 0, vel, acc, ovl, exaxisPos, -1, offset_flag, offdese);
//        robot.ExtAxisMove(pos1, 20);
//***********************************************UPD扩展轴坐标系标定 end **************************************

//***********************************************UPD扩展轴异步运动 begin **************************************
//        int tool = 1;
//        int user = 0;
//        double vel = 20;
//        double acc = 100;
//        double ovl = 100;
//        int offset_flag = 0;
//        ExaxisPos exaxisPos = new ExaxisPos( 0, 0, 0, 0 );
//        DescPose offdese = new DescPose( 0, 0, 0, 0, 0, 0 );
//
//
//        DescPose descPose0 = new DescPose(311.189,-309.688,401.836,-174.375,-1.409,-82.354);
//        JointPos jointPos0 = new JointPos(118.217,-99.669,79.928,-73.559,-85.229,-69.359);
//        //robot.MoveJ(jointPos0, descPose0, 1, 0, vel, acc, ovl, exaxisPos, -1, offset_flag, offdese);
//
//        ExaxisPos pos = new ExaxisPos( 0, 0, 0, 0 );
//        robot.ExtAxisMove(pos,40);
//
//        pos.axis1 = 20;
//        pos.axis2 = 100;
//        robot.ExtAxisMove(pos, 40);
//
//        pos.axis1 = -20;
//        pos.axis2 = -100;
//        robot.ExtAxisMove(pos, 40);
        //        robot.ExtAxisMove(pos,40);
//***********************************************UPD扩展轴异步运动 end **************************************

//***********************************************UPD扩展轴DOI begin **************************************
//        for(int i = 0; i < 128; i++)
//        {
//            robot.SetAuxDO(i, true, false, true);
//            robot.Sleep(50);
//        }
//
//        for(int i = 0; i < 128; i++)
//        {
//            robot.SetAuxDO(i, false, false, true);
//            robot.Sleep(50);
//        }
//
//        for (int i = 0; i < 4; i ++)
//        {
//            robot.SetAuxAO(i, i * 1000, false);
//            robot.Sleep(1000);
//        }
//
//        for (int i = 0; i < 4; i ++)
//        {
//            robot.SetAuxAO(i, 0, false);
//            robot.Sleep(1000);
//        }
//
//        while (true)
//        {
//            ROBOT_STATE_PKG pkg = robot.GetRobotRealTimeState();
//            System.out.println("aux DI1 DI0  is " + Integer.toBinaryString(pkg.extDIState[0]));
//        }
//***********************************************UPD扩展轴DOI end **************************************


//        robot.ExtAxisActiveECoordSys(3, 3, axisCoord, 1);

//
//        DescPose refPointPos = new DescPose(122.0, 312.0, 0, 0, 0, 0);
//        robot.SetRefPointInExAxisEnd(refPointPos);
//
//        robot.PositionorSetRefPoint(1);
//        //robot.PositionorSetRefPoint(2);
//        //robot.PositionorSetRefPoint(3);
//        //robot.PositionorSetRefPoint(4);
//
//        //DescPose coord = new DescPose();
//        //robot.PositionorComputeECoordSys(ref coord);
//
//
//        robot.ExtAxisServoOn(1, 1);
//        robot.ExtAxisSetHoming(1, 0, 10, 3);
//        robot.ExtAxisStartJog(1, 1, 100, 100, 20);
//        robot.Sleep(1000 * 2);
//        robot.ExtAxisStopJog(1);
//        robot.ExtAxisServoOn(1, 0);
//
//        ExaxisPos pos = new ExaxisPos(10, 0, 0, 0);
//        robot.ExtAxisMove(pos, 10);
    }

    private static void TestUDPWireSearch(Robot robot)
    {
        UDPComParam param = new UDPComParam("192.168.58.88", 2021, 2, 100, 3, 100, 1, 100, 10);
        robot.ExtDevSetUDPComParam(param);//udp扩展轴通讯

        robot.SetWireSearchExtDIONum(0, 0);

        int rtn0, rtn1, rtn2 = 0;
        ExaxisPos exaxisPos = new ExaxisPos(0.0, 0.0, 0.0, 0.0);
        DescPose offdese = new DescPose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

        DescPose descStart = new DescPose(-158.767, -510.596, 271.709, -179.427, -0.745, -137.349);
        JointPos jointStart = new JointPos(61.667, -79.848, 108.639, -119.682, -89.700, -70.985);

        DescPose descEnd = new DescPose(0.332, -516.427, 270.688, 178.165, 0.017, -119.989);
        JointPos jointEnd = new JointPos(79.021, -81.839, 110.752, -118.298, -91.729, -70.981);

        robot.MoveL(jointStart, descStart, 1, 0, 100, 100, 100, -1, exaxisPos, 0, 0, offdese, 0, 100);
        robot.MoveL(jointEnd, descEnd, 1, 0, 100, 100, 100, -1, exaxisPos, 0, 0, offdese, 0, 100);

        DescPose descREF0A = new DescPose(-66.106, -560.746, 270.381, 176.479, -0.126, -126.745);
        JointPos jointREF0A = new JointPos(73.531, -75.588, 102.941, -116.250, -93.347, -69.689);

        DescPose descREF0B = new DescPose(-66.109, -528.440, 270.407, 176.479, -0.129, -126.744);
        JointPos jointREF0B = new JointPos(72.534, -79.625, 108.046, -117.379, -93.366, -70.687);

        DescPose descREF1A = new DescPose(72.975, -473.242, 270.399, 176.479, -0.129, -126.744);
        JointPos jointREF1A = new JointPos(87.169, -86.509, 115.710, -117.341, -92.993, -56.034);

        DescPose descREF1B = new DescPose(31.355, -473.238, 270.405, 176.480, -0.130, -126.745);
        JointPos jointREF1B = new JointPos(82.117, -87.146, 116.470, -117.737, -93.145, -61.090);

        rtn0 = robot.WireSearchStart(0, 10, 100, 0, 10, 100, 0);
        robot.MoveL(jointREF0A, descREF0A, 1, 0, 100, 100, 100, -1, exaxisPos, 0, 0, offdese, 0, 100);  //起点
        robot.MoveL(jointREF0B, descREF0B, 1, 0, 10, 100, 100, -1, exaxisPos, 1, 0, offdese, 0, 100);  //方向点
        rtn1 = robot.WireSearchWait("REF0");
        rtn2 = robot.WireSearchEnd(0, 10, 100, 0, 10, 100, 0);

        rtn0 = robot.WireSearchStart(0, 10, 100, 0, 10, 100, 0);
        robot.MoveL(jointREF1A, descREF1A, 1, 0, 100, 100, 100, -1, exaxisPos, 0, 0, offdese, 0, 100);  //起点
        robot.MoveL(jointREF1B, descREF1B, 1, 0, 10, 100, 100, -1, exaxisPos, 1, 0, offdese, 0, 100);  //方向点
        rtn1 = robot.WireSearchWait("REF1");
        rtn2 = robot.WireSearchEnd(0, 10, 100, 0, 10, 100, 0);

        rtn0 = robot.WireSearchStart(0, 10, 100, 0, 10, 100, 0);
        robot.MoveL(jointREF0A, descREF0A, 1, 0, 100, 100, 100, -1, exaxisPos, 0, 0, offdese, 0, 100);  //起点
        robot.MoveL(jointREF0B, descREF0B, 1, 0, 10, 100, 100, -1, exaxisPos, 1, 0, offdese, 0, 100);  //方向点
        rtn1 = robot.WireSearchWait("RES0");
        rtn2 = robot.WireSearchEnd(0, 10, 100, 0, 10, 100, 0);

        rtn0 = robot.WireSearchStart(0, 10, 100, 0, 10, 100, 0);
        robot.MoveL(jointREF1A, descREF1A, 1, 0, 100, 100, 100, -1, exaxisPos, 0, 0, offdese, 0, 100);  //起点
        robot.MoveL(jointREF1B, descREF1B, 1, 0, 10, 100, 100, -1, exaxisPos, 1, 0, offdese, 0, 100);  //方向点
        rtn1 = robot.WireSearchWait("RES1");
        rtn2 = robot.WireSearchEnd(0, 10, 100, 0, 10, 100, 0);

        String[] varNameRef = {"REF0", "REF1", "#", "#", "#", "#"};
        String[] varNameRes = {"RES0", "RES1", "#", "#", "#", "#"};
        int offectFlag = 0;
        //DescPose offectPos = new DescPose(0, 0, 0, 0, 0, 0);
        DescOffset offset = new DescOffset();
        rtn0 = robot.GetWireSearchOffset(0, 0, varNameRef, varNameRes, offset);
        robot.PointsOffsetEnable(0, offset.offset);
        robot.MoveL(jointStart, descStart, 1, 0, 100, 100, 100, -1, exaxisPos, 0, 0, offdese, 0, 100);
        robot.MoveL(jointEnd, descEnd, 1, 0, 100, 100, 100, -1, exaxisPos, 1, 0, offdese, 0, 100);
        robot.PointsOffsetDisable();

    }

    private static void TestTractorMove(Robot robot)
    {
        UDPComParam param = new UDPComParam("192.168.58.88", 2021, 2, 100, 3, 100, 1, 100, 10);
        robot.ExtDevSetUDPComParam(param);//udp扩展轴通讯
        robot.ExtAxisParamConfig(1, 0, 0, 50000, -50000, 1000, 1000, 6.280, 16384, 200, 0, 0, 0);
        robot.ExtAxisParamConfig(2, 0, 0, 50000, -50000, 1000, 1000, 6.280, 16384, 200, 0, 0, 0);
        robot.SetAxisDHParaConfig(5, 0, 0, 0, 0, 0, 0, 0, 0);

        robot.TractorEnable(false);
        robot.Sleep(2000);
        robot.TractorEnable(true);
        robot.Sleep(2000);
        robot.TractorHoming();

        robot.Sleep(2000);
        robot.TractorMoveL(100, 20);
        robot.Sleep(5000);
        robot.TractorMoveL(-100, 20);
        robot.Sleep(5000);
        robot.TractorMoveC(300, 90, 20);
        robot.Sleep(2000);
        robot.TractorStop();//小车停止
        robot.TractorMoveC(300, -90, 20);
    }


    private static void  TestWeldmechineMode(Robot robot)
    {
        UDPComParam param = new UDPComParam("192.168.58.88", 2021, 2, 50, 5, 50, 1, 50, 10);
        robot.ExtDevSetUDPComParam(param);//udp扩展轴通讯
        robot.ExtDevLoadUDPDriver();

        robot.SetWeldMachineCtrlModeExtDoNum(17);//DO
        for (int i = 0; i < 5; i++)
        {
            robot.SetWeldMachineCtrlMode(0);//设置焊机控制模式
            robot.Sleep(500);
            robot.SetWeldMachineCtrlMode(1);
            robot.Sleep(500);
        }

        robot.SetWeldMachineCtrlModeExtDoNum(18);
        for (int i = 0; i < 5; i++)
        {
            robot.SetWeldMachineCtrlMode(0);
            robot.Sleep(500);
            robot.SetWeldMachineCtrlMode(1);
            robot.Sleep(500);
        }

        robot.SetWeldMachineCtrlModeExtDoNum(19);
        for (int i = 0; i < 5; i++)
        {
            robot.SetWeldMachineCtrlMode(0);//设置焊机控制模式
            robot.Sleep(500);
            robot.SetWeldMachineCtrlMode(1);
            robot.Sleep(500);
        }
    }
    private static void Program(Robot robot)
    {
// *************************************LUA文件下载 begin ********************************
        robot.LuaDownLoad("1010TestLUA.lua", "D://LUA/");
//*************************************LUA文件下载 end ********************************

//*************************************获取当前所有LUA begin ********************************
//        List<String> names = new ArrayList<String>();
//        robot.GetLuaList(names);
//        System.out.println("lua Num " + names.size() + "   " + names.get(0));
//*************************************获取当前所有LUA end ********************************

//*************************************LUA文件删除 end ********************************
//        robot.LuaDelete("1010TestLUA.lua");
//*************************************LUA文件删除 end ********************************

// *************************************LUA文件上传 begin ********************************
//        String errStr = "";
//        robot.LuaUpload("D://LUA/1010TestLUA.lua", errStr);
//        System.out.println("robot upload 1010TestLUA lua result " + errStr);
// *************************************LUA文件上传 end ********************************


// *************************************开机自动加载默认程序 begin ********************************
//        robot.LoadDefaultProgConfig(1,"/fruser/1010Test.lua");
// *************************************开机自动加载默认程序 end ********************************

// *************************************加载、获取LUA名称 begin ********************************
//        robot.Mode(0);
//        robot.ProgramLoad("/fruser/1010Test.lua");
//        String[] loadedNameStr = new String[1];
//        robot.GetLoadedProgram(loadedNameStr);
//        System.out.println("Loaded lua Name is " + loadedNameStr[0]);
//        robot.ProgramRun();
//*************************************加载、获取LUA名称 end ********************************

//*************************************获取运行状态及行号 begin ********************************
//        robot.Mode(0);
//        robot.ProgramLoad("/fruser/1010Test.lua");
//        String[] loadedNameStr = new String[1];
//        robot.GetLoadedProgram(loadedNameStr);
//        System.out.println("Loaded lua Name is " + loadedNameStr[0]);
//        robot.ProgramRun();
//
//        while(true)
//        {
//            List<Integer> results =  robot.GetCurrentLine();
//            ROBOT_STATE_PKG pkg = robot.GetRobotRealTimeState();
//            System.out.println("current line is " + results.get(1) + " Robot Runing State: " + pkg.robot_state);
//            robot.Sleep(500);
//        }
//*************************************获取运行状态及行号 end ********************************

//*************************************LUA暂停、停止、运行 begin ********************************
//        robot.Mode(0);
//        robot.ProgramLoad("/fruser/1010Test.lua");
//        String[] loadedNameStr = new String[1];
//        robot.GetLoadedProgram(loadedNameStr);
//        System.out.println("Loaded lua Name is " + loadedNameStr[0]);
//        robot.ProgramRun();//开始
//
//        robot.Sleep(1000);
//        for(int i = 0; i < 10;  i++)
//        {
//            robot.PauseMotion();//暂停运动
//            robot.Sleep(1000);
//            robot.ResumeMotion();//恢复运动
//            robot.Sleep(1000);
//        }
//
//        robot.StopMotion();//停止
//*************************************LUA暂停、停止、运行 begin ********************************

    }

    static int SegmentWeld(Robot robot)
    {
        DescPose startdescPose = new DescPose(185.859,-520.154,193.129,-177.129,1.339,-137.789);
        JointPos startjointPos = new JointPos(-60.989,-94.515,-89.479,-83.514,91.957,-13.124);

        DescPose enddescPose = new DescPose( -243.7033,-543.868,143.199,-177.954,1.528,177.758);
        JointPos endjointPos = new JointPos(-105.479,-101.919,-87.979,-78.455,91.955,-13.183);

        ExaxisPos exaxisPos = new ExaxisPos( 0, 0, 0, 0 );
        DescPose offdese = new DescPose( 0, 0, 0, 0, 0, 0 );

        robot.SegmentWeldStart(startdescPose, enddescPose, startjointPos, endjointPos, 80, 40, 0, 0, 5000, true, 0, 3, 0, 30, 30, 100, -1, exaxisPos, 0, 0, offdese);

        return 0;
    }

    static int TestAuxServo(Robot robot)
    {
//*****************************************设置和获取485扩展轴配置参数 begin *************************************
//        Axis485Param param = new Axis485Param();
//        param.servoCompany = 1;           // 伺服驱动器厂商，1-戴纳泰克
//        param.servoModel = 1;             // 伺服驱动器型号，1-FD100-750C
//        param.servoSoftVersion = 1;       // 伺服驱动器软件版本，1-V1.0
//        param.servoResolution = 131072;        // 编码器分辨率
//        param.axisMechTransRatio = 13.45;  // 机械传动比
//        robot.AuxServoSetParam(1, param);//设置485扩展轴参数
//
//        robot.AuxServoGetParam(1, param);
//        System.out.println("auxservo param servoCompany: " + param.servoCompany + "  servoModel:  " + param.servoModel +
//                "  param.servoSoftVersion:  " + param.servoSoftVersion + "  servoResolution:  " + param.servoResolution + "  axisMechTransRatio:  " + param.axisMechTransRatio);
//*****************************************设置和获取485扩展轴配置参数 end *************************************



//*****************************************485扩展轴使能、回零、速度模式 begin *************************************
//        robot.AuxServoSetControlMode(1, 1);
//        robot.Sleep(2000);
//        robot.AuxServoEnable(1, 0);
//        robot.Sleep(3000);
//        robot.AuxServoEnable(1, 1);
//        robot.Sleep(2000);
//        robot.AuxServoHoming(1, 1, 10, 10,100);
//        robot.Sleep(5000);
//
//        robot.AuxServoSetTargetSpeed(1, 100,100);
//        robot.Sleep(3000);
//        robot.AuxServoSetTargetSpeed(1, -200,100);
//        robot.Sleep(3000);
//        robot.AuxServoSetTargetSpeed(1, 0,100);
//*****************************************485扩展轴使能、回零、速度模式 end *************************************

//*****************************************485扩展轴位置模式 begin *************************************
//        robot.AuxServoSetControlMode(1, 0);
//        robot.Sleep(2000);
//        robot.AuxServoEnable(1, 1);
//        robot.Sleep(2000);
//        robot.AuxServoHoming(1, 1, 10, 10,100);
//        robot.Sleep(5000);
//        robot.AuxServoSetTargetPos(1, -100, 30,100);
//        robot.Sleep(2000);
//        robot.AuxServoSetTargetPos(1, 500, 30,100);
//*****************************************485扩展轴位置模式 end *************************************

//*****************************************获取485伺服电机状态 begin *************************************
//        robot.AuxServoSetControlMode(1, 1);
//        robot.Sleep(2000);
//        robot.AuxServoEnable(1, 0);
//        robot.Sleep(3000);
//        robot.AuxServoEnable(1, 1);
//        robot.Sleep(2000);
//        robot.AuxServoHoming(1, 1, 10, 10,100);
//        robot.Sleep(5000);
//
//        robot.AuxServoSetTargetSpeed(1, 40,100);
//        robot.Sleep(3000);
//        robot.AuxServoSetTargetSpeed(1, 40,100);
//
//        robot.AuxServosetStatusID(1);
//
//        while (true)
//        {
//            ROBOT_STATE_PKG pkg = robot.GetRobotRealTimeState();
//            System.out.println("aux servo cur Pos :" + pkg.auxState.servoPos + "  cur vel:  " + pkg.auxState.servoVel);
//            robot.Sleep(100);
//        }
//*****************************************获取485伺服电机状态 end *************************************

//*****************************************485扩展轴加速度生效 begin *************************************
        robot.AuxServoEnable(1, 0);
        robot.Sleep(1000);

        robot.AuxServoSetControlMode(1, 0);
        robot.Sleep(2000);

        robot.AuxServoEnable(1, 1);
        robot.Sleep(2000);
        robot.AuxServoHoming(1, 1, 10, 10,100);
        robot.Sleep(4000);

        while (true){
            robot.AuxServoSetAcc(500, 500);
            robot.AuxServoSetTargetPos(1, 1000, 100, 50);
            robot.Sleep(2000);
            robot.AuxServoSetTargetPos(1, 0, 500, 50);
            robot.Sleep(3000);
            robot.AuxServoSetTargetPos(1, 1000, 500, 30);
            robot.Sleep(5000);
            robot.AuxServoSetTargetPos(1, 0, 500, 30);
            robot.Sleep(5000);
        }
//*****************************************485扩展轴加速度生效 end *************************************

//*****************************************485扩展轴急停减速配置 begin *************************************
//        robot.AuxServoEnable(1, 0);
//        robot.Sleep(2000);
//        robot.AuxServoSetControlMode(1, 1);
//        robot.Sleep(2000);
//        robot.AuxServoEnable(1, 1);
//        robot.Sleep(2000);
//        robot.AuxServoHoming(1, 1, 10, 10,100);
//        robot.Sleep(4000);
//
//        robot.AuxServoSetEmergencyStopAcc(200, 200);
//        robot.Sleep(1000);
//        List<Number> rtn =  robot.AuxServoGetEmergencyStopAcc();
//        System.out.println("emergency acc is " + rtn.get(1) + "   dec is " + rtn.get(2));
//
//        robot.AuxServoSetTargetSpeed(1, 500, 100);
//
//        robot.ProgramLoad("/fruser/test1014.lua");
//        robot.ProgramRun();
//
//        int i = 0;
//        while (true) {
//            ROBOT_STATE_PKG pkg = new ROBOT_STATE_PKG();
//            pkg = robot.GetRobotRealTimeState();
//            System.out.println("time " + pkg.robotTime.second + " : "  + pkg.robotTime.millisecond + "   cur velocity is " + pkg.auxState.servoVel + "   cur 485 axis emergency state is " + ((pkg.auxState.servoState >> 7) & 0x01) + "    robot collision state is " + pkg.collisionState + "  robot emergency state is " + pkg.EmergencyStop);
//            robot.Sleep(5);
//        }
//*****************************************485扩展轴急停减速配置 end *************************************

//*****************************************485扩展轴急停状态获取 begin *************************************
//        robot.AuxServoEnable(1, 0);
//        robot.Sleep(2000);
//        robot.AuxServoSetControlMode(1, 1);
//        robot.Sleep(2000);
//
//        robot.AuxServoEnable(1, 1);
//        robot.Sleep(2000);
//
//        robot.AuxServoHoming(1, 1, 10, 10,100);
//        robot.Sleep(4000);
//
//        robot.AuxServoSetEmergencyStopAcc(200, 200);
//        robot.Sleep(1000);
//        List<Number> rtn =  robot.AuxServoGetEmergencyStopAcc();
//        System.out.println("emergency acc is " + rtn.get(1) + "   dec is " + rtn.get(2));
//
//        robot.AuxServoSetTargetSpeed(1, 500, 100);
//
//        robot.ProgramLoad("/fruser/test1014.lua");
//        robot.ProgramRun();
//
//        int i = 0;
//        while (true) {
//            i++;
//            if(i > 400)
//            {
//                robot.ResetAllError();
//                i = 0;
//
//                robot.AuxServoSetTargetSpeed(1, 200, 100);
//            }
//
//            ROBOT_STATE_PKG pkg = new ROBOT_STATE_PKG();
//            pkg = robot.GetRobotRealTimeState();
//            System.out.println("time " + pkg.robotTime.second + " : "  + pkg.robotTime.millisecond + "   cur velocity is " + pkg.auxState.servoVel + "   cur 485 axis emergency state is " + ((pkg.auxState.servoState >> 7) & 0x01) + "    robot collision state is " + pkg.collisionState + "  robot emergency state is " + pkg.EmergencyStop);
//            robot.Sleep(5);
//        }
//*****************************************485扩展轴急停状态获取 end *************************************

//*****************************************速度模式设置加速度百分比生效 end *************************************
//        robot.AuxServoEnable(1, 0);
//        robot.Sleep(1000);
//        robot.AuxServoSetControlMode(1, 1);
//        robot.Sleep(1000);
//        robot.AuxServoEnable(1, 1);
//        robot.Sleep(1000);
//        robot.AuxServoHoming(1, 1, 10, 10,100);
//        robot.Sleep(3000);
//        while(true){
//            robot.AuxServoSetTargetSpeed(1, 500, 100);
//            robot.Sleep(2000);
//            robot.AuxServoSetTargetSpeed(1, 0, 100);
//            robot.Sleep(2000);
//            robot.AuxServoSetTargetSpeed(1, 500, 10);
//            robot.Sleep(2000);
//            robot.AuxServoSetTargetSpeed(1, 0, 10);
//        }
//*****************************************位置模式设置加速度百分比生效 end *************************************
//        robot.AuxServoEnable(1, 0);
//        robot.Sleep(1000);
//        robot.AuxServoSetControlMode(1, 0);
//        robot.Sleep(1000);
//        robot.AuxServoEnable(1, 1);
//        robot.Sleep(1000);
//        robot.AuxServoHoming(1, 1, 10, 10,100);
//        robot.Sleep(3000);
//        while(true){
//            robot.AuxServoSetTargetPos(1, 1000, 500,100);
//            robot.Sleep(2000);
//            robot.AuxServoSetTargetPos(1, 0, 500,100);
//            robot.Sleep(2000);
//            robot.AuxServoSetTargetPos(1, 1000, 500,10);
//            robot.Sleep(2000);
//            robot.AuxServoSetTargetPos(1, 0, 500,100);
//            robot.Sleep(2000);
//        }
//*****************************************位置模式设置加速度百分比生效 end *************************************

//*****************************************加速度稳定性 begin *************************************
//        robot.AuxServoEnable(1, 0);
//        robot.Sleep(1000);
//        robot.AuxServoSetControlMode(1, 0);
//        robot.Sleep(1000);
//        robot.AuxServoEnable(1, 1);
//        robot.Sleep(1000);
//        robot.AuxServoHoming(1, 1, 10, 10,100);
//        robot.Sleep(3000);
//        int i=0;
//        while(true){
//            robot.AuxServoSetTargetPos(1, 1000*(i+1), 500,100);
//            robot.Sleep(2000);
//            robot.AuxServoSetTargetPos(1, 1000*(i+2), 500,20);
//            robot.Sleep(2000);
//            i=i+2;
//        }
//*****************************************加速度稳定性 end *************************************

//        return 0;
    }

    private static void TestEndLuaGripper(Robot robot)
    {
//        String rttser = "";
//        robot.AxleLuaUpload("D://zUP/AXLE_LUA_End_DaHuan.lua", rttser);

        ROBOT_STATE_PKG pkg = new ROBOT_STATE_PKG();
        AxleComParam param = new AxleComParam(7,8,1,0,5,3,1);
        //AxleComParam param = new AxleComParam(8,7,2,1,6,4,2);
        robot.SetAxleCommunicationParam(param);

        AxleComParam getParam = new AxleComParam();
        robot.GetAxleCommunicationParam(getParam);

        robot.SetAxleLuaEnable(1);
        int[] luaEnableStatus = new int[5];
        robot.GetAxleLuaEnableStatus(luaEnableStatus);
        robot.SetAxleLuaEnableDeviceType(0, 1, 0);
        int[] type = new int[10];
        robot.GetAxleLuaEnableDeviceType(type);
        //int[] func = new int[]{0,1,1,1,1,0,1,1,0,1,0,0,0,0,0,0};
        int[] func = new int[]{1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1};
        robot.SetAxleLuaGripperFunc(1, func);
        int[] getFunc = new int[16];
        robot.GetAxleLuaGripperFunc(1, getFunc);
        int[] forceEnable = new int[16];
        int[] gripperEnable = new int[16];
        int[] ioEnable = new int[16];
        robot.GetAxleLuaEnableDevice(forceEnable, gripperEnable, ioEnable);
//        robot.ActGripper(1, 0);
//        robot.Sleep(2000);
//        robot.ActGripper(1, 1);
//        robot.Sleep(2000);
//        robot.MoveGripper(1, 10, 10, 100, 50000, 0);
        int pos = 0;
//        while (true)
//        {
//            pkg = robot.GetRobotRealTimeState();
//            System.out.println("pos is " + pkg.gripper_position);
//            robot.Sleep(100);
//        }

    }

    private static void EndLuaUpload(Robot robot)
    {
//        robot.AxleLuaUpload("D://zUP/AXLE_LUA_End_DaHuan_WeiHangBad.lua");
        //robot.AxleLuaUpload("D://zUP/AXLE_LUA_End_DaHuan_WeiHang.lua");
//        robot.SetAxleLuaEnable(1);
//        TestEndLuaGripper(robot);
//        while (true)
//        {
//            ROBOT_STATE_PKG pkg = robot.GetRobotRealTimeState();
//            System.out.println("end lua err code is " + pkg.endLuaErrCode);
//            System.out.println("gripper pos is " + pkg.gripper_position);
//            robot.Sleep(100);
//        }
    }

    private static void TestEndLuaForce(Robot robot)
    {
        ROBOT_STATE_PKG pkg = new ROBOT_STATE_PKG();
        AxleComParam param = new AxleComParam(7,8,1,0,5,3,1);
        robot.SetAxleCommunicationParam(param);

        AxleComParam getParam = new AxleComParam();
        robot.GetAxleCommunicationParam(getParam);

        robot.SetAxleLuaEnable(1);
        int[] luaEnableStatus = new int[5];
        robot.GetAxleLuaEnableStatus(luaEnableStatus);
        robot.SetAxleLuaEnableDeviceType(1, 0, 0);
        int[] type = new int[10];
        robot.GetAxleLuaEnableDeviceType(type);

        int[] forceEnable = new int[16];
        int[] gripperEnable = new int[16];
        int[] ioEnable = new int[16];
        robot.GetAxleLuaEnableDevice(forceEnable, gripperEnable, ioEnable);

        robot.Sleep(1000);
        Object[] M = { 15.0, 15.0, 15.0, 0.5, 0.5, 0.1 };
        Object[] B = { 150.0, 150.0, 150.0, 5.0, 5.0, 1.0 };
        Object[] K = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
        Object[] F = { 10.0, 10.0, 10.0, 1.0, 1.0, 1.0 };
        robot.EndForceDragControl(1, 0, 0, 0,M, B, K, F, 50, 100);

        robot.Sleep(2000);

        robot.EndForceDragControl(0, 0, 0,0, M, B, K, F, 50, 100);
    }

    //肘部ARC，测试奇异点保护
    public static void TestSingularAvoidEArc(Robot robot)
    {
        DescPose startdescPose=new DescPose(-57.170, -690.147, 370.969, 176.438, -8.320, 169.881);
        JointPos startjointPos=new JointPos(78.017, -62.036, 69.561, -94.199, -98.416, -1.360);

        DescPose middescPose=new DescPose(-71.044, -743.395, 375.996, -179.499, -5.398, 168.739);
        JointPos midjointPos=new JointPos(77.417, -55.000, 58.732, -94.360, -95.385, -1.376);

        DescPose enddescPose=new DescPose(-439.979, -512.743, 396.472, 178.112, 3.625, 146.576);
        JointPos endjointPos=new JointPos(40.243, -65.402, 70.802, -92.565, -87.055, -16.465);


        ExaxisPos exaxisPos=new ExaxisPos(0, 0, 0, 0);
        DescPose offdese=new DescPose(0, 0, 0, 0, 0, 0);

        robot.MoveL(startjointPos, startdescPose, 0, 0, 30, 100, 100, -1, exaxisPos, 0, 0, offdese, 1, 1);
        robot.SingularAvoidStart(2, 10, 5, 5);
        robot.MoveC(midjointPos, middescPose, 0, 0, 30, 100, exaxisPos, 0, offdese, endjointPos, enddescPose, 0, 0, 30, 100, exaxisPos, 0, offdese, 100, -1);
        robot.SingularAvoidEnd();



//        DescPose startdescPose=new DescPose(-352.437, -88.350, 226.471, 177.222, 4.924, 86.631);
//        JointPos startjointPos=new JointPos(-3.463, -84.308, 105.579, -108.475, -85.087, -0.334);
//
//        DescPose middescPose=new DescPose(-518.339, -23.706, 207.899, -178.420, 0.171, 71.697);
//        JointPos midjointPos=new JointPos(-8.587, -51.805, 64.914, -104.695, -90.099, 9.718);
//
//        DescPose enddescPose=new DescPose(-273.934, 323.003, 227.224, 176.398, 2.783, 66.064);
//        JointPos endjointPos=new JointPos(-63.460, -71.228, 88.068, -102.291, -90.149, -39.605);
//
//
//        ExaxisPos exaxisPos=new ExaxisPos(0, 0, 0, 0);
//        DescPose offdese=new DescPose(0, 0, 0, 0, 0, 0);
//
//        robot.MoveL(startjointPos, startdescPose, 0, 0, 50, 100, 100, -1, exaxisPos, 0, 0, offdese, 1, 1);
//        robot.SingularAvoidStart(1, 100, 50, 10);
//        robot.MoveC(midjointPos, middescPose, 0, 0, 50, 100, exaxisPos, 0, offdese, endjointPos, enddescPose, 0, 0, 100, 100, exaxisPos, 0, offdese, 100, -1);
//        robot.SingularAvoidEnd();
    }

    //肩部ARC，测试奇异点保护
    public static void TestSingularAvoidSArc(Robot robot)
    {
        robot.SingularAvoidEnd();
        int rtn = 0;
        DescPose startdescPose=new DescPose(299.993, -168.982, 299.998, 179.999, -0.002, -166.415);
        JointPos startjointPos=new JointPos(-12.160, -71.236, -131.775, -66.992, 90.000, 64.255);

        DescPose middescPose=new DescPose(249.985, -140.988, 299.929, 179.996, -0.013, -166.417);
        JointPos midjointPos=new JointPos(-8.604, -60.474, -137.494, -72.046, 89.999, 67.813);

        DescPose enddescPose=new DescPose(-249.991, -168.980, 299.981, 179.999, 0.004, -107.386);
        JointPos endjointPos=new JointPos(-126.186, -63.401, -136.126, -70.477, 89.998, -108.800);

        ExaxisPos exaxisPos=new ExaxisPos(0, 0, 0, 0);
        DescPose offdese=new DescPose(0, 0, 0, 0, 0, 0);

        rtn = robot.MoveL(startjointPos, startdescPose, 0, 0, 30, 100, 100, -1, exaxisPos, 0, 0, offdese, 1, 1);
        rtn = robot.SingularAvoidStart(2, 30, 5, 5);
        rtn = robot.MoveC(midjointPos, middescPose, 0, 0, 30, 100, exaxisPos, 0, offdese, endjointPos, enddescPose, 0, 0, 30, 100, exaxisPos, 0, offdese, 100, -1);
        rtn = robot.SingularAvoidEnd();
        System.out.println("robot moving rtn is "+rtn);

//        robot.SingularAvoidEnd();
//        DescPose startdescPose=new DescPose(-379.749, -113.569, 262.288, -178.716, 2.620, 91.597);
//        JointPos startjointPos=new JointPos(1.208, -80.436, 93.788, -104.620, -87.372, -0.331);
//
//        DescPose middescPose=new DescPose(-151.941, -155.742, 262.756, 177.693, 2.571, 106.941);
//        JointPos midjointPos=new JointPos(16.727, -121.385, 124.147, -90.442, -87.440, -0.318);
//
//        DescPose enddescPose=new DescPose(-211.982, 218.852, 280.712, 176.819, -4.408, 26.857);
//        JointPos endjointPos=new JointPos(-63.754, -98.766, 105.961, -94.052, -94.435, -0.366);
//
//        ExaxisPos exaxisPos=new ExaxisPos(0, 0, 0, 0);
//        DescPose offdese=new DescPose(0, 0, 0, 0, 0, 0);
//
//        robot.MoveL(startjointPos, startdescPose, 0, 0, 30, 100, 100, -1, exaxisPos, 0, 0, offdese, 1, 1);
//        robot.SingularAvoidStart(0, 150, 50, 20);//开启保护
//        robot.MoveC(midjointPos, middescPose, 0, 0, 30, 100, exaxisPos, 0, offdese, endjointPos, enddescPose, 0, 0, 100, 100, exaxisPos, 0, offdese, 100, -1);
//        robot.SingularAvoidEnd();
    }

    //腕部Line，测试奇异点保护
    public static void TestSingularAvoidWLin(Robot robot)
    {
        DescPose startdescPose=new DescPose(-352.574, -685.606, 479.415, -15.926, -54.905, 130.693);
        JointPos startjointPos=new JointPos(49.630, -56.597, 60.013, -57.990, 42.725, 146.834);

        DescPose enddescPose=new DescPose(-653.655, -235.943, 434.585, -176.403, -54.513, -66.719);
        JointPos endjointPos=new JointPos(5.072, -58.920, 55.280, -57.939, -41.207, 146.834);

        ExaxisPos exaxisPos=new ExaxisPos(0, 0, 0, 0);
        DescPose offdese=new DescPose(0, 0, 0, 0, 0, 0);

        robot.MoveL(startjointPos, startdescPose, 0, 0, 30, 100, 100, -1, exaxisPos, 0, 0, offdese, 1, 1);
        robot.SingularAvoidStart(2, 30, 10, 3);
        robot.MoveL(endjointPos, enddescPose, 0, 0, 30, 100, 100, -1, exaxisPos, 0, 0, offdese, 1, 1);
        robot.SingularAvoidEnd();

//        DescPose startdescPose=new DescPose(-402.473, -185.876, 103.985, -175.367, 59.682, 94.221);
//        JointPos startjointPos=new JointPos(-0.095, -50.828, 109.737, -150.708, -30.225, -0.623);
//
//        DescPose enddescPose=new DescPose(-399.264, -184.434, 296.022, -4.402, 58.061, -94.161);
//        JointPos endjointPos=new JointPos(-0.095, -65.547, 105.145, -131.397, 31.851, -0.622);
//
//        ExaxisPos exaxisPos=new ExaxisPos(0, 0, 0, 0);
//        DescPose offdese=new DescPose(0, 0, 0, 0, 0, 0);
//
//        robot.MoveL(startjointPos, startdescPose, 0, 0, 50, 100, 100, -1, exaxisPos, 0, 0, offdese, 1, 1);
//        robot.SingularAvoidStart(0, 150, 50, 20);
//        robot.MoveL(endjointPos, enddescPose, 0, 0, 50, 100, 100, -1, exaxisPos, 0, 0, offdese, 1, 1);
//        robot.SingularAvoidEnd();
    }

    ///腕部ARC，测试奇异点保护
    public static void TestSingularAvoidWArc(Robot robot)
    {
        DescPose startdescPose=new DescPose(-352.575, -685.604, 479.380, -15.933, -54.906, 130.699);
        JointPos startjointPos=new JointPos(49.630, -56.597, 60.017, -57.989, 42.725, 146.834);

        DescPose middescPose=new DescPose(-437.302, -372.046, 366.764, -133.489, -62.309, -94.994);
        JointPos midjointPos=new JointPos(21.202, -72.442, 84.164, -51.660, -29.880, 146.823);

        DescPose enddescPose=new DescPose(-653.649, -235.926, 434.525, -176.386, -54.515, -66.734);
        JointPos endjointPos=new JointPos(5.070, -58.920, 55.287, -57.937, -41.207, 146.834);

        ExaxisPos exaxisPos=new ExaxisPos(0, 0, 0, 0);
        DescPose offdese=new DescPose(0, 0, 0, 0, 0, 0);

        robot.MoveL(startjointPos, startdescPose, 0, 0, 30, 100, 100, -1, exaxisPos, 0, 0, offdese, 1, 1);
        robot.SingularAvoidStart(2, 10, 5, 4);
        robot.MoveC(midjointPos, middescPose, 0, 0, 30, 100, exaxisPos, 0, offdese, endjointPos, enddescPose, 0, 0, 30, 100, exaxisPos, 0, offdese, 100, -1);
        robot.SingularAvoidEnd();

//        DescPose startdescPose=new DescPose(-352.794, -164.582, 132.122, 176.136, 50.177, 85.343);
//        JointPos startjointPos=new JointPos(-2.048, -66.683, 121.240, -141.651, -39.776, -0.564);
//
//        DescPose middescPose=new DescPose(-352.353, -3.338, 299.600, -1.730, 58.744, -136.276);
//        JointPos midjointPos=new JointPos(-30.807, -92.341, 126.259, -102.944, 33.740, -25.798);
//
//        DescPose enddescPose=new DescPose(-352.353, -3.337, 353.164, -1.729, 58.744, -136.276);
//        JointPos endjointPos=new JointPos(-30.807, -98.084, 116.943, -87.886, 33.740, -25.798);
//
//        DescPose descPose=new DescPose(-402.473, -185.876, 103.985, -175.367, 59.682, 94.221);
//
//        JointPos jointPos=new JointPos(-0.095, -50.828, 109.737, -150.708, -30.225, -0.623);
//
//        ExaxisPos exaxisPos=new ExaxisPos(0, 0, 0, 0);
//        DescPose offdese=new DescPose(0, 0, 0, 0, 0, 0);
//
//        robot.MoveL(startjointPos, startdescPose, 0, 0, 50, 100, 100, -1, exaxisPos, 0, 0, offdese, 1, 1);
//        robot.SingularAvoidStart(0, 150, 50, 20);
//        robot.MoveC(midjointPos, middescPose, 0, 0, 100, 100, exaxisPos, 0, offdese, endjointPos, enddescPose, 0, 0, 100, 100, exaxisPos, 0, offdese, 100, -1);
//        robot.MoveL(jointPos, descPose, 0, 0, 50, 100, 100, -1, exaxisPos, 0, 0, offdese, 1, 1);
//        robot.SingularAvoidEnd();
    }

    //肩部Line，测试奇异点保护
    public static void TestSingularAvoidSLin(Robot robot)
    {
        DescPose startdescPose=new DescPose(300.002, -102.991, 299.994, 180.000, -0.001, -166.416);
        JointPos startjointPos=new JointPos(-0.189, -66.345, -134.615, -69.042, 90.000, 76.227);

        DescPose enddescPose=new DescPose(-300.000, -103.001, 299.994, 179.998, 0.003, -107.384);
        JointPos endjointPos=new JointPos(-142.292, -66.345, -134.615, -69.042, 89.997, -124.908);

        ExaxisPos exaxisPos=new ExaxisPos(0, 0, 0, 0);
        DescPose offdese=new DescPose(0, 0, 0, 0, 0, 0);

        robot.MoveL(startjointPos, startdescPose, 0, 0, 30, 100, 100, -1, exaxisPos, 0, 0, offdese, 1, 1);
        robot.SingularAvoidStart(2, 30, 10, 3);
        robot.MoveL(endjointPos, enddescPose, 0, 0, 30, 100, 100, -1, exaxisPos, 0, 0, offdese, 1, 1);
        robot.SingularAvoidEnd();

//        DescPose startdescPose=new DescPose(-379.749, -113.569, 262.293, -178.715, 2.620, 91.597);
//        JointPos startjointPos=new JointPos(1.208, -80.436, 93.788, -104.620, -87.372, -0.331);
//
//        DescPose enddescPose=new DescPose(252.972, -74.287, 316.795, -177.588, 2.451, 97.588);
//        JointPos endjointPos=new JointPos(7.165, -170.868, 63.507, 14.965, -87.534, -0.319);
//
//        ExaxisPos exaxisPos=new ExaxisPos(0, 0, 0, 0);
//        DescPose offdese=new DescPose(0, 0, 0, 0, 0, 0);
//
//        robot.MoveL(startjointPos, startdescPose, 0, 0, 50, 100, 100, -1, exaxisPos, 0, 0, offdese, 1, 1);
//        robot.SingularAvoidStart(0, 150, 50, 20);
//        robot.MoveL(endjointPos, enddescPose, 0, 0, 50, 100, 100, -1, exaxisPos, 0, 0, offdese, 1, 1);
//        robot.SingularAvoidEnd();
    }
    //轨迹J文件上传与删除
    public static void UploadTrajectoryJ(Robot robot)
    {
        robot.TrajectoryJDelete("testA.txt");//删除轨迹文件
        robot.TrajectoryJUpLoad("D://zUP/testA.txt");

        int retval = 0;
        String traj_file_name= "/fruser/traj/testA.txt";
        retval = robot.LoadTrajectoryJ(traj_file_name, 100, 1);
        System.out.println("LoadTrajectoryJ %s, retval is:"+traj_file_name+retval);

        DescPose traj_start_pose=new DescPose(0,0,0,0,0,0);
        retval = robot.GetTrajectoryStartPose(traj_file_name, traj_start_pose);
        System.out.println("GetTrajectoryStartPose is: %d"+retval);
        System.out.println("desc_pos:"+"("+traj_start_pose.tran.x+","+traj_start_pose.tran.y+","+traj_start_pose.tran.z+","+traj_start_pose.rpy.rx+","+traj_start_pose.rpy.ry+","+traj_start_pose.rpy.rz+")");

        robot.SetSpeed(30);
        robot.MoveCart(traj_start_pose, 1, 0, 100, 100, 100, -1, -1);

        robot.Sleep(5000);

        int traj_num = 0;

        ROBOT_STATE_PKG pkg = robot.GetRobotRealTimeState();
        traj_num=pkg.trajectory_pnum;
        System.out.println("GetTrajectoryStartPose traj num is:"+traj_num);

        retval = robot.MoveTrajectoryJ();
        System.out.println("MoveTrajectoryJ retval is:"+retval);
    }
    //轨迹J文件上传与删除
    public static void UploadTrajectoryB(Robot robot)
    {
        robot.TrajectoryJDelete("testB.txt");//删除轨迹文件
        robot.TrajectoryJUpLoad("D://zUP/testB.txt");

        int retval = 0;
        String traj_file_name = "/fruser/traj/testB.txt";
        retval = robot.LoadTrajectoryJ(traj_file_name, 100, 1);
        System.out.println("LoadTrajectoryJ "+traj_file_name+", retval is:"+retval);

        DescPose traj_start_pose=new DescPose(0,0,0,0,0,0);
        retval = robot.GetTrajectoryStartPose(traj_file_name, traj_start_pose);
        System.out.println("GetTrajectoryStartPose is:"+retval);
        System.out.println("desc_pos:"+traj_start_pose.tran.x+","+traj_start_pose.tran.y+","+traj_start_pose.tran.z+","+traj_start_pose.rpy.rx+","+ traj_start_pose.rpy.ry+","+traj_start_pose.rpy.rz);

        robot.SetSpeed(30);
        robot.MoveCart(traj_start_pose, 1, 0, 100, 100, 100, -1, -1);

        robot.Sleep(5000);

        int traj_num = 0;
        ROBOT_STATE_PKG pkg = robot.GetRobotRealTimeState();
        traj_num=pkg.trajectory_pnum;
        System.out.println("GetTrajectoryStartPose traj num is:"+traj_num);

        retval = robot.MoveTrajectoryJ();
        System.out.println("MoveTrajectoryJ retval is:"+retval);
    }

    public static void MoveRotGripper(Robot robot, int pos, double rotPos)
    {
        robot.ResetAllError();
        robot.ActGripper(1, 1);
        robot.Sleep(1000);
        int rtn = robot.MoveGripper(1, pos, 50, 50, 5000, 1, 1, rotPos, 50, 100);
        System.out.println("move gripper rtn is:"+rtn);
        while (true)
        {
            ROBOT_STATE_PKG pkg=robot.GetRobotRealTimeState();
            if (Math.abs(pkg.gripper_position - pos) < 1.5)
            {
                break;
            }
            else
            {
                System.out.println("cur gripper pos is:"+pkg.gripper_position);
                robot.Sleep(10);
            }
        }
        System.out.println("Gripper Motion Done:"+pos);
    }

    public static void SetAO(Robot robot, float value)
    {
        robot.SetAO(0, value, 0);
        robot.SetAO(1, value, 0);
        robot.SetToolAO(0, value, 0);
        while (true)
        {
            ROBOT_STATE_PKG pkg=robot.GetRobotRealTimeState();
            if (Math.abs(pkg.cl_analog_output[0]/40.96 - value) < 0.5)
            {
                break;
            }
            else
            {
                System.out.println("cur AO value is"+pkg.cl_analog_output[0]);
                robot.Sleep(1);
            }
        }
        System.out.println("setAO Done:"+value);
    }
    public static void FIRArc(Robot robot, boolean enable)
    {
        DescPose startdescPose=new DescPose(-366.397, -572.427, 418.339, -178.972, 1.829, -142.970);
        JointPos startjointPos=new JointPos(43.651, -70.284, 91.057, -109.075, -88.768, -83.382);

        DescPose middescPose=new DescPose(-569.710, -132.595, 395.147, 178.418, -1.893, 171.051);
        JointPos midjointPos=new JointPos(-2.334, -79.300, 108.196, -120.594, -91.790, -83.386);

        DescPose enddescPose=new DescPose(-608.420, 610.692, 314.930, -176.438, -1.756, 117.333);
        JointPos endjointPos=new JointPos(-56.153, -46.964, 68.015, -113.200, -86.661, -83.479);

        ExaxisPos exaxisPos=new ExaxisPos(0, 0, 0, 0);
        DescPose offdese=new DescPose(0, 0, 0, 0, 0, 0);

        if (enable)
        {
            robot.LinArcFIRPlanningStart(1000, 1000, 1000, 1000);
            robot.MoveL(startjointPos, startdescPose, 0, 0, 100, 100, 100, -1, exaxisPos, 0, 0, offdese, 1, 1);
            robot.MoveC(midjointPos, middescPose, 0, 0, 100, 100, exaxisPos, 0, offdese, endjointPos, enddescPose, 0, 0, 100, 100, exaxisPos, 0, offdese, 100, -1);
            robot.LinArcFIRPlanningEnd();
        }
        else
        {
            robot.MoveL(startjointPos, startdescPose, 0, 0, 100, 100, 100, -1, exaxisPos, 0, 0, offdese, 1, 1);
            robot.MoveC(midjointPos, middescPose, 0, 0, 100, 100, exaxisPos, 0, offdese, endjointPos, enddescPose, 0, 0, 100, 100, exaxisPos, 0, offdese, 100, -1);
        }
    }

    public static void FIRLin(Robot robot, boolean enable)
    {
        DescPose startdescPose=new DescPose(-569.710, -132.595, 395.147, 178.418, -1.893, 171.051);
        JointPos startjointPos=new JointPos(-2.334, -79.300, 108.196, -120.594, -91.790, -83.386);

        DescPose enddescPose=new DescPose(-366.397, -572.427, 418.339, -178.972, 1.829, -142.970);
        JointPos endjointPos=new JointPos(43.651, -70.284, 91.057, -109.075, -88.768, -83.382);

        ExaxisPos exaxisPos=new ExaxisPos(0, 0, 0, 0);
        DescPose offdese=new DescPose(0, 0, 0, 0, 0, 0);

        if (enable)
        {
            robot.LinArcFIRPlanningStart(5000, 5000, 5000, 5000);
            robot.MoveL(startjointPos, startdescPose, 0, 0, 100, 100, 100, -1, exaxisPos, 0, 0, offdese, 1, 1);
            robot.MoveL(endjointPos, enddescPose, 0, 0, 100, 100, 100, -1, exaxisPos, 0, 0, offdese, 1, 1);
            robot.LinArcFIRPlanningEnd();
        }
        else
        {
            robot.MoveL(startjointPos, startdescPose, 0, 0, 100, 100, 100, -1, exaxisPos, 0, 0, offdese, 1, 1);
            robot.MoveL(endjointPos, enddescPose, 0, 0, 100, 100, 100, -1, exaxisPos, 0, 0, offdese, 1, 1);
        }
    }

    public  static void FIRLinL(Robot robot, boolean enable)
    {
        DescPose startdescPose=new DescPose(-608.420, 610.692, 314.930, -176.438, -1.756, 117.333);
        JointPos startjointPos=new JointPos(-56.153, -46.964, 68.015, -113.200, -86.661, -83.479);

        DescPose enddescPose=new DescPose(-366.397, -572.427, 418.339, -178.972, 1.829, -142.970);
        JointPos endjointPos=new JointPos(43.651, -70.284, 91.057, -109.075, -88.768, -83.382);

        ExaxisPos exaxisPos=new ExaxisPos(0, 0, 0, 0);
        DescPose offdese=new DescPose(0, 0, 0, 0, 0, 0);

        if (enable)
        {
            robot.LinArcFIRPlanningStart(5000, 5000, 5000, 5000);
            robot.MoveL(startjointPos, startdescPose, 0, 0, 100, 100, 100, -1, exaxisPos, 0, 0, offdese, 1, 1);
            robot.MoveL(endjointPos, enddescPose, 0, 0, 100, 100, 100, -1, exaxisPos, 0, 0, offdese, 1, 1);
            robot.LinArcFIRPlanningEnd();
        }
        else
        {
            robot.MoveL(startjointPos, startdescPose, 0, 0, 100, 100, 100, -1, exaxisPos, 0, 0, offdese, 1, 1);
            robot.MoveL(endjointPos, enddescPose, 0, 0, 100, 100, 100, -1, exaxisPos, 0, 0, offdese, 1, 1);
        }
    }

    public static void FIRPTP(Robot robot, boolean enable)
    {
        DescPose startdescPose=new DescPose(-569.710, -132.595, 395.147, 178.418, -1.893, 171.051);
        JointPos startjointPos=new JointPos(-2.334, -79.300, 108.196, -120.594, -91.790, -83.386);

        DescPose enddescPose=new DescPose(-366.397, -572.427, 418.339, -178.972, 1.829, -142.970);
        JointPos endjointPos=new JointPos(43.651, -70.284, 91.057, -109.075, -88.768, -83.382);

        ExaxisPos exaxisPos=new ExaxisPos(0, 0, 0, 0);
        DescPose offdese=new DescPose(0, 0, 0, 0, 0, 0);

        if (enable)
        {
            robot.PtpFIRPlanningStart(1000);
            robot.MoveJ(startjointPos, startdescPose, 0, 0, 100, 100, 100, exaxisPos, -1, 0, offdese);
            robot.MoveJ(endjointPos, enddescPose, 0, 0, 100, 100, 100, exaxisPos, -1, 0, offdese);
            robot.PtpFIRPlanningEnd();
        }
        else
        {
            robot.MoveJ(startjointPos, startdescPose, 0, 0, 100, 100, 100, exaxisPos, -1, 0, offdese);
            robot.MoveJ(endjointPos, enddescPose, 0, 0, 100, 100, 100, exaxisPos, -1, 0, offdese);
        }
    }

}

