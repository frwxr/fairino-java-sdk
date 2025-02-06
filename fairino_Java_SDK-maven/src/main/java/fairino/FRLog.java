package fairino;

import java.io.File;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.nio.file.StandardOpenOption;
import java.text.SimpleDateFormat;
import java.util.*;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

public class FRLog
{
    private FrLogType curLogType = FrLogType.DIRECT;
    private FrLogLevel curLogLevel = FrLogLevel.INFO;
    private String logFileName = "";
    private String logFilePath = "";
    private List<String> logBuf = new ArrayList<String>();
    private List<String> asyncWriteBuf = new ArrayList<String>();
    private int maxLogBufCount = 100;    //缓冲输出队列长度
    private int checkLogFileSizeFlag = 0;
    private final int CHECKFILESIZE = 1000;
    private boolean writeLogAsyncFlag = false;
    Lock slockAsyncLog = new ReentrantLock();
    private final long MAXFILESIZE = 10000000;
    private int logSaveDays = 10;
    private int logSaveFileNum = 10;
    public FRLog()
    {
        logFilePath = "";//System.CurrentDirectory;
        logFileName = GetNewLogFileName();
        //hread t = new Thread(WriteLogAsyncThread);
        //t.Start();
    }

    public FRLog(FrLogType logType, FrLogLevel logLevel, String filePath, int saveFileNum, int saveFileDays)
    {
        logFilePath = filePath;
        curLogType = logType;
        logSaveDays = saveFileDays;

        curLogLevel = logLevel;
        logSaveFileNum = saveFileNum;
        logFileName = GetNewLogFileName();

        if(logType == FrLogType.ASYNC)
        {
            writeLogAsyncFlag = true;
//            Thread t = new Thread(WriteLogAsyncThread);
//            t.Start();
        }


//        Thread t = new Thread(WriteLogAsyncThread);
//        t.Start();

    }

//    private String GetNewLogFileName()
//    {
//        return null;
//    }

    public int SetLogLevel(FrLogLevel level)
    {
        curLogLevel = level;
        return 0;
    }

    public int LogInfo(String logStr)
    {
        if(curLogLevel.compareTo(FrLogLevel.INFO) > 0)
        {
            return 0;
        }
        SimpleDateFormat formatter= new SimpleDateFormat("yyyy-MM-dd HH:mm:ss.SSS");
        Date date = new Date(System.currentTimeMillis());

        String fullLogStr = "[INFO] [" + formatter.format(date) + "] " + logStr;
        LogWrite(fullLogStr);
        return 0;
    }

    public int LogWarn(String logStr)
    {
        if (curLogLevel.compareTo(FrLogLevel.WARN) > 0)
        {
            return 0;
        }

        SimpleDateFormat formatter= new SimpleDateFormat("yyyy-MM-dd HH:mm:ss.SSS");
        Date date = new Date(System.currentTimeMillis());
        String fullLogStr = "[WARN] [" + formatter.format(date) + "] " + logStr;
        LogWrite(fullLogStr);
        return 0;
    }

    public int LogError(String methodName, int linNum, String logStr)
    {
        if (curLogLevel.compareTo(FrLogLevel.ERROR) > 0)
        {
            return 0;
        }

        SimpleDateFormat formatter= new SimpleDateFormat("yyyy-MM-dd HH:mm:ss.SSS");
        Date date = new Date(System.currentTimeMillis());
        String fullLogStr = "[ERROR] [" + formatter.format(date) + "] [" + methodName + " " + linNum + "] " + logStr;
        LogWrite(fullLogStr);
        return 0;
    }

    public int LogError(String logStr)
    {
        if (curLogLevel.compareTo(FrLogLevel.ERROR) > 0)
        {
            return 0;
        }

        SimpleDateFormat formatter= new SimpleDateFormat("yyyy-MM-dd HH:mm:ss.SSS");
        Date date = new Date(System.currentTimeMillis());
        String fullLogStr = "[ERROR] [" + formatter.format(date) + "] " + logStr;
        LogWrite(fullLogStr);
        return 0;
    }

    public int LogDebug(String logStr)
    {
        if (curLogLevel.compareTo(FrLogLevel.DEBUG) > 0)
        {
            return 0;
        }

        SimpleDateFormat formatter= new SimpleDateFormat("yyyy-MM-dd HH:mm:ss.SSS");
        Date date = new Date(System.currentTimeMillis());
        String fullLogStr = "[DEBUG] [" + formatter.format(date)  +  "] " + logStr;


        LogWrite(fullLogStr);
        return 0;
    }

    private int LogWrite(String logStr)
    {
        System.out.println(logStr);

        checkLogFileSizeFlag++;
        if(checkLogFileSizeFlag > CHECKFILESIZE) // 1000行日志检测一次大小
        {
            File fileInfo = new File(logFileName);
            long logFileSize = fileInfo.length();
            if(logFileSize > MAXFILESIZE)
            {
                logFileName = GetNewLogFileName();
                checkLogFileSizeFlag = 0;
            }
        }

        slockAsyncLog.lock();
        if (curLogType == FrLogType.DIRECT)
        {
            WriteLogDirect(logStr + "\n");
        }
        else if (curLogType == FrLogType.BUFFER)
        {
            WriteLogBuffer(logStr + "\n");
        }
        else if (curLogType == FrLogType.ASYNC)
        {
            WriteLogAsync(logStr + "\n");
        }

        slockAsyncLog.unlock();

        return 0;
    }

    private int WriteLogDirect(String logStr)
    {
        try {
            File file2 = new File(logFileName);
            if (!file2.exists()) {
                file2.createNewFile();//创建文件
            }
            Files.write(Paths.get(logFileName), logStr.getBytes(), StandardOpenOption.APPEND);
        }
        catch (Throwable e)
        {
            System.out.println("write log file fail " + e.getMessage());
            return -1;
        }

        return 0;
        }

        private int WriteLogBuffer(String logStr)
        {
            logBuf.add(logStr);
            int curLogCount = logBuf.size();
            if(curLogCount > maxLogBufCount)
            {
                try {
                    for(int i = 0; i < curLogCount; i++)
                    {
                        File file2 = new File(logFileName);
                        if (!file2.exists()) {
                            file2.createNewFile();//创建文件
                        }
                        Files.write(Paths.get(logFileName), logBuf.get(i).getBytes(), StandardOpenOption.APPEND);
                    }
                }
                catch (Throwable e)
                {
                    System.out.println("write log file fail " + e.getMessage());
                    return -1;
                }
                logBuf.clear();
            }
            return 0;
        }

    private int WriteLogAsync(String logStr)
    {
        logBuf.add(logStr);
        return 0;
    }

    private String GetNewLogFileName()
    {
        int logFileCount = 0;

        File file = new File(logFilePath);
        //用数组把文件夹下的文件存起来
        File[] files = file.listFiles();
        Arrays.sort(files, new Comparator<File>() {
            public int compare(File f1, File f2) {
                long diff = f1.lastModified() - f2.lastModified();
                if (diff > 0)
                    return 1;
                else if (diff == 0)
                    return 0;
                else
                    return -1;//如果 if 中修改为 返回-1 同时此处修改为返回 1  排序就会是递减
            }

            public boolean equals(Object obj) {
                return true;
            }

        });
        //foreach遍历数组
        for (File file2 : files) {
            if (file2.getName().contains("FrLog"))
            {
                logFileCount++;
            }
        }
        SimpleDateFormat formatter=new SimpleDateFormat("yyyy-MM-dd_HH-mm-ss");

        if(logFileCount > logSaveFileNum)
        {
            Calendar cal = Calendar.getInstance();
            cal.add(Calendar.DATE, (-1) * logSaveDays);
            Date curTime = cal.getTime();



            for (File tmpFile : files)
            {
                try {
                    String fileDataStr = tmpFile.getName().substring(6, 25);
                    Date fileDate = formatter.parse(fileDataStr);

//                    DateTimeFormatter df = DateTimeFormatter.ofPattern("yyyy-MM-dd_HH-mm-ss");
//                    LocalDateTime dateTimeT = LocalDateTime.parse(fileDataStr, df);
//                    Date fileDate = Date.from(dateTimeT.toInstant(ZoneOffset.of("+8")));


                    if (fileDate.before(curTime) && logFileCount > logSaveFileNum)
                    {
                        tmpFile.delete();
                        logFileCount--;
                    }
                }
                catch (Throwable e)
                {
                    System.out.println("delete old log file fail  :  " + e.getMessage());
                }
            }
        }


        return logFilePath + "\\FrLog_" + formatter.format(Calendar.getInstance().getTime()) + ".log";
    }

    private void WriteLogAsyncThread()
    {
        try
        {
            while(writeLogAsyncFlag)
            {
                if(curLogType == FrLogType.ASYNC && !logBuf.isEmpty())  //只要有就写
                {
                    slockAsyncLog.lock();
                    asyncWriteBuf.addAll(logBuf);
                    logBuf.clear();

                    for (int i = 0; i < asyncWriteBuf.size(); i++)
                    {
                        File file2 = new File(logFileName);
                        if (!file2.exists()) {
                            file2.createNewFile();//创建文件
                        }
                        Files.write(Paths.get(logFileName), asyncWriteBuf.get(i).getBytes(), StandardOpenOption.APPEND);
                    }

                    asyncWriteBuf.clear();
                }
                else
                {
                    Thread.sleep(50);
                }
            }
        }
        catch (Throwable e)
        {
            System.out.println("async write log failed  :  " + e.getMessage());
        }
    }

    public int LogClose()
    {
        try {
            slockAsyncLog.lock();
            int curLogCount = logBuf.size();
            if(curLogCount > 0)
            {
                for (int i = 0; i < curLogCount; i++)
                {
                    Files.write(Paths.get(logFileName), logBuf.get(i).getBytes(), StandardOpenOption.APPEND);
                }
                logBuf.clear();
            }
            slockAsyncLog.unlock();
            writeLogAsyncFlag = false;
        } catch (Throwable e) {
            System.out.println("log close failed  :  "+ e.getMessage());
        }
        return 0;
    }

}
