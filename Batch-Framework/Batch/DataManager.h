#ifndef _DATAMANAGER_H_
#define _DATAMANAGER_H_

// void Compute_Preference_list(TASK task, WORKER worker[Number_Worker],int *PT){    //输入一个task,全部的worker，求出task对worker的偏好列表,以及任务的偏好列表PT
// 4.12 21:36 OMG,文件的读取存入响应的数据结构还没实现
#include "Basic_information.h"

/*******
 * 修改了任务生成时间，要求开始时间在截止时间之前
 * 注意这里存在无线循环的情况，即，开始时间一直在截止时间之前。
 *
 */
class DataManager
{
private:
    /* data */
public:
    double endtimeX;
    double rangeX;
    double scoreX;
    double speed;
    void Prodece_Task_Reward_Minscore_Deadline(vector<TASK> &task);
    void ReadLocationForTask(vector<TASK> &task);
    void Get_Trajectory_locations(vector<WORKER> &worker, int Number_BusStop);
    void Prodece_Worker_endTime_range_score(vector<WORKER> &worker, vector<double> &Sumdis); //, double endtimeX, double rangeX, double scoreX, int speed

    DataManager(/* args */);
    DataManager(double endtimeX_, double rangeX_, double scoreX_, double speed_) : endtimeX(endtimeX_), rangeX(rangeX_), scoreX(scoreX_), speed(speed_){};

    ~DataManager();
};

DataManager::DataManager(/* args */)
{
}

DataManager::~DataManager()
{
}

/**
 * 生成task的其它属性
 */
void DataManager::Prodece_Task_Reward_Minscore_Deadline(vector<TASK> &task)
{ // 随机生成task的reward和minscore
    int Number_Task = task.size();
    default_random_engine e1, e2, e3, e4;

    uniform_real_distribution<double> u1(scoreMinRangeTask, scoreMaxRangeTask);         // score
    uniform_real_distribution<double> u2(rewardMinRangeTask, rewardMaxRangeTask);       // reward
    uniform_real_distribution<double> u3(deadlineMinRangeTask, deadlineMaxRangeTask);   // Deadline
    uniform_real_distribution<double> u4(startTimeMinRangeTask, startTimeMaxRangeTask); // startTime
    double start_time = 0;
    double end_time = 0;

    for (int i = 0; i < Number_Task; ++i)
    {
        task[i].Minscore = u1(e1);
        task[i].Reward = u2(e2);

        end_time = u3(e3);
        start_time = u4(e4);
        while (start_time >= end_time)
        {
            end_time = u3(e3);
            start_time = u4(e4);
        }

        task[i].startTime = start_time;
        task[i].Deadline = end_time;
        //   cout << "-------------score:"<< task[i].Minscore << endl;
        // cout << "-------------startTime:" << start_time << endl;
        // cout << "-------------Deadline:" << end_time << endl;
    }
    // sort(task.begin(), task.end(), cmp_task_start); // 对任务和工人分别按照开始时间进行升序排序
}
/***
 * 从文件读取获取任务位置
 */
void DataManager::ReadLocationForTask(vector<TASK> &task)
{
    int Number_Task = task.size();
    ifstream in("../../dataset\\Berlin\\Task_LocationBER.txt"); // 打开文件
    // ifstream in2("../../dataset\\RestaurantsAMS_Reward_Minscore.txt"); //打开文件
    // 读数据。
    for (int i = 0; i < Number_Task; ++i)
    {
        for (int j = 0; j < 5; ++j)
        {
            double temp;
            in >> temp;

            if (j == 1)
                task[i].X = temp;
            if (j == 2)
                task[i].Y = temp;
            // cout << j << endl;
        }
    }
    in.close(); // 关闭文件
}

/***
 * 从文件读取获取woker的轨迹
 */
void DataManager::Get_Trajectory_locations(vector<WORKER> &worker, int Number_BusStop)
{
    ifstream in1("../../dataset\\Berlin\\BusRoutesAsStopsBER.txt");
    ifstream in2("../../dataset\\Berlin\\BusStopsBER.txt");
    struct BusStop
    {
        int BusStopId;
        double X;
        double Y;
        /* data */
    };
    vector<BusStop> BusStops;
    for (int i = 0; i < Number_BusStop; ++i)
    {
        for (int j = 0; j < 5; ++j)
        {
            BusStop temp;
            double tm;
            in2 >> tm;
            if (j == 0)
                temp.BusStopId = (int)tm;

            if (j == 1)
                temp.X = tm;

            if (j == 2)
            {
                temp.Y = tm;
                BusStops.push_back(temp);
            }
        }
    }
    in2.close(); // 关闭文件

    vector<POI> trajectory;
    string line;
    regex pat_regex("[[:digit:]]+");
    int i = 0, p = 0;
    while (p < worker.size())
    { // 按行读取
        getline(in1, line);
        int j = 0;
        for (sregex_iterator it(line.begin(), line.end(), pat_regex), end_it; it != end_it; ++it)
        {
            if (j != 0)
            {
                int temp = stoi(it->str());
                vector<BusStop>::iterator itt;
                for (itt = BusStops.begin(); itt != BusStops.end(); itt++)
                {
                    int id = (*itt).BusStopId;
                    if (temp == id)
                    {
                        POI poi;
                        poi.X = (*itt).X;
                        poi.Y = (*itt).Y;
                        worker[i].trajectory.push_back(poi);
                        break;
                    }
                }
            }
            j++;
        }

        i++;
        p++;
    }
    in1.close();
}

/****
 * 生成工人信息分数，评分时间，等等等，已修改
 * 诗婷原本的代码中的截止时间计算方法为：（距离/速度）*（1+2000）
 * @rangeX区域：是定值，注意是够需要修改。
 * @endtimeX截止时间：是定值，注意是够需要修改
 */
void DataManager::Prodece_Worker_endTime_range_score(vector<WORKER> &worker, vector<double> &Sumdis) // 省略了原文的double endtimeX, double rangeX, double scoreX, int speed
{
    // 设置参数endtime,range,score,Maxdetour;
    default_random_engine e, e1;
    uniform_real_distribution<double> u(scoreMinRangeWorker, scoreMaxRangeWorker);          // score
    uniform_real_distribution<double> u1(startTimeMinRangeWorker, startTimeMaxRangeWorker); // 开始时间
    for (int i = 0; i < worker.size(); ++i)
    {
        worker[i].startTime = u1(e1);
        worker[i].endTime = worker[i].startTime + (Sumdis[i] / speed) * (1 + endtimeX);
        worker[i].range = rangeX;
        worker[i].score = u(e) * scoreX;
        /*    cout <<"endTime: "<< worker[i].endTime << endl;
           cout <<"sumdis: "<< Sumdis[i] << endl;
        cout <<"MAXdetour: "<< worker[i].MAXdetour << endl;
        cout <<"range: "<< worker[i].range << endl;
        cout <<"score: "<< worker[i].score << endl;
        */
    }
    // sort(worker.begin(), worker.end(), cmp_worker_start); // hy新增对工人排序

    // sortWork(worker, Sumdis, global_Sumdis);
}

class DataManage_G_mission
{
    /*
    需要注意的是：
        开始时间和节数时间要一个区域范围内
        因为是欧氏距离，所以范围远远小于经纬度。
        里面的生成参数需要修改
    */
private:
    /* data */

public:
    double endtimeX;
    double rangeX;
    double scoreX;
    double speed;
    void ReadLocationForTask(vector<TASK> &task);
    void Prodece_Task_Reward_Minscore_Deadline(vector<TASK> &task);

    void Get_Trajectory_locations(vector<WORKER> &worker);
    void Prodece_Worker_endTime_range_score(vector<WORKER> &worker, vector<double> &Sumdis); // double endtimeX, double rangeX, double scoreX, int speed

    DataManage_G_mission(double endtimeX_, double rangeX_, double scoreX_, double speed_) : endtimeX(endtimeX_), rangeX(rangeX_), scoreX(scoreX_), speed(speed_){};
    DataManage_G_mission(/* args */);
    ~DataManage_G_mission();
};

DataManage_G_mission::DataManage_G_mission(/* args */)
{
}

DataManage_G_mission::~DataManage_G_mission()
{
}
/***
 * 从文件读取获取任务位置
 */
void DataManage_G_mission::ReadLocationForTask(vector<TASK> &task)
{
    ifstream in("../../dataset\\G-mission\\task_information.txt"); // 打开文件
    int Number_Task = task.size();
    if (in)
    {
        for (int i = 0; i < Number_Task; ++i)
        {
            for (int j = 0; j < 5; ++j)
            {
                double temp;
                in >> temp;
                if (j == 2)
                    task[i].X = temp;
                if (j == 3)
                    task[i].Y = temp;
                if (j == 4) // 获取报酬
                    task[i].Reward = temp;
            }
        }

        in.close(); // 关闭文件
    }
}
/**
 * 生成task的其它属性
 */
void DataManage_G_mission::Prodece_Task_Reward_Minscore_Deadline(vector<TASK> &task)
{ // 随机生成task的reward和minscore
    int Number_Task = task.size();
    default_random_engine e1, e2, e3, e4;
    uniform_real_distribution<double> u1(0.7, 0.9); // score,数据集中有工人分数，因此生成区间已确定
    // uniform_real_distribution<double> u2(rewardMinRangeTask, rewardMaxRangeTask);       // reward，数据集中有
    uniform_real_distribution<double> u3(deadlineMinRangeTask, deadlineMaxRangeTask);   // Deadline
    uniform_real_distribution<double> u4(startTimeMinRangeTask, startTimeMaxRangeTask); // startTime
    double start_time = 0;
    double end_time = 0;
    for (int i = 0; i < Number_Task; ++i)
    {
        task[i].Minscore = u1(e1);

        end_time = u3(e3);
        start_time = u4(e4);
        while (start_time >= end_time)
        {
            end_time = u3(e3);
            start_time = u4(e4);
        }

        task[i].startTime = start_time;
        task[i].Deadline = end_time;
        //   cout << "-------------score:"<< task[i].Minscore << endl;
        // cout << "-------------startTime:" << start_time << endl;
        // cout << "-------------Deadline:" << end_time << endl;
    }
    // sort(task.begin(), task.end(), cmp_task_start); // 对任务和工人分别按照开始时间进行升序排序
}

/**
 *
 * 获取工人的轨迹。
 */
void DataManage_G_mission::Get_Trajectory_locations(vector<WORKER> &worker)
{
    ifstream in1("../../dataset\\G-mission\\200worker\\data.txt");
    int Number_Worker = worker.size();
    for (int i = 0; i < Number_Worker; ++i)
    {
        for (int k = 0; k < 5; k++)
        {
            POI poi;
            for (int j = 0; j < 4; ++j)
            {
                double temp;
                in1 >> temp;
                if (j == 2)
                    poi.X = temp;
                if (j == 3)
                {
                    poi.Y = temp;
                    worker[i].trajectory.push_back(poi);
                }
            }
        }
    }

    in1.close();

    // ifstream in2("../../dataset\\G-mission\\worker-score.txt");
    ifstream in2("../../dataset\\G-mission\\200worker\\score.txt");
    for (int i = 0; i < Number_Worker; ++i)
    {
        for (int k = 0; k < 3; k++)
        {
            double temp;
            in2 >> temp;
            if (k == 2)
                worker[i].score = temp;
        }
    }
    in2.close();
}

/****
 * 诗婷原本的代码中的截止时间计算方法为：（距离/速度）*（1+2000）
 * @rangeX区域：是定值，注意是够需要修改。
 * @endtimeX截止时间：是定值，注意是够需要修改
 */
void DataManage_G_mission::Prodece_Worker_endTime_range_score(vector<WORKER> &worker, vector<double> &Sumdis) //, double endtimeX, double rangeX, double scoreX, int speed
{
    // 设置参数endtime,range,score,Maxdetour;
    default_random_engine e1;
    // uniform_real_distribution<double> u(60, 100); // score
    // uniform_real_distribution<double> u1(0, 5); // 开始时间

    uniform_real_distribution<double> u(scoreMinRangeWorker, scoreMaxRangeWorker);          // score
    uniform_real_distribution<double> u1(startTimeMinRangeWorker, startTimeMaxRangeWorker); // 开始时间
    for (int i = 0; i < worker.size(); ++i)
    {
        worker[i].startTime = u1(e1);
        worker[i].endTime = worker[i].startTime + (Sumdis[i] / speed) * (1000 + endtimeX); // 这里是乘以10000，而不是以前的1，原因是欧氏距离的长度很短，如果是1生成的开始时间和节数时间间隔很短。
        worker[i].range = 1 * rangeX;
        /*    cout <<"endTime: "<< worker[i].endTime << endl;
           cout <<"sumdis: "<< Sumdis[i] << endl;
        cout <<"MAXdetour: "<< worker[i].MAXdetour << endl;
        cout <<"range: "<< worker[i].range << endl;
        cout <<"score: "<< worker[i].score << endl;
        */
    }
    // sort(worker.begin(), worker.end(), cmp_worker_start); // hy新增对工人排序
    // sortWork(worker, Sumdis, global_Sumdis);
}

double rad(double d)
{
    return d * PI / 180.0;
}
/**
 * 计算经纬度距离公式
 */
double GetDistance(double lat1, double lng1, double lat2, double lng2)
{
    double radLat1 = rad(lat1);
    double radLng1 = rad(lng1);
    double radLat2 = rad(lat2);
    double radLng2 = rad(lng2);
    double a = abs(radLng1 - radLng2);
    double b = abs(radLat1 - radLat2);
    double h = pow(sin(b / 2), 2) + cos(lat1) * cos(lat2) * pow(sin(a / 2), 2);
    double s = 2 * EARTH_RADIUS * sin(sqrt(h)) * 1000;
    return s;
}

class DataManager_T_Drive
{
private:
    /* data */
    // worker number不超过这个record就行了
public:
    double endtimeX;
    double rangeX;
    double scoreX;
    double speed;
    int Worker_Record;           // 最多有这么些个工人
    int Number_Trajectory_Point; // 每个工人取Number_Trajectory_Point个轨迹点
    vector<WORKER> workerRecord; // 数据集是筛选出的工人
    vector<double> Sumdis1;
    vector<vector<double>> Worker_subTrajectoryDis1;
    void ReproduceWorker(vector<WORKER> &worker, vector<double> &Sumdis, vector<vector<double>> &global_Worker_subTrajectoryDis);
    void Produce_Random_Task_ForTrajectoryPoint(int tasknumber, int trajectory_point, vector<WORKER> &worker, vector<TASK> &task);
    void Get_Trajectory_locations();
    void Caculate_Sumdist_Trajectory();
    // void Caculate_Sumdist_Trajectory(vector<double> &Sumdis1, vector<double> (&Worker_subTrajectoryDis1)[Worker_Record], WORKER (&workerRecord)[Worker_Record]);
    void Prodece_Task_Reward_Minscore_Deadline(vector<TASK> &task);
    void ReadLocationForTask(vector<TASK> &task);

    void Prodece_Worker_endTime_range_score(vector<WORKER> &worker, vector<double> &Sumdis); //, double endtimeX, double rangeX, double scoreX, int speed

    // void ReproduceWorker(WORKER (&worker)[Number_Worker], WORKER (&workerRecord)[Worker_Record], double (&Sumdis1)[Worker_Record], double (&Sumdis)[Number_Worker], vector<double> (&Worker_subTrajectoryDis)[Number_Worker], vector<double> (&Worker_subTrajectoryDis1)[Worker_Record]);
    // void Produce_Random_Task_ForTrajectoryPoint(int tasknumber, int trajectory_point);
    DataManager_T_Drive(double endtimeX_, double rangeX_, double scoreX_, double speed_, int Worker_Record_, int Number_Trajectory_Point_) : endtimeX(endtimeX_), rangeX(rangeX_), scoreX(scoreX_), speed(speed_), Worker_Record(Worker_Record_), Number_Trajectory_Point(Number_Trajectory_Point_)
    {
        workerRecord.resize(Worker_Record_); // 从数据集文件中获得
        Sumdis1.resize(Worker_Record_);
        Worker_subTrajectoryDis1.resize(Worker_Record_, vector<double>(Number_Trajectory_Point_));
    };

    DataManager_T_Drive(/* args */);
    ~DataManager_T_Drive();
};

DataManager_T_Drive::DataManager_T_Drive(/* args */)
{
}

DataManager_T_Drive::~DataManager_T_Drive()
{
}

/**
 * 对生成工人进行筛选
 * 要求距离要处于某个阶段
 * 不过在这里我没有设计
 */
void DataManager_T_Drive::ReproduceWorker(vector<WORKER> &worker, vector<double> &Sumdis, vector<vector<double>> &Worker_subTrajectoryDis)
{
    int j = 0;
    int Number_Worker = worker.size();
    for (int i = 0; i < Worker_Record; i++)
    {
        if (j < Number_Worker)
        {

            // if (Sumdis1[i] >= 15000 && Sumdis1[i] <= 160000)
            {
                worker[j] = workerRecord[i];
                Sumdis[j] = Sumdis1[i];
                Worker_subTrajectoryDis[j] = Worker_subTrajectoryDis1[i];

                // cout << i << "的原始信息：" << workerRecord[i].endTime << "\t" << workerRecord[i].range << "\t" << workerRecord[i].score << "\t" << workerRecord[i].trajectory.size() << endl;
                // cout << Sumdis1[i] << endl;
                // cout << endl;
                // cout << j << "工人的信息：" << worker[j].endTime << "\t" << worker[j].range << "\t" << worker[j].score << "\t" << worker[j].trajectory.size() << endl;
                // cout << Sumdis[j] << endl;
                // cout << endl;

                j++;
            }
        }
    }
    cout << "worker总数：" << j << endl;
    cout << "worker总数：" << worker.size() << endl;
}
void DataManager_T_Drive::Produce_Random_Task_ForTrajectoryPoint(int tasknumber, int trajectory_point, vector<WORKER> &worker, vector<TASK> &task)
{
    default_random_engine e;
    int tasknum = 0;
    int Number_Worker = worker.size();
    int Number_Task = task.size();

    for (int i = 0; i < Number_Worker; i++)
        for (int j = 0; j < tasknumber; j++)
        {
            int t = rand() % trajectory_point; // 返回0-39的数字

            uniform_real_distribution<double> u2(worker[i].trajectory[t].X - 0.04, worker[i].trajectory[t].X + 0.04); // 经度范围
            uniform_real_distribution<double> u3(worker[i].trajectory[t].Y - 0.04, worker[i].trajectory[t].Y + 0.04); // 纬度范围
            task[tasknum].X = u2(e);
            task[tasknum].Y = u3(e);
            // cout << task[tasknum].X <<endl;
            //  cout << task[tasknum].Y <<endl;
            tasknum++;
        }
    while (tasknum < Number_Task)
    {                                                                                                                  // 随机选工人，随机选取轨迹点
        int wi = rand() % Number_Worker;                                                                               // 返回0-NumberWorker-1的数字
        int ti = rand() % Number_Trajectory_Point;                                                                     // 返回0-Number_Trajectory_Point-1的数字
        uniform_real_distribution<double> u2(worker[wi].trajectory[ti].X - 0.005, worker[wi].trajectory[ti].X + 0.05); // 经度范围
        uniform_real_distribution<double> u3(worker[wi].trajectory[ti].Y - 0.005, worker[wi].trajectory[ti].Y + 0.05); // 纬度范围
        task[tasknum].X = u2(e);
        task[tasknum].Y = u3(e);
        //   cout<<tasknum<<"任务"<<"的经纬度："<< task[tasknum].X <<","<<task[tasknum].Y<<endl;
        tasknum++;
    }
    cout << "任务总数：" << tasknum << endl;
}
void DataManager_T_Drive::Get_Trajectory_locations()
{
    int t = 1, w = 1;
    while (w <= Worker_Record)
    {
        stringstream ss;
        string str;
        ss << t;
        ss >> str;
        ifstream in("../../dataset\\T-Drive\\" + str + ".txt");
        // cout << "sting  ss:" << str.c_str() << endl;
        if (in)
        {
            vector<POI> trajectory;
            string line;
            int tranum = 0;
            while (tranum < 40) // 按行读取
            {
                getline(in, line);
                if (tranum % 4 == 0)
                {
                    POI poi;
                    char *s_input = (char *)line.c_str();
                    const char *split = ",";
                    char *p = strtok(s_input, split);
                    double a;
                    int i = 0;
                    while (p != NULL)
                    {
                        a = atof(p);
                        if (a == 0)
                        {
                            cout << t << endl;
                            cout << endl;
                        }
                        if (i == 2)
                        {
                            if (a < 115.7 && a > 117.4) // 北京的经度范围
                            {
                                cout << t << endl;
                                cout << endl;
                            }
                            poi.X = a;
                        }
                        if (i == 3)
                        {
                            if (a < 39.4 && a > 41.6) // 北京的纬度范围
                            {
                                cout << t << endl;
                                cout << endl;
                            }
                            poi.Y = a;
                        }
                        p = strtok(NULL, split);
                        i++;
                    }
                    workerRecord[w - 1].trajectory.push_back(poi);
                }
                tranum++;
            }
            in.close();
            w++;
        }
        else
        {
            // cout<<"文件不存在！"<<endl;
        }
        t++;
    }
    cout << "工人的轨迹文件读取完成！" << endl;
    // cout<<endl;

    int p = 0, z = 0;
    vector<int> numbb;
    for (int i = 0; i < Worker_Record; i++) // 输出trajectory_location
    {
        // cout<<"工人"<<i<<"的轨迹点数量："<<workerRecord[i].trajectory.size()<<endl;

        for (int j = 0; j < workerRecord[i].trajectory.size(); j++)
        {
            // cout<<"("<<workerRecord[i].trajectory[j].X<<","<<workerRecord[i].trajectory[j].Y<<")"<<" "<<endl;
            if (workerRecord[i].trajectory[j].X == 0 || workerRecord[i].trajectory[j].Y == 0)
            {
                z++;
                numbb.push_back(i);
            }
        }
        // cout<<endl;
        if (workerRecord[i].trajectory.size() == 0)
            p++;
    }
    // cout<<"轨迹点为0的工人数目"<<p<<endl;
    // cout<<endl;
    if (z != 0)
    {
        cout << "轨迹点经纬度为0的工人有几个人：" << z << endl;
        for (int j = 0; j < numbb.size(); j++)
        {
            cout << "工人的序号为:" << numbb[j] << endl;
        }
    }
    cout << endl;
}
void DataManager_T_Drive::Caculate_Sumdist_Trajectory()
{
    for (int i = 0; i < Worker_Record; i++)
    {
        double sum = 0;
        Worker_subTrajectoryDis1[i].push_back(0); // 第一个轨迹点为0
        for (int j = 0; j < workerRecord[i].trajectory.size() - 1; j++)
        {
            sum = sum + GetDistance(workerRecord[i].trajectory[j].Y, workerRecord[i].trajectory[j].X, workerRecord[i].trajectory[j + 1].Y, workerRecord[i].trajectory[j + 1].X);
            Worker_subTrajectoryDis1[i].push_back(sum);
            //	  cout<<sum<<endl;
        }
        Sumdis1[i] = sum;
        cout << i << " "
             << "all:" << Sumdis1[i] << endl;
    }

    for (int i = 0; i < Worker_Record; i++)
    {
        double sum = 0;
        Worker_subTrajectoryDis1[i].push_back(0); // 第一个轨迹点为0
        for (int j = 0; j < workerRecord[i].trajectory.size() - 1; j++)
        {
            sum = sum + GetDistance(workerRecord[i].trajectory[j].Y, workerRecord[i].trajectory[j].X, workerRecord[i].trajectory[j + 1].Y, workerRecord[i].trajectory[j + 1].X);
            Worker_subTrajectoryDis1[i].push_back(sum);
            cout << "sssssuuummmm: " << sum << endl;
        }
        Sumdis1[i] = sum;
        //   cout<<i<<" "<<"all:" << Sumdis[i] <<endl;
    }
}
// void Caculate_Sumdist_Trajectory(vector<double> &Sumdis1, vector<double> (&Worker_subTrajectoryDis1)[Worker_Record], WORKER (&workerRecord)[Worker_Record]);
void DataManager_T_Drive::Prodece_Task_Reward_Minscore_Deadline(vector<TASK> &task)
{
    int Number_Task = task.size();
    default_random_engine e1, e2, e3, e4;
    // uniform_real_distribution<double> u1(60,100);  //score来控制随机数引擎生成60到100之间的实数。
    // uniform_int_distribution<unsigned> u2(1,10);  //reward来控制随机数引擎生成1到10之间的整数。

    normal_distribution<double> u1((deadlineMinRangeTask + scoreMaxRangeTask) / 2, 30); // 来控制随机数引擎生成均值为80，标准差为30的正态分布数据。
    normal_distribution<double> u2((rewardMinRangeTask + rewardMaxRangeTask) / 2, 10);
    // uniform_real_distribution<double> u1(scoreMinRangeTask, scoreMaxRangeTask);         // score
    // uniform_real_distribution<double> u2(rewardMinRangeTask, rewardMaxRangeTask);       // reward
    uniform_real_distribution<double> u3(deadlineMinRangeTask, deadlineMaxRangeTask);   // Deadline
    uniform_real_distribution<double> u4(startTimeMinRangeTask, startTimeMaxRangeTask); // startTime
    double start_time = 0;
    double end_time = 0;

    for (int i = 0; i < Number_Task; ++i)
    {
        task[i].Minscore = u1(e1);
        task[i].Reward = u2(e2);

        end_time = u3(e3);
        start_time = u4(e4);
        while (start_time >= end_time)
        {
            end_time = u3(e3);
            start_time = u4(e4);
        }

        task[i].startTime = start_time;
        task[i].Deadline = end_time;
        //   cout << "-------------score:"<< task[i].Minscore << endl;
        // cout << "-------------startTime:" << start_time << endl;
        // cout << "-------------Deadline:" << end_time << endl;
    }
}

void DataManager_T_Drive::Prodece_Worker_endTime_range_score(vector<WORKER> &worker, vector<double> &Sumdis)
{
    int Number_Worker = worker.size();
    // 设置参数endtime,range,score,Maxdetour;
    default_random_engine e, e1;
    // // uniform_real_distribution<double> u(60,100);  //score
    // normal_distribution<double> u(80, 10);       // 正态分布
    // uniform_real_distribution<double> u1(0, 50); // startTime

    uniform_real_distribution<double> u((scoreMinRangeWorker + scoreMaxRangeWorker) / 2, (scoreMinRangeWorker - scoreMaxRangeWorker) / 2); // score
    uniform_real_distribution<double> u1(startTimeMinRangeWorker, startTimeMaxRangeWorker);                                                // startTime

    for (int i = 0; i < Number_Worker; ++i)
    {

        worker[i].startTime = u1(e1);
        worker[i].endTime = worker[i].startTime + (Sumdis[i] / speed) * (1 + endtimeX);
        worker[i].range = rangeX;
        worker[i].score = u(e) * scoreX;
        /*    cout <<"endTime: "<< worker[i].endTime << endl;
           cout <<"sumdis: "<< Sumdis[i] << endl;
        cout <<"MAXdetour: "<< worker[i].MAXdetour << endl;
        cout <<"range: "<< worker[i].range << endl;
        cout <<"score: "<< worker[i].score << endl;
        */
    }
    // sortWork(worker, Sumdis, global_Sumdis);
}

#endif // _DATAMANAGER_H_