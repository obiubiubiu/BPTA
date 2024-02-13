#ifndef _DATAMANAGER_H_
#define _DATAMANAGER_H_

#include "Basic_information.h"
#include <chrono>

int scoreMinRangeTask = 60;
int scoreMaxRangeTask = 100;

int rewardMinRangeTask = 60;
int rewardMaxRangeTask = 80;

int deadlineMinRangeTask = 30;
int deadlineMaxRangeTask = 80;

int startTimeMinRangeTask = 0;
int startTimeMaxRangeTask = 50;

int scoreMinRangeWorker = 50;
int scoreMaxRangeWorker = 110;

int startTimeMinRangeWorker = 0;
int startTimeMaxRangeWorker = 60;

/*******
 * 修改了任务生成时间，要求开始时间在截止时间之前
 * 注意这里存在无线循环的情况，即，开始时间一直在截止时间之前。
 *
 */
class DataManager
{
private:
public:
    double endtimeX;
    double taskEndtimeX;
    double rangeX;
    double scoreX;
    double speed;
    bool distributionOption;
    void Prodece_Task_Reward_Minscore_Deadline(vector<TASK> &task);
    void ReadLocationForTask(vector<TASK> &task);
    void Get_Trajectory_locations(vector<WORKER> &worker, int Number_BusStop);
    void Prodece_Worker_endTime_range_score(vector<WORKER> &worker, vector<double> &Sumdis);

    DataManager();
    DataManager(bool distributionOption_, double endtimeX_, double taskEndtimeX_, double rangeX_, double scoreX_, double speed_) : distributionOption(distributionOption_), endtimeX(endtimeX_), taskEndtimeX(taskEndtimeX_), rangeX(rangeX_), scoreX(scoreX_), speed(speed_){};

    ~DataManager();
};

DataManager::DataManager()
{
}

DataManager::~DataManager()
{
}

/**
 * 生成task的其它属性
 */
void DataManager::Prodece_Task_Reward_Minscore_Deadline(vector<TASK> &task)
{
    int Number_Task = task.size();

    double start_time = 0;
    double end_time = 0;
    default_random_engine e1, e2, e3, e4;

    if (distributionOption)
    {
        uniform_real_distribution<double> u1(scoreMinRangeTask, scoreMaxRangeTask);
        uniform_real_distribution<double> u2(rewardMinRangeTask, rewardMaxRangeTask);
        uniform_real_distribution<double> u3(deadlineMinRangeTask, deadlineMaxRangeTask);
        uniform_real_distribution<double> u4(startTimeMinRangeTask, startTimeMaxRangeTask);

        e1.seed(chrono::system_clock::now().time_since_epoch().count());
        e2.seed(chrono::system_clock::now().time_since_epoch().count() + 0.01);
        e3.seed(chrono::system_clock::now().time_since_epoch().count() + 0.02);
        e4.seed(chrono::system_clock::now().time_since_epoch().count() + 0.03);

        for (int i = 0; i < Number_Task; ++i)
        {
            task[i].Minscore = abs(u1(e1));
            task[i].Reward = abs(u2(e2));

            end_time = abs(u3(e3));
            start_time = abs(u4(e4));
            while (start_time >= end_time)
            {
                end_time = abs(u3(e3));
            }

            task[i].startTime = start_time;
            task[i].Deadline = end_time;
        }
    }
    else
    {
        e1.seed(chrono::system_clock::now().time_since_epoch().count());
        e2.seed(chrono::system_clock::now().time_since_epoch().count() + 0.01);
        e3.seed(chrono::system_clock::now().time_since_epoch().count() + 0.02);
        e4.seed(chrono::system_clock::now().time_since_epoch().count() + 0.03);

        normal_distribution<double> u1((scoreMinRangeTask + scoreMaxRangeTask) / 2, 10);
        normal_distribution<double> u2((rewardMinRangeTask + rewardMaxRangeTask) / 2, 20);
        normal_distribution<double> u3((deadlineMinRangeTask + deadlineMaxRangeTask) / 2, 30);
        normal_distribution<double> u4((startTimeMinRangeTask + startTimeMaxRangeTask) / 2, 20);

        for (int i = 0; i < Number_Task; ++i)
        {
            task[i].Minscore = abs(u1(e1));
            task[i].Reward = abs(u2(e2));

            end_time = abs(u3(e3));
            start_time = abs(u4(e4));
            while (start_time >= end_time)
            {
                end_time = abs(u3(e3));
            }

            task[i].startTime = start_time;
            task[i].Deadline = end_time;
        }
    }
}
/***
 * 从文件读取获取任务位置
 */
void DataManager::ReadLocationForTask(vector<TASK> &task)
{

    int Number_Task = task.size();
    ifstream in("../../dataset/Berlin/Task_LocationBER.txt");

    for (int i = 0; i < Number_Task; ++i)
    {

        for (int j = 0; j < 5; ++j)
        {
            double temp;
            in >> temp;

            if (j == 1)
            {
                task[i].X = temp;
            }

            if (j == 2)
            {
                task[i].Y = temp;
            }
        }
    }
    in.close();
}

/***
 * 从文件读取获取woker的轨迹
 */
void DataManager::Get_Trajectory_locations(vector<WORKER> &worker, int Number_BusStop)
{
    ifstream in1("../../dataset/Berlin/BusRoutesAsStopsBER.txt");
    ifstream in2("../../dataset/Berlin/BusStopsBER.txt");
    struct BusStop
    {
        int BusStopId;
        double X;
        double Y;
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
            {
                temp.BusStopId = (int)tm;
            }

            if (j == 1)
            {
                temp.X = tm;
            }

            if (j == 2)
            {
                temp.Y = tm;
            }

            BusStops.push_back(temp);
        }
    }
    in2.close();

    vector<POI> trajectory;
    string line;
    regex pat_regex("[[:digit:]]+");
    int i = 0, p = 0;

    while (p < worker.size() && p <= 235)
    {
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
void DataManager::Prodece_Worker_endTime_range_score(vector<WORKER> &worker, vector<double> &Sumdis)
{

    std::default_random_engine e(9999), e1(871238);

    if (distributionOption)
    {
        uniform_real_distribution<double> u(scoreMinRangeWorker, scoreMaxRangeWorker);
        uniform_real_distribution<double> u1(startTimeMinRangeWorker, startTimeMaxRangeWorker);
        e1.seed(chrono::system_clock::now().time_since_epoch().count());
        e.seed(chrono::system_clock::now().time_since_epoch().count() + 0.01);

        for (int i = 0; i < worker.size(); ++i)
        {
            worker[i].startTime = abs(u1(e1));
            worker[i].endTime = worker[i].startTime + (Sumdis[i] / speed) * (1 + endtimeX);
            worker[i].range = rangeX;
            worker[i].score = abs(u(e)) * scoreX;
        }
    }
    else
    {
        normal_distribution<double> u((scoreMinRangeWorker + scoreMaxRangeWorker) / 2, 20);
        normal_distribution<double> u1((startTimeMinRangeWorker + startTimeMaxRangeWorker) / 2, 20);

        e1.seed(chrono::system_clock::now().time_since_epoch().count());
        e.seed(chrono::system_clock::now().time_since_epoch().count() + 0.01);
        for (int i = 0; i < worker.size(); ++i)
        {
            worker[i].startTime = abs(u1(e1));
            worker[i].endTime = worker[i].startTime + (Sumdis[i] / speed) * (1 + endtimeX);
            worker[i].range = rangeX;
            worker[i].score = abs(u(e)) * scoreX;
        }
    }
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
public:
    double endtimeX;
    double taskEndtimeX;
    double rangeX;
    double scoreX;
    double speed;
    bool distributionOption;
    void ReadLocationForTask(vector<TASK> &task);
    void Prodece_Task_Reward_Minscore_Deadline(vector<TASK> &task);

    void Get_Trajectory_locations(vector<WORKER> &worker);
    void Prodece_Worker_endTime_range_score(vector<WORKER> &worker, vector<double> &Sumdis);

    DataManage_G_mission(bool distributionOption_, double endtimeX_, double taskEndtimeX_, double rangeX_, double scoreX_, double speed_) : distributionOption(distributionOption_), endtimeX(endtimeX_), taskEndtimeX(taskEndtimeX_), rangeX(rangeX_), scoreX(scoreX_), speed(speed_){};
    DataManage_G_mission();
    ~DataManage_G_mission();
};

DataManage_G_mission::DataManage_G_mission()
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
    ifstream in("../../dataset/G-mission/task_information.txt");
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
                if (j == 4)
                    task[i].Reward = temp;
            }
        }

        in.close();
    }
}
/**
 * 生成task的其它属性
 */
void DataManage_G_mission::Prodece_Task_Reward_Minscore_Deadline(vector<TASK> &task)
{
    int Number_Task = task.size();
    std::default_random_engine e1(2013467), e2(365485449), e3(3333), e4(12387);
    double start_time = 0;
    double end_time = 0;

    if (distributionOption)
    {
        uniform_real_distribution<double> u1(0.7, 0.9);

        uniform_real_distribution<double> u3(deadlineMinRangeTask, deadlineMaxRangeTask);
        uniform_real_distribution<double> u4(startTimeMinRangeTask, startTimeMaxRangeTask);

        e1.seed(chrono::system_clock::now().time_since_epoch().count());

        e3.seed(chrono::system_clock::now().time_since_epoch().count() + 0.02);
        e4.seed(chrono::system_clock::now().time_since_epoch().count() + 0.03);

        for (int i = 0; i < Number_Task; ++i)
        {
            task[i].Minscore = abs(u1(e1));

            end_time = abs(u3(e3));
            start_time = abs(u4(e4));
            while (start_time >= end_time)
            {
                end_time = abs(u3(e3));
            }

            task[i].startTime = start_time;
            task[i].Deadline = end_time;
        }
    }
    else
    {
        normal_distribution<double> u1(0.8, 0.1);
        normal_distribution<double> u3((deadlineMinRangeTask + deadlineMaxRangeTask) / 2, 30);
        normal_distribution<double> u4((startTimeMinRangeTask + startTimeMaxRangeTask) / 2, 10);

        e1.seed(chrono::system_clock::now().time_since_epoch().count());
        e2.seed(chrono::system_clock::now().time_since_epoch().count() + 0.01);
        e3.seed(chrono::system_clock::now().time_since_epoch().count() + 0.02);
        e4.seed(chrono::system_clock::now().time_since_epoch().count() + 0.03);

        for (int i = 0; i < Number_Task; ++i)
        {

            task[i].Minscore = abs(u1(e1));

            end_time = abs(u3(e3));
            start_time = abs(u4(e4));

            while (start_time >= end_time)
            {
                end_time = abs(u3(e3));
            }

            task[i].startTime = start_time;
            task[i].Deadline = end_time;
        }
    }
}

/**
 *
 * 获取工人的轨迹。
 */
void DataManage_G_mission::Get_Trajectory_locations(vector<WORKER> &worker)
{
    ifstream in1("../../dataset/G-mission/200worker/data.txt");
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

    ifstream in2("../../dataset/G-mission/200worker/score.txt");
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
void DataManage_G_mission::Prodece_Worker_endTime_range_score(vector<WORKER> &worker, vector<double> &Sumdis)
{

    std::default_random_engine e1(7451233);

    if (distributionOption)
    {

        uniform_real_distribution<double> u1(startTimeMinRangeWorker, startTimeMaxRangeWorker);

        for (int i = 0; i < worker.size(); ++i)
        {
            e1.seed(chrono::system_clock::now().time_since_epoch().count());
            worker[i].startTime = abs(u1(e1));
            worker[i].endTime = worker[i].startTime + (Sumdis[i] / speed) * (1000 + endtimeX);
            worker[i].range = 1 * rangeX;
            /*    cout <<"endTime: "<< worker[i].endTime << endl;
               cout <<"sumdis: "<< Sumdis[i] << endl;
            cout <<"MAXdetour: "<< worker[i].MAXdetour << endl;
            cout <<"range: "<< worker[i].range << endl;
            cout <<"score: "<< worker[i].score << endl;
            */
        }
    }
    else
    {

        normal_distribution<double> u1((startTimeMinRangeWorker + startTimeMaxRangeWorker) / 2, 20);
        e1.seed(chrono::system_clock::now().time_since_epoch().count());

        for (int i = 0; i < worker.size(); ++i)
        {
            worker[i].startTime = abs(u1(e1));
            worker[i].endTime = worker[i].startTime + (Sumdis[i] / speed) * (1000 + endtimeX);
            worker[i].range = 1 * rangeX;
            /*    cout <<"endTime: "<< worker[i].endTime << endl;
               cout <<"sumdis: "<< Sumdis[i] << endl;
            cout <<"MAXdetour: "<< worker[i].MAXdetour << endl;
            cout <<"range: "<< worker[i].range << endl;
            cout <<"score: "<< worker[i].score << endl;
            */
        }
    }

    cout << "test" << endl;
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
public:
    double endtimeX;
    double taskEndtimeX;
    double rangeX;
    double scoreX;
    double speed;
    int Worker_Record;
    int worker_num_T;
    int Number_Trajectory_Point;
    bool distributionOption;

    void ReproduceWorker(vector<WORKER> &worker, vector<double> &Sumdis, vector<vector<double>> &global_Worker_subTrajectoryDis);
    void Produce_Random_Task_ForTrajectoryPoint(int tasknumber, int trajectory_point, vector<WORKER> &worker, vector<TASK> &task);
    void Get_Trajectory_locations(vector<WORKER> &worker);
    void Caculate_Sumdist_Trajectory(vector<WORKER> &worker, vector<double> &Sumdis, vector<vector<double>> &global_Worker_subTrajectoryDis);

    void Prodece_Task_Reward_Minscore_Deadline(vector<TASK> &task);
    void ReadLocationForTask(vector<TASK> &task);

    void Prodece_Worker_endTime_range_score(vector<WORKER> &worker, vector<double> &Sumdis);

    DataManager_T_Drive(bool distributionOption_, double endtimeX_, double taskEndtimeX_, double rangeX_, double scoreX_, double speed_, int Worker_Record_, int T_Worker_num_, int Number_Trajectory_Point_) : distributionOption(distributionOption_), endtimeX(endtimeX_), taskEndtimeX(taskEndtimeX_), rangeX(rangeX_), scoreX(scoreX_), speed(speed_), Worker_Record(Worker_Record_), worker_num_T(T_Worker_num_), Number_Trajectory_Point(Number_Trajectory_Point_){

                                                                                                                                                                                                                                                                                                                                                                                                                        };

    DataManager_T_Drive();
    ~DataManager_T_Drive();
};

DataManager_T_Drive::DataManager_T_Drive()
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

            {

                j++;
            }
        }
    }
}
void DataManager_T_Drive::Produce_Random_Task_ForTrajectoryPoint(int tasknumber, int trajectory_point, vector<WORKER> &worker, vector<TASK> &task)
{
    default_random_engine e(365485449);
    int tasknum = 0;
    int Number_Worker = worker.size();
    int Number_Task = task.size();

    int numSkip = Number_Worker / Number_Task;
    std::mt19937 generator(555);
    uniform_int_distribution<int> distribution(0, trajectory_point - 1);
    if (numSkip > 0)
    {

        for (int j = 0; j < tasknumber; j++)
        {
            int t = distribution(generator);

            if (distributionOption)
            {
                uniform_real_distribution<double> u2(worker[j * numSkip].trajectory[t].X - 0.04, worker[j * numSkip].trajectory[t].X + 0.04);
                uniform_real_distribution<double> u3(worker[j * numSkip].trajectory[t].Y - 0.04, worker[j * numSkip].trajectory[t].Y + 0.04);
                e.seed(chrono::system_clock::now().time_since_epoch().count());
                task[tasknum].X = abs(u2(e));
                task[tasknum].Y = abs(u3(e));
            }
            else
            {
                normal_distribution<double> u2(worker[j * numSkip].trajectory[t].X, 0.04);
                normal_distribution<double> u3(worker[j * numSkip].trajectory[t].Y, 0.04);
                e.seed(chrono::system_clock::now().time_since_epoch().count());

                task[tasknum].X = abs(u2(e));
                task[tasknum].Y = abs(u3(e));
            }

            tasknum++;
        }
    }
    else
    {

        int numReapt = Number_Task / Number_Worker;
        if (numReapt > trajectory_point)
        {
            numReapt = trajectory_point;
        }

        for (int i = 0; i < Number_Worker; i++)
        {
            for (int j = 0; j < numReapt; j++)
            {
                int t = distribution(generator);

                if (distributionOption)
                {
                    uniform_real_distribution<double> u2(worker[i].trajectory[t].X - 0.04, worker[i].trajectory[t].X + 0.04);
                    uniform_real_distribution<double> u3(worker[i].trajectory[t].Y - 0.04, worker[i].trajectory[t].Y + 0.04);
                    e.seed(chrono::system_clock::now().time_since_epoch().count());

                    task[tasknum].X = abs(u2(e));
                    task[tasknum].Y = abs(u3(e));
                }
                else
                {
                    normal_distribution<double> u2(worker[i].trajectory[t].X, 0.04);
                    normal_distribution<double> u3(worker[i].trajectory[t].Y, 0.04);
                    e.seed(chrono::system_clock::now().time_since_epoch().count());

                    task[tasknum].X = abs(u2(e));
                    task[tasknum].Y = abs(u3(e));
                }

                tasknum++;
            }
        }
    }

    while (tasknum < Number_Task)
    {

        std::mt19937 generator(123);
        uniform_int_distribution<int> distribution(0, Number_Worker - 1);
        uniform_int_distribution<int> distribution1(0, Number_Trajectory_Point - 1);

        int wi = distribution(generator);
        int ti = distribution1(generator);

        if (distributionOption)
        {
            uniform_real_distribution<double> u2(worker[wi].trajectory[ti].X - 0.005, worker[wi].trajectory[ti].X + 0.05);
            uniform_real_distribution<double> u3(worker[wi].trajectory[ti].Y - 0.005, worker[wi].trajectory[ti].Y + 0.05);
            e.seed(chrono::system_clock::now().time_since_epoch().count());

            task[tasknum].X = abs(u2(e));
            task[tasknum].Y = abs(u3(e));
        }
        else
        {
            normal_distribution<double> u2(worker[wi].trajectory[ti].X, 0.05);
            normal_distribution<double> u3(worker[wi].trajectory[ti].Y, 0.05);
            e.seed(chrono::system_clock::now().time_since_epoch().count());

            task[tasknum].X = abs(u2(e));
            task[tasknum].Y = abs(u3(e));
        }

        tasknum++;
    }
}
void DataManager_T_Drive::Get_Trajectory_locations(vector<WORKER> &worker)
{
    int t = 1, w = 1, realW = 1;
    while (w <= Worker_Record && realW <= worker_num_T)
    {
        int lineNum = Number_Trajectory_Point * 4;
        bool sign = false;
        stringstream ss;
        string str;
        ss << t;
        ss >> str;
        ifstream in("../../dataset/T-Drive/" + str + ".txt");

        if (in)
        {
            vector<POI> trajectory;
            string line;
            int tranum = 0;
            while (tranum < lineNum)
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

                        if (i == 2)
                        {
                            if (a < 115.7 || a > 117.4)
                            {
                                sign = true;
                                break;
                            }
                            poi.X = a;
                        }

                        if (i == 3)
                        {
                            if (a < 39.4 || a > 41.6)
                            {
                                sign = true;
                                break;
                            }
                            poi.Y = a;
                        }
                        p = strtok(NULL, split);
                        i++;
                    }

                    worker[realW - 1].trajectory.push_back(poi);
                }
                if (sign)
                {
                    break;
                }

                tranum++;
            }

            if (!sign)
            {
                realW++;
            }

            in.close();
            w++;
        }
        else
        {
        }
        t++;
    }

    int p = 0, z = 0;
    vector<int> numbb;
    for (int i = 0; i < worker_num_T; i++)
    {

        for (int j = 0; j < worker[i].trajectory.size(); j++)
        {

            if (worker[i].trajectory[j].X == 0 || worker[i].trajectory[j].Y == 0)
            {
                z++;
                numbb.push_back(i);
            }
        }

        if (worker[i].trajectory.size() == 0)
            p++;
    }
}
void DataManager_T_Drive::Caculate_Sumdist_Trajectory(vector<WORKER> &worker, vector<double> &Sumdis, vector<vector<double>> &global_Worker_subTrajectoryDis)
{

    for (int i = 0; i < worker_num_T; i++)
    {
        double sum = 0;
        global_Worker_subTrajectoryDis[i].push_back(0);

        for (int j = 0; j < worker[i].trajectory.size() - 1; j++)
        {
            sum = sum + GetDistance(worker[i].trajectory[j].Y, worker[i].trajectory[j].X, worker[i].trajectory[j + 1].Y, worker[i].trajectory[j + 1].X);
            global_Worker_subTrajectoryDis[i].push_back(sum);
        }
        Sumdis[i] = sum;
    }
}

void DataManager_T_Drive::Prodece_Task_Reward_Minscore_Deadline(vector<TASK> &task)
{
    int Number_Task = task.size();
    std::default_random_engine e4(2013467), e3(365485449), e2(3333), e1(12387);
    double start_time = 0;
    double end_time = 0;

    if (distributionOption)
    {
        uniform_real_distribution<double> u1(scoreMinRangeTask, scoreMaxRangeTask);
        uniform_real_distribution<double> u2(rewardMinRangeTask, rewardMaxRangeTask);

        uniform_real_distribution<double> u3(deadlineMinRangeTask, deadlineMaxRangeTask);
        uniform_real_distribution<double> u4(startTimeMinRangeTask, startTimeMaxRangeTask);

        e1.seed(chrono::system_clock::now().time_since_epoch().count());
        e2.seed(chrono::system_clock::now().time_since_epoch().count() + 0.01);
        e3.seed(chrono::system_clock::now().time_since_epoch().count() + 0.02);
        e4.seed(chrono::system_clock::now().time_since_epoch().count() + 0.03);

        for (int i = 0; i < Number_Task; ++i)
        {
            task[i].Minscore = abs(u1(e1));
            task[i].Reward = abs(u2(e2));

            end_time = abs(u3(e3));
            start_time = abs(u4(e4));
            while (start_time >= end_time)
            {
                end_time = abs(u3(e3));
            }

            task[i].startTime = start_time;
            task[i].Deadline = end_time;
        }
    }
    else
    {
        normal_distribution<double> u1((deadlineMinRangeTask + scoreMaxRangeTask) / 2, 30);
        normal_distribution<double> u2((rewardMinRangeTask + rewardMaxRangeTask) / 2, 10);
        normal_distribution<double> u3((deadlineMinRangeTask + deadlineMaxRangeTask) / 2, 10);
        normal_distribution<double> u4((startTimeMinRangeTask + startTimeMaxRangeTask) / 2, 10);

        e1.seed(chrono::system_clock::now().time_since_epoch().count());
        e2.seed(chrono::system_clock::now().time_since_epoch().count() + 0.01);
        e3.seed(chrono::system_clock::now().time_since_epoch().count() + 0.02);
        e4.seed(chrono::system_clock::now().time_since_epoch().count() + 0.03);

        for (int i = 0; i < Number_Task; ++i)
        {
            task[i].Minscore = abs(u1(e1));
            task[i].Reward = abs(u2(e2));

            end_time = abs(u3(e3));
            start_time = abs(u4(e4));
            while (start_time >= end_time)
            {
                end_time = abs(u3(e3));
            }
            task[i].startTime = start_time;
            task[i].Deadline = end_time;
        }
    }
}

void DataManager_T_Drive::Prodece_Worker_endTime_range_score(vector<WORKER> &worker, vector<double> &Sumdis)
{
    int Number_Worker = worker.size();

    default_random_engine e(7451233), e1(2013467), e2(365485449);

    if (distributionOption)
    {
        uniform_real_distribution<double> u(scoreMinRangeWorker, scoreMaxRangeWorker);
        uniform_real_distribution<double> u1(startTimeMinRangeWorker, startTimeMaxRangeWorker);

        e1.seed(chrono::system_clock::now().time_since_epoch().count());
        e.seed(chrono::system_clock::now().time_since_epoch().count() + 0.01);
        for (int i = 0; i < Number_Worker; ++i)
        {

            worker[i].startTime = abs(u1(e1));
            worker[i].endTime = worker[i].startTime + (Sumdis[i] / speed) * (1 + endtimeX);
            worker[i].range = rangeX;
            worker[i].score = abs(u(e)) * scoreX;
        }
    }
    else
    {
        normal_distribution<double> u((scoreMinRangeWorker + scoreMaxRangeWorker) / 2, 20);
        normal_distribution<double> u1((startTimeMinRangeWorker + startTimeMaxRangeWorker) / 2, 20);
        e1.seed(chrono::system_clock::now().time_since_epoch().count());
        e.seed(chrono::system_clock::now().time_since_epoch().count() + 0.01);

        for (int i = 0; i < Number_Worker; ++i)
        {

            worker[i].startTime = abs(u1(e1));
            worker[i].endTime = worker[i].startTime + (Sumdis[i] / speed) * (1 + endtimeX);
            worker[i].range = rangeX;
            worker[i].score = abs(u(e)) * scoreX;
        }
    }
}

#endif