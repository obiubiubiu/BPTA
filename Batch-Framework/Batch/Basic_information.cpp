#include "Basic_information.h"
#include <chrono>

void Basic_information::Initialize_group(vector<double> &AD, vector<double> &Sumdis, int current_workID, double nowTime, vector<CURRENT_WORKERS_GROUP> &current_workerGroup) // 已修改，变为截止时间-当前窗口截止时间
{
    // cout << "\t\t\t进入了Initialize_group" << endl;
    // 初始化AD
    AD.clear();
    int wid = 0;
    for (int j = 0; j < current_workerGroup.size(); j++)
    {
        wid = current_workerGroup[j].Original_Local;
        double cuAD = (current_workerGroup[j].worker.endTime - nowTime) * speed - Sumdis[wid];

        if (current_workerGroup[j].worker.endTime <= nowTime || cuAD <= 0 || current_workerGroup[j].sign == false)
        {
            cout << "删除了工人：" << wid << "   绕路" << Sumdis[wid] << "\t剩余绕路距离 \t " << cuAD << "\t\t当前时间:" << nowTime << ",原本开始he 结束:" << current_workerGroup[j].worker.startTime << "," << current_workerGroup[j].worker.endTime << endl;
            current_workerGroup.erase(current_workerGroup.begin() + j);
            j--;
            continue;
        }
        AD.push_back(cuAD);
        cout << "\t\t\t当前工人idddD" << wid << "\t剩余绕路距离 \t " << cuAD << "\t\t本身路程消耗距离:" << Sumdis[wid] << "\t\t当前时间:" << nowTime << ",原本开始时间:" << current_workerGroup[j].worker.startTime << endl;
    }
    // for (int j = current_workID; j < global_Current_workID; j++)
    // {
    //     cout << "\t\t\t当前工人idddD" << j;

    //     AD.push_back((current_workerGroup[j].worker.endTime - nowTime) * speed - Sumdis[j]);
    //     cout << "\t剩余绕路距离 \t " << AD[j] << "\t\t本身路程消耗距离:" << Sumdis[j] << "\t\t当前时间:" << nowTime << ",原本开始时间:" << current_workerGroup[j].worker.startTime << endl;
    // }
}

double Basic_information::Caculate_mindist_global(int workerid, int taskid, vector<vector<int>> &poi)
{                                                     // 输入一个task,一个worker,返回task与worker的最近绕路距离,以及最近的POI点在worker轨迹中的第几个
    vector<POI> Trajectory;                           // 定义Trajectory指针
    Trajectory = global_workers[workerid].trajectory; // 指针，访问
    double detour_distance, mindis = 100000;
    int j = -1;
    for (vector<POI>::iterator it = Trajectory.begin(); it != Trajectory.end(); it++)
    {
        j++;
        detour_distance = GetDistance(global_tasks[taskid].Y, global_tasks[taskid].X, (*it).Y, (*it).X);
        if (detour_distance < mindis)
        {
            mindis = detour_distance;
            poi[taskid][workerid] = j; // 记录轨迹中的第j个点是最小的
        }
    }
    return mindis;
}

/**
 * 求解全局内工人和任务的偏好
 */
void Basic_information::Compute_global_PTPW_Group(vector<vector<pair<int, double>>> &PT, vector<vector<pair<int, double>>> &PW)
{
    // 计算PT,PW
    int Number_current_taskGroup = global_tasks.size();
    int Number_current_workerGroup = global_workers.size();
    int num = 0;
    for (int i = 0; i < Number_current_taskGroup; i++)
    {
        for (int j = 0; j < Number_current_workerGroup; j++)
        {
            if (!(global_tasks[i].Deadline < global_workers[j].startTime || global_tasks[i].startTime > global_workers[j].endTime)) // 满足时间约束
            {
                if (global_tasks[i].Minscore <= global_workers[j].score) // 否则绕路距离为0！！错误，改为否则绕路距离为无穷大，正确：分数不满足时不会进入到偏好列表中
                {
                    // 工人的分数满足任务的最小约束                                                                                    //为何错误？？？？
                    global_detour_distance[i][j] = Caculate_mindist_global(j, i, global_POI); // 计算每个任务和每个worker之间的最小绕路距离

                    if (global_detour_distance[i][j] <= global_workers[j].range)
                    {
                        double stime = max(global_workers[j].startTime, global_tasks[i].startTime); // 开始时间
                        double etime = min(global_workers[j].endTime, global_tasks[i].Deadline);    // 结束时间
                        double subdis = global_Worker_subTrajectoryDis[j][global_POI[i][j]];        // 工人j到任务taskid的绕路点的距离

                        double taskCost = subdis + global_detour_distance[i][j];               // 任务耗时=前路程+绕路
                        double workCost = global_Sumdis[j] + 2 * global_detour_distance[i][j]; // 工人路程耗时=自身路径+2*绕路

                        if (taskCost < (etime - stime) * speed && workCost < (global_workers[j].endTime - stime) * speed) // 任务和路程的重合时间为任务的时间，重合的开始时间和工人的截止时间为工人的时间
                        {
                            double preference1 = global_tasks[i].Reward - 2 * global_detour_distance[i][j] * c; // 工人的偏好值
                            if (preference1 > 0)                                                                // 利润大于0
                            {
                                double preference2 = global_workers[j].score; // 任务的偏好值
                                PT[i].push_back(make_pair(j, preference2));
                                PW[j].push_back(make_pair(i, preference1));
                                cout << "(任务,工人)可配对情况：  (" << i << "," << j << ")" << endl;
                                num++;
                            }
                            else
                            {
                                // cout<<taskid<<"\t"<<j<<"距离太远!"<<endl;
                            }
                        }
                    }
                }
            }
        }
    }
    cout << "总的任意匹配对数：" << num << endl;
    for (int i = 0; i < global_workers.size(); i++) // 计算工人偏好列表
    {
        // 对工人的偏序列表排序
        sort(PW[i].begin(), PW[i].end(), cmp);
    }
    for (int i = 0; i < global_tasks.size(); i++) // 计算工人偏好列表
    {
        // 对工人的偏序列表排序
        sort(PT[i].begin(), PT[i].end(), cmp);
    }

    cout << endl;
}

double Basic_information::Caculate_mindist(int workerid, int taskid, vector<int> poi[], vector<CURRENT_TASK_GROUP> &task, vector<CURRENT_WORKERS_GROUP> &worker) // 返回最小距离
{                                                                                                                                                                // 输入一个task,一个worker,返回task与worker的最近绕路距离,以及最近的POI点在worker轨迹中的第几个
    vector<POI> Trajectory;                                                                                                                                      // 定义Trajectory指针
    Trajectory = worker[workerid].worker.trajectory;                                                                                                             // 指针，访问
    double detour_distance, mindis = 100000;
    int j = -1;
    for (vector<POI>::iterator it = Trajectory.begin(); it != Trajectory.end(); it++)
    {
        j++;
        detour_distance = GetDistance(task[taskid].task.Y, task[taskid].task.X, (*it).Y, (*it).X);

        if (detour_distance < mindis)
        {
            mindis = detour_distance;
            poi[taskid][workerid] = j; // 记录轨迹中的第j个点是最小的
        }
    }

    return mindis;
}

/****
 *已修改，增加了三个
 *  1-3 简单的剪枝，4具体的操作
 *
 * 1.判断分数约束
 * 2.判断绕路距离与工人范围关系
 * 3.判断收益约束
 * 4.判断工人和任务之间的时间距离关系CurrentTask_Satisfy

 */
int Basic_information::FindLatestWorkerNew_Greedy(int taskid, vector<int> &current_Group_worker_AW, vector<double> &current_Group_workerAD, vector<double> current_detour_distance[], vector<vector<double>> &current_Group_worker_subTrajectoryDis, vector<int> CT_Worker[], double MaxDistanceTask[], double *current_task_NeedTime, vector<CURRENT_TASK_GROUP> &current_taskGroup, vector<CURRENT_WORKERS_GROUP> &current_workerGroup, vector<int> poi[], double nowtime)
{
    // 工人i,AW可用工人集合、AD剩余可用偏移距离、DD绕路距离、WS工人轨迹、CT_Worker工人当前工作、MaxDistanceTask最大距离、current_task_NeedTime当前任务时间

    //     CurrentTask_Satisfy(detour_distance, global_Worker_subTrajectoryDis,CT_Worker,MaxDistanceTask,*current_task_NeedTime);
    // 找最近的工人
    int latest_workerid = -1;
    double mindist = __DBL_MAX__;
    for (int j = 0; j < current_Group_worker_AW.size(); j++)
    { // 遍历所有可用工人
        int workerid = current_Group_worker_AW[j];

        if (current_taskGroup[taskid].task.Minscore <= current_workerGroup[workerid].worker.score) // worker分数满足大于MinScore
        {
            double dist = Caculate_mindist(workerid, taskid, poi, current_taskGroup, current_workerGroup); // 计算最小绕路距离

            // 新增绕路detour_distance的计算
            current_detour_distance[taskid][workerid] = dist;
            if (current_workerGroup[workerid].worker.range >= dist) // 计算绕路距离距离小于range
            {
                /********+任务的Deadline限制*/

                if (current_taskGroup[taskid].task.Reward - (2 * dist * c) > 0) // 利润大于0
                {

                    if (CurrentTask_Satisfy(current_taskGroup, workerid, taskid, current_Group_workerAD, current_detour_distance, current_Group_worker_subTrajectoryDis, CT_Worker, poi, MaxDistanceTask, current_task_NeedTime, nowtime)) // 未修改
                    // if(SatisfiedDeadline(workerid, taskid, AD, dist)) //满足deadline约束+其它约束
                    {

                        if (dist < mindist)
                        {
                            mindist = dist;
                            latest_workerid = workerid;
                        }
                    }
                }
            }
        }
    }
    return latest_workerid;
}

// void Compute_Preference_list(TASK task, WORKER worker[Number_Worker],int *PT){    //输入一个task,全部的worker，求出task对worker的偏好列表,以及任务的偏好列表PT
// 4.12 21:36 OMG,文件的读取存入响应的数据结构还没实现

void Basic_information::ReadLocationForTask(vector<TASK> &task)
{
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
// // Get_Trajectory_locations(global_workers);
// void Basic_information::Get_Trajectory_locations(vector<WORKER> &worker)
// {
//     ifstream in1("../../dataset\\Berlin\\BusRoutesAsStopsBER.txt");
//     ifstream in2("../../dataset\\Berlin\\BusStopsBER.txt");
//     struct BusStop
//     {
//         int BusStopId;
//         double X;
//         double Y;
//         /* data */
//     };
//     vector<BusStop> BusStops;
//     for (int i = 0; i < Number_BusStop; ++i)
//     {
//         for (int j = 0; j < 5; ++j)
//         {
//             BusStop temp;
//             double tm;
//             in2 >> tm;
//             if (j == 0)
//                 temp.BusStopId = (int)tm;

//             if (j == 1)
//                 temp.X = tm;

//             if (j == 2)
//             {
//                 temp.Y = tm;
//                 BusStops.push_back(temp);
//             }
//         }
//     }
//     in2.close(); // 关闭文件

//     vector<POI> trajectory;
//     string line;
//     regex pat_regex("[[:digit:]]+");
//     int i = 0, p = 0;
//     while (p < Number_Worker)
//     { // 按行读取
//         getline(in1, line);
//         int j = 0;
//         for (sregex_iterator it(line.begin(), line.end(), pat_regex), end_it; it != end_it; ++it)
//         {
//             if (j != 0)
//             {
//                 int temp = stoi(it->str());
//                 vector<BusStop>::iterator itt;
//                 for (itt = BusStops.begin(); itt != BusStops.end(); itt++)
//                 {
//                     int id = (*itt).BusStopId;
//                     if (temp == id)
//                     {
//                         POI poi;
//                         poi.X = (*itt).X;
//                         poi.Y = (*itt).Y;
//                         worker[i].trajectory.push_back(poi);
//                         break;
//                     }
//                 }
//             }
//             j++;
//         }

//         i++;
//         p++;
//     }
//     in1.close();
// }

/****
 * 诗婷原本的代码中的截止时间计算方法为：（距离/速度）*（1+2000）
 * @rangeX区域：是定值，注意是够需要修改。
 * @endtimeX截止时间：是定值，注意是够需要修改
 */
void Basic_information::Prodece_Worker_endTime_range_score(vector<WORKER> &worker, vector<double> &Sumdis, double endtimeX, double rangeX, double scoreX)
{
    // 设置参数endtime,range,score,Maxdetour;
    default_random_engine e, e1;
    uniform_real_distribution<double> u(60, 100); // score
    uniform_int_distribution<unsigned> u1(0, 60); // 开始时间
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
    // sort(worker.begin(), worker.end(), cmp_worker_start); // hy新增对工人排序
}

void Basic_information::showTask(vector<TASK> &task)
{
    for (int i = 0; i < Number_Task; i++)
    {
        cout << "id: " << i << " ";
        cout << "X：" << task[i].X << " ";
        cout << "Y：" << task[i].Y << " ";
        cout << "minscore：" << task[i].Minscore << " ";
        cout << "reward：" << task[i].Reward << " ";
        cout << endl;
    }
}

void Basic_information::showWorker(vector<WORKER> &worker)
{
    for (int i = 0; i < Number_Worker; i++)
    {
        cout << "工人id: " << i << endl;
        for (int j = 0; j < worker[i].trajectory.size(); j++)
        {
            cout << "X：" << worker[i].trajectory[j].X << " ";
            cout << "Y：" << worker[i].trajectory[j].Y << " ";
            cout << endl;
        }

        /*   cout<<"endtime："<<worker[i].endTime <<" ";
           cout<<"range："<<worker[i].range <<" ";
         cout<<"score："<<worker[i].score<<" ";
           cout<<endl;
           */
    }
}

double Basic_information::rad(double d)
{
    return d * PI / 180.0;
}

double Basic_information::GetDistance(double lat1, double lng1, double lat2, double lng2)
{
    double s = 0;
    if (dataOption == 2)
    {

        s = sqrt(pow((lat1 - lat2), 2) + pow((lng1 - lng2), 2)); // 欧氏距离
    }
    else
    {
        double radLat1 = rad(lat1);
        double radLng1 = rad(lng1);
        double radLat2 = rad(lat2);
        double radLng2 = rad(lng2);
        double a = abs(radLng1 - radLng2);
        double b = abs(radLat1 - radLat2);
        double h = pow(sin(b / 2), 2) + cos(lat1) * cos(lat2) * pow(sin(a / 2), 2);
        s = 2 * EARTH_RADIUS * sin(sqrt(h)) * 1000;
    }
    return s;
}
double Basic_information::Caculate_Task_Satisfaction_sum(vector<vector<int>> &CT_Worker, vector<vector<pair<int, double>>> &PT)
{
    double sum = 0;

    for (int i = 0; i < Number_Worker; i++)
    {
        int workerID = i;
        for (int j = 0; j < CT_Worker[i].size(); j++)
        {
            int taskid = CT_Worker[i][j];
            /*
            int index = GetIndex_PT(i, taskid, PT);
            //     cout<<"任务taskid"<<taskid<<"工人"<<i<<endl;
            if (PT[taskid].size() == index)
                cout << "未在任务列表中找到工人！" << endl;
            else
            {
                double s = ((PT[taskid][index].second - 0) / (PT[taskid][0].second - 0));
                //      cout<<s<<"任务的满意度"<<endl;
                sum = sum + s;
            }
            */

            auto it = find_if(PT[taskid].begin(), PT[taskid].end(), [workerID](const auto &p)
                              { return p.first == workerID; });
            if (it != PT[taskid].end())
            {
                int index = distance(PT[taskid].begin(), it);
                double s = ((PT[taskid][index].second - 0) / (PT[taskid][0].second - 0));
                //      cout<<s<<"任务的满意度"<<endl;
                sum = sum + s;
            }
        }
    }

    return sum;
}

/***
 * 打印任务满意度avg
 */
double Basic_information::Caculate_Task_Satisfaction_avg(vector<vector<int>> &CT_Worker, vector<vector<pair<int, double>>> &PT)
{

    double sum = 0;

    for (int i = 0; i < Number_Worker; i++)
    {
        int workerID = i;
        for (int j = 0; j < CT_Worker[i].size(); j++)
        {
            int taskid = CT_Worker[i][j];

            /*
             int index = GetIndex_PT(i, taskid, PT);
             // cout << "taskid:  " << taskid << "\t index:  " << index << endl;
             //     cout<<"任务taskid"<<taskid<<"工人"<<i<<endl;
             if (PT[taskid].size() == index)
                 cout << "未在任务列表中找到工人！" << endl;
             else
             {
                 double s = ((PT[taskid][index].second - 0) / (PT[taskid][0].second - 0));
                 //      cout<<s<<"任务的满意度"<<endl;
                 sum = sum + s;
             }
            */

            auto it = find_if(PT[taskid].begin(), PT[taskid].end(), [workerID](const auto &p)
                              { return p.first == workerID; });
            if (it != PT[taskid].end())
            {
                int index = distance(PT[taskid].begin(), it);
                double s = ((PT[taskid][index].second - 0) / (PT[taskid][0].second - 0));
                //      cout<<s<<"任务的满意度"<<endl;
                sum = sum + s;
            }
        }
    }

    return sum / Number_Task * 100;
}
/***
 * 打印工人满意度avg
 */
double Basic_information::Caculate_Worker_Satisfaction_avg(vector<vector<int>> &CT_Worker, vector<vector<pair<int, double>>> &PW)
{
    double allsum = 0;
    for (int i = 0; i < Number_Worker; i++)
    {
        int workerID = i;
        if (CT_Worker[i].size() != 0)
        {
            double sum = 0, avg = 0;
            for (int j = 0; j < CT_Worker[i].size(); j++)
            {
                int taskid = CT_Worker[i][j];
                /*
                int index = GetIndex_PW(i, taskid, PW);
                if (PW[i].size() == index)
                    cout << "未在工人i中找到task！" << endl;
                else
                {
                    double s = (PW[i][index].second - 0) / (PW[i][0].second - 0);
                    sum = sum + s;
                }

                */

                auto it = find_if(PW[workerID].begin(), PW[workerID].end(), [taskid](const auto &p)
                                  { return p.first == taskid; });
                if (it != PW[workerID].end())
                {
                    int index = distance(PW[workerID].begin(), it);
                    double s = ((PW[workerID][index].second - 0) / (PW[workerID][0].second - 0));
                    //      cout<<s<<"任务的满意度"<<endl;
                    sum = sum + s;
                }
            }
            avg = sum / CT_Worker[i].size();
            allsum = allsum + avg;
            // cout<<allsum<<endl;
        }
    }
    return allsum / Number_Worker * 100;
}
/***
 * 打印工人满意度总和
 */
double Basic_information::Caculate_Worker_Satisfaction_sum(vector<vector<int>> &CT_Worker, vector<vector<pair<int, double>>> &PW)
{
    double allsum = 0;
    for (int i = 0; i < Number_Worker; i++)
    {
        int workerID = i;
        if (CT_Worker[i].size() != 0)
        {
            double sum = 0, avg = 0;
            for (int j = 0; j < CT_Worker[i].size(); j++)
            {
                int taskid = CT_Worker[i][j];
                /*
                 int index = GetIndex_PW(i, taskid, PW);
                 if (PW[i].size() == index)
                     cout << "未在工人i中找到task！" << endl;
                 else
                 {
                     double s = (PW[i][index].second - 0) / (PW[i][0].second - 0);
                     sum = sum + s;
                 }

                */

                auto it = find_if(PW[workerID].begin(), PW[workerID].end(), [taskid](const auto &p)
                                  { return p.first == taskid; });
                if (it != PW[workerID].end())
                {
                    int index = distance(PW[workerID].begin(), it);
                    double s = ((PW[workerID][index].second - 0) / (PW[workerID][0].second - 0));
                    //      cout<<s<<"任务的满意度"<<endl;
                    sum = sum + s;
                }
                //  cout<<CT_Worker[i].size()<<endl;
                //   cout<<i<<"\t"<<taskid<<"\t"<<index<<"\t"<<PW[i].size()<<"\t"<<s<<"\t"<< sum <<endl;
                //  cout<<endl;
            }
            avg = sum;
            allsum = allsum + avg;
            // cout<<allsum<<endl;
        }
    }
    return allsum;
}

int Basic_information::GetIndex_PW(int workerid, int taskid, vector<vector<pair<int, double>>> &PW)
{
    // 获取任务在PW中的排序位置
    int i = 0;
    for (i; i < PW[workerid].size(); i++)
    {
        if (PW[workerid][i].first == taskid)
            break;
    }
    return i;
}
int Basic_information::GetIndex_PT(int workerid, int taskid, vector<vector<pair<int, double>>> &PT)
{
    int i = 0;
    for (i; i < PT[taskid].size(); i++)
    {
        if (PT[taskid][i].first == workerid)
            break;
    }
    return i;
}

/***
 * 为当前分组内任务计算工人最大绕行距离。
 */
void Basic_information::computeMaxDitanceTask(double MaxDistanceTask[], vector<CURRENT_TASK_GROUP> currentTask, double current_window_endTime) // 每个任务最长可走的路程，已修改
{
    for (int i = 0; i < currentTask.size(); i++)
    {
        MaxDistanceTask[i] = (currentTask[i].task.Deadline - current_window_endTime) * speed; // 工人最大可走的路长需要满足任务的DEADLINE
    }
}

/****
 *
 * 以任务为主导判断工人和任务能否匹配。
 *
 * 1. 判断工人到新任务的绕路距离与工人截止时间关系
 * 2. 判断插入新点对前后点的影响
 *      对前面的点无影响
 *      对后面的点的截止时间影响
 *
 */
bool Basic_information::IfReplace(int workerid, int taskid, int assignedtaskid, vector<double> &AD, vector<double> detour_distance[], vector<vector<double>> &Worker_subTrajectoryDis, vector<vector<int>> &CT_Worker, vector<int> poi[], double MaxDistanceTask[], double *current_task_NeedTime, double beforesumDis, vector<CURRENT_TASK_GROUP> &current_taskGroup, double nowtime)
{

    // 1.工人的AD，任务的Deadline，其它任务的Deadline
    // 相比Greedy-worker多一个容量的判断
    int assignedNumber = CT_Worker[workerid].size();
    if (assignedNumber == 0) // 没有已匹配的任务，立即返回false;
        return false;

    // cout<<"工人已匹配任务数量："<<assignedNumber<<endl;
    // 先判断工人的绕路距离约束是否满足？
    double ADD = AD[workerid] + 2 * detour_distance[assignedtaskid][workerid] - 2 * detour_distance[taskid][workerid]; // 工人的绕路距离约束
    // cout<<"剩余AD:"<<AD[workerid]<<",任务绕路"<<2 * detour_distance[taskid][workerid]<<","<<ADD<<endl;
    if (ADD < 0)
    {
        // cout<<"不满足工人的Deadline!"<<endl;
        return false;
    }

    //+任务的Deadline约束
    // 对插入点之前的工人已分配任务的已绕路距离进行计算
    int current_detourid = poi[taskid][workerid]; // 插入任务的偏移轨迹点
    // cout<<"任务的偏移点序号:"<<current_detourid<<endl;
    // 判断起点是否已有任务偏移
    bool flag_first = false;

    // 替换出去的任务poi点在当前任务的poi点之前,则当前任务的偏移距离更新
    int replacepoi = poi[assignedtaskid][workerid];
    if (replacepoi <= current_detourid)
        beforesumDis = beforesumDis - 2 * detour_distance[assignedtaskid][workerid];

    double Time_TO_Task;                                                                                                             //+偏移点到起点的时间
    Time_TO_Task = (detour_distance[taskid][workerid] + Worker_subTrajectoryDis[workerid][current_detourid] + beforesumDis) / speed; // 偏移距离+已匹配任务的绕路+起点到任务位置的轨迹路程
    *current_task_NeedTime = Time_TO_Task;                                                                                           // 记录到任务位置所需时间并返回
    // 是否满足任务的Deadline约束，//最后一个偏移点也考虑了
    if (current_taskGroup[taskid].task.Deadline > Time_TO_Task + nowtime)
    {
        // cout<<"偏移点到起点的时间:"<<Time_TO_Task<<",任务的DEADLINE"<<task[taskid].Deadline<<endl;

        // 判断对替换对已匹配任务的影响
        int noinflu_count = 0; // 判断是否在队列最后插入
        for (int i = 0; i < assignedNumber; i++)
        {
            int assigntask_id = CT_Worker[workerid][i];
            int detourpoint = poi[assigntask_id][workerid]; // 已匹配任务的偏移点
            double temporalMaxd = 0;
            //******新增去除任务assignedtaskid去除的影响
            if (replacepoi < detourpoint && assigntask_id != assignedtaskid) // 更新任务的MaxDistanceTask
            {
                // 更新暂时的任务的最大可走路程记录
                temporalMaxd = MaxDistanceTask[assigntask_id] + (2 * detour_distance[assignedtaskid][workerid]); //+替换的任务绕路路程
            }
            // 继续判断新增任务taskid的影响
            if (current_detourid < detourpoint && assigntask_id != assignedtaskid) // 只对偏移点之后的任务有影响, 判断是否满足后面任务的Deadline 更新任务的MaxDistanceTask
            {
                // 判断插入对后续任务的影响
                if (temporalMaxd < (2 * detour_distance[taskid][workerid]))
                {
                    // cout<<"任务影响后续任务的dealine不可插入"<<endl;
                    return false; // 不可插入，立马返回false
                }
                else
                {
                    // 在Deadline范围内，返回函数之后再更新任务的最大可走路程
                    // MaxDistanceTask[assigntask_id]=MaxDistanceTask[assigntask_id]+(2*detour_distance[taskid][workerid]); //插入的任务绕路路程
                }
            }

            if (current_detourid > detourpoint) // 在已匹配任务之后插入，无影响
            {
                noinflu_count++;
            }
        }
        if (noinflu_count == assignedNumber) // 在队列最后插入，直接返回true，不需要判断后续任务的影响
        {
            // 已满足Deadline
            // cout<<"不影响所有任务"<<noinflu_count <<endl;
            return true;
        }

        // cout<<"所有任务不受影响且不在最后插入返回true"<<endl;
        return true; // 所有任务不受影响且不在最后插入，返回true.
    }
    else // 不满足任务的Deadline直接返回false
    {
        // cout<<"不满足任务的Deadline直接返回false"<<endl;
        return false;
    }
}

bool Basic_information::CurrentTask_Satisfy_TSDA(vector<CURRENT_TASK_GROUP> &current_taskGroup, int workerid, int taskid, vector<double> &AD, vector<double> current_detour_distance[], vector<vector<double>> &current_Worker_subTrajectoryDis, vector<vector<int>> &CT_Worker, vector<int> poi[], double MaxDistanceTask[], double *current_task_NeedTime, double nowtime)
{
    int assignedNumber = CT_Worker[workerid].size(); // 工人已匹配任务数量
    if (assignedNumber == Capacity)                  // 不满足容量
    {
        return false;
    }
    // cout<<"工人已匹配任务数量："<<assignedNumber<<endl;
    // 先判断工人的绕路距离约束是否满足？
    double ADD = AD[workerid] - 2 * current_detour_distance[taskid][workerid]; // 工人的绕路距离约束--------

    // cout<<"剩余AD:"<<AD[workerid]<<",任务绕路"<<2 * detour_distance[taskid][workerid]<<","<<ADD<<endl;
    if (ADD < 0)
    { // cout<<"不满足工人的Deadline!"<<endl;
        return false;
    }

    //+任务的Deadline约束

    // 对插入点之前的工人已分配任务的已绕路距离进行计算
    int current_detourid = poi[taskid][workerid]; // 任务的偏移轨迹点
    // cout<<"任务的偏移点序号:"<<current_detourid<<endl;
    // 判断起点是否已有任务偏移
    bool flag_first = false;
    double SumdetouDis = 0.0;
    for (int i = 0; i < assignedNumber; i++) // 计算插入前任务的总偏移距离
    {
        int assigntask_id = CT_Worker[workerid][i];
        int detourpoint = poi[assigntask_id][workerid]; // 已匹配任务的偏移点
        // cout<<"已匹配任务"<<assigntask_id<<"偏移点序号"<<detourpoint<<endl;
        if (current_detourid >= detourpoint)
        {
            SumdetouDis = SumdetouDis + 2 * current_detour_distance[assigntask_id][workerid];
            // cout<<"已匹配任务总偏移距离："<<SumdetouDis<<endl;
        }
        if (detourpoint == 0) // 出发点已有任务
        {
            flag_first = true;
        }
    }

    double Time_TO_Task; //+起点到最新任务点偏移的总时间----hy未修改

    Time_TO_Task = (current_detour_distance[taskid][workerid] + current_Worker_subTrajectoryDis[workerid][current_detourid] + SumdetouDis) / speed; // 偏移距离+已匹配任务的绕路+起点到任务位置的轨迹路程
    *current_task_NeedTime = Time_TO_Task;                                                                                                          // 记录到任务位置所需时间并返回
    // 是否满足任务的Deadline约束，//最后一个偏移点也考虑了
    if (current_taskGroup[taskid].task.Deadline > Time_TO_Task + nowtime)
    {
        // cout<<"偏移点到起点的时间:"<<Time_TO_Task<<",任务的DEADLINE"<<task[taskid].Deadline<<endl;
        if (assignedNumber == 0) // 没有已匹配的任务
        {
            // cout<<"满足任务的DEADLINE"<<endl;
            return true;
        }
        else // 判断对已匹配任务的影响
        {
            int noinflu_count = 0; // 判断是否在队列最后插入
            for (int i = 0; i < assignedNumber; i++)
            {
                int assigntask_id = CT_Worker[workerid][i];
                int detourpoint = poi[assigntask_id][workerid]; // 已匹配任务的偏移点
                //    if(trajectory_id == detourpoint ) //偏移点为同一个暂时不管
                // cout<<"已匹配任务的偏移点:"<<detourpoint<<endl;
                if (current_detourid < detourpoint) // 只对偏移点之后的任务有影响, 判断是否满足后面任务的Deadline 更新任务的MaxDistanceTask
                {
                    // 判断插入对后续任务的影响
                    if (MaxDistanceTask[assigntask_id] < (2 * current_detour_distance[taskid][workerid])) // 偏移距离耗时不能超过任务的起止时间
                    {
                        // cout<<"任务影响后续任务的dealine不可插入"<<endl;
                        return false; // 不可插入，立马返回false
                    }
                    else
                    {
                        // 在Deadline范围内，返回函数之后再更新任务的最大可走路程
                        //  MaxDistanceTask[assigntask_id]=MaxDistanceTask[assigntask_id]+(2*detour_distance[taskid][workerid]); //插入的任务绕路路程
                    }
                }

                if (current_detourid > detourpoint) // 在已匹配任务之后插入，无影响
                {
                    noinflu_count++;
                }
            }
            if (noinflu_count == assignedNumber) // 在队列最后插入，直接返回true，不需要判断后续任务的影响
            {
                // 已满足Deadline
                // cout<<"不影响所有任务"<<noinflu_count <<endl;
                return true;
            }
            // cout<<"所有任务不受影响且不在最后插入返回true"<<endl;
            return true; // 所有任务不受影响且不在最后插入，返回true.x
        }
    }
    else // 不满足任务的Deadline直接返回false
    {
        // cout<<"不满足任务的Deadline直接返回false"<<endl;
        return false;
    }
}

bool Basic_information::CurrentTask_Satisfy(vector<CURRENT_TASK_GROUP> &current_taskGroup, int workerid, int taskid, vector<double> &AD, vector<double> current_detour_distance[], vector<vector<double>> &current_Worker_subTrajectoryDis, vector<int> CT_Worker[], vector<int> poi[], double MaxDistanceTask[], double *current_task_NeedTime, double nowtime)
{
    int assignedNumber = CT_Worker[workerid].size(); // 工人已匹配任务数量
    // 先判断工人的绕路距离约束是否满足？
    double ADD = AD[workerid] - 2 * current_detour_distance[taskid][workerid]; // 工人的绕路距离约束--------

    // cout<<"剩余AD:"<<AD[workerid]<<",任务绕路"<<2 * detour_distance[taskid][workerid]<<","<<ADD<<endl;
    if (ADD < 0)
    {
        // cout<<"不满足工人的Deadline!"<<endl;
        return false;
    }

    //+任务的Deadline约束

    // 对插入点之前的工人已分配任务的已绕路距离进行计算
    int current_detourid = poi[taskid][workerid]; // 任务的偏移轨迹点
    // cout<<"任务的偏移点序号:"<<current_detourid<<endl;
    // 判断起点是否已有任务偏移
    bool flag_first = false;
    double SumdetouDis = 0.0;
    for (int i = 0; i < assignedNumber; i++) // 计算插入前任务的总偏移距离
    {
        int assigntask_id = CT_Worker[workerid][i];
        int detourpoint = poi[assigntask_id][workerid]; // 已匹配任务的偏移点
        // cout<<"已匹配任务"<<assigntask_id<<"偏移点序号"<<detourpoint<<endl;
        if (current_detourid >= detourpoint)
        {
            SumdetouDis = SumdetouDis + 2 * current_detour_distance[assigntask_id][workerid];
            // cout<<"已匹配任务总偏移距离："<<SumdetouDis<<endl;
        }
        if (detourpoint == 0) // 出发点已有任务
        {
            flag_first = true;
        }
    }

    double Time_TO_Task; //+起点到最新任务点偏移的总时间----hy未修改

    Time_TO_Task = (current_detour_distance[taskid][workerid] + current_Worker_subTrajectoryDis[workerid][current_detourid] + SumdetouDis) / speed; // 偏移距离+已匹配任务的绕路+起点到任务位置的轨迹路程

    *current_task_NeedTime = Time_TO_Task;
    // 记录到任务位置所需时间并返回
    // 是否满足任务的Deadline约束，//最后一个偏移点也考虑了

    if (current_taskGroup[taskid].task.Deadline > Time_TO_Task + nowtime)
    {

        // cout<<"偏移点到起点的时间:"<<Time_TO_Task<<",任务的DEADLINE"<<task[taskid].Deadline<<endl;
        if (assignedNumber == 0) // 没有已匹配的任务
        {
            // cout<<"满足任务的DEADLINE"<<endl;
            return true;
        }
        else // 判断对已匹配任务的影响
        {
            int noinflu_count = 0; // 判断是否在队列最后插入
            for (int i = 0; i < assignedNumber; i++)
            {
                int assigntask_id = CT_Worker[workerid][i];
                int detourpoint = poi[assigntask_id][workerid]; // 已匹配任务的偏移点
                //    if(trajectory_id == detourpoint ) //偏移点为同一个暂时不管
                // cout<<"已匹配任务的偏移点:"<<detourpoint<<endl;
                if (current_detourid < detourpoint) // 只对偏移点之后的任务有影响, 判断是否满足后面任务的Deadline 更新任务的MaxDistanceTask
                {
                    // 判断插入对后续任务的影响
                    if (MaxDistanceTask[assigntask_id] < (2 * current_detour_distance[taskid][workerid])) // 偏移距离耗时不能超过任务的起止时间
                    {
                        // cout<<"任务影响后续任务的dealine不可插入"<<endl;
                        return false; // 不可插入，立马返回false
                    }
                    else
                    {
                        // 在Deadline范围内，返回函数之后再更新任务的最大可走路程
                        //  MaxDistanceTask[assigntask_id]=MaxDistanceTask[assigntask_id]+(2*detour_distance[taskid][workerid]); //插入的任务绕路路程
                    }
                }

                if (current_detourid > detourpoint) // 在已匹配任务之后插入，无影响
                {
                    noinflu_count++;
                }
            }
            if (noinflu_count == assignedNumber) // 在队列最后插入，直接返回true，不需要判断后续任务的影响
            {
                // 已满足Deadline
                // cout<<"不影响所有任务"<<noinflu_count <<endl;
                return true;
            }
            // cout<<"所有任务不受影响且不在最后插入返回true"<<endl;
            return true; // 所有任务不受影响且不在最后插入，返回true.x
        }
    }
    else // 不满足任务的Deadline直接返回false
    {
        // cout<<"不满足任务的Deadline直接返回false"<<endl;
        return false;
    }
}

void Basic_information::UpdateTaskDeadline(int workerid, int taskid, vector<double> current_detour_distance[], vector<int> CT_Worker[], vector<int> poi[], double MaxDistanceTask[], double current_task_NeedTime)
{
    for (int i = 0; i < CT_Worker[workerid].size(); i++)
    {
        int assigntask_id = CT_Worker[workerid][i];
        int assignedpoint = poi[assigntask_id][workerid]; // 已匹配任务的偏移点
        int detourpoint = poi[taskid][workerid];
        //    if(trajectory_id == detourpoint ) //偏移点为同一个暂时不管

        if (assigntask_id == taskid) // 刚插入的任务
        {
            // cout<<"原最大可走："<< MaxDistanceTask[taskid]<<endl;
            MaxDistanceTask[taskid] = MaxDistanceTask[taskid] - current_task_NeedTime * speed; // 当前任务剩余可走的最大距离，也就是时间。
                                                                                               // cout<<"本次需要走："<<current_task_NeedTime*speed<<endl;
                                                                                               // cout<<"当前任务最大可走距离更新为："<< MaxDistanceTask[taskid]<<endl;
        }
        else
        {

            if (detourpoint < assignedpoint) // 只对偏移点之后的任务有影响, 判断是否满足后面任务的Deadline 更新任务的MaxDistanceTask
            {
                // 更新任务的最大可走路程
                MaxDistanceTask[assigntask_id] = MaxDistanceTask[assigntask_id] - (2 * current_detour_distance[taskid][workerid]); // 减去插入的任务绕路路程
                                                                                                                                   // cout<<"影响的任务"<<assigntask_id<<"的最大可走距离更新为："<< MaxDistanceTask[assigntask_id]<<endl;
            }
        }
    }
}

/*******
 * 修改了任务生成时间，要求开始时间在截止时间之前
 * 注意这里存在无线循环的情况，即，开始时间一直在截止时间之前。
 *
 */
void Basic_information::Prodece_Task_Reward_Minscore_Deadline(vector<TASK> &task)
{ // 随机生成task的reward和minscore
    default_random_engine e1, e2, e3, e4;
    uniform_real_distribution<double> u1(60, 100); // score
    uniform_real_distribution<double> u2(1, 10);   // reward
    uniform_real_distribution<double> u3(20, 60);  // Deadline
    uniform_real_distribution<double> u4(0, 50);   // startTime
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
    sort(task.begin(), task.end(), cmp_task_start); // 对任务和工人分别按照开始时间进行升序排序
}

/**
 * Sumdis获取每个工人的轨迹距离之和，无需修改
 */

void Basic_information::Caculate_Sumdist_Trajectory(vector<double> &Sumdis, vector<vector<double>> &global_Worker_subTrajectoryDis, vector<WORKER> &worker)
{
    for (int i = 0; i < Number_Worker; i++)
    {

        double sum = 0;
        double subsum = 0;
        global_Worker_subTrajectoryDis[i].push_back(0); // 第一个轨迹点为0

        for (int j = 0; j < worker[i].trajectory.size() - 1; j++)
        {
            cout << "工人：" << i << "  轨迹：" << j << "  工人总数：" << Number_Worker << " 轨迹总数：" << worker[i].trajectory.size() << endl;

            worker[i].trajectory[j].Y;
            worker[i].trajectory[j + 1].X;

            double distance = GetDistance(worker[i].trajectory[j].Y, worker[i].trajectory[j].X, worker[i].trajectory[j + 1].Y, worker[i].trajectory[j + 1].X);

            sum = sum + distance;
            global_Worker_subTrajectoryDis[i].push_back(sum); // 记录每个轨迹点前的距之和
        }
        Sumdis[i] = sum;
        //   cout<<i<<" "<<"all:" << Sumdis[i] <<endl;
    }
}

// void Basic_information::print_groupTasks_addEndTime()
// {
//     for (int i = 0; i < taskGroups_addEndTime_result.size(); i++)
//     {
//         for (auto t : taskGroups_addEndTime_result[i])
//         {
//             cout << t.startTime << ",,," << t.Deadline << endl;
//         }
//     }
// }

void Basic_information::begin_Algorithm(string alg_num)
{
    global_CT_Worker.clear();
    global_CT_Worker.resize(Number_Worker);
    global_Current_workID = 0;
    global_Current_taskNumber = 0;
    cout << "---------" << alg_num << "--------" << endl;
}
void Basic_information::updata_current_Info(vector<CURRENT_TASK_GROUP> &current_taskGroup, vector<CURRENT_WORKERS_GROUP> &current_workerGroup, vector<double> &current_Group_workerAD, vector<vector<double>> &current_Group_worker_subTrajectoryDis, vector<double> &current_Group_workerSumdis)
{
    vector<CURRENT_WORKERS_GROUP> temp_work_group;

    for (vector<CURRENT_WORKERS_GROUP>::iterator it_curret_Worker = current_workerGroup.begin(); it_curret_Worker != current_workerGroup.end(); ++it_curret_Worker) // 移除工人
    {
        if ((*it_curret_Worker).sign)
        {
            temp_work_group.push_back(*it_curret_Worker);
        }
    }
    current_workerGroup.clear();
    current_workerGroup = temp_work_group;
    temp_work_group.clear();

    vector<CURRENT_TASK_GROUP> temp_task_group;

    for (vector<CURRENT_TASK_GROUP>::iterator it_curret_task = current_taskGroup.begin(); it_curret_task != current_taskGroup.end(); ++it_curret_task) // 移除工人
    {
        if ((*it_curret_task).sign)
        {
            temp_task_group.push_back(*it_curret_task);
        }
    }
    current_taskGroup.clear();
    current_taskGroup = temp_task_group;
    temp_task_group.clear();
}

/***
 * 窗口划分框架
 * 首先根据当前任务分组，获取可以服务的工人
 * 其次考虑当前分组中任务未匹配且能继续分配到下一组的情况。
 *
 * 要求：修改groupTasks_addEndTime的第3、4条
 *
 * 1. 每一组task处于连续的时间窗口Wmax，Wmax；
 * 2. 一组内的task数量不超过Tmax。
 * 3. 若一组任务数量达到为Tmax时，则以当前current_taskGroup的时间重新创建新的窗口。
 *      3.1 任务和工人进行匹配
 *      3.2 对于未匹配的任务划分到下一组，修改任务的开始时间（为了防止计算下一分组开始时间出错）
 * 4. 若已到达当前窗口时间current_window_endTime，纵然当前窗口current_window_endTime的task数量位到达Tmax也创建新的窗口。
 *      4.1 任务和工人进行匹配
 *      4.2 对于未匹配的任务划分到下一组，修改任务的开始时间（为了防止计算下一分组开始时间出错）
 */
void Basic_information::Grouping_Framework_Greedy(vector<TASK> &tasks, vector<WORKER> &workers, double Wmax, int Tmax, vector<double> &Sumdis, vector<vector<double>> &global_Worker_subTrajectoryDis)

{
    vector<CURRENT_TASK_GROUP> current_taskGroup;              // 当前时间窗口内的任务列表，已获得
    vector<CURRENT_WORKERS_GROUP> current_Group_worker;        // 当前窗口内工人，已获得
    vector<double> current_Group_workerSumdis;                 // 当前窗口内工人轨迹的总距离。已获得
    vector<double> current_Group_workerAD;                     // 当前窗口内工人剩余可用距离，已获得
    double current_window_endTime = tasks[0].startTime + Wmax; // 当前时间窗口的结束时间，已获得
    double current_window_startTime = tasks[0].startTime;      // 当前时间窗口的起始时间，已获得
    int current_workID = global_Current_workID;                // 判断当前窗口的工人从几号开始的，已获得

    vector<vector<double>> current_Group_worker_subTrajectoryDis; // 当前窗口内工人轨迹点之和，已获得
    // vector<vector<double>> current_detour_distance;               // 当前窗口内最大绕路距离

    int sign = 0;
    cout << "------------------------开始分组--------------------" << endl;

    for (int i = 0; i < tasks.size(); i++)
    {

        TASK task = tasks[i];
        sign++;

        CURRENT_TASK_GROUP taskg;
        taskg.task = task;
        taskg.Original_Local = i;
        current_taskGroup.push_back(taskg);

        // 如果当前分组数量已达到Tmax，则创建新的分组
        // 如果任务的开始时间大于等于当前窗口的截止时间，则创建新的分组

        if ((current_taskGroup.size() == Tmax) || (task.startTime >= current_window_endTime))
        {

            current_workID = global_Current_workID;

            current_window_endTime = task.startTime; // 下一窗口的开始时间，也就是上一窗口的结束时间

            // 计算当前窗口满足时间约束的work。
            groupWork_according_TaskGroup(workers, Sumdis, current_Group_worker, current_Group_workerSumdis, current_window_endTime, current_workID, current_Group_workerAD, global_Worker_subTrajectoryDis, current_Group_worker_subTrajectoryDis); // 计算当前窗口满足时间约束的work。
            determine_Window_Task_Timeout(current_taskGroup, current_window_endTime);

            // 任务工人配对，并计算当工人和任务未匹配时,能进入下一窗口的工人和任务，

            // match_WorkerTask(current_taskGroup, current_Group_worker, current_window_endTime, current_Group_workerAD, current_Group_worker_subTrajectoryDis);        // 当前任务组,当前工人组,当前窗口截止时间，当前工人剩余绕行距离
            match_WorkerTask_Greedy(current_taskGroup, current_Group_worker, current_window_endTime, current_Group_workerAD, current_Group_worker_subTrajectoryDis); // 当前任务组,当前工人组,当前窗口截止时间，当前工人剩余绕行距离

            // 为下一窗口做准备，即最大的截止时间
            current_window_startTime = current_window_endTime;
            current_window_endTime += Wmax;
            cout << "------------------------开始结束--------------------\n"
                 << endl;
            cout << "------------------------开始分组--------------------" << endl;

            // updata_nextWindow_Worker_Task(current_taskGroup, current_Group_worker, current_window_endTime); // 将剩余工人加入当前组。并更新他们的开始时间为当前截止时间
            updata_current_Info(current_taskGroup, current_Group_worker, current_Group_workerAD, current_Group_worker_subTrajectoryDis, current_Group_workerSumdis);
        }

        // 将剩余任务加入当前分组，并更新他们的开始时间为当前时间。
    }
    // 将最后一个分组加入分组列表
    if (current_taskGroup.size() > 0)
    {

        current_workID = global_Current_workID;
        current_window_startTime = current_taskGroup.back().task.startTime;
        cout << sign << "   最后一组，下一时间窗口为：" << current_window_startTime << endl;
        // 计算分组内的工人以及他们剩余绕行距离
        groupWork_according_TaskGroup(workers, Sumdis, current_Group_worker, current_Group_workerSumdis, current_window_endTime, current_workID, current_Group_workerAD, global_Worker_subTrajectoryDis, current_Group_worker_subTrajectoryDis); // 计算当前窗口满足时间约束的work。

        determine_Window_Task_Timeout(current_taskGroup, current_window_endTime);
        // 对分组内任务和工人进行匹配。
        match_WorkerTask_Greedy(current_taskGroup, current_Group_worker, current_window_endTime, current_Group_workerAD, current_Group_worker_subTrajectoryDis); // 当前任务组,当前工人组,当前窗口截止时间，当前工人剩余绕行距离
        updata_current_Info(current_taskGroup, current_Group_worker, current_Group_workerAD, current_Group_worker_subTrajectoryDis, current_Group_workerSumdis);
        // updata_nextWindow_Worker_Task(current_taskGroup, current_Group_worker, current_window_startTime);
        // current_taskGroup.clear();
    }
    // print_groupTasks_addEndTime();
}

/*****
 *
 * 根据任务分组工人
 * 当前窗口出现的工人
 * 获取轨迹距离
 * 计算剩余绕行距离
 *
 * 所有工人，所有工人轨迹距离，当前工人分组，当前工人分组的轨迹距离，当前时间、当前工人的ID，当前工人分组的剩余可用距离，所有工人的节点前项和,当前工人节点距离前项和
 *
 * 这里的
 */

void Basic_information::groupWork_according_TaskGroup(vector<WORKER> &workers, vector<double> &Sumdis, vector<CURRENT_WORKERS_GROUP> &current_workerGroup,
                                                      vector<double> &current_Group_workerSumdis, double nowTime, int current_workID, vector<double> &current_Group_workerAD,
                                                      vector<vector<double>> &global_Worker_subTrajectoryDis, vector<vector<double>> &current_Group_worker_subTrajectoryDis)
{
    CURRENT_WORKERS_GROUP workerg;
    cout << "\t当前时间" << nowTime << "当前工人id：" << global_Current_workID << endl;
    int ttt = current_workID;

    for (auto it = (workers.begin() + current_workID); it != workers.end();)
    {

        if (it->startTime < nowTime && it->endTime > nowTime)
        {
            cout << "\t分组内工人：" << global_Current_workID << "(" << (*it).startTime << "," << (*it).endTime << ")   距离：" << Sumdis[global_Current_workID] << endl;
            workerg.worker = *it;
            workerg.Original_Local = global_Current_workID;
            current_workerGroup.push_back(workerg);                              // 当前分组内的工人
            current_Group_workerSumdis.push_back(Sumdis[global_Current_workID]); // 当前分组内的工人轨迹距离

            current_Group_worker_subTrajectoryDis.push_back(global_Worker_subTrajectoryDis[global_Current_workID]); // 当前分组内工人的轨迹点距离前项和
            // current_Group_workerAD.push_back(((*it).endTime - nowTime) * speed - Sumdis[current_WorkNumber]); // 当前分组内工人的剩余绕行距离

            // Initialize_group(current_Group_workerAD, current_Group_workerSumdis, current_workID, nowTime, current_workerGroup); // hy 初始化剩余绕路距离当前时间-剩余时间，已修改到分组划分的位置,
            //                                                                                                                     // it = workers.erase(it);                                                                                                                    // ++current_WorkNumber;
        }
        ttt++;

        // else if (it->endTime < nowTime)
        // {
        //   // it = workers.erase(it);
        //   ++current_WorkNumber;
        // }

        if (it->startTime >= nowTime)
        {

            cout << ttt << "\t" << it->startTime << "\t开始时间太大了,当前的工人编号" << global_Current_workID << endl;
            // current_workerGroup.erase(std::remove_if(current_workerGroup.begin(), current_workerGroup.end(), [nowTime](CURRENT_WORKERS_GROUP w)
            //                                          { return w.worker.endTime < nowTime; }),
            //                           current_workerGroup.end());
            // cout<<"跳出循环了，啊"<<endl;
            Initialize_group(current_Group_workerAD, current_Group_workerSumdis, current_workID, nowTime, current_workerGroup); // hy 初始化剩余绕路距离当前时间-剩余时间，已修改到分组划分的位置,
                                                                                                                                // it = workers.erase(it);
            break;
        }
        ++global_Current_workID;

        ++it;
    }

    // print_groupWork(current_workerGroup);
}

/******
 * 计算当前窗口中满足时间约束的task，
 * 即查看有没分组内超时的任务
 */
void Basic_information::determine_Window_Task_Timeout(std::vector<CURRENT_TASK_GROUP> &taskList, double &nowTime)
{
    // 检查分组内的截止时间是否小于当前时间，如果是则删除该任务
    taskList.erase(std::remove_if(taskList.begin(), taskList.end(), [nowTime](CURRENT_TASK_GROUP task)
                                  { return task.task.Deadline < nowTime || task.sign == false; }),
                   taskList.end());

    for (int i = 0; i < taskList.size(); i++)
    {
        cout << "\t\t当前任务:" << taskList[i].Original_Local << "(" << taskList[i].task.startTime << "\t" << taskList[i].task.Deadline << ") 当前时间：" << nowTime << endl;
    }
}

/**ShowCTMatching
 * 展示组内和全局的匹配结果
 * 有两种方式是因为我懒得修改了。
 * 其中vector<vector<int>>指的全局
 * vector<int> [number]:指的组内
 */
void Basic_information::ShowCTMatching(vector<vector<int>> &CT_Worker, int current_Number_Workers) // 修改
{
    int sumtasks = 0;
    int workers = 0;
    for (int i = 0; i < current_Number_Workers; i++)
    {

        for (int j = 0; j < CT_Worker[i].size(); j++)
        {
            cout << "配对:（工人" << i << "，任务" << CT_Worker[i][j] << endl;
            // cout << "work时间:\t" << global_workers[i].startTime << "\t" << global_workers[i].endTime << "\t";
            cout << "task时间:\t" << global_tasks[CT_Worker[i][j]].startTime << "\t\t" << global_tasks[CT_Worker[i][j]].Deadline << endl;

            sumtasks++;
        }

        if (CT_Worker[i].size() != 0)
            workers++;
    }
    cout << "匹配的任务总数:" << sumtasks << endl;
    cout << "匹配的工人总数:" << workers << endl;
}
/**ShowCTMatching
 * 有两种方式是因为我懒得修改了。
 * 其中vector<vector<int>>指的全局
 * vector<int> [number]:指的组内
 */

void Basic_information::ShowCTMatching(vector<int> CT_Worker[], int current_Number_Worker) // 修改
{
    int sumtasks = 0;
    int workers = 0;
    for (int i = 0; i < current_Number_Worker; i++)
    {
        for (int j = 0; j < CT_Worker[i].size(); j++)
        {
            // cout << "配对:" << i << "\t" << CT_Worker[i][j] << endl;
            // cout << "work:\t" << worker[i].startTime << "\t" << worker[i].endTime << endl;
            // cout << "task:\t" << task[CT_Worker[i][j]].startTime << "\t" << task[CT_Worker[i][j]].Deadline << endl;
            sumtasks++;
        }
        if (CT_Worker[i].size() != 0)
            workers++;
    }
    cout << "匹配的任务总数:" << sumtasks << endl;
    cout << "匹配的工人总数:" << workers << endl;
}
/***
 * 对任务和工人执行配对（这里嵌套诗婷的代码）
 * 将原本诗婷写在main函数的内容嵌套进来
 * 对于每个任务，计算任务的可用工人：、
 *      首先，满足range约束，满足最小声誉分数约束；
 *      其次，在匹配时再判断是否满足deadline约束，容量约束
 *      接着，然后计算与每个可用工人的距离，选择最近的工人
 */
void Basic_information::match_WorkerTask_Greedy(vector<CURRENT_TASK_GROUP> &current_taskGroup, vector<CURRENT_WORKERS_GROUP> &current_workerGroup, double current_window_endTime, vector<double> &current_Group_workerAD, vector<vector<double>> &current_Group_worker_subTrajectoryDis)
{
    int current_Number_Worker = current_workerGroup.size();
    int current_Number_Task = current_taskGroup.size();
    vector<double> current_detour_distance[current_Number_Task]; // 存储当前分组内，任务到每个工人的最近绕路距离，并初始化，防止数组越界。
    vector<int> current_poi[current_Number_Task];                // 存储poiid，记录哪个点到任务距离是最小的
    for (int i = 0; i < current_Number_Task; i++)
    {
        current_detour_distance[i] = vector<double>(current_Number_Worker, 0);
        // current_detour_distance[i].push_back(vector<double>(current_Number_Worker, 0));
        current_poi[i] = vector<int>(current_Number_Worker, 0);
    }

    //  ----------------- 下面是诗婷的代码----------------;

    vector<int> current_CT_Worker[current_Number_Worker]; // 记录当前worker已分配的任务，已修改
    double current_MaxDistanceTask[current_Number_Task];  // 任务允许的最大绕路距离，根据任务的开始和截止时间计算初始化，后面会根据work和task位置关系进一步更新，用于剪枝
    double current_task_NeedTime = 0.0;                   // 记录到任务位置所需时间并返回
    // // 存储detour_distance
    // double AD[current_Number_Worker]; // hy  AT记录worker的剩余可用时间，AD记录worker的剩余可用偏移距离。
    // Initialize(AD, Sumdis);   / / hy 初始化剩余绕路距离当前时间-剩余时间，已修改到分组划分的位置

    computeMaxDitanceTask(current_MaxDistanceTask, current_taskGroup, current_window_endTime); // 初始化各任务在满足deadline限制的条件下，每个任务可走的最长距离。

    // 实现基于任务距离的贪心算法
    vector<int> current_Group_worker_AW; // 定义可用工人的集合为vector变量

    for (int i = 0; i < current_Number_Worker; i++) // 初始化可用工人集合
    {
        current_Group_worker_AW.push_back(i);
    }

    for (int i = 0; i < current_Number_Task; i++) // 匹配每个任务
    {

        if (current_Group_worker_AW.size() != 0) // 当还有可用工人时，可对任务继续进行匹配
        {
            int workerid = -1;
            // workerid = FindLatestWorker(i,AW,AD); //为任务匹配最近的工人，计算剩余AD
            workerid = FindLatestWorkerNew_Greedy(i, current_Group_worker_AW, current_Group_workerAD, current_detour_distance, current_Group_worker_subTrajectoryDis, current_CT_Worker, current_MaxDistanceTask, &current_task_NeedTime, current_taskGroup, current_workerGroup, current_poi, current_window_endTime); // 寻找最近的工人来进行匹配

            // 找到工人后，更新信息，以便分组内其他的匹配
            if (workerid != -1) // 有可匹配的工人
            {
                cout << "\t任务:" << i << ",工人" << workerid << "匹配成功!" << endl;
                cout << "\t原始任务:" << current_taskGroup[i].Original_Local << ",工人" << current_workerGroup[workerid].Original_Local << "匹配成功!" << endl;
                // cout << "任务的时间是：" << endl;
                // cout << global_tasks[current_workerGroup[workerid].Original_Local].startTime << "  " << global_tasks[current_workerGroup[workerid].Original_Local].Deadline << endl;
                // Matching.push_back(make_pair(i, workerid));
                current_workerGroup[workerid].sign = false; // 当前分组内已被使用，不能成为下一分组的元素
                current_taskGroup[i].sign = false;          // 当前分组内已被使用，不能成为下一分组的元素
                current_CT_Worker[workerid].push_back(i);
                global_CT_Worker[current_workerGroup[workerid].Original_Local].push_back(current_taskGroup[i].Original_Local);
                // 更新AD,...
                current_Group_workerAD[workerid] = current_Group_workerAD[workerid] - 2 * current_detour_distance[i][workerid]; // 计算剩余AD[workerid]
                // 更新任务的最大可走距离
                UpdateTaskDeadline(workerid, i, current_detour_distance, current_CT_Worker, current_poi, current_MaxDistanceTask, current_task_NeedTime); // 更新，由于work增加新的task因此更新，约束条件的状态。
                if (current_CT_Worker[workerid].size() == Capacity)                                                                                       // 工人容量已达上限，不再参与匹配//这里看出current_Group_worker_AW和current_Group_worker性质一样，所以都要删除，增加了current_Group_worker删除操作
                {
                    for (vector<int>::iterator it = current_Group_worker_AW.begin(); it != current_Group_worker_AW.end(); ++it) // 移除工人
                    {
                        if (*it == workerid)
                        {
                            it = current_Group_worker_AW.erase(it);

                            //        cout<<workerid<<"工人已移除！"<<endl;
                            //        cout<<endl;
                            break;
                        }
                    }
                }
            }
            else
            {
                // cout << i << "任务无可加入的工人！" << endl;
                // cout << endl;
            }
        }
        else
            break; // 无可用工人则跳出循环，任务匹配结束。
    }

    ShowCTMatching(current_CT_Worker, current_Number_Worker); // 输出配对
}

/****
 *
 * 计算当工人和任务未匹配时,更新能进入下一窗口的工人和任务时间
 */

void Basic_information::print_groupWork(vector<CURRENT_WORKERS_GROUP> selected_workers)
{
    std::cout << " 当前组内的workers: ";
    for (const auto &w : selected_workers)
    {
        std::cout << "(" << w.worker.startTime << ", " << w.worker.endTime << ") ";
    }
    std::cout << std::endl;

    std::cout << "Remaining workers: ";
    for (const auto &w : global_workers)
    {
        std::cout << "(" << w.startTime << ", " << w.endTime << ") ";
    }
    std::cout << std::endl;
}

// /***
//  *  仅按任务的开始时间升序分组task。
//  */

// void Basic_information::print_groupTasks_startTime_simple()
// {
//     for (int i = 0; i < taskGroups_simple_result.size(); i++)
//     {
//         for (auto t : taskGroups_simple_result[i])
//         {
//             cout << t.task.startTime << ",,," << t.task.Deadline << endl;
//         }
//     }
// }

/**
 * 打印工人和任务的所有信息
 */
void Basic_information::print_info()
{
    // 打印100个WORKER的信息
    for (int i = 0; i < Number_Worker; i++)
    {
        cout << "Worker " << i << ":" << endl;
        cout << "\t Trajectory: ";
        for (auto poi : global_workers[i].trajectory)
        {
            cout << "(" << poi.X << ", " << poi.Y << ") ";
        }
        cout << endl;
        cout << "\t StartTime: " << global_workers[i].startTime << endl;
        cout << "\t EndTime: " << global_workers[i].endTime << endl;
        cout << "\t range: " << global_workers[i].range << endl;
        cout << "\t score: " << global_workers[i].score << endl;
    }

    // 打印100个task的信息
    for (int i = 0; i < Number_Task; i++)
    {
        cout << "task " << i << ":" << endl;
        cout << "\t X: " << global_tasks[i].X << endl;
        cout << "\t Y: " << global_tasks[i].Y << endl;
        cout << "\t Reward: " << global_tasks[i].Reward << endl;
        cout << "\t Minscore: " << global_tasks[i].Minscore << endl;
        cout << "\t StartTime: " << global_tasks[i].startTime << endl;
        cout << "\t Deadline: " << global_tasks[i].Deadline << endl;
    }
}

void Basic_information::printf_Satisfaction_Results(string alg_name, double run_time)
{

    ShowCTMatching(global_CT_Worker, Number_Worker); // 输出配对
    double task_satis_results;                       // 任务满意度
    double worker_satis_results;                     // 工人满意度

    if (Satisfaction_sign)
    {                                                                                         // 输出平均满意度
        task_satis_results = Caculate_Task_Satisfaction_avg(global_CT_Worker, global_PT);     // 任务满意度
        worker_satis_results = Caculate_Worker_Satisfaction_avg(global_CT_Worker, global_PW); // 工人满意度
        cout << "任务的平均满意度：" << task_satis_results << endl;

        cout << "工人的平均满意度：" << worker_satis_results << endl;
        printf_Results_to_txt(task_satis_results, worker_satis_results, "平均", alg_name, run_time);
    }
    else // 满意度求和
    {

        task_satis_results = Caculate_Task_Satisfaction_sum(global_CT_Worker, global_PT);     // 任务满意度
        worker_satis_results = Caculate_Worker_Satisfaction_sum(global_CT_Worker, global_PW); // 工人满意度
        cout << "任务的求和满意度：" << task_satis_results << endl;

        cout << "工人的求和满意度：" << worker_satis_results << endl;
        printf_Results_to_txt(task_satis_results, worker_satis_results, "求和", alg_name, run_time);
    }

    cout << endl;
}
void Basic_information::printf_Results_to_txt(double task_satis_results, double worker_satis_results, string sati_name, string alg_name, double run_time)
{

    // double task_satis_results = Caculate_Task_Satisfaction_avg(global_CT_Worker, global_PT);     // 任务满意度
    // double worker_satis_results = Caculate_Worker_Satisfaction_avg(global_CT_Worker, global_PW); // 工人满意度
    int sumtasks = 0;
    int workers = 0;
    ofstream out("../Satisfaction_Results.txt", ios::app);

    for (int i = 0; i < Number_Worker; i++)
    {

        if (global_CT_Worker[i].size() != 0)
        {
            sumtasks += global_CT_Worker[i].size();
            workers++;
        }
    }

    // out << "\t任务数：" << Number_Task << "\t 工人数：" << Number_Worker << "\t工人容量:" << Capacity << "\t工人成本/单位" << c << "\t移动速度:" << speed
    //     << "\t窗口时间：" << Wmax << "\t窗口任务：" << Tmax << "\n";
    out << "\t ---------------           \n";
    out << "\t 算法名：\t" << alg_name << "\t\t满意度计算方式：\t" << sati_name << "\n";
    out << "\t 任务的" << sati_name << "满意度：\t" << task_satis_results << "\n";
    out << "\t 工人的" << sati_name << "满意度：\t" << worker_satis_results << "\n";
    out << "\t (任务)匹配数：" << sumtasks << "\t 工人匹配数：" << workers << "\n";
    out << "\t 运行时间:\t" << run_time << "ms\n";
    out << "\n";
    out.close();

    cout << endl;
}
/***
 *
 * 开始workerBatch的代码
 */
void Basic_information::Grouping_Framework_WorkerBatch(vector<TASK> &tasks, vector<WORKER> &workers, double Wmax, int Tmax, vector<double> &Sumdis, vector<vector<double>> &global_Worker_subTrajectoryDis)

{
    vector<CURRENT_TASK_GROUP> current_taskGroup;              // 当前时间窗口内的任务列表，已获得
    vector<CURRENT_WORKERS_GROUP> current_Group_worker;        // 当前窗口内工人，已获得
    vector<double> current_Group_workerSumdis;                 // 当前窗口内工人轨迹的总距离。已获得
    vector<double> current_Group_workerAD;                     // 当前窗口内工人剩余可用距离，已获得
    double current_window_endTime = tasks[0].startTime + Wmax; // 当前时间窗口的结束时间，已获得
    double current_window_startTime = tasks[0].startTime;      // 当前时间窗口的起始时间，已获得
    int current_workID = global_Current_workID;                // 判断当前窗口的工人从几号开始的，已获得

    vector<vector<double>> current_Group_worker_subTrajectoryDis; // 当前窗口内工人轨迹点之和，已获得
    // vector<vector<double>> current_detour_distance;               // 当前窗口内最大绕路距离

    int sign = 0;
    cout << "\n------------------分组开始--------------" << endl;
    for (int i = 0; i < tasks.size(); i++)
    {

        TASK task = tasks[i];
        sign++;

        CURRENT_TASK_GROUP taskg;
        taskg.task = task;
        taskg.Original_Local = i;
        current_taskGroup.push_back(taskg);

        // 如果当前分组数量已达到Tmax，则创建新的分组
        // 如果任务的开始时间大于等于当前窗口的截止时间，则创建新的分组
        if ((current_taskGroup.size() == Tmax) || (task.startTime >= current_window_endTime))
        {
            current_workID = global_Current_workID;
            current_window_endTime = task.startTime; // 下一窗口的开始时间，也就是上一窗口的结束时间
            cout << sign << " 1\t当前窗口截止时间：" << current_window_endTime << endl;
            // 计算当前窗口满足时间约束的work。
            groupWork_according_TaskGroup(workers, Sumdis, current_Group_worker, current_Group_workerSumdis, current_window_endTime, current_workID, current_Group_workerAD, global_Worker_subTrajectoryDis, current_Group_worker_subTrajectoryDis); // 计算当前窗口满足时间约束的work。
            determine_Window_Task_Timeout(current_taskGroup, current_window_endTime);

            // 任务工人配对，并计算当工人和任务未匹配时,能进入下一窗口的工人和任务，

            // match_WorkerTask(current_taskGroup, current_Group_worker, current_window_endTime, current_Group_workerAD, current_Group_worker_subTrajectoryDis);        // 当前任务组,当前工人组,当前窗口截止时间，当前工人剩余绕行距离
            match_WorkerTask_workerBatch(current_taskGroup, current_Group_worker, current_window_endTime, current_Group_workerAD, current_Group_worker_subTrajectoryDis); // 当前任务组,当前工人组,当前窗口截止时间，当前工人剩余绕行距离

            // 为下一窗口做准备，即最大的截止时间
            current_window_startTime = current_window_endTime;
            current_window_endTime += Wmax;

            // updata_nextWindow_Worker_Task(current_taskGroup, current_Group_worker, current_window_endTime); // 将剩余工人加入当前组。并更新他们的开始时间为当前截止时间
            cout << "\n-----------------------------分组结束\n"
                 << endl;
            updata_current_Info(current_taskGroup, current_Group_worker, current_Group_workerAD, current_Group_worker_subTrajectoryDis, current_Group_workerSumdis);
        }

        // 将剩余任务加入当前分组，并更新他们的开始时间为当前时间。
    }
    // 将最后一个分组加入分组列表
    if (current_taskGroup.size() > 0)
    {

        current_workID = global_Current_workID;
        current_window_startTime = current_taskGroup.back().task.startTime;
        cout << sign << "   最后一组，下一时间窗口为：" << current_window_startTime << endl;
        // 计算分组内的工人以及他们剩余绕行距离
        groupWork_according_TaskGroup(workers, Sumdis, current_Group_worker, current_Group_workerSumdis, current_window_endTime, current_workID, current_Group_workerAD, global_Worker_subTrajectoryDis, current_Group_worker_subTrajectoryDis); // 计算当前窗口满足时间约束的work。

        determine_Window_Task_Timeout(current_taskGroup, current_window_endTime);
        // 对分组内任务和工人进行匹配。
        match_WorkerTask_workerBatch(current_taskGroup, current_Group_worker, current_window_endTime, current_Group_workerAD, current_Group_worker_subTrajectoryDis); // 当前任务组,当前工人组,当前窗口截止时间，当前工人剩余绕行距离
                                                                                                                                                                      // updata_nextWindow_Worker_Task(current_taskGroup, current_Group_worker, current_window_startTime);
                                                                                                                                                                      // current_taskGroup.clear();
    }
    // print_groupTasks_addEndTime();
}

/**
 * 求解分组内工人和任务的偏好
 */
void Basic_information::Compute_PTPW_Group_workerBatch(vector<vector<pair<int, double>>> &current_PT, vector<vector<pair<int, double>>> &current_PW, vector<double> current_detour_distance[], vector<CURRENT_TASK_GROUP> &current_taskGroup, vector<CURRENT_WORKERS_GROUP> &current_workerGroup, vector<int> current_poi[]) // 诗婷增加, 已完全修改
{
    // 计算PT,PW
    cout << "进入分组内的偏好计算" << endl;
    int Number_current_taskGroup = current_taskGroup.size();
    int Number_current_workerGroup = current_workerGroup.size();
    for (int i = 0; i < Number_current_taskGroup; i++)
    {
        for (int j = 0; j < Number_current_workerGroup; j++)
        {
            if (!(current_taskGroup[i].task.Deadline < current_workerGroup[j].worker.startTime || current_taskGroup[i].task.startTime > current_workerGroup[j].worker.endTime)) // 满足时间约束
            {
                if (current_taskGroup[i].task.Minscore <= current_workerGroup[j].worker.score) // 否则绕路距离为0！！错误，改为否则绕路距离为无穷大，正确：分数不满足时不会进入到偏好列表中
                {
                    // 工人的分数满足任务的最小约束                                                                                    //为何错误？？？？
                    current_detour_distance[i][j] = Caculate_mindist(j, i, current_poi, current_taskGroup, current_workerGroup); // 计算每个任务和每个worker之间的最小绕路距离

                    if (current_detour_distance[i][j] <= current_workerGroup[j].worker.range)
                    {
                        double preference1 = current_taskGroup[i].task.Reward - 2 * current_detour_distance[i][j] * c; // 工人的偏好值
                        if (preference1 > 0)                                                                           // 利润大于0
                        {
                            double preference2 = current_workerGroup[j].worker.score; // 任务的偏好值
                            current_PT[i].push_back(make_pair(j, preference2));
                            cout << "\t\t当前分组内偏好计算, 任务" << i << "\t工人" << j << "偏好计算成功" << endl;
                            cout << "\t\t原始任务:" << current_taskGroup[i].Original_Local << ",工人" << current_workerGroup[j].Original_Local << "匹配成功!" << endl;

                            current_PW[j].push_back(make_pair(i, preference1));
                        }
                        else
                        {
                            // cout<<taskid<<"\t"<<j<<"距离太远!"<<endl;
                        }
                    }
                }
            }
        }
    }

    for (int i = 0; i < current_workerGroup.size(); i++) // 计算工人偏好列表
    {
        // 对工人的偏序列表排序
        sort(current_PW[i].begin(), current_PW[i].end(), cmp);
    }

    cout << endl;
}

void Basic_information::match_WorkerTask_workerBatch(vector<CURRENT_TASK_GROUP> &current_taskGroup, vector<CURRENT_WORKERS_GROUP> &current_workerGroup, double current_window_endTime, vector<double> &current_Group_workerAD, vector<vector<double>> &current_Group_worker_subTrajectoryDis)
{
    int current_Number_Worker = current_workerGroup.size();
    int current_Number_Task = current_taskGroup.size();
    // double current_detour_distance[current_Number_Task][current_Number_Worker];
    // int poi[current_Number_Task][current_Number_Worker]; // 存储poiid，记录哪个点是最小的
    vector<double> current_detour_distance[current_Number_Task]; // 存储当前分组内，任务到每个工人的最近绕路距离，并初始化，防止数组越界。
    vector<int> current_poi[current_Number_Task];                // 存储poiid，记录哪个点到任务距离是最小的
    vector<vector<pair<int, double>>> current_PT(current_Number_Task);
    vector<vector<pair<int, double>>> current_PW(current_Number_Worker);
    for (int i = 0; i < current_Number_Task; i++)
    {
        current_detour_distance[i] = vector<double>(current_Number_Worker, 0);
        // current_detour_distance[i].push_back(vector<double>(current_Number_Worker, 0));
        current_poi[i] = vector<int>(current_Number_Worker, 0);
    }
    Compute_PTPW_Group_workerBatch(current_PT, current_PW, current_detour_distance, current_taskGroup, current_workerGroup, current_poi); // 诗婷增加, 已完全修改

    //  ----------------- 下面是诗婷的代码----------------;

    vector<int> current_CT_Worker[current_Number_Worker]; // 记录当前worker已分配的任务，已修改
    double current_MaxDistanceTask[current_Number_Task];  // 任务允许的最大绕路距离，根据任务的开始和截止时间计算初始化，后面会根据work和task位置关系进一步更新，用于剪枝
    double current_task_NeedTime = 0.0;                   // 记录到任务位置所需时间并返回
    // // 存储detour_distance
    // double AD[current_Number_Worker]; // hy  AT记录worker的剩余可用时间，AD记录worker的剩余可用偏移距离。
    // Initialize(AD, Sumdis);   / / hy 初始化剩余绕路距离当前时间-剩余时间，已修改到分组划分的位置

    computeMaxDitanceTask(current_MaxDistanceTask, current_taskGroup, current_window_endTime); // 初始化各任务在满足deadline限制的条件下，每个任务可走的最长距离。

    /**
     * 迭代匹配
     * 工人批量时，工人不可用或者任务不可用停止循环匹配
     * 未匹配或匹配失败的批次工人优先匹配
     * 偏好列表ORDER
     */

    iterator_Match_WorkBatch(current_workerGroup, current_taskGroup, current_PW,
                             current_Group_workerAD, current_detour_distance, current_Group_worker_subTrajectoryDis,
                             current_CT_Worker, current_MaxDistanceTask, &current_task_NeedTime, current_poi, current_window_endTime);

    ShowCTMatching(current_CT_Worker, current_Number_Worker); // 输出配对
}

void Basic_information::iterator_Match_WorkBatch(vector<CURRENT_WORKERS_GROUP> &current_workerGroup, vector<CURRENT_TASK_GROUP> &current_taskGroup, vector<vector<pair<int, double>>> &current_PW,
                                                 vector<double> &current_Group_workerAD, vector<double> current_detour_distance[], vector<vector<double>> &current_Group_worker_subTrajectoryDis,
                                                 vector<int> CT_Worker[], double MaxDistanceTask[], double *current_task_NeedTime, vector<int> current_poi[], double nowtime)
{
    int Number_Worker = current_workerGroup.size();
    int Number_Task = current_taskGroup.size();
    int Worker_Available[Number_Worker] = {0}; // 为0时上次分配失败，1表示上次分配成功,2 表示容量已满或者已遍历完【偏好列表】
    int Task_Available[Number_Task] = {0};     // 0表示未分配，1表示已分配
    int count_NAWorker = 0;                    // 记录容量已满或者已达偏好列表
    int count_NATask = 0;                      // 表示已匹配的任务数
    int CurrentTaskInPW[Number_Worker] = {0};  // 当前访问列表里的oder//上面为诗婷新增的代码
    int unmatchC = 0;
    for (int i = 0; i < current_workerGroup.size(); i++)
    {
        if (current_PW[i].size() == 0) // 工人的偏好列表中没有任务
        {
            Worker_Available[i] = 3; // 不可能参与匹配
            unmatchC++;
            //   cout<<"没有偏好列表的工人id"<<i<<endl;
        }
        else
        {
            //   WorkerAndPW[i] = make_pair(i, PW[i].size());
        }
    }
    int diedai = 0;
    while ((count_NAWorker + unmatchC) < Number_Worker && count_NATask < Number_Task) // 工人不可用或者任务不可用停止循环
    {

        diedai++;
        // cout<<"第"<<diedai<<"次迭代开始***********"<<endl;
        // cout<<"不可用工人数"<<count_NAWorker+unmatchC<<endl;
        // cout<<"不可用任务数"<<count_NAWorker+unmatchC<<endl;
        vector<int> updateObj; // 两个批次都分配结束后才更新第一批次的工人状态；

        // 先对cout<<"匹配失败的工人开始匹配*********"<<endl;
        // 后对匹配的工人工人再次匹配。
        // int flag[2] = {0, 1}; // 当flag为0时为匹配失败的工人开始匹配，当为1时，成功的再匹配
        for (int flag = 0; flag < 2; flag++)
        {
            for (int i = 0; i < Number_Worker; i++)
            // for(int j=0;j<Number_Worker;j++)
            {
                //  int i = WorkerAndPW[j].first; //i表示工人id
                if (Worker_Available[i] == flag) // 0为未匹配或匹配失败的批次工人优先匹配，1为已匹配的工人延后匹配
                {

                    int orderinPW = CurrentTaskInPW[i]; // 偏好列表ORDER
                    // cout<<"工人"<<i<<"遍历pw的任务序号："<<orderinPW<<endl;
                    int current_task_id = current_PW[i][orderinPW].first; // 当前访问的任务编号
                    // cout<<"当前访问的任务:"<<current_task_id <<endl;
                    CurrentTaskInPW[i]++;                     // 序号+1,下一个任务
                    if (Task_Available[current_task_id] == 0) // 当前任务未匹配
                    {
                        // 是否满足所有任务的Deadline限制，如果满足则更新后续影响任务的最大可走距离
                        if (CurrentTask_Satisfy(current_taskGroup, i, current_task_id, current_Group_workerAD, current_detour_distance, current_Group_worker_subTrajectoryDis, CT_Worker, current_poi, MaxDistanceTask, current_task_NeedTime, nowtime)) // 变量地址传递
                        {
                            // cout<<"可插入任务"<<endl;
                            Task_Available[current_task_id] = 1; // 更新任务为已匹配状态1
                            count_NATask++;                      // 已分配任务数+1
                            CT_Worker[i].push_back(current_task_id);

                            // hy黄阳↓↓↓↓↓↓↓↓

                            current_workerGroup[i].sign = false;                                                                                  // 当前分组内工人已被使用，不能成为下一分组的元素
                            current_taskGroup[current_task_id].sign = false;                                                                      // 当前分组内任务已被使用
                            global_CT_Worker[current_workerGroup[i].Original_Local].push_back(current_taskGroup[current_task_id].Original_Local); // 标记全局中已经匹配的结果

                            // hy黄阳↑↑↑↑↑↑↑

                            // 更新工人的已匹配任务；
                            current_Group_workerAD[i] = current_Group_workerAD[i] - 2 * current_detour_distance[current_task_id][i]; // 计算剩余AD[workerid]
                            // 更新工人的可绕路距离;
                            UpdateTaskDeadline(i, current_task_id, current_detour_distance, CT_Worker, current_poi, MaxDistanceTask, *current_task_NeedTime); // 更新，由于work增加新的task因此更新，约束条件的状态。
                            //*******更新任务的最大可行驶距离

                            if (CT_Worker[i].size() == Capacity)
                            // 判断工人容量是否已达标
                            {
                                count_NAWorker++;        // 不可用+1
                                Worker_Available[i] = 3; // 不可能再参与匹配
                                                         // cout<<"工人"<<i<<"容量达标"<<endl;
                            }

                            else if (flag == 0)
                            {
                                updateObj.push_back(i); // 记录需要更新为状态1的工人
                                                        // cout<<" 记录需要更新为状态1的工人:"<<i<<endl;
                            }
                        }
                        else if (flag == 0) // 不满足约束Deadline约束，仍然失败
                        {
                        }
                        else if (flag == 1) // 当前任务已匹配，本次匹配失败
                        {
                            Worker_Available[i] = 0; // 更新为匹配失败状态
                        }
                    }
                    else if (flag == 1) // 当前任务已匹配，本次匹配失败
                    {
                        Worker_Available[i] = 0; // 更新为匹配失败状态
                    }

                    if (CurrentTaskInPW[i] == current_PW[i].size()) // 已遍历偏好列表
                    {
                        if (Worker_Available[i] != 3) // 容量未达标
                        {
                            count_NAWorker++;        // 已达结尾工人，不可用+1
                            Worker_Available[i] = 3; // 更新为不可再用的工人
                                                     // cout<<"工人"<<i<<"PW遍历完成"<<endl;
                        }
                    }
                    else // 还可继续遍历，下次继续仍然是失败状态
                    {
                    }
                    //   }
                }
            }
        }

        // 更新第一批次还可以继续匹配的工人的状态。黄阳我不懂

        for (int i = 0; i < updateObj.size(); i++)
        {
            int updateworkerid = updateObj[i];
            if (Worker_Available[updateworkerid] != 3) // 没遍历到偏好列表尾部
            {
                Worker_Available[updateworkerid] = 1; // 更新为匹配成功的工人状态
                                                      // cout<<"更新从0变为1的工人"<<updateworkerid<<endl;
            }
        }
    }
}

/***
 * TPPG算法
 */

/***
 * 窗口划分框架
 * 首先根据当前任务分组，获取可以服务的工人
 * 其次考虑当前分组中任务未匹配且能继续分配到下一组的情况。
 *
 * 要求：修改groupTasks_addEndTime的第3、4条
 *
 * 1. 每一组task处于连续的时间窗口Wmax，Wmax；
 * 2. 一组内的task数量不超过Tmax。
 * 3. 若一组任务数量达到为Tmax时，则以当前current_taskGroup的时间重新创建新的窗口。
 *      3.1 任务和工人进行匹配
 *      3.2 对于未匹配的任务划分到下一组，修改任务的开始时间（为了防止计算下一分组开始时间出错）
 * 4. 若已到达当前窗口时间current_window_endTime，纵然当前窗口current_window_endTime的task数量位到达Tmax也创建新的窗口。
 *      4.1 任务和工人进行匹配
 *      4.2 对于未匹配的任务划分到下一组，修改任务的开始时间（为了防止计算下一分组开始时间出错）
 */
void Basic_information::Grouping_Framework_TPPG(vector<TASK> &tasks, vector<WORKER> &workers, double Wmax, int Tmax, vector<double> &Sumdis, vector<vector<double>> &global_Worker_subTrajectoryDis)

{
    vector<CURRENT_TASK_GROUP> current_taskGroup;              // 当前时间窗口内的任务列表，已获得
    vector<CURRENT_WORKERS_GROUP> current_Group_worker;        // 当前窗口内工人，已获得
    vector<double> current_Group_workerSumdis;                 // 当前窗口内工人轨迹的总距离。已获得
    vector<double> current_Group_workerAD;                     // 当前窗口内工人剩余可用距离，已获得
    double current_window_endTime = tasks[0].startTime + Wmax; // 当前时间窗口的结束时间，已获得
    double current_window_startTime = tasks[0].startTime;      // 当前时间窗口的起始时间，已获得
    int current_workID = global_Current_workID;                // 判断当前窗口的工人从几号开始的，已获得

    vector<vector<double>> current_Group_worker_subTrajectoryDis; // 当前窗口内工人轨迹点之和，已获得
    // vector<vector<double>> current_detour_distance;               // 当前窗口内最大绕路距离

    int sign = 0;

    for (int i = 0; i < tasks.size(); i++)
    {

        TASK task = tasks[i];
        sign++;

        CURRENT_TASK_GROUP taskg;
        taskg.task = task;
        taskg.Original_Local = i;
        current_taskGroup.push_back(taskg);

        // 如果当前分组数量已达到Tmax，则创建新的分组
        // 如果任务的开始时间大于等于当前窗口的截止时间，则创建新的分组
        if ((current_taskGroup.size() == Tmax) || (task.startTime >= current_window_endTime))
        {

            current_workID = global_Current_workID;

            current_window_endTime = task.startTime; // 下一窗口的开始时间，也就是上一窗口的结束时间

            cout << sign << " 1\t当前窗口截止时间：" << current_window_endTime << endl;
            // 计算当前窗口满足时间约束的work。
            groupWork_according_TaskGroup(workers, Sumdis, current_Group_worker, current_Group_workerSumdis, current_window_endTime, current_workID, current_Group_workerAD, global_Worker_subTrajectoryDis, current_Group_worker_subTrajectoryDis); // 计算当前窗口满足时间约束的work。
            determine_Window_Task_Timeout(current_taskGroup, current_window_endTime);
            cout << "2\t开始分组内进行匹配" << endl;

            // 任务工人配对，并计算当工人和任务未匹配时,能进入下一窗口的工人和任务，

            // match_WorkerTask(current_taskGroup, current_Group_worker, current_window_endTime, current_Group_workerAD, current_Group_worker_subTrajectoryDis);        // 当前任务组,当前工人组,当前窗口截止时间，当前工人剩余绕行距离
            match_WorkerTask_TPPG(current_taskGroup, current_Group_worker, current_window_endTime, current_Group_workerAD, current_Group_worker_subTrajectoryDis); // 当前任务组,当前工人组,当前窗口截止时间，当前工人剩余绕行距离

            // 为下一窗口做准备，即最大的截止时间
            current_window_startTime = current_window_endTime;
            current_window_endTime += Wmax;

            // updata_nextWindow_Worker_Task(current_taskGroup, current_Group_worker, current_window_endTime); // 将剩余工人加入当前组。并更新他们的开始时间为当前截止时间
        }
        updata_current_Info(current_taskGroup, current_Group_worker, current_Group_workerAD, current_Group_worker_subTrajectoryDis, current_Group_workerSumdis);

        // 将剩余任务加入当前分组，并更新他们的开始时间为当前时间。
    }
    // 将最后一个分组加入分组列表
    if (current_taskGroup.size() > 0)
    {

        current_workID = global_Current_workID;
        current_window_startTime = current_taskGroup.back().task.startTime;
        cout << sign << "   最后一组，下一时间窗口为：" << current_window_startTime << endl;
        // 计算分组内的工人以及他们剩余绕行距离
        groupWork_according_TaskGroup(workers, Sumdis, current_Group_worker, current_Group_workerSumdis, current_window_endTime, current_workID, current_Group_workerAD, global_Worker_subTrajectoryDis, current_Group_worker_subTrajectoryDis); // 计算当前窗口满足时间约束的work。

        determine_Window_Task_Timeout(current_taskGroup, current_window_endTime);
        // 对分组内任务和工人进行匹配。
        match_WorkerTask_TPPG(current_taskGroup, current_Group_worker, current_window_endTime, current_Group_workerAD, current_Group_worker_subTrajectoryDis); // 当前任务组,当前工人组,当前窗口截止时间，当前工人剩余绕行距离
                                                                                                                                                               // updata_nextWindow_Worker_Task(current_taskGroup, current_Group_worker, current_window_startTime);
                                                                                                                                                               // current_taskGroup.clear();
    }
    // print_groupTasks_addEndTime();
}

void Basic_information::match_WorkerTask_TPPG(vector<CURRENT_TASK_GROUP> &current_taskGroup, vector<CURRENT_WORKERS_GROUP> &current_workerGroup, double current_window_endTime, vector<double> &current_Group_workerAD, vector<vector<double>> &current_Group_worker_subTrajectoryDis)
{
    int current_Number_Worker = current_workerGroup.size();
    int current_Number_Task = current_taskGroup.size();
    // double current_detour_distance[current_Number_Task][current_Number_Worker];
    // int poi[current_Number_Task][current_Number_Worker]; // 存储poiid，记录哪个点是最小的
    vector<double> current_detour_distance[current_Number_Task]; // 存储当前分组内，任务到每个工人的最近绕路距离，并初始化，防止数组越界。
    vector<int> poi[current_Number_Task];                        // 存储poiid，记录哪个点到任务距离是最小的
    for (int i = 0; i < current_Number_Task; i++)
    {
        current_detour_distance[i] = vector<double>(current_Number_Worker, 0);
        // current_detour_distance[i].push_back(vector<double>(current_Number_Worker, 0));
        poi[i] = vector<int>(current_Number_Worker, 0);
    }

    cout << "---------------任务开始---------" << endl;

    //  ----------------- 下面是诗婷的代码----------------;

    // ShowDetour_Distance(detour_distance,poi);
    // 对于每个任务，计算任务的可用工人：首先，满足range约束，满足最小声誉分数约束；其次，在匹配时再判断是否满足deadline约束，容量约束
    // 然后计算与每个可用工人的距离，选择最近的工人
    vector<int> current_CT_Worker[current_Number_Worker]; // 记录当前worker已分配的任务，已修改
    double current_MaxDistanceTask[current_Number_Task];  // 任务允许的最大绕路距离，根据任务的开始和截止时间计算初始化，后面会根据work和task位置关系进一步更新，用于剪枝
    double current_task_NeedTime = 0.0;                   // 记录到任务位置所需时间并返回
    // // 存储detour_distance
    // double AD[current_Number_Worker]; // hy  AT记录worker的剩余可用时间，AD记录worker的剩余可用偏移距离。
    // Initialize(AD, Sumdis);   / / hy 初始化剩余绕路距离当前时间-剩余时间，已修改到分组划分的位置

    computeMaxDitanceTask(current_MaxDistanceTask, current_taskGroup, current_window_endTime); // 初始化各任务在满足deadline限制的条件下，每个任务可走的最长距离。

    // 实现基于任务距离的贪心算法
    vector<int> current_Group_worker_AW; // 定义可用工人的集合为vector变量
    // int AWW[Number_Worker]; //0为可用，1不可用

    for (int i = 0; i < current_Number_Worker; i++) // 初始化可用工人集合
    {
        current_Group_worker_AW.push_back(i);
    }

    for (int i = 0; i < current_Number_Task; i++) // 匹配每个任务
    {
        // int taskid = i;

        if (current_Group_worker_AW.size() != 0) // 当还有可用工人时，可对任务继续进行匹配
        {
            int workerid = -1;
            // workerid = FindLatestWorker(i,AW,AD); //为任务匹配最近的工人，计算剩余AD
            //  hhh(poi);
            //  hhh1(current_detour_distance);

            // workerid = FindLatestWorkerNew(i, current_Group_worker_AW, current_Group_workerAD, current_detour_distance, current_Group_worker_subTrajectoryDis, current_CT_Worker, current_MaxDistanceTask, &current_task_NeedTime, current_taskGroup, current_workerGroup, poi); // 寻找最近的工人来进行匹配
            workerid = FindPreferedWorkerNew_TPPG(i, current_Group_worker_AW, current_Group_workerAD, current_detour_distance, current_Group_worker_subTrajectoryDis, current_CT_Worker, current_MaxDistanceTask, &current_task_NeedTime, current_taskGroup, current_workerGroup, poi, current_window_endTime); // 寻找最近的工人来进行匹配
            // 找到工人后，更新信息，以便分组内其他的匹配
            if (workerid != -1) // 有可匹配的工人
            {
                cout << "任务:" << i << ",工人" << workerid << "匹配成功!" << endl;
                // Matching.push_back(make_pair(i, workerid));
                current_workerGroup[workerid].sign = false; // 当前分组内已被使用，不能成为下一分组的元素
                current_taskGroup[i].sign = false;          // 当前分组内已被使用，不能成为下一分组的元素
                current_CT_Worker[workerid].push_back(i);
                global_CT_Worker[current_workerGroup[workerid].Original_Local].push_back(current_taskGroup[i].Original_Local);
                // 更新AD,...
                current_Group_workerAD[workerid] = current_Group_workerAD[workerid] - 2 * current_detour_distance[i][workerid]; // 计算剩余AD[workerid]
                // 更新任务的最大可走距离
                UpdateTaskDeadline(workerid, i, current_detour_distance, current_CT_Worker, poi, current_MaxDistanceTask, current_task_NeedTime); // 更新，由于work增加新的task因此更新，约束条件的状态。

                //  cout<<i<<"工人满足deadline约束！"<<workerid<<endl;
                // 匹配;

                // cout<<workerid<<"已匹配的任务数："<<assignedtask<<endl;
                if (current_CT_Worker[workerid].size() == Capacity) // 工人容量已达上限，不再参与匹配//这里看出current_Group_worker_AW和current_Group_worker性质一样，所以都要删除，增加了current_Group_worker删除操作
                {
                    for (vector<int>::iterator it = current_Group_worker_AW.begin(); it != current_Group_worker_AW.end(); ++it) // 移除工人
                    {
                        if (*it == workerid)
                        {
                            it = current_Group_worker_AW.erase(it);

                            //        cout<<workerid<<"工人已移除！"<<endl;
                            //        cout<<endl;
                            break;
                        }
                    }
                }
            }
            else
            {
                // cout << i << "任务无可加入的工人！" << endl;
                // cout << endl;
            }
        }
        else
            break; // 无可用工人则跳出循环，任务匹配结束。

        // -- -修改到这里了。是否记录当前一匹配的任务，工人，用于记分或者，求解满意度
    }
    ShowCTMatching(current_CT_Worker, current_Number_Worker); // 输出配对
}

int Basic_information::FindPreferedWorkerNew_TPPG(int taskid, vector<int> &current_Group_worker_AW, vector<double> &current_Group_workerAD, vector<double> current_detour_distance[], vector<vector<double>> &current_Group_worker_subTrajectoryDis, vector<int> CT_Worker[], double MaxDistanceTask[], double *current_task_NeedTime, vector<CURRENT_TASK_GROUP> &current_taskGroup, vector<CURRENT_WORKERS_GROUP> &current_workerGroup, vector<int> poi[], double nowtime)
{
    // 工人i,AW可用工人集合、AD剩余可用偏移距离、DD绕路距离、WS工人轨迹、CT_Worker工人当前工作、MaxDistanceTask最大距离、current_task_NeedTime当前任务时间

    //     CurrentTask_Satisfy(detour_distance, global_Worker_subTrajectoryDis,CT_Worker,MaxDistanceTask,*current_task_NeedTime);
    // 找最近的工人
    int best_workerid = -1;
    double maxpre = -__DBL_MAX__;
    double best_distance = 0;
    for (int j = 0; j < current_Group_worker_AW.size(); j++)
    { // 遍历所有可用工人
        int workerid = current_Group_worker_AW[j];
        if (current_taskGroup[taskid].task.Minscore <= current_workerGroup[workerid].worker.score) // worker分数满足大于MinScore
        {
            double preference = current_workerGroup[workerid].worker.score;
            if (preference > maxpre) // 偏好值大于约束,********
            {

                double dist = Caculate_mindist(workerid, taskid, poi, current_taskGroup, current_workerGroup); // 计算最小绕路距离

                // 新增绕路detour_distance的计算
                current_detour_distance[taskid][workerid] = dist;

                if (current_workerGroup[workerid].worker.range >= dist) // 计算绕路距离距离小于range
                {
                    /********+任务的Deadline限制*/

                    if (current_taskGroup[taskid].task.Reward - (2 * dist * c) > 0) // 利润大于0
                    {
                        if (CurrentTask_Satisfy(current_taskGroup, workerid, taskid, current_Group_workerAD, current_detour_distance, current_Group_worker_subTrajectoryDis, CT_Worker, poi, MaxDistanceTask, current_task_NeedTime, nowtime)) // 未修改
                        // if(SatisfiedDeadline(workerid, taskid, AD, dist)) //满足deadline约束+其它约束
                        {

                            maxpre = preference;
                            best_workerid = workerid;
                            best_distance = dist;
                        }
                    }
                }
            }
        }
    }
    return best_workerid;
}

/**
 * 4.4TSDA
 */

void Basic_information::Grouping_Framework_TSDA(vector<TASK> &tasks, vector<WORKER> &workers, double Wmax, int Tmax, vector<double> &Sumdis, vector<vector<double>> &global_Worker_subTrajectoryDis)

{
    vector<CURRENT_TASK_GROUP> current_taskGroup;              // 当前时间窗口内的任务列表，已获得
    vector<CURRENT_WORKERS_GROUP> current_Group_worker;        // 当前窗口内工人，已获得
    vector<double> current_Group_workerSumdis;                 // 当前窗口内工人轨迹的总距离。已获得
    vector<double> current_Group_workerAD;                     // 当前窗口内工人剩余可用距离，已获得
    double current_window_endTime = tasks[0].startTime + Wmax; // 当前时间窗口的结束时间，已获得
    double current_window_startTime = tasks[0].startTime;      // 当前时间窗口的起始时间，已获得
    int current_workID = global_Current_workID;                // 判断当前窗口的工人从几号开始的，已获得

    vector<vector<double>> current_Group_worker_subTrajectoryDis; // 当前窗口内工人轨迹点之和，已获得
    // vector<vector<double>> current_detour_distance;               // 当前窗口内最大绕路距离

    int sign = 0;
    cout << "\n------------------分组开始--------------" << endl;
    for (int i = 0; i < tasks.size(); i++)
    {

        TASK task = tasks[i];
        sign++;

        CURRENT_TASK_GROUP taskg;
        taskg.task = task;
        taskg.Original_Local = i;
        current_taskGroup.push_back(taskg);

        // 如果当前分组数量已达到Tmax，则创建新的分组
        // 如果任务的开始时间大于等于当前窗口的截止时间，则创建新的分组
        if ((current_taskGroup.size() == Tmax) || (task.startTime >= current_window_endTime))
        {
            current_workID = global_Current_workID;
            current_window_endTime = task.startTime; // 下一窗口的开始时间，也就是上一窗口的结束时间
            cout << sign << " 1\t当前窗口截止时间：" << current_window_endTime << endl;
            // 计算当前窗口满足时间约束的work。
            groupWork_according_TaskGroup(workers, Sumdis, current_Group_worker, current_Group_workerSumdis, current_window_endTime, current_workID, current_Group_workerAD, global_Worker_subTrajectoryDis, current_Group_worker_subTrajectoryDis); // 计算当前窗口满足时间约束的work。
            determine_Window_Task_Timeout(current_taskGroup, current_window_endTime);

            // 任务工人配对，并计算当工人和任务未匹配时,能进入下一窗口的工人和任务，

            // match_WorkerTask(current_taskGroup, current_Group_worker, current_window_endTime, current_Group_workerAD, current_Group_worker_subTrajectoryDis);        // 当前任务组,当前工人组,当前窗口截止时间，当前工人剩余绕行距离
            match_WorkerTask_TSDA(current_taskGroup, current_Group_worker, current_window_endTime, current_Group_workerAD, current_Group_worker_subTrajectoryDis); // 当前任务组,当前工人组,当前窗口截止时间，当前工人剩余绕行距离

            // 为下一窗口做准备，即最大的截止时间
            current_window_startTime = current_window_endTime;
            current_window_endTime += Wmax;

            // updata_nextWindow_Worker_Task(current_taskGroup, current_Group_worker, current_window_endTime); // 将剩余工人加入当前组。并更新他们的开始时间为当前截止时间
            cout << "\n-----------------------------分组结束\n"
                 << endl;
            updata_current_Info(current_taskGroup, current_Group_worker, current_Group_workerAD, current_Group_worker_subTrajectoryDis, current_Group_workerSumdis);
        }

        // 将剩余任务加入当前分组，并更新他们的开始时间为当前时间。
    }
    // 将最后一个分组加入分组列表
    if (current_taskGroup.size() > 0)
    {

        current_workID = global_Current_workID;
        current_window_startTime = current_taskGroup.back().task.startTime;
        cout << sign << "   最后一组，下一时间窗口为：" << current_window_startTime << endl;
        // 计算分组内的工人以及他们剩余绕行距离
        groupWork_according_TaskGroup(workers, Sumdis, current_Group_worker, current_Group_workerSumdis, current_window_endTime, current_workID, current_Group_workerAD, global_Worker_subTrajectoryDis, current_Group_worker_subTrajectoryDis); // 计算当前窗口满足时间约束的work。

        determine_Window_Task_Timeout(current_taskGroup, current_window_endTime);
        // 对分组内任务和工人进行匹配。
        match_WorkerTask_workerBatch(current_taskGroup, current_Group_worker, current_window_endTime, current_Group_workerAD, current_Group_worker_subTrajectoryDis); // 当前任务组,当前工人组,当前窗口截止时间，当前工人剩余绕行距离
                                                                                                                                                                      // updata_nextWindow_Worker_Task(current_taskGroup, current_Group_worker, current_window_startTime);
                                                                                                                                                                      // current_taskGroup.clear();
    }
    // print_groupTasks_addEndTime();
}

void Basic_information::match_WorkerTask_TSDA(vector<CURRENT_TASK_GROUP> &current_taskGroup, vector<CURRENT_WORKERS_GROUP> &current_workerGroup, double current_window_endTime, vector<double> &current_Group_workerAD, vector<vector<double>> &current_Group_worker_subTrajectoryDis)
{
    int current_Number_Worker = current_workerGroup.size();
    int current_Number_Task = current_taskGroup.size();
    // double current_detour_distance[current_Number_Task][current_Number_Worker];
    // int poi[current_Number_Task][current_Number_Worker]; // 存储poiid，记录哪个点是最小的
    vector<double> current_detour_distance[current_Number_Task]; // 存储当前分组内，任务到每个工人的最近绕路距离，并初始化，防止数组越界。
    vector<int> current_poi[current_Number_Task];                // 存储poiid，记录哪个点到任务距离是最小的
    vector<vector<pair<int, double>>> current_PT(Number_Task);
    vector<vector<pair<int, double>>> current_PW(Number_Worker);

    int current_CW_Task[current_Number_Task];                    // 记录number_task个task的当前对象
    int current_num_of_chased_worker[current_Number_Task] = {0}; // 任务追求过的工人的数量
    vector<int> current_ActiveTask;                              // 记录当前活动任务集
    vector<int> current_NextActiveTask(current_ActiveTask);
    // vector<int> current_CT_Worker[current_Number_Worker];                            // 记录当前worker已分配的任务，已修改
    vector<vector<int>> current_CT_Worker(current_Number_Worker);
    double current_MaxDistanceTask[current_Number_Task] = {0.0}; // 任务允许的最大绕路距离，根据任务的开始和截止时间计算初始化，后面会根据work和task位置关系进一步更新，用于剪枝
    double current_task_NeedTime = 0.0;                          // 记录到任务位置所需时间并返回

    for (int i = 0; i < current_Number_Task; i++)
    {
        current_detour_distance[i] = vector<double>(current_Number_Worker, 0);
        // current_detour_distance[i].push_back(vector<double>(current_Number_Worker, 0));
        current_poi[i] = vector<int>(current_Number_Worker, 0);
    }

    Compute_PTPW_Group_workerBatch(current_PT, current_PW, current_detour_distance, current_taskGroup, current_workerGroup, current_poi); // 诗婷增加, 已完全修改
    for (int i = 0; i < current_taskGroup.size(); i++)
    {
        if (current_PT[i].size() != 0) // 偏好列表不为0才是可用任务
        {
            current_ActiveTask.push_back(i);
        }
        current_CW_Task[i] = -1;
    }
    if (current_ActiveTask.empty())
    {
        return;
    }
    // // 存储detour_distance
    // double AD[current_Number_Worker]; // hy  AT记录worker的剩余可用时间，AD记录worker的剩余可用偏移距离。
    // Initialize(AD, Sumdis);   / / hy 初始化剩余绕路距离当前时间-剩余时间，已修改到分组划分的位置

    computeMaxDitanceTask(current_MaxDistanceTask, current_taskGroup, current_window_endTime); // 初始化各任务在满足deadline限制的条件下，每个任务可走的最长距离。

    /**
     * 迭代匹配
     * 工人批量时，工人不可用或者任务不可用停止循环匹配
     * 未匹配或匹配失败的批次工人优先匹配
     * 偏好列表ORDER
     */
    cout << "current_taskGroup.size(): " << current_ActiveTask.size() << current_ActiveTask.empty() << endl;
    while (!current_ActiveTask.empty())
    {
        //  cout << "第" << ++matchingTimes << "轮匹配开始！！！！！" << endl;

        current_NextActiveTask.clear();

        for (int i = 0; i < current_ActiveTask.size(); i++) // 为ActiveTask重新赋值为NextACTIVETASK
            current_NextActiveTask.push_back(current_ActiveTask[i]);

        int matchingnumber = 0, replacematching = 0;
        for (int i = 0; i < current_ActiveTask.size(); i++)
        {
            int taskid = current_ActiveTask[i];
            int order = current_num_of_chased_worker[taskid];
            int worker_to_chase = current_PT[taskid][order].first;
            //   cout << taskid << "目前已经追求的工人数量:" << order << endl;
            //  cout << "任务的偏好工人数：" << PT[taskid].size() << endl;
            current_num_of_chased_worker[taskid] = order + 1; // 无论匹配成功与否，该任务追过的工人数量加1。

            if (IFTaskExist(worker_to_chase, taskid, current_PW) != current_PW[worker_to_chase].end()) // 任务存在PW
            {

                if (CurrentTask_Satisfy_TSDA(current_taskGroup, worker_to_chase, taskid, current_Group_workerAD, current_detour_distance, current_Group_worker_subTrajectoryDis, current_CT_Worker, current_poi, current_MaxDistanceTask, &current_task_NeedTime, current_window_endTime))
                // if (AcceptTask(taskid, worker_to_chase, PW)) //修改+Deadline
                {
                    // 任务被工人直接接受
                    current_CW_Task[taskid] = worker_to_chase;
                    current_CT_Worker[worker_to_chase].push_back(taskid);
                    // current_workerGroup[worker_to_chase].sign = false; // 当前分组内工人已被使用，不能成为下一分组的元素
                    // current_taskGroup[taskid].sign = false;
                    // global_CT_Worker[current_workerGroup[worker_to_chase].Original_Local].push_back(taskid);

                    //   cout << "工人：" << worker_to_chase << "目前已匹配的任务数：" << CT_Worker[worker_to_chase].size() << endl;
                    current_NextActiveTask.erase(find(current_NextActiveTask.begin(), current_NextActiveTask.end(), taskid)); // 更新Activetask

                    Update_AD1(worker_to_chase, taskid, current_Group_workerAD, current_detour_distance); // 更新可用偏移距离
                                                                                                          //       cout << worker_to_chase << "工人的可用时间：" << AT[worker_to_chase] << endl;
                                                                                                          //      cout << "工人-任务匹配成功！！！" << endl;
                                                                                                          //      cout << " 配对为:"
                                                                                                          //           << "(" << taskid << "," << worker_to_chase << ")" << endl;
                    //*********************************
                    // 更新任务的最大可行驶距离,以及其它任务的
                    UpdateTaskDeadline_TSDA(false, 0, worker_to_chase, taskid, current_detour_distance, current_CT_Worker, current_poi, current_MaxDistanceTask, current_task_NeedTime);
                    matchingnumber++;
                    //  int taskindex = GetIndex_PW(worker_to_chase, taskid, PW);
                }
                else
                {
                    //     for (int i = 0; i < After_Task.size(); i++)
                    //      cout << "aftertask:" << After_Task[i].first << endl;
                    //      for (int i = 0; i < RPTask.size(); i++)
                    //      cout << "replacetask:" << RPTask[i] << endl;
                    //      cout << "endddd!" << endl;
                    //*************修改寻找替换任务
                    //  int MinReplaceTask =  Find_ReplaceTask(worker_to_chase, taskid, PW); //找到报酬最小的任务
                    int MinReplaceTask = FindReplaceTaskNew_TSDA(worker_to_chase, taskid, current_PW, current_Group_workerAD, current_detour_distance, current_Group_worker_subTrajectoryDis, current_CT_Worker, current_poi, current_MaxDistanceTask, &current_task_NeedTime, current_taskGroup, current_window_endTime);
                    if (MinReplaceTask != -1) // 存在可替换的任务
                    {
                        current_CW_Task[taskid] = worker_to_chase;
                        current_CW_Task[MinReplaceTask] = -1; // 被替换出去的工人的任务变为-1
                        current_CT_Worker[worker_to_chase].push_back(taskid);
                        // RemoveReplaceTask(worker_to_chase, taskid, MinReplaceTask);  //从工人已匹配中移除MinReplaceTask
                        current_CT_Worker[worker_to_chase].erase(find(current_CT_Worker[worker_to_chase].begin(), current_CT_Worker[worker_to_chase].end(), MinReplaceTask));
                        // AddTask_into_AT(worker_to_chase, taskid, MinReplaceTask);   //将 MinReplaceTask加入AT
                        if (current_num_of_chased_worker[MinReplaceTask] < current_PT[MinReplaceTask].size()) //****4.29：21：13 Active任务需要还有可求婚的对象
                            current_NextActiveTask.push_back(MinReplaceTask);
                        current_NextActiveTask.erase(find(current_NextActiveTask.begin(), current_NextActiveTask.end(), taskid)); // 将taskid任务移除AT；

                        Update_AD2(worker_to_chase, taskid, MinReplaceTask, current_Group_workerAD, current_detour_distance); // 更新可用偏移距离
                                                                                                                              //*********************************
                        // 更新已插入任务的最大可行驶距离,replace任务的最大可行驶距离,以及其它以匹配任务的最大可行驶距离
                        UpdateTaskDeadline_TSDA(true, MinReplaceTask, worker_to_chase, taskid, current_detour_distance, current_CT_Worker, current_poi, current_MaxDistanceTask, current_task_NeedTime);
                        //******替换任务的Deadline更新为初始的值
                        current_MaxDistanceTask[MinReplaceTask] = current_taskGroup[MinReplaceTask].task.Deadline * speed;
                        //   cout <<"任务"<<MinReplaceTask<<"替换成功!" <<taskid<< endl;
                        replacematching++;
                        //  int taskindex = GetIndex_PW(worker_to_chase, taskid, PW);
                        //   int MinRTindex = GetIndex_PW(worker_to_chase, MinReplaceTask, PW);
                        //   int workerindex = GetIndex_PT(worker_to_chase, MinReplaceTask, PT);
                    }
                    else
                    {
                        //      cout << "无可替换任务！！！" << endl; //该任务追过的工人数量加1
                        //      cout << "llll" << endl;
                    }
                }
            }
            if (current_num_of_chased_worker[taskid] == current_PT[taskid].size()) // 匹配到偏好列表的最后一个，则从ActivetASK中移除！
            {

                vector<int>::iterator iter1 = find(current_NextActiveTask.begin(), current_NextActiveTask.end(), taskid);
                if (iter1 != current_NextActiveTask.end())
                {
                    current_NextActiveTask.erase(find(current_NextActiveTask.begin(), current_NextActiveTask.end(), taskid));
                }
                vector<int>::iterator iter2 = find(current_NextActiveTask.begin(), current_NextActiveTask.end(), taskid);
                if (iter2 == current_NextActiveTask.end())
                {
                    //    cout << "任务偏好列表已达最后一个，任务已移除！" << endl;
                    //    cout << endl;
                }
            }
        }

        //   cout << "第" << matchingTimes << "轮匹配结束！！！！！" << endl;
        // cout << "本轮匹配到配对数量：" << matchingnumber << "\t"
        //   << "本轮替换的次数:" << replacematching << endl;

        current_ActiveTask.clear();
        for (int i = 0; i < current_NextActiveTask.size(); i++) // 为ActiveTask重新赋值为NextACTIVETASK
            current_ActiveTask.push_back(current_NextActiveTask[i]);
        //   cout << "下一轮匹配的任务数：" << NextActiveTask.size() << endl;
        //   cout << "end" << endl;
    }

    for (int i = 0; i < current_CT_Worker.size(); i++)
    {
        for (auto m : current_CT_Worker[i])
        {
            current_taskGroup[m].sign = false;
            global_CT_Worker[current_workerGroup[i].Original_Local].push_back(current_taskGroup[m].Original_Local);
        }
        current_workerGroup[i].sign = false;
    }

    ShowCTMatching(current_CT_Worker, current_Number_Worker); // 输出配对
}

vector<pair<int, double>>::iterator Basic_information::IFTaskExist(int workerid, int taskid, vector<vector<pair<int, double>>> &PW)
{
    // 判断任务在PW的位置
    vector<pair<int, double>>::iterator iter = PW[workerid].begin();
    for (iter; iter < PW[workerid].end(); iter++)
        if ((*iter).first == taskid)
            break;
    return iter;
}

void Basic_information::Update_AD1(int workerid, int taskid, vector<double> &AD, vector<double> current_detour_distance[])
{ // 加入任务后，更新工人的可用偏移距离
    double dd = 2 * current_detour_distance[taskid][workerid];
    AD[workerid] = AD[workerid] - dd;
}

void Basic_information::Update_AD2(int workerid, int taskid, int MinReplaceTask, vector<double> &AD, vector<double> detour_distance[])
{
    // 更新替换后的可用绕路距离
    double dd = 2 * detour_distance[taskid][workerid];         // 加入的任务
    double DD = 2 * detour_distance[MinReplaceTask][workerid]; // 移除的任务
    AD[workerid] = AD[workerid] - dd + DD;
}

void Basic_information::UpdateTaskDeadline_TSDA(bool replace, int replaceid, int workerid, int taskid, vector<double> current_detour_distance[], vector<vector<int>> &CT_Worker, vector<int> poi[], double MaxDistanceTask[], double current_task_NeedTime)
{
    int replacepoi = 0;
    if (replace)
    {
        replacepoi = poi[replaceid][workerid];
    }

    for (int i = 0; i < CT_Worker[workerid].size(); i++)
    {
        int assigntask_id = CT_Worker[workerid][i];
        int assignedpoint = poi[assigntask_id][workerid]; // 已匹配任务的偏移点
        int detourpoint = poi[taskid][workerid];
        //    if(trajectory_id == detourpoint ) //偏移点为同一个暂时不管
        if (assigntask_id == taskid) // 刚插入的任务
        {
            // cout<<"原最大可走："<< MaxDistanceTask[taskid]<<endl;
            MaxDistanceTask[taskid] = MaxDistanceTask[taskid] - current_task_NeedTime * speed;
            // cout<<"本次需要走："<<current_task_NeedTime*speed<<endl;
            // cout<<"当前任务最大可走距离更新为："<< MaxDistanceTask[taskid]<<endl;
        }
        else
        {
            if (detourpoint < assignedpoint) // 只对插入偏移点之后的任务有影响, 判断是否满足后面任务的Deadline 更新任务的MaxDistanceTask
            {
                // 更新任务的最大可走路程
                MaxDistanceTask[assigntask_id] = MaxDistanceTask[assigntask_id] - (2 * current_detour_distance[taskid][workerid]); // 减去插入的任务绕路路程
                                                                                                                                   // cout<<"影响的任务"<<assigntask_id<<"的最大可走距离更新为："<< MaxDistanceTask[assigntask_id]<<endl;
            }
            if (replace == true && replacepoi < assignedpoint) // 只对替换任务之后的Deadline有影响
            {
                // 更新任务的最大可走路程
                MaxDistanceTask[assigntask_id] = MaxDistanceTask[assigntask_id] + (2 * current_detour_distance[replaceid][workerid]);
            }
        }
    }
}

int Basic_information::FindReplaceTaskNew_TSDA(int workerid, int taskid, vector<vector<pair<int, double>>> &PW, vector<double> &AD, vector<double> detour_distance[], vector<vector<double>> &Worker_subTrajectoryDis, vector<vector<int>> &CT_Worker, vector<int> poi[], double MaxDistanceTask[], double *current_task_NeedTime, vector<CURRENT_TASK_GROUP> &current_taskGroup, double nowtime)
{
    int reTaskindex = -1;
    int replacetaskid = -1;         // 最不偏好的工人
    double minProfit = __DBL_MAX__; // 极大值
    int t1 = GetIndex_PW(workerid, taskid, PW);

    // 插入任务前已走的距离
    double SumdetouDis = 0.0;
    int detourpoint = poi[taskid][workerid]; // 要插入任务的偏移点
    for (vector<int>::iterator it = CT_Worker[workerid].begin(); it != CT_Worker[workerid].end(); it++)
    {
        int assignpoi = poi[*it][workerid]; // 以匹配任务的偏移点
        if (assignpoi <= detourpoint)       // 计算插入点之前的
        {
            SumdetouDis = SumdetouDis + 2 * detour_distance[*it][workerid];
        }
    }

    for (vector<int>::iterator it = CT_Worker[workerid].begin(); it != CT_Worker[workerid].end(); it++)
    {
        int t2 = GetIndex_PW(workerid, *it, PW);
        if (t1 < t2) //*it 排在 taskid的后面
        {
            if (IfReplace(workerid, taskid, *it, AD, detour_distance, Worker_subTrajectoryDis, CT_Worker, poi, MaxDistanceTask, current_task_NeedTime, SumdetouDis, current_taskGroup, nowtime))
            // if( AD[workerid] + 2 * detour_distance[workerid][*it] > 2 * detour_distance[workerid][taskid] )
            {
                if (t2 > reTaskindex)
                {
                    reTaskindex = t2;
                    replacetaskid = *it;
                }
            }
        }
    }
    return replacetaskid;
}

// 4.5 WPPG处理方法

void Basic_information::Grouping_Framework_WPPG(vector<TASK> &tasks, vector<WORKER> &workers, double Wmax, int Tmax, vector<double> &Sumdis, vector<vector<double>> &global_Worker_subTrajectoryDis)

{
    vector<CURRENT_TASK_GROUP> current_taskGroup;              // 当前时间窗口内的任务列表，已获得
    vector<CURRENT_WORKERS_GROUP> current_Group_worker;        // 当前窗口内工人，已获得
    vector<double> current_Group_workerSumdis;                 // 当前窗口内工人轨迹的总距离。已获得
    vector<double> current_Group_workerAD;                     // 当前窗口内工人剩余可用距离，已获得
    double current_window_endTime = tasks[0].startTime + Wmax; // 当前时间窗口的结束时间，已获得
    double current_window_startTime = tasks[0].startTime;      // 当前时间窗口的起始时间，已获得
    int current_workID = global_Current_workID;                // 判断当前窗口的工人从几号开始的，已获得

    vector<vector<double>> current_Group_worker_subTrajectoryDis; // 当前窗口内工人轨迹点之和，已获得
    // vector<vector<double>> current_detour_distance;               // 当前窗口内最大绕路距离

    int sign = 0;

    for (int i = 0; i < tasks.size(); i++)
    {

        TASK task = tasks[i];
        sign++;

        CURRENT_TASK_GROUP taskg;
        taskg.task = task;
        taskg.Original_Local = i;
        current_taskGroup.push_back(taskg);

        // 如果当前分组数量已达到Tmax，则创建新的分组
        // 如果任务的开始时间大于等于当前窗口的截止时间，则创建新的分组
        if ((current_taskGroup.size() == Tmax) || (task.startTime >= current_window_endTime))
        {

            current_workID = global_Current_workID;

            current_window_endTime = task.startTime; // 下一窗口的开始时间，也就是上一窗口的结束时间

            cout << sign << " \t当前窗口截止时间：" << current_window_endTime << endl;
            // 计算当前窗口满足时间约束的work。
            groupWork_according_TaskGroup(workers, Sumdis, current_Group_worker, current_Group_workerSumdis, current_window_endTime, current_workID, current_Group_workerAD, global_Worker_subTrajectoryDis, current_Group_worker_subTrajectoryDis); // 计算当前窗口满足时间约束的work。
            determine_Window_Task_Timeout(current_taskGroup, current_window_endTime);
            cout << "2\t开始分组内进行匹配" << endl;

            // 任务工人配对，并计算当工人和任务未匹配时,能进入下一窗口的工人和任务，

            // match_WorkerTask(current_taskGroup, current_Group_worker, current_window_endTime, current_Group_workerAD, current_Group_worker_subTrajectoryDis);        // 当前任务组,当前工人组,当前窗口截止时间，当前工人剩余绕行距离
            match_WorkerTask_WPPG(current_taskGroup, current_Group_worker, current_window_endTime, current_Group_workerAD, current_Group_worker_subTrajectoryDis); // 当前任务组,当前工人组,当前窗口截止时间，当前工人剩余绕行距离

            // 为下一窗口做准备，即最大的截止时间
            current_window_startTime = current_window_endTime;
            current_window_endTime += Wmax;

            // updata_nextWindow_Worker_Task(current_taskGroup, current_Group_worker, current_window_endTime); // 将剩余工人加入当前组。并更新他们的开始时间为当前截止时间
        }
        updata_current_Info(current_taskGroup, current_Group_worker, current_Group_workerAD, current_Group_worker_subTrajectoryDis, current_Group_workerSumdis);

        // 将剩余任务加入当前分组，并更新他们的开始时间为当前时间。
    }
    // 将最后一个分组加入分组列表
    if (current_taskGroup.size() > 0)
    {

        current_workID = global_Current_workID;
        current_window_startTime = current_taskGroup.back().task.startTime;
        cout << sign << "   最后一组，下一时间窗口为：" << current_window_startTime << endl;
        // 计算分组内的工人以及他们剩余绕行距离
        groupWork_according_TaskGroup(workers, Sumdis, current_Group_worker, current_Group_workerSumdis, current_window_endTime, current_workID, current_Group_workerAD, global_Worker_subTrajectoryDis, current_Group_worker_subTrajectoryDis); // 计算当前窗口满足时间约束的work。

        determine_Window_Task_Timeout(current_taskGroup, current_window_endTime);
        // 对分组内任务和工人进行匹配。
        match_WorkerTask_WPPG(current_taskGroup, current_Group_worker, current_window_endTime, current_Group_workerAD, current_Group_worker_subTrajectoryDis); // 当前任务组,当前工人组,当前窗口截止时间，当前工人剩余绕行距离
                                                                                                                                                               // updata_nextWindow_Worker_Task(current_taskGroup, current_Group_worker, current_window_startTime);
                                                                                                                                                               // current_taskGroup.clear();
    }
    // print_groupTasks_addEndTime();
}

void Basic_information::match_WorkerTask_WPPG(vector<CURRENT_TASK_GROUP> &current_taskGroup, vector<CURRENT_WORKERS_GROUP> &current_workerGroup, double current_window_endTime, vector<double> &current_Group_workerAD, vector<vector<double>> &current_Group_worker_subTrajectoryDis)
{
    int current_Number_Worker = current_workerGroup.size();
    int current_Number_Task = current_taskGroup.size();
    // double current_detour_distance[current_Number_Task][current_Number_Worker];
    // int poi[current_Number_Task][current_Number_Worker]; // 存储poiid，记录哪个点是最小的
    vector<double> current_detour_distance[current_Number_Task]; // 存储当前分组内，任务到每个工人的最近绕路距离，并初始化，防止数组越界。
    vector<int> current_poi[current_Number_Task];                // 存储poiid，记录哪个点到任务距离是最小的
    vector<vector<pair<int, double>>> current_PT(current_Number_Task);
    vector<vector<pair<int, double>>> current_PW(current_Number_Worker);

    for (int i = 0; i < current_Number_Task; i++)
    {
        current_detour_distance[i] = vector<double>(current_Number_Worker, 0);
        // current_detour_distance[i].push_back(vector<double>(current_Number_Worker, 0));
        current_poi[i] = vector<int>(current_Number_Worker, 0);
    }

    cout << "---------------任务开始---------" << endl;

    //  ----------------- 下面是诗婷的代码----------------;

    // ShowDetour_Distance(detour_distance,poi);
    // 对于每个任务，计算任务的可用工人：首先，满足range约束，满足最小声誉分数约束；其次，在匹配时再判断是否满足deadline约束，容量约束
    // 然后计算与每个可用工人的距离，选择最近的工人
    vector<int> current_CT_Worker[current_Number_Worker]; // 记录当前worker已分配的任务，已修改
    double current_MaxDistanceTask[current_Number_Task];  // 任务允许的最大绕路距离，根据任务的开始和截止时间计算初始化，后面会根据work和task位置关系进一步更新，用于剪枝
    double current_task_NeedTime = 0.0;                   // 记录到任务位置所需时间并返回
    // // 存储detour_distance
    // double AD[current_Number_Worker]; // hy  AT记录worker的剩余可用时间，AD记录worker的剩余可用偏移距离。
    // Initialize(AD, Sumdis);   / / hy 初始化剩余绕路距离当前时间-剩余时间，已修改到分组划分的位置

    computeMaxDitanceTask(current_MaxDistanceTask, current_taskGroup, current_window_endTime); // 初始化各任务在满足deadline限制的条件下，每个任务可走的最长距离。

    // 实现基于任务距离的贪心算法
    vector<int> current_Group_worker_AT;          // 定义可用任务的集合为vector变量
    for (int i = 0; i < current_Number_Task; i++) // 初始化可用任务集合
    {
        current_Group_worker_AT.push_back(i);
    }

    for (int i = 0; i < current_Number_Worker; i++) // 匹配每个工人
    {
        int workerid = i; // 对于每个工人worker

        if (current_Group_worker_AT.size() != 0) // 当还有可用任务时，可对工人继续进行匹配
        {

            ComputePWforAT_WPPG(workerid, current_PW, current_Group_worker_AT, current_detour_distance, current_poi, current_workerGroup, current_taskGroup); // 为工人生成AT的偏好列表
            sort(current_PW[workerid].begin(), current_PW[workerid].end(), cmp);
            if (current_PW[workerid].size() > 0) // worker有偏好列表时才进行匹配
            {
                int assigned_task = 0;                                // 计算工人已匹配的任务数量
                for (int j = 0; j < current_PW[workerid].size(); j++) // 遍历完PW里面的任务，尽可能多的匹配最前面的任务
                {
                    int ct_task_id = current_PW[workerid][j].first;
                    if (current_taskGroup[ct_task_id].sign) // 当前任务没有被使用
                        if (assigned_task < Capacity)       // 工人已匹配任务数不超过容量时继续匹配
                        {
                            if (find(current_Group_worker_AT.begin(), current_Group_worker_AT.end(), current_PW[workerid][j].first) != current_Group_worker_AT.end()) // 任务为可用任务
                            {
                                if (CurrentTask_Satisfy(current_taskGroup, workerid, current_PW[workerid][j].first, current_Group_workerAD, current_detour_distance, current_Group_worker_subTrajectoryDis, current_CT_Worker, current_poi, current_MaxDistanceTask, &current_task_NeedTime, current_window_endTime))
                                //  if (CurrentTask_Available(i, PW[i][j].first, AT, AD, detour_distance)) //当前任务PW[i][j]可以加入，属于AT，满足AD
                                {
                                    current_CT_Worker[workerid].push_back(current_PW[workerid][j].first);

                                    // hy黄阳↓↓↓↓↓↓↓↓

                                    current_workerGroup[workerid].sign = false; // 当前分组内工人已被使用，不能成为下一分组的元素
                                    current_taskGroup[ct_task_id].sign = false;
                                    cout << "\t\t\t对于工人:" << current_workerGroup[workerid].Original_Local << ",当前任务可使用:" << current_taskGroup[ct_task_id].Original_Local << endl; // 当前分组内任务已被使用
                                    global_CT_Worker[current_workerGroup[workerid].Original_Local].push_back(current_taskGroup[ct_task_id].Original_Local);                                  // 标记全局中已经匹配的结果

                                    // hy黄阳↑↑↑↑↑↑↑
                                    // Matching.push_back(make_pair(i, PW[i][j].first));     //工人，任务            //加入匹配M;
                                    // cout<<"index:"<<j<<","<<"("<<i<<","<<PW[i][j].first<<")"<<"匹配成功！"<<"匹配数:"<<Matching.size()<<endl;
                                    current_Group_workerAD[workerid] = current_Group_workerAD[workerid] - 2 * current_detour_distance[current_PW[workerid][j].first][workerid]; // 更新可用绕路距离
                                    UpdateTaskDeadline(workerid, current_PW[workerid][j].first, current_detour_distance, current_CT_Worker, current_poi, current_MaxDistanceTask, current_task_NeedTime);

                                    assigned_task++;                                                                                            // 更新已匹配数量
                                    for (vector<int>::iterator it = current_Group_worker_AT.begin(); it != current_Group_worker_AT.end(); ++it) // 移除任务
                                    {
                                        if (*it == current_PW[workerid][j].first)
                                        {
                                            cout << "\t\t\t\t任务:" << current_taskGroup[ct_task_id].Original_Local << "可从剩余任务中删除:" << *it << endl;
                                            vector<int>::iterator itt = current_Group_worker_AT.erase(it);
                                            //     cout<<PW[i][j].first<<"has erased!"<<endl;  //为啥移除不成功呀？？？ 4.20 20：46
                                            break;
                                        }
                                    }
                                    // assigned_task++;
                                }
                            }
                        }
                        else // 工人容量已达上限
                            break;
                }
            }
        }
        else
            break; // 无可用任务则跳出循环，工人匹配结束。
    }

    ShowCTMatching(current_CT_Worker, current_Number_Worker); // 输出配对
}

void Basic_information::ComputePWforAT_WPPG(int workerid, vector<vector<pair<int, double>>> &PW, vector<int> &AT, vector<double> current_detour_distance[], vector<int> current_poi[], vector<CURRENT_WORKERS_GROUP> &current_workerGroup, vector<CURRENT_TASK_GROUP> &current_taskGroup)
{
    // 计算AT中的taskid任务的可用任务并加入偏序列表
#pragma omp parallel for num_threads(2) // 并行处理，并行中断
    for (int i = 0; i < AT.size(); i++) // 修改循环遍历方式
    {
        int j = AT[i];
        if (current_taskGroup[j].task.Minscore <= current_workerGroup[workerid].worker.score)
        {                                                                                                                              // 工人的分数满足任务的最小约束                                                                                    //为何错误？？？？
                                                                                                                                       // current_detour_distance[j][workerid] = Caculate_mindist(workerid, j, poi); // 计算每个任务和每个worker之间的最小绕路距离
            current_detour_distance[j][workerid] = Caculate_mindist(workerid, j, current_poi, current_taskGroup, current_workerGroup); // 计算最小绕路距离
                                                                                                                                       //
            if (current_detour_distance[j][workerid] <= current_workerGroup[workerid].worker.range)
            {                                                                                                          // 当任务的绕路距离小于工人的最大绕路距离，计算偏好                                                                                    //  double preference= worker[j].trajectory[poi].poitime+detour_distance/speed;      //偏好值单任务的等待时间
                double preference = current_taskGroup[j].task.Reward - (2 * current_detour_distance[j][workerid] * c); // 偏好值为score
                if (preference > 0)
                {
#pragma omp critical // 控制单线程访问
                    {
                        PW[workerid].push_back(make_pair(j, preference)); // I任务的j worker对应的preference
                    }
                }
            }
        }
    }
}

// 4.6 WSDA处理方法
void Basic_information::Grouping_Framework_WSDA(vector<TASK> &tasks, vector<WORKER> &workers, double Wmax, int Tmax, vector<double> &Sumdis, vector<vector<double>> &global_Worker_subTrajectoryDis)

{
    vector<CURRENT_TASK_GROUP> current_taskGroup;              // 当前时间窗口内的任务列表，已获得
    vector<CURRENT_WORKERS_GROUP> current_Group_worker;        // 当前窗口内工人，已获得
    vector<double> current_Group_workerSumdis;                 // 当前窗口内工人轨迹的总距离。已获得
    vector<double> current_Group_workerAD;                     // 当前窗口内工人剩余可用距离，已获得
    double current_window_endTime = tasks[0].startTime + Wmax; // 当前时间窗口的结束时间，已获得
    double current_window_startTime = tasks[0].startTime;      // 当前时间窗口的起始时间，已获得
    int current_workID = global_Current_workID;                // 判断当前窗口的工人从几号开始的，已获得

    vector<vector<double>> current_Group_worker_subTrajectoryDis; // 当前窗口内工人轨迹点之和，已获得
    // vector<vector<double>> current_detour_distance;               // 当前窗口内最大绕路距离

    int sign = 0;
    cout << "\n------------------分组开始--------------" << endl;
    for (int i = 0; i < tasks.size(); i++)
    {

        TASK task = tasks[i];
        sign++;

        CURRENT_TASK_GROUP taskg;
        taskg.task = task;
        taskg.Original_Local = i;

        // 如果当前分组数量已达到Tmax，则创建新的分组
        // 如果任务的开始时间大于等于当前窗口的截止时间，则创建新的分组
        if ((current_taskGroup.size() == Tmax) || (task.startTime >= current_window_endTime))
        {
            current_workID = global_Current_workID;
            current_window_endTime = task.startTime; // 下一窗口的开始时间，也就是上一窗口的结束时间
            cout << sign << " 1\t当前窗口截止时间：" << current_window_endTime << endl;
            // 计算当前窗口满足时间约束的work。
            groupWork_according_TaskGroup(workers, Sumdis, current_Group_worker, current_Group_workerSumdis, current_window_endTime, current_workID, current_Group_workerAD, global_Worker_subTrajectoryDis, current_Group_worker_subTrajectoryDis); // 计算当前窗口满足时间约束的work。
            determine_Window_Task_Timeout(current_taskGroup, current_window_endTime);

            // 任务工人配对，并计算当工人和任务未匹配时,能进入下一窗口的工人和任务，

            // match_WorkerTask(current_taskGroup, current_Group_worker, current_window_endTime, current_Group_workerAD, current_Group_worker_subTrajectoryDis);        // 当前任务组,当前工人组,当前窗口截止时间，当前工人剩余绕行距离
            match_WorkerTask_WSDA(current_taskGroup, current_Group_worker, current_window_endTime, current_Group_workerAD, current_Group_worker_subTrajectoryDis); // 当前任务组,当前工人组,当前窗口截止时间，当前工人剩余绕行距离

            // 为下一窗口做准备，即最大的截止时间
            current_window_startTime = current_window_endTime;
            current_window_endTime += Wmax;

            // updata_nextWindow_Worker_Task(current_taskGroup, current_Group_worker, current_window_endTime); // 将剩余工人加入当前组。并更新他们的开始时间为当前截止时间
            cout << "\n-----------------------------分组结束\n"
                 << endl;
            updata_current_Info(current_taskGroup, current_Group_worker, current_Group_workerAD, current_Group_worker_subTrajectoryDis, current_Group_workerSumdis);
        }

        // 将剩余任务加入当前分组，并更新他们的开始时间为当前时间。
        current_taskGroup.push_back(taskg);
    }
    // 将最后一个分组加入分组列表
    if (current_taskGroup.size() > 0)
    {

        current_workID = global_Current_workID;
        current_window_startTime = current_taskGroup.back().task.startTime;
        cout << sign << "   最后一组，下一时间窗口为：" << current_window_startTime << endl;
        // 计算分组内的工人以及他们剩余绕行距离
        groupWork_according_TaskGroup(workers, Sumdis, current_Group_worker, current_Group_workerSumdis, current_window_endTime, current_workID, current_Group_workerAD, global_Worker_subTrajectoryDis, current_Group_worker_subTrajectoryDis); // 计算当前窗口满足时间约束的work。

        determine_Window_Task_Timeout(current_taskGroup, current_window_endTime);
        // 对分组内任务和工人进行匹配。
        match_WorkerTask_WSDA(current_taskGroup, current_Group_worker, current_window_endTime, current_Group_workerAD, current_Group_worker_subTrajectoryDis); // 当前任务组,当前工人组,当前窗口截止时间，当前工人剩余绕行距离
                                                                                                                                                               // updata_nextWindow_Worker_Task(current_taskGroup, current_Group_worker, current_window_startTime);
                                                                                                                                                               // current_taskGroup.clear();
    }
    //   print_groupTasks_addEndTime();
}
void Update_AD2_WSDA(int oldworkerid, int taskid, vector<double> &AD, vector<double> detour_distance[])
{
    // 更新替换后的可用绕路距离
    // 更新替换后的可用绕路距离
    double dd = 2 * detour_distance[taskid][oldworkerid]; // 移除的任务
    AD[oldworkerid] = AD[oldworkerid] + dd;
}

vector<pair<int, double>>::iterator IFWorkerExist(int workerid, int taskid, vector<vector<pair<int, double>>> &PT)
{

    vector<pair<int, double>>::iterator iter = PT[taskid].begin();
    for (iter; iter < PT[taskid].end(); iter++)
        if ((*iter).first == workerid)
            break;
    return iter;
}

void Basic_information::match_WorkerTask_WSDA(vector<CURRENT_TASK_GROUP> &current_taskGroup, vector<CURRENT_WORKERS_GROUP> &current_workerGroup, double current_window_endTime, vector<double> &current_Group_workerAD, vector<vector<double>> &current_Group_worker_subTrajectoryDis)
{
    int current_Number_Worker = current_workerGroup.size();
    int current_Number_Task = current_taskGroup.size();

    // double current_detour_distance[current_Number_Task][current_Number_Worker];
    // int poi[current_Number_Task][current_Number_Worker]; // 存储poiid，记录哪个点是最小的
    vector<double> current_detour_distance[current_Number_Task]; // 存储当前分组内，任务到每个工人的最近绕路距离，并初始化，防止数组越界。
    vector<int> current_poi[current_Number_Task];                // 存储poiid，记录哪个点到任务距离是最小的
    vector<vector<pair<int, double>>> current_PT(Number_Task);
    vector<vector<pair<int, double>>> current_PW(Number_Worker);

    int current_CW_Task[current_Number_Task];                     // 记录number_task个task的当前对象
    int current_num_of_chased_tasks[current_Number_Worker] = {0}; // 工人呢追求过的任务的数量

    vector<int> current_ActiveWorker; // 记录当前活动工人集

    vector<int> current_NextActiveWorker(current_ActiveWorker);
    // vector<int> current_CT_Worker[current_Number_Worker];                            // 记录当前worker已分配的任务，已修改
    vector<vector<int>> current_CT_Worker(current_Number_Worker);
    double current_MaxDistanceTask[current_Number_Task] = {0.0}; // 任务允许的最大绕路距离，根据任务的开始和截止时间计算初始化，后面会根据work和task位置关系进一步更新，用于剪枝
    double current_task_NeedTime = 0.0;                          // 记录到任务位置所需时间并返回

    for (int i = 0; i < current_Number_Task; i++)
    {
        current_detour_distance[i] = vector<double>(current_Number_Worker, 0);
        // current_detour_distance[i].push_back(vector<double>(current_Number_Worker, 0));
        current_poi[i] = vector<int>(current_Number_Worker, 0);
        current_CW_Task[i] = -1;
    }
    // 完成
    Compute_PTPW_Group_workerBatch(current_PT, current_PW, current_detour_distance, current_taskGroup, current_workerGroup, current_poi); // 诗婷增加, 已完全修改

    // 初始化ActiveTask,AD
    for (int i = 0; i < current_workerGroup.size(); i++)
    {
        if (current_PW[i].size() != 0)         // 偏好列表不为0才是可用工人//hy完成
        {                                      //   cout<<i<<"任务YOU偏好列表"<<endl;
                                               //   cout<<PT[i].size()<<endl;
            current_ActiveWorker.push_back(i); // hy完成
        }
    }

    if (current_ActiveWorker.empty()) //
    {
        return;
    }

    computeMaxDitanceTask(current_MaxDistanceTask, current_taskGroup, current_window_endTime); // 初始化各任务在满足deadline限制的条件下，每个任务可走的最长距离。

    /**
     * 迭代匹配
     * 工人批量时，工人不可用或者任务不可用停止循环匹配
     * 未匹配或匹配失败的批次工人优先匹配
     * 偏好列表ORDER
     */
    cout << "current_taskGroup.size(): " << current_ActiveWorker.size() << current_ActiveWorker.empty() << endl;
    int matchingTimes = 0;
    while (!current_ActiveWorker.empty())
    {
        cout << "第" << ++matchingTimes << "轮匹配开始！！！！！" << endl;

        current_NextActiveWorker.clear();

        for (int i = 0; i < current_ActiveWorker.size(); i++) // 为ActiveWorker重新赋值为NextACTIVETASK
            current_NextActiveWorker.push_back(current_ActiveWorker[i]);

        int matchingnumber = 0, replacematching = 0;
        for (int i = 0; i < current_ActiveWorker.size(); i++)
        {
            int workerid = current_ActiveWorker[i];
            int order = current_num_of_chased_tasks[workerid];

            int task_to_chase = current_PW[workerid][order].first;
            // cout << workerid << "目前已经追求的任务数量:" << order << endl;
            // cout << "任务的偏好工人数：" << PT[taskid].size() << endl;
            current_num_of_chased_tasks[workerid] = order + 1; // 无论匹配成功与否，该工人追过的任务数量加1。

            // for (auto m : current_taskGroup)
            // {
            //   cout << m.Original_Local << "  " << current_Number_Task << endl;
            // }
            // for (auto m : current_workerGroup)
            // {
            //   cout << "当前工人：" << m.Original_Local << endl;
            // }
            cout << "测试到这里111" << endl;

            if (IFWorkerExist(workerid, task_to_chase, current_PT) != current_PT[task_to_chase].end()) // 工人是否在任务的偏好列表里
            {
                cout << "测试到这里22" << endl;
                // cout << "工人：" << workerid << "目前已匹配的任务数：" << current_CT_Worker[workerid].size() << endl;
                if (CurrentTask_Satisfy_TSDA(current_taskGroup, workerid, task_to_chase, current_Group_workerAD, current_detour_distance, current_Group_worker_subTrajectoryDis, current_CT_Worker, current_poi, current_MaxDistanceTask, &current_task_NeedTime, current_window_endTime))
                // if (AcceptTask(taskid, worker_to_chase, PW)) //修改+Deadline
                {
                    cout << "测试到这里33" << endl;
                    if (current_CW_Task[task_to_chase] == -1) // 任务还未匹配
                    {
                        cout << "测试到这里44" << endl;
                        // 任务被工人直接接受
                        current_CW_Task[task_to_chase] = workerid;
                        current_CT_Worker[workerid].push_back(task_to_chase);
                        // current_workerGroup[worker_to_chase].sign = false; // 当前分组内工人已被使用，不能成为下一分组的元素
                        // current_taskGroup[taskid].sign = false;
                        // global_CT_Worker[current_workerGroup[worker_to_chase].Original_Local].push_back(taskid);

                        cout << "工人：" << workerid << "目前已匹配的任务数：" << current_CT_Worker[workerid].size() << endl;
                        //---- current_NextActiveWorker.erase(find(current_NextActiveWorker.begin(), current_NextActiveWorker.end(), workerid)); // 更新Activetask

                        Update_AD1(workerid, task_to_chase, current_Group_workerAD, current_detour_distance); // 更新可用偏移距离
                                                                                                              //       cout << worker_to_chase << "工人的可用时间：" << AT[worker_to_chase] << endl;
                                                                                                              //      cout << "工人-任务匹配成功！！！" << endl;
                                                                                                              //      cout << " 配对为:"
                                                                                                              //           << "(" << taskid << "," << worker_to_chase << ")" << endl;
                        //*********************************
                        // 更新任务的最大可行驶距离,以及其它任务的
                        UpdateTaskDeadline_WSDA(false, 0, workerid, task_to_chase, current_detour_distance, current_CT_Worker, current_poi, current_MaxDistanceTask, current_task_NeedTime);
                        matchingnumber++;
                        //  int taskindex = GetIndex_PW(worker_to_chase, taskid, PW);//hy 在这里注销了
                    }
                    else // 任务已经暂时匹配给其它工人了
                    {
                        cout << "测试到这里55" << endl;
                        // 判断任务已匹配的工人和当前工人哪个在偏好列表里排在更前面
                        int oldworkerid = current_CW_Task[task_to_chase]; // 原工人id
                        if (GetIndex_PT(workerid, task_to_chase, current_PT) < GetIndex_PT(oldworkerid, task_to_chase, current_PT))
                        {
                            cout << "测试到这里66" << endl;

                            current_CW_Task[task_to_chase] = workerid;
                            current_CT_Worker[workerid].push_back(task_to_chase); // 当前工人加入新任务

                            Update_AD1(workerid, task_to_chase, current_Group_workerAD, current_detour_distance); // 更新工人可用偏移距离
                            cout << "size:" << current_CT_Worker[oldworkerid].size() << endl;
                            for (auto m : current_CT_Worker[oldworkerid])
                            {
                                cout << m << "  " << task_to_chase << endl;
                            }
                            // cout << task_to_chase << "任务已经加入！" << endl;
                            // cout << task_to_chase << "任务需要从旧工人中移除" << endl;
                            // cout << "旧工人的当前任务如下:" << endl;
                            // for (int j = 0; j < current_CT_Worker[oldworkerid].size(); j++)
                            //   cout << current_CT_Worker[oldworkerid][j] << endl;

                            current_CT_Worker[oldworkerid].erase(find(current_CT_Worker[oldworkerid].begin(), current_CT_Worker[oldworkerid].end(), task_to_chase)); // erase出错啦！！！ 2021:5.12
                            cout << "测试到这里77" << endl;
                            cout << "删除任务后旧工人的当前任务如下:" << endl;
                            for (int j = 0; j < current_CT_Worker[oldworkerid].size(); j++)
                                cout << current_CT_Worker[oldworkerid][j] << endl;

                            Update_AD2_WSDA(oldworkerid, task_to_chase, current_Group_workerAD, current_detour_distance); // //更新旧工人的可用绕路

                            // 更新可用时间和可用偏移距离
                            //         cout << "替换成功！" << endl;

                            // 更新已插入任务的最大可行驶距离,replace任务的最大可行驶距离,以及其它以匹配任务的最大可行驶距离
                            current_MaxDistanceTask[task_to_chase] = (current_taskGroup[task_to_chase].task.Deadline - current_window_endTime) * speed;
                            UpdateTaskDeadline_WSDA(true, oldworkerid, workerid, task_to_chase, current_detour_distance, current_CT_Worker, current_poi, current_MaxDistanceTask, current_task_NeedTime);

                            replacematching++;
                        }
                        else
                        {
                            //       cout << "无可替换任务！！！" << endl; //该任务追过的工人数量加1
                            //      cout << "llll" << endl;
                        }
                    }
                }
            }
            if (current_num_of_chased_tasks[workerid] >= current_PW[workerid].size()) // 匹配到偏好列表的最后一个，则从Activetworker中移除！
            {
                vector<int>::iterator iter1 = find(current_NextActiveWorker.begin(), current_NextActiveWorker.end(), workerid);
                if (iter1 != current_NextActiveWorker.end())
                {
                    current_NextActiveWorker.erase(find(current_NextActiveWorker.begin(), current_NextActiveWorker.end(), workerid));
                }

                /*     vector<int>::iterator iter2 = find(NextActiveWorker.begin(), NextActiveWorker.end(), task_to_chase);
                    if (iter2 == NextActiveWorker.end())
                    {
                     cout << "任务偏好列表已达最后一个，任务已移除！" << endl;
                     cout << endl;
                    }
                    */
            }
        }

        current_ActiveWorker.clear();
        for (int i = 0; i < current_NextActiveWorker.size(); i++) // 为ActiveWorker重新赋值为NextACTIVETASK
            current_ActiveWorker.push_back(current_NextActiveWorker[i]);
        //     cout << "下一轮匹配的工人数：" << NextActiveWorker.size() << endl;
        //      cout << "end" << endl;
    }

    //   cout << "第" << matchingTimes << "轮匹配结束！！！！！" << endl;
    // cout << "本轮匹配到配对数量：" << matchingnumber << "\t"
    //   << "本轮替换的次数:" << replacematching << endl;

    for (int i = 0; i < current_CT_Worker.size(); i++) // 这里多余 增加时长
    {
        for (auto m : current_CT_Worker[i])
        {
            current_taskGroup[m].sign = false;
            global_CT_Worker[current_workerGroup[i].Original_Local].push_back(current_taskGroup[m].Original_Local);
        }
        current_workerGroup[i].sign = false;
    }

    ShowCTMatching(current_CT_Worker, current_Number_Worker); // 输出配对
}
void Basic_information::UpdateTaskDeadline_WSDA(bool replace, int replaceWorkid, int workerid, int taskid, vector<double> current_detour_distance[], vector<vector<int>> &CT_Worker, vector<int> poi[], double MaxDistanceTask[], double current_task_NeedTime)
{
    // int replacepoi = 0;
    // if (replace)
    // {
    //   replacepoi = poi[taskid][replaceWorkid]; // 原始工人的任务位置
    // }

    for (int i = 0; i < CT_Worker[workerid].size(); i++) // 对于当前工人来说，只会影响后面的
    {
        int assigntask_id = CT_Worker[workerid][i];
        int assignedpoint = poi[assigntask_id][workerid]; // 已匹配任务的偏移点
        int detourpoint = poi[taskid][workerid];
        //    if(trajectory_id == detourpoint ) //偏移点为同一个暂时不管
        if (assigntask_id == taskid)
        {
            // cout<<"原最大可走："<< MaxDistanceTask[taskid]<<endl;
            MaxDistanceTask[taskid] = MaxDistanceTask[taskid] - current_task_NeedTime * speed;
            // cout<<"本次需要走："<<current_task_NeedTime*speed<<endl;
            // cout<<"当前任务最大可走距离更新为："<< MaxDistanceTask[taskid]<<endl;
        }
        else
        {
            if (detourpoint < assignedpoint) // 只对插入偏移点之后的任务有影响, 判断是否满足后面任务的Deadline 更新任务的MaxDistanceTask
            {
                // 更新任务的最大可走路程
                MaxDistanceTask[assigntask_id] = MaxDistanceTask[assigntask_id] - (2 * current_detour_distance[taskid][workerid]); // 减去插入的任务绕路路程
                                                                                                                                   // cout<<"影响的任务"<<assigntask_id<<"的最大可走距离更新为："<< MaxDistanceTask[assigntask_id]<<endl;
            }
        }
    }

    if (replace == true) // 如果是替换的
    {
        for (int i = 0; i < CT_Worker[replaceWorkid].size(); i++) // 也是只会影响后面的
        {
            int assigntask_id = CT_Worker[replaceWorkid][i];
            int assignedpoint = poi[assigntask_id][replaceWorkid]; // 已匹配任务的偏移点
            int detourpoint = poi[taskid][replaceWorkid];
            if (assigntask_id == taskid)
            {
                // cout<<"原最大可走："<< MaxDistanceTask[taskid]<<endl;
                MaxDistanceTask[taskid] = MaxDistanceTask[taskid] - current_task_NeedTime * speed;
                // cout<<"本次需要走："<<current_task_NeedTime*speed<<endl;
                // cout<<"当前任务最大可走距离更新为："<< MaxDistanceTask[taskid]<<endl;
            }
            else
            {
                if (detourpoint < assignedpoint) // 只对插入偏移点之后的任务有影响, 判断是否满足后面任务的Deadline 更新任务的MaxDistanceTask
                {
                    // 更新任务的最大可走路程
                    MaxDistanceTask[assigntask_id] = MaxDistanceTask[assigntask_id] + (2 * current_detour_distance[taskid][replaceWorkid]); // 减去插入的任务绕路路程
                                                                                                                                            // cout<<"影响的任务"<<assigntask_id<<"的最大可走距离更新为："<< MaxDistanceTask[assigntask_id]<<endl;
                }
            }
        }
    }
}

/***
 * 4.7
 * 根据混合排序得到已经排好序的（任务、工人）数据时间
 * 逐个初始化每个信息
 * 并按照时间进行匹配计算
 *  计算分为任务（或工人）两种形式
 *
 */

void Basic_information::whole_Greedy_Framework()
{
    vector<pair<WORKER, TASK>> hybird_datasets;                 // 合并排序后的 workers 和 tasks
    sort_hybrid(global_tasks, global_workers, hybird_datasets); // 混合排序

    vector<CURRENT_TASK_GROUP> current_taskGroup;      // 当前时间窗口内的任务列表，已获得
    vector<CURRENT_WORKERS_GROUP> current_workerGroup; // 当前窗口内工人，已获得
    vector<CURRENT_WORKER_STATE> current_workerState;  // 存储当前工人的状态
    vector<CURRENT_TASK_STATE> current_taskState;      // 存储当前任务的状态

    double current_time = 0;
    int arrive_global_workerID = -1; // 当前已到达全局工人的第几个
    int arrive_global_taskID = -1;   // 当前已到达全局任务的第几个
    CURRENT_TASK_GROUP taskg;
    CURRENT_WORKERS_GROUP workerg;
    CURRENT_WORKER_STATE workerStateg;
    CURRENT_TASK_STATE taskStateg;
    bool sign; //  false当前出现是工人，true是任务

    int nummm = 0;
    for (auto p : hybird_datasets)
    {

        if (p.first.startTime != 0) // 对于按时间到来的工人进行数据的初始化
        {
            arrive_global_workerID++;
            workerg.worker = p.first;
            workerg.Original_Local = arrive_global_workerID;
            current_workerGroup.push_back(workerg);
            current_time = p.first.startTime;

            workerStateg.current_alltaskCost = 0;
            workerStateg.current_worker_subTrajectoryDis = global_Worker_subTrajectoryDis[arrive_global_workerID]; // 当前窗口内工人轨迹点之和
            workerStateg.workerAD_orig = (global_workers[arrive_global_workerID].endTime - current_time) * speed - global_Sumdis[arrive_global_workerID];

            // workerStateg.current_workerSumdis = Sumdis[current_workerID];
            current_workerState.push_back(workerStateg);

            sign = false;
        }
        else // 对于任务进行初始化获取
        {
            arrive_global_taskID++;
            taskg.task = p.second;
            taskg.Original_Local = arrive_global_taskID;
            current_taskGroup.push_back(taskg);
            current_time = p.second.startTime;

            taskStateg.MaxDistanceTask_orig = (p.second.Deadline - current_time) * speed; // 初始化各任务在满足deadline限制的条件下，每个任务可走的最长距离。
            // taskStateg.current_taskCost = 0;
            // taskStateg其他三个参数在后面获得

            // taskStateg.current_task_detour_distance;
            current_taskState.push_back(taskStateg);
            sign = true;
        }

        if (current_taskGroup.size() > 0 && current_workerGroup.size() > 0) // 根据时间更新当前剩余工人和任务
        {

            erase_Task_worker_Timeout(current_taskGroup, current_workerGroup, current_time, current_workerState, current_taskState); // 移除超时的任务和工人
        }
        else
        {
            // cout << nummm++ << "不用处理的" << endl;
            continue;
        }

        match_Whole(current_taskGroup, current_workerGroup, current_time, current_workerState, current_taskState, sign); // 当前任务组,当前工人组,当前窗口截止时间，当前工人剩余绕行距离
        // cout << nummm++ << endl;                                                                                         // cout << nummm++ << endl;
    }
}

/***
 * 计算并存储当前tasklist对应工人最小的poi点
 * 并返回最小的距离
 */
double Basic_information::Caculate_mindist_whole(int global_workerid, int current_taskid, vector<CURRENT_TASK_GROUP> &task, vector<CURRENT_TASK_STATE> &taskState) // 返回最小距离
{                                                                                                                                                                  // 输入一个task,一个worker,返回task与worker的最近绕路距离,以及最近的POI点在worker轨迹中的第几个
    vector<POI> Trajectory;                                                                                                                                        // 定义Trajectory指针
    Trajectory = global_workers[global_workerid].trajectory;                                                                                                       // 指针，访问
    double detour_distance, mindis = 100000;
    int j = -1;
    for (vector<POI>::iterator it = Trajectory.begin(); it != Trajectory.end(); it++)
    {
        j++;

        detour_distance = GetDistance(task[current_taskid].task.Y, task[current_taskid].task.X, (*it).Y, (*it).X);

        if (detour_distance < mindis)
        {

            mindis = detour_distance;
            taskState[current_taskid].poi.push_back(j); // 记录轨迹中的第j个点是最小的
            // whole_global_POI[task[current_taskid].Original_Local][global_workerid];
        }
    }

    return mindis;
}

/***
 * 对新加入的信息进行处理
 * 先判断是否超时
 * 若加入工人
 *    计算每个任务和新工人之间的信息，并配对
 * 若加入任务
 *    计算每个工人和新任务之间的信息，并配对
 */
void Basic_information::match_Whole(vector<CURRENT_TASK_GROUP> &current_taskGroup, vector<CURRENT_WORKERS_GROUP> &current_workerGroup, double current_Time, vector<CURRENT_WORKER_STATE> &workStates, vector<CURRENT_TASK_STATE> &taskStates, bool sign)
{

    int current_Number_Worker = current_workerGroup.size();
    int current_Number_Task = current_taskGroup.size();
    vector<vector<int>> current_CT_Worker(current_Number_Worker, vector<int>(current_Number_Task));
    double current_task_NeedTime = 0;

    if (sign == false)
    { // 如果加入的是工人
        int tempTid = 0;
        int arrive_global_workerID;

        // CURRENT_WORKERS_GROUP *new_wg = &current_workerGroup.back(); // 新工人
        // CURRENT_WORKER_STATE *new_ws = &workStates.back();          // 新工人状态

        for (auto &cu_ts : taskStates) // 计算任务与当前新工人的关联信息
        {
            if (workStates.back().matched_task.size() < Capacity) // 当工人的容量<额定
            {
                if (current_taskGroup[tempTid].task.Minscore <= current_workerGroup[current_Number_Worker - 1].worker.score) // worker分数满足大于MinScore
                {
                    /********+任务的Deadline限制*/
                    //  double dist= taskStates[tempTid].poi[current_Number_Worker - 1];
                    arrive_global_workerID = current_workerGroup.back().Original_Local;

                    double dist = Caculate_mindist_whole(arrive_global_workerID, tempTid, current_taskGroup, taskStates); // 计算tasklist对应新工人最小的poi点
                    cu_ts.current_task_detour_distance.push_back(dist);                                                   // 存储当前任务对应工人的最小距离
                    // cu_ts.workid.push_back(arrive_global_workerID);                                                 // 存储工人的全局ID信息

                    //  double dist= taskStates[currentTaskid].poi[currentWorkerid];                                               // 距离工人最近的点
                    if (current_workerGroup[current_Number_Worker - 1].worker.range >= dist) // 计算绕路距离距离小于range
                    {

                        if (current_taskGroup[tempTid].task.Reward - (2 * dist * c) > 0) // 利润大于0
                        {
                            if (CurrentTask_Satisfy_whole(current_taskGroup, current_workerGroup, tempTid, taskStates, workStates, current_Time, current_Number_Worker - 1, &current_task_NeedTime)) // 未修改                                                                                                                              // if(SatisfiedDeadline(workerid, taskid, AD, dist)) //满足deadline约束+其它约束
                            {
                                // 添加为新加入的工人任务对
                                workStates.back().matched_task.push_back(current_taskGroup[tempTid]); // 记录加入工人的基础信息
                                workStates.back().current_alltaskCost += dist;                        // 记录当前所有绕路花费
                                workStates.back().detour_poi.push_back(cu_ts.poi.back());             // 记录轨迹中的第j个点是最小的;

                                // workStates.back().matched_current_taskCost.push(dist);
                                // cout << "cu_ts.MaxDistanceTask_orig:" << cu_ts.MaxDistanceTask_orig << endl;

                                workStates.back().matched_MaxDistanceTask_orig.push_back(cu_ts.MaxDistanceTask_orig); // 初始化剩余最大绕路距离
                                /**
                                 * 更新
                                 */
                                current_taskGroup[tempTid].sign = false;                                                       // 更新任务为已配对状态
                                global_CT_Worker[arrive_global_workerID].push_back(current_taskGroup[tempTid].Original_Local); // 更新全局匹配对
                                cout << "工人，任务(" << arrive_global_workerID << ", " << current_taskGroup[tempTid].Original_Local << ") 匹配成功" << endl;
                                cout << current_taskGroup[tempTid].task.startTime;

                                cout << "\tWORKER (开始t" << current_workerGroup[current_Number_Worker - 1].worker.startTime
                                     << ",结束t " << current_workerGroup[current_Number_Worker - 1].worker.endTime
                                     << ", 分数" << current_workerGroup[current_Number_Worker - 1].worker.score
                                     << ",范围" << current_workerGroup[current_Number_Worker - 1].worker.range
                                     << ",剩余绕路" << workStates[current_Number_Worker - 1].workerAD_orig - (current_workerGroup[current_Number_Worker - 1].worker.startTime - current_Time) * speed << ")" << endl;

                                cout << "\tTASK (开始t：" << current_taskGroup[tempTid].task.startTime << ",结束t： " << current_taskGroup[tempTid].task.Deadline
                                     << ", 分数：" << current_taskGroup[tempTid].task.Minscore << ", 报酬"
                                     << current_taskGroup[tempTid].task.Reward << ",剩余绕路 " << taskStates[tempTid].MaxDistanceTask_orig << ", 绕路距离："
                                     << dist << " )" << endl;

                                UpdateTaskDeadline_whole(current_Number_Worker - 1, tempTid, workStates, taskStates, current_task_NeedTime, current_taskGroup, current_workerGroup); // 更新，由于work增加新的task因此更新，约束条件的状态。
                            }
                        }
                    }
                }
            }
            else
            { // 超过额定就标记为要删除状态
                current_workerGroup.back().sign = false;
                break;
            }

            tempTid++;
        }
    }
    else // 加入的是任务
    {
        int tempWid = 0;
        int arrive_global_workerID;

        // CURRENT_WORKERS_GROUP *new_wg = &current_workerGroup.back(); // 新工人
        // CURRENT_WORKER_STATE *new_ws = &workStates.back();          // 新工人状态

        for (auto &cu_ws : workStates) // 计算任务与当前新工人的关联信息
        {

            if (current_taskGroup[current_Number_Task - 1].task.Minscore <= current_workerGroup[tempWid].worker.score) // worker分数满足大于MinScore
            {
                /********+任务的Deadline限制*/
                //  double dist= taskStates[tempTid].poi[current_Number_Worker - 1];
                arrive_global_workerID = current_workerGroup[tempWid].Original_Local;

                double dist = Caculate_mindist_whole(arrive_global_workerID, current_Number_Task - 1, current_taskGroup, taskStates); // 计算tasklist对应新工人最小的poi点
                taskStates.back().current_task_detour_distance.push_back(dist);                                                       // 存储当前任务对应工人的最小距离
                // taskStates.back().workid.push_back(arrive_global_workerID);                                                     // 存储工人的全局ID信息

                //  double dist= taskStates[currentTaskid].poi[currentWorkerid];                                               // 距离工人最近的点
                if (current_workerGroup[tempWid].worker.range >= dist) // 计算绕路距离距离小于range
                {

                    if (current_taskGroup[current_Number_Task - 1].task.Reward - (2 * dist * c) > 0) // 利润大于0
                    {
                        if (CurrentTask_Satisfy_whole(current_taskGroup, current_workerGroup, current_Number_Task - 1, taskStates, workStates, current_Time, tempWid, &current_task_NeedTime)) // 未修改                                                                                                                              // if(SatisfiedDeadline(workerid, taskid, AD, dist)) //满足deadline约束+其它约束
                        {
                            // 添加为新加入的工人
                            workStates[tempWid].matched_task.push_back(current_taskGroup[current_Number_Task - 1]); // 记录加入工人的基础信息
                            workStates[tempWid].current_alltaskCost += dist;                                        // 记录当前所有绕路花费
                            workStates[tempWid].detour_poi.push_back(taskStates.back().poi.back());                 // 记录轨迹中的第j个点是最小的;

                            // workStates.back().matched_current_taskCost.push(dist);

                            workStates[tempWid].matched_MaxDistanceTask_orig.push_back(taskStates.back().MaxDistanceTask_orig - dist); // 初始化初始-自绕
                            /**
                             * 更新
                             */
                            current_taskGroup[current_Number_Task - 1].sign = false;                                                                            // 更新任务为已配对状态
                            global_CT_Worker[current_workerGroup[tempWid].Original_Local].push_back(current_taskGroup[current_Number_Task - 1].Original_Local); // 更新全局匹配对
                            if (cu_ws.matched_task.size() == Capacity)                                                                                          // 当工人的容量<额定
                            {
                                current_workerGroup[tempWid].sign = false;
                            }
                            // 更新当前工人已匹配任务的最大可走距离
                            UpdateTaskDeadline_whole(tempWid, current_Number_Task - 1, workStates, taskStates, current_task_NeedTime, current_taskGroup, current_workerGroup); // 更新，由于work增加新的task因此更新，约束条件的状态。
                            break;
                        }
                    }
                }
            }

            tempWid++;
        }
    }

    // ShowCTMatching(current_CT_Worker, current_Number_Worker); // 输出配对
}

void Basic_information::UpdateTaskDeadline_whole(int current_workerid, int taskid, vector<CURRENT_WORKER_STATE> &workStates, vector<CURRENT_TASK_STATE> &taskStates, double current_task_NeedTime, vector<CURRENT_TASK_GROUP> &current_taskGroup, vector<CURRENT_WORKERS_GROUP> &current_workerGroup)
{
    for (int i = 0; i < workStates[current_workerid].matched_task.size(); i++) // 根据当前工人的状态更新任务的时间
    {
        int assigntask_id = workStates[current_workerid].matched_task[i].Original_Local; // 已分配任务的原始id
        int detourpoint = workStates[current_workerid].detour_poi[i];                    // 已匹配任务的偏移点

        int assignedpoint = taskStates[taskid].poi.back(); // 新加入任务的偏移点

        //    if(trajectory_id == detourpoint ) //偏移点为同一个暂时不管

        if (assigntask_id == current_taskGroup[taskid].Original_Local) // 若是刚插入的任务
        {
            // cout<<"原最大可走："<< MaxDistanceTask[taskid]<<endl;
            // taskStates[taskid].MaxDistanceTask_orig -= current_task_NeedTime * speed; // 当前任务剩余可走的最大距离，也就是时间。

            workStates[current_workerid].matched_MaxDistanceTask_orig[i] -= current_task_NeedTime * speed; // 当前任务剩余可走的最大距离，也就是时间。
                                                                                                           // cout<<"本次需要走："<<current_task_NeedTime*speed<<endl;
                                                                                                           // cout<<"当前任务最大可走距离更新为："<< MaxDistanceTask[taskid]<<endl;
        }
        else
        {

            if (detourpoint < assignedpoint) // 只对偏移点之后的任务有影响, 判断是否满足后面任务的Deadline 更新任务的MaxDistanceTask
            {
                // 更新任务的最大可走路程
                workStates[current_workerid].matched_MaxDistanceTask_orig[i] -= (2 * taskStates[taskid].current_task_detour_distance[current_workerid]); // 当前任务剩余可走的最大距离，也就是时间。

                // MaxDistanceTask[assigntask_id] = MaxDistanceTask[assigntask_id] - (2 * current_detour_distance[taskid][workerid]); // 减去插入的任务绕路路程
                // cout<<"影响的任务"<<assigntask_id<<"的最大可走距离更新为："<< MaxDistanceTask[assigntask_id]<<endl;
            }
        }
    }
}

/****
 *
 * 以任务为主导判断工人和任务能否匹配。
 *
 * 1. 判断工人到新任务的绕路距离与工人截止时间关系
 * 2. 判断插入新点对前后点的影响
 *      对前面的点无影响
 *      对后面的点的截止时间影响
 *
 */
bool Basic_information::CurrentTask_Satisfy_whole(vector<CURRENT_TASK_GROUP> &current_taskGroup, vector<CURRENT_WORKERS_GROUP> &current_workerGroup, int current_taskid, vector<CURRENT_TASK_STATE> &taskStates, vector<CURRENT_WORKER_STATE> &workStates, double current_Time, int current_workerid, double *current_task_NeedTime)
{

    int assignedNumber = workStates[current_workerid].matched_task.size(); // 工人已匹配任务数量
    // cout<<"工人已匹配任务数量："<<assignedNumber<<endl;
    // 先判断工人的绕路距离约束是否满足？
    double ADD = workStates[current_workerid].workerAD_orig - workStates[current_workerid].current_alltaskCost -
                 (current_Time - current_workerGroup[current_workerid].worker.startTime) * speed -
                 2 * taskStates[current_taskid].current_task_detour_distance.back(); // 原始剩余距离-已匹配任务的绕路距离-当前时间消耗的距离-当前可能增加的绕路距离

    // cout<<"剩余AD:"<<AD[workerid]<<",任务绕路"<<2 * detour_distance[taskid][workerid]<<","<<ADD<<endl;
    if (ADD < 0)
    {
        // cout<<"不满足工人的Deadline!"<<endl;
        return false;
    }

    //+任务的Deadline约束

    // 对插入点之前的工人已分配的任务 绕路距离进行计算
    int current_detourid = taskStates[current_taskid].poi.back(); // 当前任务的偏移轨迹点
    // cout<<"任务的偏移点序号:"<<current_detourid<<endl;
    // 判断起点是否已有任务偏移
    // bool flag_first = false;
    double SumdetouDis = 0.0;                // 绕路总距离
    for (int i = 0; i < assignedNumber; i++) // 计算插入前任务的总偏移距离
    {
        int assigntask_id = workStates[current_workerid].matched_task[i].Original_Local; // 已分配任务的原始id
        int detourpoint = workStates[current_workerid].detour_poi[i];                    // 已匹配任务的偏移点
        if ((workStates[current_workerid].matched_MaxDistanceTask_orig[i] -
             (current_Time - global_tasks[assigntask_id].startTime)) < 0) // 已匹配的任务超时了，整个工人不能再匹配了
        {
            current_workerGroup[current_workerid].sign = false;

            return false;
        }
        if (current_detourid >= detourpoint) // 求和当前插入点前的绕路距离之和
        {

            double x = global_tasks[assigntask_id].X;
            double y = global_tasks[assigntask_id].Y;
            double x1 = current_workerGroup[current_workerid].worker.trajectory[detourpoint].X;
            double y1 = current_workerGroup[current_workerid].worker.trajectory[detourpoint].Y;

            SumdetouDis += 2 * GetDistance(y, x, y1, x1); // 已匹配任务总偏移距离
                                                          // cout<<"已匹配任务总偏移距离："<<SumdetouDis<<endl;
        }
        // if (detourpoint == 0) // 出发点已有任务
        // {
        //   flag_first = true;
        // }
    }

    double Time_TO_Task; //+起点到最新任务点偏移的花费的总时间----hy未修改
    Time_TO_Task = (taskStates[current_taskid].current_task_detour_distance.back() +
                    workStates[current_workerid].current_worker_subTrajectoryDis[current_detourid] + SumdetouDis) /
                   speed; // 偏移距离+已匹配任务的绕路+起点到任务位置的轨迹路程
    *current_task_NeedTime = Time_TO_Task;
    // Time_TO_Task = (current_detour_distance[taskid][workerid] + current_Worker_subTrajectoryDis[workerid][current_detourid] + SumdetouDis) / speed; // 偏移距离+已匹配任务的绕路+起点到任务位置的轨迹路程
    // 是否满足任务的Deadline约束，//最后一个偏移点也考虑了
    if (current_taskGroup[current_taskid].task.Deadline > (Time_TO_Task + current_Time))
    {
        // cout<<"偏移点到起点的时间:"<<Time_TO_Task<<",任务的DEADLINE"<<task[taskid].Deadline<<endl;
        if (assignedNumber == 0) // 没有已匹配的任务
        {
            // cout<<"满足任务的DEADLINE"<<endl;
            return true;
        }
        else // 判断对已匹配任务的影响
        {
            int noinflu_count = 0; // 判断是否在队列最后插入
            for (int i = 0; i < assignedNumber; i++)
            {
                int assigntask_id = workStates[current_workerid].matched_task[i].Original_Local; // 已分配任务的原始id
                int detourpoint = workStates[current_workerid].detour_poi[i];                    // 已匹配任务的偏移点
                if (current_detourid < detourpoint)                                              // 只对偏移点之后的任务有影响, 判断是否满足后面任务的Deadline 更新任务的MaxDistanceTask
                {
                    // 判断插入对后续任务的影响

                    double x = global_tasks[assigntask_id].X;
                    double y = global_tasks[assigntask_id].Y;
                    double x1 = current_workerGroup[current_workerid].worker.trajectory[detourpoint].X;
                    double y1 = current_workerGroup[current_workerid].worker.trajectory[detourpoint].Y;

                    if (workStates[current_workerid].matched_MaxDistanceTask_orig[i] - (current_Time - global_tasks[assigntask_id].startTime) * speed <
                        (2 * GetDistance(y, x, y1, x1))) // 偏移距离耗时不能超过任务的起止时间
                    // if (MaxDistanceTask[assigntask_id] < (2 * current_detour_distance[taskid][workerid])) // 偏移距离耗时不能超过任务的起止时间
                    {
                        // cout<<"任务影响后续任务的dealine不可插入"<<endl;
                        return false; // 不可插入，立马返回false
                    }
                    else
                    {
                        // 在Deadline范围内，返回函数之后再更新任务的最大可走路程
                        //  MaxDistanceTask[assigntask_id]=MaxDistanceTask[assigntask_id]+(2*detour_distance[taskid][workerid]); //插入的任务绕路路程
                    }
                }

                if (current_detourid > detourpoint) // 在已匹配任务之后插入，无影响
                {
                    noinflu_count++;
                }
            }
            if (noinflu_count == assignedNumber) // 在队列最后插入，直接返回true，不需要判断后续任务的影响
            {
                // 已满足Deadline
                // cout<<"不影响所有任务"<<noinflu_count <<endl;
                return true;
            }
            // cout<<"所有任务不受影响且不在最后插入返回true"<<endl;
            return true; // 所有任务不受影响且不在最后插入，返回true.x
        }
    }
    else // 不满足任务的Deadline直接返回false
    {
        // cout<<"不满足任务的Deadline直接返回false"<<endl;
        return false;
    }
}

/******
 * 移除不满足时间约束的工人和任务
 * 同时，工人的当前剩余绕路距离也会删除
 * 任务的相关信息也会删除
 */
void Basic_information::erase_Task_worker_Timeout(std::vector<CURRENT_TASK_GROUP> &taskList, vector<CURRENT_WORKERS_GROUP> &workerList, double nowTime, vector<CURRENT_WORKER_STATE> &current_workerState, vector<CURRENT_TASK_STATE> &current_taskState)
{
    // 检查分组内的截止时间是否小于当前时间，如果是则删除该任务

    for (int i = taskList.size() - 1; i >= 0; i--)
    {
        if (taskList[i].task.Deadline < nowTime || taskList[i].sign == false)
        {
            taskList.erase(taskList.begin() + i);
            current_taskState.erase(current_taskState.begin() + i);
        }
    }

    // 检查分组内的截止时间是否小于当前时间，如果是则删除该工人
    for (int i = workerList.size() - 1; i >= 0; i--)
    {
        if (workerList[i].worker.endTime < nowTime || workerList[i].sign == false)
        {

            workerList.erase(workerList.begin() + i);
            current_workerState.erase(current_workerState.begin() + i);

            // for (int j = taskList.size() - 1; j >= 0; j--) // 同时删除任务列表中与该工人相关的信息(工3个)
            // {
            //   current_taskState[j].poi.erase(current_taskState[j].poi.begin() + i);
            //   current_taskState[j].current_task_detour_distance.erase(current_taskState[j].current_task_detour_distance.begin() + i);
            //   current_taskState[j].workid.erase(current_taskState[j].workid.begin() + i);
            //   cout << "\t\t\t2.1.2。1" << endl;
            // }
        }
    }
}

/***
 * 读取全文需要的数据
 * 从本地获取数据
 * 以及对没用的数据进行生产
 * dataOption
 *      1：Berlin，   2：G-mission   3:T-drive
 */
bool optionDataset(int dataOption, Basic_information &info, vector<vector<double>> &global_Worker_subTrajectoryDis, vector<double> &Sumdis, double endtimeX)
{
    ofstream out("../Satisfaction_Results.txt", ios::app);
    auto start1 = std::chrono::system_clock::now();
    std::time_t tt;
    tt = std::chrono::system_clock::to_time_t(start1);
    string t = ctime(&tt);
    out << "-------------------------\n";
    if (dataOption == 1)
    {
        if (info.Number_Worker > 236)
        {
            cout << "本数据集的工人数量最多为：236" << endl;
            return false;
        }
        // 任务的：分数、奖励、时间 都是生成的。
        // 工人的：绕路范围、分数、时间都是生成的
        double rangeX = 2000;      // 工人绕路范围，固定数值
        double scoreX = 1;         // 工人的分数，这是一个参数
        int Number_BusStop = 4346; // 公交站数量，为数据集固定的，轨迹数量也是固定的

        DataManager datasets(endtimeX, rangeX, scoreX, info.speed); //  生成工人信息分数，评分时间，等等等

        datasets.ReadLocationForTask(info.global_tasks);                                               // 从文件读取获取任务位置，无需修改
        datasets.Prodece_Task_Reward_Minscore_Deadline(info.global_tasks);                             // 生成任务数据，增加开始时间并排序，已修改
        datasets.Get_Trajectory_locations(info.global_workers, Number_BusStop);                        // 从文件读取获取woker的轨迹，无需修改
        info.Caculate_Sumdist_Trajectory(Sumdis, global_Worker_subTrajectoryDis, info.global_workers); // 获取woker的整条轨迹的距离，无需修改
        datasets.Prodece_Worker_endTime_range_score(info.global_workers, Sumdis);                      // 生成工人信息分数，评分时间，等等等，已修改
        out << "数据集：Berlin\t" << t << "\n";
        out << "工人轨迹点数（自定义）：" << 30 << "\n";
    }
    else if (dataOption == 2)
    {
        if (info.Number_Worker > 200 || info.Number_Task > 713)
        {
            cout << "本数据集的工人数量最多为：200；任务数量为713" << endl;
            return false;
        }

        // 任务：奖励是从数据集获取的
        // 工人：分数是从数据集获取的

        // 任务：评分（根据工人分数生成，不要变）、时间（自动生成）
        // 工人：范围（根据任务奖励生成，不要变）、时间（自动生成）

        double rangeX = 0.3;                                                 // 接任务的范围，需要调整
        double scoreX = 1;                                                   // 工人的分数，这是一个参数
        DataManage_G_mission datasets(endtimeX, rangeX, scoreX, info.speed); //    生成工人信息分数，评分时间，等等等，已修改
        //
        // Prodece_Task_Reward_Minscore(task);

        datasets.ReadLocationForTask(info.global_tasks);                                               // 从文件读取获取任务位置，无需修改
        datasets.Prodece_Task_Reward_Minscore_Deadline(info.global_tasks);                             // 生成任务数据，增加开始时间并排序，已修改
        datasets.Get_Trajectory_locations(info.global_workers);                                        // 从文件读取获取woker的轨迹，无需修改
        info.Caculate_Sumdist_Trajectory(Sumdis, global_Worker_subTrajectoryDis, info.global_workers); // 获取woker的整条轨迹的距离，无需修改
        datasets.Prodece_Worker_endTime_range_score(info.global_workers, Sumdis);                      // 生成工人信息分数，评分时间，等等等，已修改
        out << "数据集：G_mission\t" << t << "\n";
        out << "工人轨迹点数（数据集自带）：" << 5 << "\n";
    }
    else if (dataOption == 3)
    {
        int Worker_Record = 5823;         // 工人最大的数量
        int Number_Trajectory_Point = 30; // 每个工人取30个轨迹点
        if (info.Number_Worker > Worker_Record)
        {
            cout << "工人数量最多为：5823" << endl;
            return false;
        }

        // cin >> Worker_Record >> Number_Trajectory_Point;
        DataManager_T_Drive datasets(endtimeX, 2000, 1, info.speed, Worker_Record, Number_Trajectory_Point); // endtimeX;rangeX;  scoreX;   speed;    Worker_Record; Number_Trajectory_Point;

        datasets.Get_Trajectory_locations();
        datasets.Caculate_Sumdist_Trajectory();
        datasets.ReproduceWorker(info.global_workers, Sumdis, global_Worker_subTrajectoryDis);
        datasets.Prodece_Worker_endTime_range_score(info.global_workers, Sumdis);
        datasets.Produce_Random_Task_ForTrajectoryPoint(int(info.Number_Task / info.Number_Worker), Number_Trajectory_Point, info.global_workers, info.global_tasks);
        datasets.Prodece_Task_Reward_Minscore_Deadline(info.global_tasks);
        out << "数据集：T_drive\t" << t << "\n";
        out << "工人轨迹点数（自定义）：" << 30 << "\n";
    }
    else
    {
        cout << "输错了，要输入1-3！" << endl;
        out << "数据集：输错了\t" << t << "\n";
        return false;
    }

    out << "任务数：" << info.Number_Task << "\t 工人数：" << info.Number_Worker << "\t工人容量:" << info.Capacity << "\t工人成本/单位" << info.c << "\t移动速度:" << info.speed
        << "\t窗口时间：" << info.Wmax << "\t窗口任务：" << info.Tmax << "\n";
    out.close();
    return true;
}

/***
 * 下面是多余的函数
 */
// 已经合并
// void Basic_information::Compute_AvailableWorker(int taskid, vector<vector<pair<int, double>>> &PT, vector<vector<double>> &detour_distance, vector<vector<int>> &global_POI)
// {
//     // 计算taskid任务的可用工人并加入偏序列表
//     // cout << "全局匹配满足要求的r任务：" << taskid << "\t\t";
//     for (int j = 0; j < Number_Worker; j++)
//     {
//         if (!(global_tasks[taskid].Deadline < global_workers[j].startTime || global_tasks[taskid].startTime > global_workers[j].endTime)) // 满足时间约束

//         {
//             if (global_tasks[taskid].Minscore <= global_workers[j].score)
//             {
//                 // 工人的分数满足任务的最小约束                                                                                    //为何错误？？？？
//                 detour_distance[taskid][j] = Caculate_mindist(j, taskid, global_POI); // 计算每个任务和每个worker之间的最小绕路距离

//                 if (detour_distance[taskid][j] <= global_workers[j].range)
//                 {                                                                                            // 当任务的绕路距离小于工人的最大绕路距离，计算偏好
//                     double preference2 = global_tasks[taskid].Reward - (2 * detour_distance[taskid][j] * c); // 利润大于0
//                     if (preference2 > 0)
//                     {
//                         double preference = global_workers[j].score;    // 偏好值为score
//                         PT[taskid].push_back(make_pair(j, preference)); // I任务的j worker对应的preference
//                         cout << "(任务,工人)可配对情况：  (" << taskid << "," << j << ")" << endl;
//                     }
//                 }
//                 else
//                 {
//                     //           cout<<taskid<<"\t"<<j<<"距离太远!"<<detour_distance[taskid][j]<<endl;
//                 }
//                 //    cout<<endl;
//             }
//         }
//     }
// }

// void Basic_information::Compute_AvailableTask(int workerid, vector<vector<pair<int, double>>> &PW, vector<vector<double>> &detour_distance, vector<vector<int>> &poi)
// { // 计算taskid任务的可用任务并加入偏序列表
//     for (int j = 0; j < Number_Task; j++)
//     {
//         if (global_tasks[j].Minscore <= global_workers[workerid].score)
//         {                                                                      // 工人的分数满足任务的最小约束                                                                                    //为何错误？？？？
//             detour_distance[j][workerid] = Caculate_mindist(workerid, j, poi); // 计算每个任务和每个worker之间的最小绕路距离
//             if (detour_distance[j][workerid] <= global_workers[workerid].range)
//             { // 当任务的绕路距离小于工人的最大绕路距离，计算偏好
//                 //   double preference= worker[j].trajectory[poi].poitime+detour_distance/speed;      //偏好值单任务的等待时间
//                 double preference = global_tasks[j].Reward - (2 * detour_distance[j][workerid] * c); // 偏好值为score
//                 if (preference > 0)
//                 {
//                     PW[workerid].push_back(make_pair(j, preference)); // I任务的j worker对应的preference
//                 }
//             }
//         }
//     }
// }
