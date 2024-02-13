#include "Basic_information.h"
#include <chrono>
#include <unordered_map>

void Basic_information::Initialize_group(vector<double> &AD, vector<double> &current_Sumdis, vector<vector<double>> &current_Group_worker_subTrajectoryDis, int current_workID, double nowTime, vector<CURRENT_WORKERS_GROUP> &current_workerGroup)
{

    AD.clear();

    for (int j = 0; j < current_workerGroup.size(); j++)
    {

        double cuAD = (current_workerGroup[j].worker.endTime - nowTime) * speed - current_Sumdis[j];

        if (current_workerGroup[j].worker.endTime <= nowTime || cuAD <= 0 || current_workerGroup[j].sign == false || current_Sumdis[j] < 0.01)
        {

            current_workerGroup.erase(current_workerGroup.begin() + j);
            current_Sumdis.erase(current_Sumdis.begin() + j);
            current_Group_worker_subTrajectoryDis.erase(current_Group_worker_subTrajectoryDis.begin() + j);

            j--;
            continue;
        }
        AD.push_back(cuAD);
    }
}

double Basic_information::Caculate_mindist_global(int workerid, int taskid, vector<vector<int>> &poi)
{
    vector<POI> Trajectory;
    Trajectory = global_workers[workerid].trajectory;
    double detour_distance, mindis = 100000;
    int j = -1;

    for (vector<POI>::iterator it = Trajectory.begin(); it != Trajectory.end(); it++)
    {
        j++;
        detour_distance = GetDistance(global_tasks[taskid].Y, global_tasks[taskid].X, (*it).Y, (*it).X);

        if (detour_distance < mindis)
        {
            mindis = detour_distance;
            poi[taskid][workerid] = j;
        }
    }
    return mindis;
}

/**
 * 求解全局内工人和任务的偏好
 */
int Basic_information::Compute_global_PTPW_Group(vector<vector<pair<int, double>>> &PT, vector<vector<pair<int, double>>> &PW)
{

    int Number_current_taskGroup = global_tasks.size();
    int Number_current_workerGroup = global_workers.size();
    int num = 0;

    for (int i = 0; i < Number_current_taskGroup; i++)
    {
        for (int j = 0; j < Number_current_workerGroup; j++)
        {
            if (!(global_tasks[i].Deadline < global_workers[j].startTime || global_tasks[i].startTime > global_workers[j].endTime))
            {

                global_detour_distance[i][j] = Caculate_mindist_global(j, i, global_POI);

                if (global_detour_distance[i][j] <= global_workers[j].range)
                {
                    double stime = max(global_workers[j].startTime, global_tasks[i].startTime);
                    double etime = min(global_workers[j].endTime, global_tasks[i].Deadline);
                    double subdis = global_Worker_subTrajectoryDis[j][global_POI[i][j]];

                    double taskCost = subdis + global_detour_distance[i][j];
                    double workCost = global_Sumdis[j] + 2 * global_detour_distance[i][j];

                    if (taskCost < (etime - stime) * speed && workCost < (global_workers[j].endTime - stime) * speed)
                    {
                        double preference1 = global_tasks[i].Reward - 2 * global_detour_distance[i][j] * c;
                        if (preference1 > 0)
                        {
                            PW[j].push_back(make_pair(i, preference1));
                            num++;
                        }
                        if (global_tasks[i].Minscore <= global_workers[j].score)
                        {
                            double preference2 = global_workers[j].score;
                            PT[i].push_back(make_pair(j, preference2));
                        }
                    }
                }
            }
        }
    }

    cout << "总的任意匹配对数：" << num << endl;

    for (int i = 0; i < global_workers.size(); i++)
    {

        sort(PW[i].begin(), PW[i].end(), cmp);
    }

    for (int i = 0; i < global_tasks.size(); i++)
    {

        sort(PT[i].begin(), PT[i].end(), cmp);
    }

    return num;
}

double Basic_information::Caculate_mindist(int workerid, int taskid, vector<int> poi[], vector<CURRENT_TASK_GROUP> &task, vector<CURRENT_WORKERS_GROUP> &worker)
{
    vector<POI> Trajectory;
    Trajectory = worker[workerid].worker.trajectory;
    double detour_distance, mindis = 100000;
    int j = -1;
    for (vector<POI>::iterator it = Trajectory.begin(); it != Trajectory.end(); it++)
    {
        j++;
        detour_distance = GetDistance(task[taskid].task.Y, task[taskid].task.X, (*it).Y, (*it).X);

        if (detour_distance < mindis)
        {
            mindis = detour_distance;
            poi[taskid][workerid] = j;
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
int Basic_information::FindLatestWorkerNew_Greedy(int taskid, vector<int> &current_Group_worker_AW, vector<double> &current_Group_workerAD, vector<double> current_detour_distance[], vector<vector<double>> &current_Group_worker_subTrajectoryDis, vector<int> CT_Worker[], double MaxDistanceTask[], double *current_task_NeedTime, vector<CURRENT_TASK_GROUP> &current_taskGroup, vector<CURRENT_WORKERS_GROUP> &current_workerGroup, vector<int> current_poi[], double nowtime)
{

    int latest_workerid = -1;
    double time = 0;
    double mindist = __DBL_MAX__;
    for (int j = 0; j < current_Group_worker_AW.size(); j++)
    {
        int workerid = current_Group_worker_AW[j];

        if (current_taskGroup[taskid].task.Minscore <= current_workerGroup[workerid].worker.score)
        {
            double dist = Caculate_mindist(workerid, taskid, current_poi, current_taskGroup, current_workerGroup);

            current_detour_distance[taskid][workerid] = dist;
            if (current_workerGroup[workerid].worker.range >= dist)
            {

                if (current_taskGroup[taskid].task.Reward - (2 * dist * c) > 0)
                {

                    if (CurrentTask_Satisfy(current_taskGroup, workerid, taskid, current_Group_workerAD, current_detour_distance, current_Group_worker_subTrajectoryDis, CT_Worker, current_poi, MaxDistanceTask, current_task_NeedTime, nowtime))

                    {

                        if (dist < mindist)
                        {
                            mindist = dist;
                            latest_workerid = workerid;
                            time = *current_task_NeedTime;
                        }
                    }
                }
            }
        }
    }
    *current_task_NeedTime = time;

    return latest_workerid;
}

void Basic_information::ReadLocationForTask(vector<TASK> &task)
{
    ifstream in("../../dataset\\Berlin\\Task_LocationBER.txt");

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
        }
    }
    in.close();
}

/****
 * 诗婷原本的代码中的截止时间计算方法为：（距离/速度）*（1+2000）
 * @rangeX区域：是定值，注意是够需要修改。
 * @endtimeX截止时间：是定值，注意是够需要修改
 */
void Basic_information::Prodece_Worker_endTime_range_score(vector<WORKER> &worker, vector<double> &Sumdis, double endtimeX, double rangeX, double scoreX)
{

    default_random_engine e(3333), e1(365485449);
    uniform_real_distribution<double> u(60, 100);
    uniform_int_distribution<unsigned> u1(0, 60);

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

        s = sqrt(pow((lat1 - lat2), 2) + pow((lng1 - lng2), 2));
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

            if (PT[taskid].size() == index)
                cout << "未在任务列表中找到工人！" << endl;
            else
            {
                double s = ((PT[taskid][index].second - 0) / (PT[taskid][0].second - 0));

                sum = sum + s;
            }
            */

            auto it = find_if(PT[taskid].begin(), PT[taskid].end(), [workerID](const auto &p)
                              { return p.first == workerID; });

            int ttt = 0;
            if (it != PT[taskid].end())
            {
                int index = distance(PT[taskid].begin(), it);

                double s = ((PT[taskid][index].second - global_tasks[taskid].Minscore + 1) / (PT[taskid][ttt].second - global_tasks[taskid].Minscore + 1));

                sum = sum + s;
                ttt++;
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
    int matchnum = 0;

    for (int i = 0; i < Number_Worker; i++)
    {
        int workerID = i;
        for (int j = 0; j < CT_Worker[i].size(); j++)
        {
            matchnum++;
            int taskid = CT_Worker[i][j];

            /*
             int index = GetIndex_PT(i, taskid, PT);


             if (PT[taskid].size() == index)
                 cout << "未在任务列表中找到工人！" << endl;
             else
             {
                 double s = ((PT[taskid][index].second - 0) / (PT[taskid][0].second - 0));

                 sum = sum + s;
             }
            */

            auto it = find_if(PT[taskid].begin(), PT[taskid].end(), [workerID](const auto &p)
                              { return p.first == workerID; });

            int ttt = 0;
            if (it != PT[taskid].end())
            {
                int index = distance(PT[taskid].begin(), it);
                double s = ((PT[taskid][index].second - global_tasks[taskid].Minscore + 1) / (PT[taskid][ttt].second - global_tasks[taskid].Minscore + 1));

                s *= global_MaxDistanceTask[taskid] / ((global_tasks[taskid].Deadline - global_tasks[taskid].startTime) * speed - global_detour_distance[taskid][workerID]);

                sum = sum + s;
                ttt++;
            }
        }
    }

    return 0.2 * matchnum / Number_Task + 0.4 * sum / matchnum;
}
/***
 * 打印工人满意度avg
 */
double Basic_information::Caculate_Worker_Satisfaction_avg(vector<vector<int>> &CT_Worker, vector<vector<pair<int, double>>> &PW)
{
    double allsum = 0;
    int matchNumber = 0;
    for (int i = 0; i < Number_Worker; i++)
    {
        int workerID = i;
        if (CT_Worker[i].size() != 0)
        {

            matchNumber++;
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
                int ttt = 0;
                if (it != PW[workerID].end())
                {
                    int index = distance(PW[workerID].begin(), it);

                    double s = ((PW[workerID][index].second - 0) / (PW[workerID][ttt].second - 0));

                    sum = sum + s;
                    ttt++;
                }
            }

            avg = sum / Capacity;
            allsum = allsum + avg;
        }
    }

    return 0.4 * allsum / Number_Worker + 0 * matchNumber / Number_Worker;
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
                int ttt = 0;
                if (it != PW[workerID].end())
                {
                    int index = distance(PW[workerID].begin(), it);
                    double s = ((PW[workerID][index].second - 0) / (PW[workerID][ttt].second - 0));

                    sum = sum + s;
                    ttt++;
                }
            }
            avg = sum;
            allsum = allsum + avg;
        }
    }
    return allsum / c;
}

int Basic_information::GetIndex_PW(int workerid, int taskid, vector<vector<pair<int, double>>> &PW)
{

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
void Basic_information::computeMaxDitanceTask(double MaxDistanceTask[], vector<CURRENT_TASK_GROUP> currentTask, double current_window_endTime)
{

    for (int i = 0; i < currentTask.size(); i++)
    {
        MaxDistanceTask[i] = (currentTask[i].task.Deadline - current_window_endTime) * speed;
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

    int assignedNumber = CT_Worker[workerid].size();
    if (assignedNumber == 0)
        return false;

    double ADD = AD[workerid] + 2 * detour_distance[assignedtaskid][workerid] - 2 * detour_distance[taskid][workerid];

    if (ADD < 0)
    {

        return false;
    }

    int current_detourid = poi[taskid][workerid];

    bool flag_first = false;

    int replacepoi = poi[assignedtaskid][workerid];
    if (replacepoi <= current_detourid)
        beforesumDis = beforesumDis - 2 * detour_distance[assignedtaskid][workerid];

    double Time_TO_Task;
    Time_TO_Task = (detour_distance[taskid][workerid] + Worker_subTrajectoryDis[workerid][current_detourid] + beforesumDis) / speed;
    *current_task_NeedTime = Time_TO_Task;

    if (current_taskGroup[taskid].task.Deadline > Time_TO_Task + nowtime)
    {

        int noinflu_count = 0;
        for (int i = 0; i < assignedNumber; i++)
        {
            int assigntask_id = CT_Worker[workerid][i];
            int detourpoint = poi[assigntask_id][workerid];
            double temporalMaxd = 0;

            if (replacepoi < detourpoint && assigntask_id != assignedtaskid)
            {

                temporalMaxd = MaxDistanceTask[assigntask_id] + (2 * detour_distance[assignedtaskid][workerid]);
            }

            if (current_detourid < detourpoint && assigntask_id != assignedtaskid)
            {

                if (temporalMaxd < (2 * detour_distance[taskid][workerid]))
                {

                    return false;
                }
                else
                {
                }
            }

            if (current_detourid > detourpoint)
            {
                noinflu_count++;
            }
        }
        if (noinflu_count == assignedNumber)
        {

            return true;
        }

        return true;
    }
    else
    {

        return false;
    }
}

bool Basic_information::CurrentTask_Satisfy_TSDA(vector<CURRENT_TASK_GROUP> &current_taskGroup, int workerid, int taskid, vector<double> &AD, vector<double> current_detour_distance[], vector<vector<double>> &current_Worker_subTrajectoryDis, vector<vector<int>> &CT_Worker, vector<int> poi[], double MaxDistanceTask[], double *current_task_NeedTime, double nowtime)
{
    int assignedNumber = CT_Worker[workerid].size();
    if (assignedNumber == Capacity)
    {
        return false;
    }

    double ADD = AD[workerid] - 2 * current_detour_distance[taskid][workerid];

    if (ADD < 0)
    {
        return false;
    }

    int current_detourid = poi[taskid][workerid];

    bool flag_first = false;
    double SumdetouDis = 0.0;
    for (int i = 0; i < assignedNumber; i++)
    {
        int assigntask_id = CT_Worker[workerid][i];
        int detourpoint = poi[assigntask_id][workerid];

        if (current_detourid >= detourpoint)
        {
            SumdetouDis = SumdetouDis + 2 * current_detour_distance[assigntask_id][workerid];
        }
        if (detourpoint == 0)
        {
            flag_first = true;
        }
    }

    double Time_TO_Task = 0.0;

    Time_TO_Task = (current_detour_distance[taskid][workerid] + current_Worker_subTrajectoryDis[workerid][current_detourid] + SumdetouDis) / speed;

    if (current_taskGroup[taskid].task.Deadline > Time_TO_Task + nowtime)
    {
        *current_task_NeedTime = Time_TO_Task;

        if (assignedNumber == 0)
        {

            return true;
        }
        else
        {
            int noinflu_count = 0;
            for (int i = 0; i < assignedNumber; i++)
            {
                int assigntask_id = CT_Worker[workerid][i];
                int detourpoint = poi[assigntask_id][workerid];

                if (current_detourid < detourpoint)
                {

                    if (MaxDistanceTask[assigntask_id] < (2 * current_detour_distance[taskid][workerid]))
                    {

                        return false;
                    }
                    else
                    {
                    }
                }

                if (current_detourid > detourpoint)
                {
                    noinflu_count++;
                }
            }
            if (noinflu_count == assignedNumber)
            {

                return true;
            }

            return true;
        }
    }
    else
    {

        return false;
    }
}

bool Basic_information::CurrentTask_Satisfy(vector<CURRENT_TASK_GROUP> &current_taskGroup, int workerid, int taskid, vector<double> &AD, vector<double> current_detour_distance[], vector<vector<double>> &current_Worker_subTrajectoryDis, vector<int> CT_Worker[], vector<int> poi[], double MaxDistanceTask[], double *current_task_NeedTime, double nowtime)
{
    int assignedNumber = CT_Worker[workerid].size();

    double ADD = AD[workerid] - 2 * current_detour_distance[taskid][workerid];

    if (ADD < 0)
    {

        return false;
    }

    int current_detourid = poi[taskid][workerid];

    bool flag_first = false;
    double SumdetouDis = 0.0;
    for (int i = 0; i < assignedNumber; i++)
    {
        int assigntask_id = CT_Worker[workerid][i];
        int detourpoint = poi[assigntask_id][workerid];

        if (current_detourid >= detourpoint)
        {
            SumdetouDis = SumdetouDis + 2 * current_detour_distance[assigntask_id][workerid];
        }
        if (detourpoint == 0)
        {
            flag_first = true;
        }
    }

    double Time_TO_Task;

    Time_TO_Task = (current_detour_distance[taskid][workerid] + current_Worker_subTrajectoryDis[workerid][current_detourid] + SumdetouDis) / speed;

    if (current_taskGroup[taskid].task.Deadline > Time_TO_Task + nowtime)
    {
        *current_task_NeedTime = Time_TO_Task;

        if (assignedNumber == 0)
        {

            return true;
        }
        else
        {
            int noinflu_count = 0;
            for (int i = 0; i < assignedNumber; i++)
            {
                int assigntask_id = CT_Worker[workerid][i];
                int detourpoint = poi[assigntask_id][workerid];

                if (current_detourid < detourpoint)
                {

                    if (MaxDistanceTask[assigntask_id] < (2 * current_detour_distance[taskid][workerid]))
                    {

                        return false;
                    }
                    else
                    {
                    }
                }

                if (current_detourid > detourpoint)
                {
                    noinflu_count++;
                }
            }
            if (noinflu_count == assignedNumber)
            {

                return true;
            }

            return true;
        }
    }
    else
    {

        return false;
    }
}

void Basic_information::UpdateTaskDeadline(vector<CURRENT_TASK_GROUP> &current_taskGroup, int workerid, int taskid, vector<double> current_detour_distance[], vector<int> CT_Worker[], vector<int> poi[], double MaxDistanceTask[], double current_task_NeedTime)
{

    for (int i = 0; i < CT_Worker[workerid].size(); i++)
    {
        int assigntask_id = CT_Worker[workerid][i];
        int assignedpoint = poi[assigntask_id][workerid];
        int detourpoint = poi[taskid][workerid];

        if (assigntask_id == taskid)
        {

            MaxDistanceTask[taskid] = MaxDistanceTask[taskid] - current_task_NeedTime * speed;

            global_MaxDistanceTask[current_taskGroup[taskid].Original_Local] = MaxDistanceTask[taskid];
        }
        else
        {

            if (detourpoint < assignedpoint)
            {

                MaxDistanceTask[assigntask_id] = MaxDistanceTask[assigntask_id] - (2 * current_detour_distance[taskid][workerid]);

                global_MaxDistanceTask[current_taskGroup[assigntask_id].Original_Local] = MaxDistanceTask[assigntask_id];
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
{
    std::default_random_engine e1(2013467), e2(365485449), e3(3333), e4(12387);
    uniform_real_distribution<double> u1(60, 100);
    uniform_real_distribution<double> u2(1, 10);
    uniform_real_distribution<double> u3(20, 60);
    uniform_real_distribution<double> u4(0, 50);
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
    }
    sort(task.begin(), task.end(), cmp_task_start);
}

/**
 * Sumdis获取每个工人的轨迹距离之和，无需修改
 */

void Basic_information::Caculate_Sumdist_Trajectory(vector<double> &Sumdis, vector<vector<double>> &global_Worker_subTrajectoryDis, vector<WORKER> &worker)
{

    int RandomNumber = 0;
    for (int i = 0; i < Number_Worker; i++)
    {

        double sum = 0;
        double subsum = 0;
        if (i > 235)
        {
            srand((unsigned)time(NULL));
            RandomNumber = rand() % 236;
            Sumdis[i] = Sumdis[RandomNumber];
            for (int mmm = 0; mmm < global_Worker_subTrajectoryDis[RandomNumber].size(); mmm++)
            {
                global_Worker_subTrajectoryDis[i].push_back(global_Worker_subTrajectoryDis[RandomNumber][mmm]);
                worker[i].trajectory.push_back(worker[RandomNumber].trajectory[mmm]);
            }
        }
        else
        {
            global_Worker_subTrajectoryDis[i].push_back(0);

            for (int j = 0; j < worker[i].trajectory.size() - 1; j++)
            {

                double distance = GetDistance(worker[i].trajectory[j].Y, worker[i].trajectory[j].X, worker[i].trajectory[j + 1].Y, worker[i].trajectory[j + 1].X);
                sum = sum + distance;
                global_Worker_subTrajectoryDis[i].push_back(sum);
            }
            Sumdis[i] = sum;
        }
    }
}

void Basic_information::begin_Algorithm(string alg_num)
{
    global_CT_Worker.clear();
    global_CT_Worker.resize(Number_Worker);

    global_Current_workID = 0;
    global_Current_taskNumber = 0;

    global_MaxDistanceTask.clear();
    global_MaxDistanceTask.resize(0);

    worknextMaxDistanceTask.clear();
    worknextMaxDistanceTask.resize(0);

    nextWorkSet.clear();
    nextWorkSet.resize(0);
}
void Basic_information::updata_current_Info(vector<CURRENT_TASK_GROUP> &current_taskGroup, vector<CURRENT_WORKERS_GROUP> &current_workerGroup, vector<double> &current_Group_workerAD, vector<vector<double>> &current_Group_worker_subTrajectoryDis, vector<double> &current_Group_workerSumdis)
{
    vector<CURRENT_WORKERS_GROUP> temp_work_group;
    vector<vector<double>> Temp_current_Group_worker_subTrajectoryDis;
    vector<double> Temp_current_Group_workerSumdis;

    for (vector<CURRENT_WORKERS_GROUP>::iterator it_curret_Worker = current_workerGroup.begin(); it_curret_Worker != current_workerGroup.end(); ++it_curret_Worker)
    {
        if ((*it_curret_Worker).sign)
        {
            temp_work_group.push_back(*it_curret_Worker);
            Temp_current_Group_worker_subTrajectoryDis.push_back(global_Worker_subTrajectoryDis[(*it_curret_Worker).Original_Local]);
            Temp_current_Group_workerSumdis.push_back(global_Sumdis[(*it_curret_Worker).Original_Local]);
        }
    }
    current_workerGroup.clear();
    current_Group_worker_subTrajectoryDis.clear();
    current_Group_workerSumdis.clear();

    current_workerGroup = temp_work_group;
    current_Group_worker_subTrajectoryDis = Temp_current_Group_worker_subTrajectoryDis;
    current_Group_workerSumdis = Temp_current_Group_workerSumdis;

    temp_work_group.clear();
    Temp_current_Group_worker_subTrajectoryDis.clear();
    Temp_current_Group_workerSumdis.clear();

    vector<CURRENT_TASK_GROUP> temp_task_group;

    for (vector<CURRENT_TASK_GROUP>::iterator it_curret_task = current_taskGroup.begin(); it_curret_task != current_taskGroup.end(); ++it_curret_task)
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
    vector<CURRENT_TASK_GROUP> current_taskGroup;
    vector<CURRENT_WORKERS_GROUP> current_Group_worker;
    vector<double> current_Group_workerSumdis;
    vector<double> current_Group_workerAD;
    double current_window_endTime = tasks[0].startTime + Wmax;
    double current_window_startTime = tasks[0].startTime;
    int current_workID = global_Current_workID;

    vector<vector<double>> current_Group_worker_subTrajectoryDis;

    int sign = 0;

    int temp = 0;

    for (int i = 0; i < tasks.size(); i++)
    {

        TASK task = tasks[i];
        sign++;

        CURRENT_TASK_GROUP taskg;
        taskg.task = task;
        taskg.Original_Local = i;
        current_taskGroup.push_back(taskg);

        if ((sign - temp == Tmax) || (task.startTime >= current_window_endTime))

        {
            temp = current_taskGroup.size();

            current_workID = global_Current_workID;

            current_window_endTime = task.startTime;

            groupWork_according_TaskGroup(workers, Sumdis, current_Group_worker, current_Group_workerSumdis, current_window_endTime, current_workID, current_Group_workerAD, global_Worker_subTrajectoryDis, current_Group_worker_subTrajectoryDis);
            determine_Window_Task_Timeout(current_taskGroup, current_window_endTime);

            match_WorkerTask_Greedy(current_taskGroup, current_Group_worker, current_window_endTime, current_Group_workerAD, current_Group_worker_subTrajectoryDis);

            current_window_startTime = current_window_endTime;
            current_window_endTime += Wmax;

            updata_current_Info(current_taskGroup, current_Group_worker, current_Group_workerAD, current_Group_worker_subTrajectoryDis, current_Group_workerSumdis);
        }
    }

    if (current_taskGroup.size() > 0)
    {

        current_workID = global_Current_workID;
        current_window_startTime = current_taskGroup.back().task.startTime;

        groupWork_according_TaskGroup(workers, Sumdis, current_Group_worker, current_Group_workerSumdis, current_window_endTime, current_workID, current_Group_workerAD, global_Worker_subTrajectoryDis, current_Group_worker_subTrajectoryDis);

        determine_Window_Task_Timeout(current_taskGroup, current_window_endTime);

        match_WorkerTask_Greedy(current_taskGroup, current_Group_worker, current_window_endTime, current_Group_workerAD, current_Group_worker_subTrajectoryDis);
        updata_current_Info(current_taskGroup, current_Group_worker, current_Group_workerAD, current_Group_worker_subTrajectoryDis, current_Group_workerSumdis);
    }
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

    int ttt = current_workID;

    for (auto it = (workers.begin() + current_workID); it != workers.end();)
    {

        if (it->startTime < nowTime && it->endTime > nowTime && Sumdis[global_Current_workID] > 0.01)
        {

            workerg.worker = *it;
            workerg.Original_Local = global_Current_workID;
            current_workerGroup.push_back(workerg);
            current_Group_workerSumdis.push_back(Sumdis[global_Current_workID]);

            current_Group_worker_subTrajectoryDis.push_back(global_Worker_subTrajectoryDis[global_Current_workID]);
        }
        ttt++;

        if (it->startTime >= nowTime)
        {

            Initialize_group(current_Group_workerAD, current_Group_workerSumdis, current_Group_worker_subTrajectoryDis, current_workID, nowTime, current_workerGroup);

            break;
        }
        ++global_Current_workID;

        ++it;
    }
}

/******
 * 计算当前窗口中满足时间约束的task，
 * 即查看有没分组内超时的任务
 */
void Basic_information::determine_Window_Task_Timeout(std::vector<CURRENT_TASK_GROUP> &taskList, double &nowTime)
{

    taskList.erase(std::remove_if(taskList.begin(), taskList.end(), [nowTime](CURRENT_TASK_GROUP task)
                                  { return task.task.Deadline < nowTime || task.sign == false; }),
                   taskList.end());
}

/**ShowCTMatching
 * 展示组内和全局的匹配结果
 * 有两种方式是因为我懒得修改了。
 * 其中vector<vector<int>>指的全局
 * vector<int> [number]:指的组内
 */
void Basic_information::ShowCTMatching(vector<vector<int>> &CT_Worker, int current_Number_Workers)
{
    int sumtasks = 0;
    int workers = 0;
    for (int i = 0; i < current_Number_Workers; i++)
    {

        sumtasks += CT_Worker[i].size();

        if (CT_Worker[i].size() != 0)
            workers++;
    }
}

void Basic_information::ShowCTMatching(const char *filename, vector<vector<int>> &CT_Worker, int current_Number_Workers)
{
    int sumtasks = 0;
    int workers = 0;
    ofstream outFile;
    outFile.open(filename, ios::out);

    int mmm = 0;
    for (auto d : CT_Worker)
    {
        if (d.size() == 0)
        {
            mmm++;
            continue;
        }
        outFile << "work:" << mmm++ << "    task:" << endl;
        for (int i = 0; i < d.size(); i++)
        {
            if (i == d.size() - 1)
                outFile << d[i] << endl;
            else
                outFile << d[i] << ",";
            sumtasks++;
        }

        if (d.size() != 0)
            workers++;
    }

    outFile.close();
}

/**ShowCTMatching
 * 有两种方式是因为我懒得修改了。
 * 其中vector<vector<int>>指的全局
 * vector<int> [number]:指的组内
 */

void Basic_information::ShowCTMatching(vector<int> CT_Worker[], int current_Number_Worker)
{
    int sumtasks = 0;
    int workers = 0;

    for (int i = 0; i < current_Number_Worker; i++)
    {

        sumtasks += CT_Worker[i].size();
        if (CT_Worker[i].size() != 0)
            workers++;
    }
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
    vector<double> current_detour_distance[current_Number_Task];
    vector<int> current_poi[current_Number_Task];
    for (int i = 0; i < current_Number_Task; i++)
    {
        current_detour_distance[i] = vector<double>(current_Number_Worker, 0);

        current_poi[i] = vector<int>(current_Number_Worker, 0);
    }

    vector<int> current_CT_Worker[current_Number_Worker];
    double current_MaxDistanceTask[current_Number_Task] = {0};
    double current_task_NeedTime = 0.0;

    computeMaxDitanceTask(current_MaxDistanceTask, current_taskGroup, current_window_endTime);

    vector<int> current_Group_worker_AW;

    for (int i = 0; i < current_Number_Worker; i++)
    {
        current_Group_worker_AW.push_back(i);
    }

    for (int i = 0; i < current_Number_Task; i++)
    {

        if (current_Group_worker_AW.size() != 0)
        {
            int workerid = -1;

            workerid = FindLatestWorkerNew_Greedy(i, current_Group_worker_AW, current_Group_workerAD, current_detour_distance, current_Group_worker_subTrajectoryDis, current_CT_Worker, current_MaxDistanceTask, &current_task_NeedTime, current_taskGroup, current_workerGroup, current_poi, current_window_endTime);

            if (workerid != -1)
            {

                current_workerGroup[workerid].sign = false;
                current_taskGroup[i].sign = false;
                current_CT_Worker[workerid].push_back(i);
                global_CT_Worker[current_workerGroup[workerid].Original_Local].push_back(current_taskGroup[i].Original_Local);

                current_Group_workerAD[workerid] = current_Group_workerAD[workerid] - 2 * current_detour_distance[i][workerid];
                global_workers[current_workerGroup[workerid].Original_Local].ADdis = current_Group_workerAD[workerid];

                UpdateTaskDeadline(current_taskGroup, workerid, i, current_detour_distance, current_CT_Worker, current_poi, current_MaxDistanceTask, current_task_NeedTime);
                if (current_CT_Worker[workerid].size() == Capacity)
                {
                    for (vector<int>::iterator it = current_Group_worker_AW.begin(); it != current_Group_worker_AW.end(); ++it)
                    {
                        if (*it == workerid)
                        {
                            it = current_Group_worker_AW.erase(it);

                            break;
                        }
                    }
                }
            }
            else
            {
            }
        }
        else
            break;
    }

    ShowCTMatching(current_CT_Worker, current_Number_Worker);
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

/**
 * 打印工人和任务的所有信息
 */
void Basic_information::print_info()
{

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

    double task_satis_results;
    double worker_satis_results;

    if (Satisfaction_sign)
    {
        task_satis_results = Caculate_Task_Satisfaction_avg(global_CT_Worker, global_PT);
        worker_satis_results = Caculate_Worker_Satisfaction_avg(global_CT_Worker, global_PW);

        printf_Results_to_txt(task_satis_results, worker_satis_results, "平均", alg_name, run_time);
    }
    else
    {

        task_satis_results = Caculate_Task_Satisfaction_sum(global_CT_Worker, global_PT);
        worker_satis_results = Caculate_Worker_Satisfaction_sum(global_CT_Worker, global_PW);

        printf_Results_to_txt(task_satis_results, worker_satis_results, "求和", alg_name, run_time);
    }
}
void Basic_information::printf_Results_to_txt(double task_satis_results, double worker_satis_results, string sati_name, string alg_name, double run_time)
{

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

    out << "--------------- " << alg_name << sum_repeat << "---------------\n";
    out << "\t任务数：" << Number_Task << "\t 工人数：" << Number_Worker << "\t工人容量:" << Capacity << "\t工人成本/单位" << c << "\t移动速度:" << speed
        << "\t窗口时间：" << Wmax << "\t窗口任务：" << Tmax << "\n";
    out << "任务的" << sati_name << "满意度：\t" << task_satis_results << "\n";
    out << "工人的" << sati_name << "满意度：\t" << worker_satis_results << "\n";
    out << "任务匹配对数：\t" << sumtasks << "\n";
    out << "工人匹配对数：\t" << workers << "\n";
    out << "运行时间(ms):\t" << run_time << "\n";
    out << "\n";
    out.close();

    avg_task_sati += task_satis_results;
    avg_work_sati += worker_satis_results;
    avg_work_match_num += workers;
    avg_task_match_num += sumtasks;
    avg_runtime += run_time;
}
/***
 *
 * 开始workerBatch的代码
 */
void Basic_information::Grouping_Framework_WorkerBatch(vector<TASK> &tasks, vector<WORKER> &workers, double Wmax, int Tmax, vector<double> &Sumdis, vector<vector<double>> &global_Worker_subTrajectoryDis)

{
    vector<CURRENT_TASK_GROUP> current_taskGroup;
    vector<CURRENT_WORKERS_GROUP> current_Group_worker;
    vector<double> current_Group_workerSumdis;
    vector<double> current_Group_workerAD;
    double current_window_endTime = tasks[0].startTime + Wmax;
    double current_window_startTime = tasks[0].startTime;
    int current_workID = global_Current_workID;

    vector<vector<double>> current_Group_worker_subTrajectoryDis;

    int sign = 0;
    int temp = 0;

    for (int i = 0; i < tasks.size(); i++)
    {

        TASK task = tasks[i];
        sign++;

        CURRENT_TASK_GROUP taskg;
        taskg.task = task;
        taskg.Original_Local = i;
        current_taskGroup.push_back(taskg);

        if ((sign - temp == Tmax) || (task.startTime >= current_window_endTime))
        {
            temp = current_taskGroup.size();
            current_workID = global_Current_workID;
            current_window_endTime = task.startTime;

            groupWork_according_TaskGroup(workers, Sumdis, current_Group_worker, current_Group_workerSumdis, current_window_endTime, current_workID, current_Group_workerAD, global_Worker_subTrajectoryDis, current_Group_worker_subTrajectoryDis);
            determine_Window_Task_Timeout(current_taskGroup, current_window_endTime);

            match_WorkerTask_workerBatch(current_taskGroup, current_Group_worker, current_window_endTime, current_Group_workerAD, current_Group_worker_subTrajectoryDis);

            current_window_startTime = current_window_endTime;
            current_window_endTime += Wmax;

            updata_current_Info(current_taskGroup, current_Group_worker, current_Group_workerAD, current_Group_worker_subTrajectoryDis, current_Group_workerSumdis);
        }
    }

    if (current_taskGroup.size() > 0)
    {

        current_workID = global_Current_workID;
        current_window_startTime = current_taskGroup.back().task.startTime;

        groupWork_according_TaskGroup(workers, Sumdis, current_Group_worker, current_Group_workerSumdis, current_window_endTime, current_workID, current_Group_workerAD, global_Worker_subTrajectoryDis, current_Group_worker_subTrajectoryDis);

        determine_Window_Task_Timeout(current_taskGroup, current_window_endTime);

        match_WorkerTask_workerBatch(current_taskGroup, current_Group_worker, current_window_endTime, current_Group_workerAD, current_Group_worker_subTrajectoryDis);
    }
}

/**
 * 求解分组内工人和任务的偏好
 */
void Basic_information::Compute_PTPW_Group_workerBatch(vector<vector<pair<int, double>>> &current_PT, vector<vector<pair<int, double>>> &current_PW, vector<double> current_detour_distance[], vector<CURRENT_TASK_GROUP> &current_taskGroup, vector<CURRENT_WORKERS_GROUP> &current_workerGroup, vector<int> current_poi[])
{

    int Number_current_taskGroup = current_taskGroup.size();
    int Number_current_workerGroup = current_workerGroup.size();
    for (int i = 0; i < Number_current_taskGroup; i++)
    {
        for (int j = 0; j < Number_current_workerGroup; j++)
        {
            if (!(current_taskGroup[i].task.Deadline < current_workerGroup[j].worker.startTime || current_taskGroup[i].task.startTime > current_workerGroup[j].worker.endTime))
            {
                if (current_taskGroup[i].task.Minscore <= current_workerGroup[j].worker.score)
                {

                    current_detour_distance[i][j] = Caculate_mindist(j, i, current_poi, current_taskGroup, current_workerGroup);

                    if (current_detour_distance[i][j] <= current_workerGroup[j].worker.range)
                    {

                        double preference1 = current_taskGroup[i].task.Reward - 2 * current_detour_distance[i][j] * c;
                        if (preference1 > 0)
                        {
                            current_PW[j].push_back(make_pair(i, preference1));
                        }

                        double preference2 = current_workerGroup[j].worker.score;
                        if (current_taskGroup[i].task.Minscore <= preference2)
                        {
                            current_PT[i].push_back(make_pair(j, preference2));
                        }
                    }
                }
            }
        }
    }

    for (int i = 0; i < current_workerGroup.size(); i++)
    {

        sort(current_PW[i].begin(), current_PW[i].end(), cmp);
    }

    for (int i = 0; i < current_taskGroup.size(); i++)
    {

        sort(current_PT[i].begin(), current_PT[i].end(), cmp);
    }
}

void Basic_information::match_WorkerTask_workerBatch(vector<CURRENT_TASK_GROUP> &current_taskGroup, vector<CURRENT_WORKERS_GROUP> &current_workerGroup, double current_window_endTime, vector<double> &current_Group_workerAD, vector<vector<double>> &current_Group_worker_subTrajectoryDis)
{
    int current_Number_Worker = current_workerGroup.size();
    int current_Number_Task = current_taskGroup.size();

    vector<double> current_detour_distance[current_Number_Task];
    vector<int> current_poi[current_Number_Task];
    vector<vector<pair<int, double>>> current_PT(current_Number_Task);
    vector<vector<pair<int, double>>> current_PW(current_Number_Worker);
    for (int i = 0; i < current_Number_Task; i++)
    {
        current_detour_distance[i] = vector<double>(current_Number_Worker, 0);

        current_poi[i] = vector<int>(current_Number_Worker, 0);
    }
    Compute_PTPW_Group_workerBatch(current_PT, current_PW, current_detour_distance, current_taskGroup, current_workerGroup, current_poi);

    vector<int> current_CT_Worker[current_Number_Worker];
    double current_MaxDistanceTask[current_Number_Task];
    double current_task_NeedTime = 0.0;

    computeMaxDitanceTask(current_MaxDistanceTask, current_taskGroup, current_window_endTime);

    /**
     * 迭代匹配
     * 工人批量时，工人不可用或者任务不可用停止循环匹配
     * 未匹配或匹配失败的批次工人优先匹配
     * 偏好列表ORDER
     */

    iterator_Match_WorkBatch(current_workerGroup, current_taskGroup, current_PW, current_PT, current_Group_workerAD, current_detour_distance, current_Group_worker_subTrajectoryDis,
                             current_CT_Worker, current_MaxDistanceTask, &current_task_NeedTime, current_poi, current_window_endTime);

    ShowCTMatching(current_CT_Worker, current_Number_Worker);
}

void Basic_information::iterator_Match_WorkBatch(vector<CURRENT_WORKERS_GROUP> &current_workerGroup, vector<CURRENT_TASK_GROUP> &current_taskGroup, vector<vector<pair<int, double>>> &current_PW, vector<vector<pair<int, double>>> &current_PT,
                                                 vector<double> &current_Group_workerAD, vector<double> current_detour_distance[], vector<vector<double>> &current_Group_worker_subTrajectoryDis,
                                                 vector<int> current_CT_Worker[], double MaxDistanceTask[], double *current_task_NeedTime, vector<int> current_poi[], double nowtime)
{
    int Number_Worker = current_workerGroup.size();
    int Number_Task = current_taskGroup.size();
    int Worker_Available[Number_Worker] = {0};
    int Task_Available[Number_Task] = {0};
    int count_NAWorker = 0;
    int count_NATask = 0;
    int CurrentTaskInPW[Number_Worker] = {0};
    int unmatchC = 0;
    for (int i = 0; i < current_workerGroup.size(); i++)
    {
        if (current_PW[i].size() == 0)
        {
            Worker_Available[i] = 3;
            unmatchC++;
        }
        else
        {
        }
    }
    int diedai = 0;
    while ((count_NAWorker + unmatchC) < Number_Worker && count_NATask < Number_Task)
    {

        diedai++;

        vector<int> updateObj;

        for (int flag = 0; flag < 2; flag++)
        {
            for (int i = 0; i < Number_Worker; i++)

            {

                if (Worker_Available[i] == flag)
                {

                    int orderinPW = CurrentTaskInPW[i];

                    int current_task_id = current_PW[i][orderinPW].first;

                    CurrentTaskInPW[i]++;
                    if (Task_Available[current_task_id] == 0)
                    /*
                    && (find_if(current_PT[current_task_id].begin(), current_PT[current_task_id].end(), [i](const std::pair<int, double> &p)
                                                                         { return p.second == i; }) != current_PT[current_task_id].end())

                    */
                    {
                        double ttttt = 0;

                        if (CurrentTask_Satisfy(current_taskGroup, i, current_task_id, current_Group_workerAD, current_detour_distance, current_Group_worker_subTrajectoryDis, current_CT_Worker, current_poi, MaxDistanceTask, current_task_NeedTime, nowtime))
                        {

                            Task_Available[current_task_id] = 1;
                            count_NATask++;
                            current_CT_Worker[i].push_back(current_task_id);

                            current_workerGroup[i].sign = false;
                            current_taskGroup[current_task_id].sign = false;
                            global_CT_Worker[current_workerGroup[i].Original_Local].push_back(current_taskGroup[current_task_id].Original_Local);

                            current_Group_workerAD[i] = current_Group_workerAD[i] - 2 * current_detour_distance[current_task_id][i];

                            global_workers[current_workerGroup[i].Original_Local].ADdis = current_Group_workerAD[i];
                            UpdateTaskDeadline(current_taskGroup, i, current_task_id, current_detour_distance, current_CT_Worker, current_poi, MaxDistanceTask, *current_task_NeedTime);

                            if (current_CT_Worker[i].size() == Capacity)

                            {
                                count_NAWorker++;
                                Worker_Available[i] = 3;
                            }

                            else if (flag == 0)
                            {
                                updateObj.push_back(i);
                            }
                        }
                        else if (flag == 0)
                        {
                        }
                        else if (flag == 1)
                        {
                            Worker_Available[i] = 0;
                        }
                    }
                    else if (flag == 1)
                    {
                        Worker_Available[i] = 0;
                    }

                    if (CurrentTaskInPW[i] == current_PW[i].size())
                    {
                        if (Worker_Available[i] != 3)
                        {
                            count_NAWorker++;
                            Worker_Available[i] = 3;
                        }
                    }
                    else
                    {
                    }
                }
            }
        }

        for (int i = 0; i < updateObj.size(); i++)
        {
            int updateworkerid = updateObj[i];
            if (Worker_Available[updateworkerid] != 3)
            {
                Worker_Available[updateworkerid] = 1;
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
    vector<CURRENT_TASK_GROUP> current_taskGroup;
    vector<CURRENT_WORKERS_GROUP> current_Group_worker;
    vector<double> current_Group_workerSumdis;
    vector<double> current_Group_workerAD;
    double current_window_endTime = tasks[0].startTime + Wmax;
    double current_window_startTime = tasks[0].startTime;
    int current_workID = global_Current_workID;

    vector<vector<double>> current_Group_worker_subTrajectoryDis;

    int sign = 0;
    int temp = 0;

    for (int i = 0; i < tasks.size(); i++)
    {

        TASK task = tasks[i];

        sign++;

        CURRENT_TASK_GROUP taskg;
        taskg.task = task;
        taskg.Original_Local = i;
        current_taskGroup.push_back(taskg);

        if ((sign - temp == Tmax) || (task.startTime >= current_window_endTime))
        {
            temp = current_taskGroup.size();
            current_workID = global_Current_workID;

            current_window_endTime = task.startTime;

            groupWork_according_TaskGroup(workers, Sumdis, current_Group_worker, current_Group_workerSumdis, current_window_endTime, current_workID, current_Group_workerAD, global_Worker_subTrajectoryDis, current_Group_worker_subTrajectoryDis);
            determine_Window_Task_Timeout(current_taskGroup, current_window_endTime);

            match_WorkerTask_TPPG(current_taskGroup, current_Group_worker, current_window_endTime, current_Group_workerAD, current_Group_worker_subTrajectoryDis);

            current_window_startTime = current_window_endTime;
            current_window_endTime += Wmax;
        }
        updata_current_Info(current_taskGroup, current_Group_worker, current_Group_workerAD, current_Group_worker_subTrajectoryDis, current_Group_workerSumdis);
    }

    if (current_taskGroup.size() > 0)
    {

        current_workID = global_Current_workID;
        current_window_startTime = current_taskGroup.back().task.startTime;

        groupWork_according_TaskGroup(workers, Sumdis, current_Group_worker, current_Group_workerSumdis, current_window_endTime, current_workID, current_Group_workerAD, global_Worker_subTrajectoryDis, current_Group_worker_subTrajectoryDis);

        determine_Window_Task_Timeout(current_taskGroup, current_window_endTime);

        match_WorkerTask_TPPG(current_taskGroup, current_Group_worker, current_window_endTime, current_Group_workerAD, current_Group_worker_subTrajectoryDis);
    }
}

void Basic_information::match_WorkerTask_TPPG(vector<CURRENT_TASK_GROUP> &current_taskGroup, vector<CURRENT_WORKERS_GROUP> &current_workerGroup, double current_window_endTime, vector<double> &current_Group_workerAD, vector<vector<double>> &current_Group_worker_subTrajectoryDis)
{
    int current_Number_Worker = current_workerGroup.size();
    int current_Number_Task = current_taskGroup.size();

    vector<double> current_detour_distance[current_Number_Task];
    vector<int> poi[current_Number_Task];
    for (int i = 0; i < current_Number_Task; i++)
    {
        current_detour_distance[i] = vector<double>(current_Number_Worker, 0);

        poi[i] = vector<int>(current_Number_Worker, 0);
    }

    vector<int> current_CT_Worker[current_Number_Worker];
    double current_MaxDistanceTask[current_Number_Task];
    double current_task_NeedTime = 0.0;

    computeMaxDitanceTask(current_MaxDistanceTask, current_taskGroup, current_window_endTime);

    vector<int> current_Group_worker_AW;

    for (int i = 0; i < current_Number_Worker; i++)
    {
        current_Group_worker_AW.push_back(i);
    }

    for (int i = 0; i < current_Number_Task; i++)
    {

        if (current_Group_worker_AW.size() != 0)
        {
            int workerid = -1;

            workerid = FindPreferedWorkerNew_TPPG(i, current_Group_worker_AW, current_Group_workerAD, current_detour_distance, current_Group_worker_subTrajectoryDis, current_CT_Worker, current_MaxDistanceTask, &current_task_NeedTime, current_taskGroup, current_workerGroup, poi, current_window_endTime);

            if (workerid != -1)
            {

                current_workerGroup[workerid].sign = false;
                current_taskGroup[i].sign = false;
                current_CT_Worker[workerid].push_back(i);
                global_CT_Worker[current_workerGroup[workerid].Original_Local].push_back(current_taskGroup[i].Original_Local);

                current_Group_workerAD[workerid] = current_Group_workerAD[workerid] - 2 * current_detour_distance[i][workerid];

                global_workers[current_workerGroup[workerid].Original_Local].ADdis = current_Group_workerAD[workerid];
                UpdateTaskDeadline(current_taskGroup, workerid, i, current_detour_distance, current_CT_Worker, poi, current_MaxDistanceTask, current_task_NeedTime);

                if (current_CT_Worker[workerid].size() == Capacity)
                {
                    for (vector<int>::iterator it = current_Group_worker_AW.begin(); it != current_Group_worker_AW.end(); ++it)
                    {
                        if (*it == workerid)
                        {
                            it = current_Group_worker_AW.erase(it);

                            break;
                        }
                    }
                }
            }
            else
            {
            }
        }
        else
            break;
    }
    ShowCTMatching(current_CT_Worker, current_Number_Worker);
}

int Basic_information::FindPreferedWorkerNew_TPPG(int taskid, vector<int> &current_Group_worker_AW, vector<double> &current_Group_workerAD, vector<double> current_detour_distance[], vector<vector<double>> &current_Group_worker_subTrajectoryDis, vector<int> CT_Worker[], double MaxDistanceTask[], double *current_task_NeedTime, vector<CURRENT_TASK_GROUP> &current_taskGroup, vector<CURRENT_WORKERS_GROUP> &current_workerGroup, vector<int> poi[], double nowtime)
{

    double tttttt = 0;
    int best_workerid = -1;
    double maxpre = -__DBL_MAX__;
    double best_distance = 0;
    for (int j = 0; j < current_Group_worker_AW.size(); j++)
    {
        int workerid = current_Group_worker_AW[j];
        if (current_taskGroup[taskid].task.Minscore <= current_workerGroup[workerid].worker.score)
        {
            double preference = current_workerGroup[workerid].worker.score;
            if (preference > maxpre)
            {

                double dist = Caculate_mindist(workerid, taskid, poi, current_taskGroup, current_workerGroup);

                current_detour_distance[taskid][workerid] = dist;

                if (current_workerGroup[workerid].worker.range >= dist)
                {

                    if (current_taskGroup[taskid].task.Reward - (2 * dist * c) > 0)
                    {
                        if (CurrentTask_Satisfy(current_taskGroup, workerid, taskid, current_Group_workerAD, current_detour_distance, current_Group_worker_subTrajectoryDis, CT_Worker, poi, MaxDistanceTask, current_task_NeedTime, nowtime))

                        {

                            maxpre = preference;
                            best_workerid = workerid;
                            best_distance = dist;
                            tttttt = *current_task_NeedTime;
                        }
                    }
                }
            }
        }
    }
    *current_task_NeedTime = tttttt;
    return best_workerid;
}

/***
 * TPPG_batch算法
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
void Basic_information::Grouping_Framework_TPPG_Batch(vector<TASK> &tasks, vector<WORKER> &workers, double Wmax, int Tmax, vector<double> &Sumdis, vector<vector<double>> &global_Worker_subTrajectoryDis)

{
    vector<CURRENT_TASK_GROUP> current_taskGroup;
    vector<CURRENT_WORKERS_GROUP> current_Group_worker;
    vector<double> current_Group_workerSumdis;
    vector<double> current_Group_workerAD;
    double current_window_endTime = tasks[0].startTime + Wmax;
    double current_window_startTime = tasks[0].startTime;
    int current_workID = global_Current_workID;

    vector<vector<double>> current_Group_worker_subTrajectoryDis;

    int sign = 0;
    int temp = 0;

    for (int i = 0; i < tasks.size(); i++)
    {

        TASK task = tasks[i];
        sign++;

        CURRENT_TASK_GROUP taskg;
        taskg.task = task;
        taskg.Original_Local = i;
        current_taskGroup.push_back(taskg);

        if ((sign - temp == Tmax) || (task.startTime >= current_window_endTime))
        {
            temp = current_taskGroup.size();

            current_workID = global_Current_workID;

            current_window_endTime = task.startTime;

            groupWork_according_TaskGroup(workers, Sumdis, current_Group_worker, current_Group_workerSumdis, current_window_endTime, current_workID, current_Group_workerAD, global_Worker_subTrajectoryDis, current_Group_worker_subTrajectoryDis);
            determine_Window_Task_Timeout(current_taskGroup, current_window_endTime);

            match_WorkerTask_TPPG_Batch(current_taskGroup, current_Group_worker, current_window_endTime, current_Group_workerAD, current_Group_worker_subTrajectoryDis);

            current_window_startTime = current_window_endTime;
            current_window_endTime += Wmax;
        }
        updata_current_Info(current_taskGroup, current_Group_worker, current_Group_workerAD, current_Group_worker_subTrajectoryDis, current_Group_workerSumdis);
    }

    if (current_taskGroup.size() > 0)
    {

        current_workID = global_Current_workID;
        current_window_startTime = current_taskGroup.back().task.startTime;

        groupWork_according_TaskGroup(workers, Sumdis, current_Group_worker, current_Group_workerSumdis, current_window_endTime, current_workID, current_Group_workerAD, global_Worker_subTrajectoryDis, current_Group_worker_subTrajectoryDis);

        determine_Window_Task_Timeout(current_taskGroup, current_window_endTime);

        match_WorkerTask_TPPG_Batch(current_taskGroup, current_Group_worker, current_window_endTime, current_Group_workerAD, current_Group_worker_subTrajectoryDis);
    }
}

void Basic_information::match_WorkerTask_TPPG_Batch(vector<CURRENT_TASK_GROUP> &current_taskGroup, vector<CURRENT_WORKERS_GROUP> &current_workerGroup, double current_window_endTime, vector<double> &current_Group_workerAD, vector<vector<double>> &current_Group_worker_subTrajectoryDis)
{
    int current_Number_Worker = current_workerGroup.size();
    int current_Number_Task = current_taskGroup.size();

    vector<double> current_detour_distance[current_Number_Task];
    vector<int> current_poi[current_Number_Task];
    vector<vector<pair<int, double>>> current_PT(Number_Task);
    vector<vector<pair<int, double>>> current_PW(Number_Worker);

    int current_CW_Task[current_Number_Task];
    int current_num_of_chased_worker[current_Number_Task] = {0};
    vector<int> current_ActiveTask;
    vector<int> current_NextActiveTask;

    vector<int> current_CT_Worker[current_Number_Worker];
    double current_MaxDistanceTask[current_Number_Task] = {0.0};
    double current_task_NeedTime = 0.0;

    for (int i = 0; i < current_Number_Task; i++)
    {
        current_detour_distance[i] = vector<double>(current_Number_Worker, 0);

        current_poi[i] = vector<int>(current_Number_Worker, 0);
    }
    Compute_PTPW_Group_workerBatch(current_PT, current_PW, current_detour_distance, current_taskGroup, current_workerGroup, current_poi);

    for (int i = 0; i < current_taskGroup.size(); i++)
    {
        if (current_PT[i].size() != 0)
        {
            current_ActiveTask.push_back(i);
            current_NextActiveTask.push_back(i);
        }
        current_CW_Task[i] = -1;
    }
    if (current_ActiveTask.empty())
    {
        return;
    }

    vector<vector<int>> workerWasRequest(Number_Worker, vector<int>(0));
    vector<int> workerRID;

    computeMaxDitanceTask(current_MaxDistanceTask, current_taskGroup, current_window_endTime);
    int current_chaseNum = 0;
    int times = 0;
    while (!current_ActiveTask.empty())
    {
        ++times;

        /**
         * 每个任务开始进行匹配请求
         */

        for (int i = 0; i < current_ActiveTask.size(); i++)
        {
            int taskid = current_ActiveTask[i];
            if ((current_chaseNum) >= current_PT[taskid].size())
            {

                current_NextActiveTask.erase(find(current_NextActiveTask.begin(), current_NextActiveTask.end(), current_ActiveTask[i]));
                continue;
            }
            int workid = current_PT[taskid][current_chaseNum].first;

            if (current_CT_Worker[workid].size() >= Capacity)
            {
                continue;
            }

            if (workerWasRequest[workid].empty())
            {
                workerRID.push_back(workid);
            }

            workerWasRequest[workid].push_back(taskid);
        }

        for (int j = 0; j < workerRID.size(); j++)
        {
            int workerid = workerRID[j];

            /**
             * 1
             * 下面根据工人的偏好列表
             * 对发出请求的工人进行排序
             * workerWasRequest最后得到的是排好序的任务请求列表
             */

            std::unordered_map<int, int> index_map;
            for (int i = 0; i < current_PW[workerid].size(); ++i)
            {
                index_map[current_PW[workerid][i].first] = i;
            }

            std::sort(workerWasRequest[workerid].begin(), workerWasRequest[workerid].end(),
                      [&index_map](const int &left, const int &right)
                      {
                          return index_map[left] < index_map[right];
                      });
            /**
             * 2
             * 拍完顺序开始匹配判断是否满足约束条件
             * 若满足，则task删除。
             * 若工人匹配出超过容量，则删除
             */

            for (int k = 0; k < workerWasRequest[workerid].size(); k++)
            {

                int taskIIID = workerWasRequest[workerid][k];

                if (current_taskGroup[taskIIID].task.Minscore <= current_workerGroup[workerid].worker.score)
                {
                    double preference = current_workerGroup[workerid].worker.score;

                    if (current_CT_Worker[workerid].size() >= Capacity)
                    {
                        break;
                    }
                    if (CurrentTask_Satisfy(current_taskGroup, workerid, taskIIID, current_Group_workerAD, current_detour_distance, current_Group_worker_subTrajectoryDis, current_CT_Worker, current_poi, current_MaxDistanceTask, &current_task_NeedTime, current_window_endTime))

                    {

                        current_workerGroup[workerid].sign = false;
                        current_taskGroup[taskIIID].sign = false;

                        current_CT_Worker[workerid].push_back(taskIIID);
                        global_CT_Worker[current_workerGroup[workerid].Original_Local].push_back(current_taskGroup[taskIIID].Original_Local);

                        current_NextActiveTask.erase(find(current_NextActiveTask.begin(), current_NextActiveTask.end(), taskIIID));

                        Update_AD1(workerid, taskIIID, current_Group_workerAD, current_detour_distance);
                        global_workers[current_workerGroup[workerid].Original_Local].ADdis = current_Group_workerAD[workerid];
                        UpdateTaskDeadline(current_taskGroup, workerid, taskIIID, current_detour_distance, current_CT_Worker, current_poi, current_MaxDistanceTask, current_task_NeedTime);
                    }
                }
            }
            workerWasRequest[workerid].clear();
            workerWasRequest[workerid].resize(0);
        }
        current_ActiveTask.clear();
        current_ActiveTask.resize(current_NextActiveTask.size());

        for (int i = 0; i < current_NextActiveTask.size(); i++)
            current_ActiveTask[i] = current_NextActiveTask[i];

        workerWasRequest.clear();
        workerWasRequest.resize(current_Number_Worker, vector<int>(0));

        workerRID.clear();
        workerRID.resize(0);
        current_chaseNum++;
    }

    ShowCTMatching(current_CT_Worker, current_Number_Worker);
}

/**
 * 4.4TSDA
 */

void Basic_information::Grouping_Framework_TSDA(vector<TASK> &tasks, vector<WORKER> &workers, double Wmax, int Tmax, vector<double> &Sumdis, vector<vector<double>> &global_Worker_subTrajectoryDis)

{
    vector<CURRENT_TASK_GROUP> current_taskGroup;
    vector<CURRENT_WORKERS_GROUP> current_Group_worker;
    vector<double> current_Group_workerSumdis;
    vector<double> current_Group_workerAD;
    double current_window_endTime = tasks[0].startTime + Wmax;
    double current_window_startTime = tasks[0].startTime;
    int current_workID = global_Current_workID;

    vector<vector<double>> current_Group_worker_subTrajectoryDis;

    int sign = 0;
    int temp = 0;

    for (int i = 0; i < tasks.size(); i++)
    {

        TASK task = tasks[i];
        sign++;

        CURRENT_TASK_GROUP taskg;
        taskg.task = task;
        taskg.Original_Local = i;
        current_taskGroup.push_back(taskg);

        if ((sign - temp == Tmax) || (task.startTime >= current_window_endTime))
        {
            temp = current_taskGroup.size();
            current_workID = global_Current_workID;
            current_window_endTime = task.startTime;

            groupWork_according_TaskGroup(workers, Sumdis, current_Group_worker, current_Group_workerSumdis, current_window_endTime, current_workID, current_Group_workerAD, global_Worker_subTrajectoryDis, current_Group_worker_subTrajectoryDis);
            determine_Window_Task_Timeout(current_taskGroup, current_window_endTime);

            match_WorkerTask_TSDA(current_taskGroup, current_Group_worker, current_window_endTime, current_Group_workerAD, current_Group_worker_subTrajectoryDis);

            current_window_startTime = current_window_endTime;
            current_window_endTime += Wmax;

            updata_current_Info(current_taskGroup, current_Group_worker, current_Group_workerAD, current_Group_worker_subTrajectoryDis, current_Group_workerSumdis);
        }
    }

    if (current_taskGroup.size() > 0)
    {

        current_workID = global_Current_workID;
        current_window_startTime = current_taskGroup.back().task.startTime;

        groupWork_according_TaskGroup(workers, Sumdis, current_Group_worker, current_Group_workerSumdis, current_window_endTime, current_workID, current_Group_workerAD, global_Worker_subTrajectoryDis, current_Group_worker_subTrajectoryDis);

        determine_Window_Task_Timeout(current_taskGroup, current_window_endTime);

        match_WorkerTask_TSDA(current_taskGroup, current_Group_worker, current_window_endTime, current_Group_workerAD, current_Group_worker_subTrajectoryDis);
    }
}

void Basic_information::match_WorkerTask_TSDA(vector<CURRENT_TASK_GROUP> &current_taskGroup, vector<CURRENT_WORKERS_GROUP> &current_workerGroup, double current_window_endTime, vector<double> &current_Group_workerAD, vector<vector<double>> &current_Group_worker_subTrajectoryDis)
{
    int current_Number_Worker = current_workerGroup.size();
    int current_Number_Task = current_taskGroup.size();

    vector<double> current_detour_distance[current_Number_Task];
    vector<int> current_poi[current_Number_Task];
    vector<vector<pair<int, double>>> current_PT(Number_Task);
    vector<vector<pair<int, double>>> current_PW(Number_Worker);

    int current_CW_Task[current_Number_Task];
    int current_num_of_chased_worker[current_Number_Task] = {0};

    vector<int> current_ActiveTask;
    vector<int> current_NextActiveTask(current_ActiveTask);

    vector<vector<int>> current_CT_Worker(current_Number_Worker);
    double current_MaxDistanceTask[current_Number_Task] = {0.0};
    double current_task_NeedTime = 0.0;

    for (int i = 0; i < current_Number_Task; i++)
    {
        current_detour_distance[i] = vector<double>(current_Number_Worker, 0);

        current_poi[i] = vector<int>(current_Number_Worker, 0);
    }

    Compute_PTPW_Group_workerBatch(current_PT, current_PW, current_detour_distance, current_taskGroup, current_workerGroup, current_poi);
    for (int i = 0; i < current_taskGroup.size(); i++)
    {
        if (current_PT[i].size() != 0)
        {
            current_ActiveTask.push_back(i);
        }
        current_CW_Task[i] = -1;
    }
    if (current_ActiveTask.empty())
    {
        return;
    }

    computeMaxDitanceTask(current_MaxDistanceTask, current_taskGroup, current_window_endTime);

    /**
     * 迭代匹配
     * 工人批量时，工人不可用或者任务不可用停止循环匹配
     * 未匹配或匹配失败的批次工人优先匹配
     * 偏好列表ORDER
     */

    while (!current_ActiveTask.empty())
    {

        current_NextActiveTask.clear();

        for (int i = 0; i < current_ActiveTask.size(); i++)
            current_NextActiveTask.push_back(current_ActiveTask[i]);

        int matchingnumber = 0, replacematching = 0;
        for (int i = 0; i < current_ActiveTask.size(); i++)
        {
            int taskid = current_ActiveTask[i];
            int order = current_num_of_chased_worker[taskid];
            int worker_to_chase = current_PT[taskid][order].first;

            current_num_of_chased_worker[taskid] = order + 1;

            if (IFTaskExist(worker_to_chase, taskid, current_PW) != current_PW[worker_to_chase].end())
            {

                if (CurrentTask_Satisfy_TSDA(current_taskGroup, worker_to_chase, taskid, current_Group_workerAD, current_detour_distance, current_Group_worker_subTrajectoryDis, current_CT_Worker, current_poi, current_MaxDistanceTask, &current_task_NeedTime, current_window_endTime))

                {

                    current_CW_Task[taskid] = worker_to_chase;
                    current_CT_Worker[worker_to_chase].push_back(taskid);

                    current_NextActiveTask.erase(find(current_NextActiveTask.begin(), current_NextActiveTask.end(), taskid));

                    Update_AD1(worker_to_chase, taskid, current_Group_workerAD, current_detour_distance);

                    global_workers[current_workerGroup[worker_to_chase].Original_Local].ADdis = current_Group_workerAD[worker_to_chase];

                    UpdateTaskDeadline_TSDA(current_taskGroup, false, 0, worker_to_chase, taskid, current_detour_distance, current_CT_Worker, current_poi, current_MaxDistanceTask, current_task_NeedTime);
                    matchingnumber++;
                }
                else
                {

                    int MinReplaceTask = FindReplaceTaskNew_TSDA(worker_to_chase, taskid, current_PW, current_Group_workerAD, current_detour_distance, current_Group_worker_subTrajectoryDis, current_CT_Worker, current_poi, current_MaxDistanceTask, &current_task_NeedTime, current_taskGroup, current_window_endTime);
                    if (MinReplaceTask != -1)
                    {
                        current_CW_Task[taskid] = worker_to_chase;
                        current_CW_Task[MinReplaceTask] = -1;
                        current_CT_Worker[worker_to_chase].push_back(taskid);

                        current_CT_Worker[worker_to_chase].erase(find(current_CT_Worker[worker_to_chase].begin(), current_CT_Worker[worker_to_chase].end(), MinReplaceTask));

                        if (current_num_of_chased_worker[MinReplaceTask] < current_PT[MinReplaceTask].size())
                            current_NextActiveTask.push_back(MinReplaceTask);
                        current_NextActiveTask.erase(find(current_NextActiveTask.begin(), current_NextActiveTask.end(), taskid));

                        Update_AD2(worker_to_chase, taskid, MinReplaceTask, current_Group_workerAD, current_detour_distance);
                        global_workers[current_workerGroup[worker_to_chase].Original_Local].ADdis = current_Group_workerAD[worker_to_chase];

                        UpdateTaskDeadline_TSDA(current_taskGroup, true, MinReplaceTask, worker_to_chase, taskid, current_detour_distance, current_CT_Worker, current_poi, current_MaxDistanceTask, current_task_NeedTime);

                        current_MaxDistanceTask[MinReplaceTask] = current_taskGroup[MinReplaceTask].task.Deadline * speed;
                        global_MaxDistanceTask[current_taskGroup[MinReplaceTask].Original_Local] = current_MaxDistanceTask[MinReplaceTask];

                        replacematching++;
                    }
                    else
                    {
                    }
                }
            }
            if (current_num_of_chased_worker[taskid] == current_PT[taskid].size())
            {

                vector<int>::iterator iter1 = find(current_NextActiveTask.begin(), current_NextActiveTask.end(), taskid);
                if (iter1 != current_NextActiveTask.end())
                {
                    current_NextActiveTask.erase(find(current_NextActiveTask.begin(), current_NextActiveTask.end(), taskid));
                }
            }
        }

        current_ActiveTask.clear();
        for (int i = 0; i < current_NextActiveTask.size(); i++)
            current_ActiveTask.push_back(current_NextActiveTask[i]);
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

    ShowCTMatching(current_CT_Worker, current_Number_Worker);
}

vector<pair<int, double>>::iterator Basic_information::IFTaskExist(int workerid, int taskid, vector<vector<pair<int, double>>> &PW)
{

    vector<pair<int, double>>::iterator iter = PW[workerid].begin();
    for (iter; iter < PW[workerid].end(); iter++)
        if ((*iter).first == taskid)
            break;
    return iter;
}

void Basic_information::Update_AD1(int workerid, int taskid, vector<double> &AD, vector<double> current_detour_distance[])
{
    double dd = 2 * current_detour_distance[taskid][workerid];
    AD[workerid] = AD[workerid] - dd;
}

void Basic_information::Update_AD2(int workerid, int taskid, int MinReplaceTask, vector<double> &AD, vector<double> detour_distance[])
{

    double dd = 2 * detour_distance[taskid][workerid];
    double DD = 2 * detour_distance[MinReplaceTask][workerid];
    AD[workerid] = AD[workerid] - dd + DD;
}

void Basic_information::UpdateTaskDeadline_TSDA(vector<CURRENT_TASK_GROUP> &current_taskGroup, bool replace, int replaceid, int workerid, int taskid, vector<double> current_detour_distance[], vector<vector<int>> &CT_Worker, vector<int> poi[], double MaxDistanceTask[], double current_task_NeedTime)
{
    int replacepoi = 0;
    if (replace)
    {
        replacepoi = poi[replaceid][workerid];
    }

    for (int i = 0; i < CT_Worker[workerid].size(); i++)
    {
        int assigntask_id = CT_Worker[workerid][i];
        int assignedpoint = poi[assigntask_id][workerid];
        int detourpoint = poi[taskid][workerid];

        if (assigntask_id == taskid)
        {

            MaxDistanceTask[taskid] = MaxDistanceTask[taskid] - current_task_NeedTime * speed;

            global_MaxDistanceTask[current_taskGroup[taskid].Original_Local] = MaxDistanceTask[taskid];
        }
        else
        {
            if (detourpoint < assignedpoint)
            {

                MaxDistanceTask[assigntask_id] = MaxDistanceTask[assigntask_id] - (2 * current_detour_distance[taskid][workerid]);

                global_MaxDistanceTask[current_taskGroup[assigntask_id].Original_Local] = MaxDistanceTask[assigntask_id];
            }
            if (replace == true && replacepoi < assignedpoint)
            {

                MaxDistanceTask[assigntask_id] = MaxDistanceTask[assigntask_id] + (2 * current_detour_distance[replaceid][workerid]);
                global_MaxDistanceTask[current_taskGroup[assigntask_id].Original_Local] = MaxDistanceTask[assigntask_id];
            }
        }
    }
}

int Basic_information::FindReplaceTaskNew_TSDA(int workerid, int taskid, vector<vector<pair<int, double>>> &PW, vector<double> &AD, vector<double> detour_distance[], vector<vector<double>> &Worker_subTrajectoryDis, vector<vector<int>> &CT_Worker, vector<int> poi[], double MaxDistanceTask[], double *current_task_NeedTime, vector<CURRENT_TASK_GROUP> &current_taskGroup, double nowtime)
{
    int reTaskindex = -1;
    int replacetaskid = -1;
    double minProfit = __DBL_MAX__;
    int t1 = GetIndex_PW(workerid, taskid, PW);

    double SumdetouDis = 0.0;
    int detourpoint = poi[taskid][workerid];
    for (vector<int>::iterator it = CT_Worker[workerid].begin(); it != CT_Worker[workerid].end(); it++)
    {
        int assignpoi = poi[*it][workerid];
        if (assignpoi <= detourpoint)
        {
            SumdetouDis = SumdetouDis + 2 * detour_distance[*it][workerid];
        }
    }

    for (vector<int>::iterator it = CT_Worker[workerid].begin(); it != CT_Worker[workerid].end(); it++)
    {
        int t2 = GetIndex_PW(workerid, *it, PW);
        if (t1 < t2)
        {
            if (IfReplace(workerid, taskid, *it, AD, detour_distance, Worker_subTrajectoryDis, CT_Worker, poi, MaxDistanceTask, current_task_NeedTime, SumdetouDis, current_taskGroup, nowtime))

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

void Basic_information::Grouping_Framework_WPPG(vector<TASK> &tasks, vector<WORKER> &workers, double Wmax, int Tmax, vector<double> &Sumdis, vector<vector<double>> &global_Worker_subTrajectoryDis)

{
    vector<CURRENT_TASK_GROUP> current_taskGroup;
    vector<CURRENT_WORKERS_GROUP> current_Group_worker;
    vector<double> current_Group_workerSumdis;
    vector<double> current_Group_workerAD;
    double current_window_endTime = tasks[0].startTime + Wmax;
    double current_window_startTime = tasks[0].startTime;
    int current_workID = global_Current_workID;

    vector<vector<double>> current_Group_worker_subTrajectoryDis;

    int sign = 0;
    int temp = 0;

    for (int i = 0; i < tasks.size(); i++)
    {

        TASK task = tasks[i];
        sign++;

        CURRENT_TASK_GROUP taskg;
        taskg.task = task;
        taskg.Original_Local = i;
        current_taskGroup.push_back(taskg);

        if ((sign - temp == Tmax) || (task.startTime >= current_window_endTime))
        {
            temp = current_taskGroup.size();

            current_workID = global_Current_workID;

            current_window_endTime = task.startTime;

            groupWork_according_TaskGroup(workers, Sumdis, current_Group_worker, current_Group_workerSumdis, current_window_endTime, current_workID, current_Group_workerAD, global_Worker_subTrajectoryDis, current_Group_worker_subTrajectoryDis);
            determine_Window_Task_Timeout(current_taskGroup, current_window_endTime);

            match_WorkerTask_WPPG(current_taskGroup, current_Group_worker, current_window_endTime, current_Group_workerAD, current_Group_worker_subTrajectoryDis);

            current_window_startTime = current_window_endTime;
            current_window_endTime += Wmax;
        }
        updata_current_Info(current_taskGroup, current_Group_worker, current_Group_workerAD, current_Group_worker_subTrajectoryDis, current_Group_workerSumdis);
    }

    if (current_taskGroup.size() > 0)
    {

        current_workID = global_Current_workID;
        current_window_startTime = current_taskGroup.back().task.startTime;

        groupWork_according_TaskGroup(workers, Sumdis, current_Group_worker, current_Group_workerSumdis, current_window_endTime, current_workID, current_Group_workerAD, global_Worker_subTrajectoryDis, current_Group_worker_subTrajectoryDis);

        determine_Window_Task_Timeout(current_taskGroup, current_window_endTime);

        match_WorkerTask_WPPG(current_taskGroup, current_Group_worker, current_window_endTime, current_Group_workerAD, current_Group_worker_subTrajectoryDis);
    }
}

void Basic_information::match_WorkerTask_WPPG(vector<CURRENT_TASK_GROUP> &current_taskGroup, vector<CURRENT_WORKERS_GROUP> &current_workerGroup, double current_window_endTime, vector<double> &current_Group_workerAD, vector<vector<double>> &current_Group_worker_subTrajectoryDis)
{
    int current_Number_Worker = current_workerGroup.size();
    int current_Number_Task = current_taskGroup.size();

    vector<double> current_detour_distance[current_Number_Task];
    vector<int> current_poi[current_Number_Task];
    vector<vector<pair<int, double>>> current_PT(current_Number_Task);
    vector<vector<pair<int, double>>> current_PW(current_Number_Worker);

    for (int i = 0; i < current_Number_Task; i++)
    {
        current_detour_distance[i] = vector<double>(current_Number_Worker, 0);

        current_poi[i] = vector<int>(current_Number_Worker, 0);
    }

    vector<int> current_CT_Worker[current_Number_Worker];
    double current_MaxDistanceTask[current_Number_Task];
    double current_task_NeedTime = 0.0;

    computeMaxDitanceTask(current_MaxDistanceTask, current_taskGroup, current_window_endTime);

    vector<int> current_Group_worker_AT;
    for (int i = 0; i < current_Number_Task; i++)
    {
        current_Group_worker_AT.push_back(i);
    }

    for (int i = 0; i < current_Number_Worker; i++)
    {
        int workerid = i;

        if (current_Group_worker_AT.size() != 0)
        {

            ComputePWforAT_WPPG(workerid, current_PW, current_Group_worker_AT, current_detour_distance, current_poi, current_workerGroup, current_taskGroup);
            sort(current_PW[workerid].begin(), current_PW[workerid].end(), cmp);
            if (current_PW[workerid].size() > 0)
            {
                int assigned_task = 0;
                for (int j = 0; j < current_PW[workerid].size(); j++)
                {
                    int ct_task_id = current_PW[workerid][j].first;
                    if (current_taskGroup[ct_task_id].sign)
                        if (assigned_task < Capacity)
                        {
                            if (find(current_Group_worker_AT.begin(), current_Group_worker_AT.end(), current_PW[workerid][j].first) != current_Group_worker_AT.end())
                            {
                                if (CurrentTask_Satisfy(current_taskGroup, workerid, current_PW[workerid][j].first, current_Group_workerAD, current_detour_distance, current_Group_worker_subTrajectoryDis, current_CT_Worker, current_poi, current_MaxDistanceTask, &current_task_NeedTime, current_window_endTime))

                                {
                                    current_CT_Worker[workerid].push_back(current_PW[workerid][j].first);

                                    current_workerGroup[workerid].sign = false;
                                    current_taskGroup[ct_task_id].sign = false;

                                    global_CT_Worker[current_workerGroup[workerid].Original_Local].push_back(current_taskGroup[ct_task_id].Original_Local);

                                    current_Group_workerAD[workerid] = current_Group_workerAD[workerid] - 2 * current_detour_distance[current_PW[workerid][j].first][workerid];
                                    global_workers[current_workerGroup[workerid].Original_Local].ADdis = current_Group_workerAD[workerid];
                                    UpdateTaskDeadline(current_taskGroup, workerid, current_PW[workerid][j].first, current_detour_distance, current_CT_Worker, current_poi, current_MaxDistanceTask, current_task_NeedTime);

                                    assigned_task++;
                                    for (vector<int>::iterator it = current_Group_worker_AT.begin(); it != current_Group_worker_AT.end(); ++it)
                                    {
                                        if (*it == current_PW[workerid][j].first)
                                        {

                                            vector<int>::iterator itt = current_Group_worker_AT.erase(it);

                                            break;
                                        }
                                    }
                                }
                            }
                        }
                        else
                            break;
                }
            }
        }
        else
            break;
    }

    ShowCTMatching(current_CT_Worker, current_Number_Worker);
}

void Basic_information::ComputePWforAT_WPPG(int workerid, vector<vector<pair<int, double>>> &PW, vector<int> &AT, vector<double> current_detour_distance[], vector<int> current_poi[], vector<CURRENT_WORKERS_GROUP> &current_workerGroup, vector<CURRENT_TASK_GROUP> &current_taskGroup)
{

    for (int i = 0; i < AT.size(); i++)
    {
        int j = AT[i];
        if (current_taskGroup[j].task.Minscore <= current_workerGroup[workerid].worker.score)
        {

            current_detour_distance[j][workerid] = Caculate_mindist(workerid, j, current_poi, current_taskGroup, current_workerGroup);

            if (current_detour_distance[j][workerid] <= current_workerGroup[workerid].worker.range)
            {
                double preference = current_taskGroup[j].task.Reward - (2 * current_detour_distance[j][workerid] * c);
                if (preference > 0)
                {

                    {
                        PW[workerid].push_back(make_pair(j, preference));
                    }
                }
            }
        }
    }
}

void Basic_information::Grouping_Framework_WSDA(vector<TASK> &tasks, vector<WORKER> &workers, double Wmax, int Tmax, vector<double> &Sumdis, vector<vector<double>> &global_Worker_subTrajectoryDis)

{
    vector<CURRENT_TASK_GROUP> current_taskGroup;
    vector<CURRENT_WORKERS_GROUP> current_Group_worker;
    vector<double> current_Group_workerSumdis;
    vector<double> current_Group_workerAD;
    double current_window_endTime = tasks[0].startTime + Wmax;
    double current_window_startTime = tasks[0].startTime;
    int current_workID = global_Current_workID;

    vector<vector<double>> current_Group_worker_subTrajectoryDis;

    int sign = 0;
    int temp = 0;

    for (int i = 0; i < tasks.size(); i++)
    {

        TASK task = tasks[i];
        sign++;

        CURRENT_TASK_GROUP taskg;
        taskg.task = task;
        taskg.Original_Local = i;

        if ((sign - temp == Tmax) || (task.startTime >= current_window_endTime))
        {
            temp = current_taskGroup.size();
            current_workID = global_Current_workID;
            current_window_endTime = task.startTime;

            groupWork_according_TaskGroup(workers, Sumdis, current_Group_worker, current_Group_workerSumdis, current_window_endTime, current_workID, current_Group_workerAD, global_Worker_subTrajectoryDis, current_Group_worker_subTrajectoryDis);
            determine_Window_Task_Timeout(current_taskGroup, current_window_endTime);

            match_WorkerTask_WSDA(current_taskGroup, current_Group_worker, current_window_endTime, current_Group_workerAD, current_Group_worker_subTrajectoryDis);

            current_window_startTime = current_window_endTime;
            current_window_endTime += Wmax;

            updata_current_Info(current_taskGroup, current_Group_worker, current_Group_workerAD, current_Group_worker_subTrajectoryDis, current_Group_workerSumdis);
        }

        current_taskGroup.push_back(taskg);
    }

    if (current_taskGroup.size() > 0)
    {

        current_workID = global_Current_workID;
        current_window_startTime = current_taskGroup.back().task.startTime;

        groupWork_according_TaskGroup(workers, Sumdis, current_Group_worker, current_Group_workerSumdis, current_window_endTime, current_workID, current_Group_workerAD, global_Worker_subTrajectoryDis, current_Group_worker_subTrajectoryDis);

        determine_Window_Task_Timeout(current_taskGroup, current_window_endTime);

        match_WorkerTask_WSDA(current_taskGroup, current_Group_worker, current_window_endTime, current_Group_workerAD, current_Group_worker_subTrajectoryDis);
    }
}
void Update_AD2_WSDA(int oldworkerid, int taskid, vector<double> &AD, vector<double> detour_distance[])
{

    double dd = 2 * detour_distance[taskid][oldworkerid];
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

    vector<double> current_detour_distance[current_Number_Task];
    vector<int> current_poi[current_Number_Task];
    vector<vector<pair<int, double>>> current_PT(Number_Task);
    vector<vector<pair<int, double>>> current_PW(Number_Worker);

    int current_CW_Task[current_Number_Task];
    int current_num_of_chased_tasks[current_Number_Worker] = {0};

    vector<int> current_ActiveWorker;

    vector<int> current_NextActiveWorker(current_ActiveWorker);

    vector<vector<int>> current_CT_Worker(current_Number_Worker);
    double current_MaxDistanceTask[current_Number_Task] = {0.0};
    double current_task_NeedTime = 0.0;

    for (int i = 0; i < current_Number_Task; i++)
    {
        current_detour_distance[i] = vector<double>(current_Number_Worker, 0);

        current_poi[i] = vector<int>(current_Number_Worker, 0);
        current_CW_Task[i] = -1;
    }

    Compute_PTPW_Group_workerBatch(current_PT, current_PW, current_detour_distance, current_taskGroup, current_workerGroup, current_poi);

    for (int i = 0; i < current_workerGroup.size(); i++)
    {
        if (current_PW[i].size() != 0)
        {

            current_ActiveWorker.push_back(i);
        }
    }

    if (current_ActiveWorker.empty())
    {
        return;
    }

    computeMaxDitanceTask(current_MaxDistanceTask, current_taskGroup, current_window_endTime);

    /**
     * 迭代匹配
     * 工人批量时，工人不可用或者任务不可用停止循环匹配
     * 未匹配或匹配失败的批次工人优先匹配
     * 偏好列表ORDER
     */

    int matchingTimes = 0;
    while (!current_ActiveWorker.empty())
    {

        current_NextActiveWorker.clear();

        for (int i = 0; i < current_ActiveWorker.size(); i++)
            current_NextActiveWorker.push_back(current_ActiveWorker[i]);

        int matchingnumber = 0, replacematching = 0;
        for (int i = 0; i < current_ActiveWorker.size(); i++)
        {
            int workerid = current_ActiveWorker[i];
            int order = current_num_of_chased_tasks[workerid];

            int task_to_chase = current_PW[workerid][order].first;

            current_num_of_chased_tasks[workerid] = order + 1;

            if (IFWorkerExist(workerid, task_to_chase, current_PT) != current_PT[task_to_chase].end())
            {

                if (CurrentTask_Satisfy_TSDA(current_taskGroup, workerid, task_to_chase, current_Group_workerAD, current_detour_distance, current_Group_worker_subTrajectoryDis, current_CT_Worker, current_poi, current_MaxDistanceTask, &current_task_NeedTime, current_window_endTime))

                {

                    if (current_CW_Task[task_to_chase] == -1)
                    {

                        current_CW_Task[task_to_chase] = workerid;
                        current_CT_Worker[workerid].push_back(task_to_chase);

                        Update_AD1(workerid, task_to_chase, current_Group_workerAD, current_detour_distance);

                        global_workers[current_workerGroup[workerid].Original_Local].ADdis = current_Group_workerAD[workerid];

                        UpdateTaskDeadline_WSDA(current_taskGroup, false, 0, workerid, task_to_chase, current_detour_distance, current_CT_Worker, current_poi, current_MaxDistanceTask, current_task_NeedTime);
                        matchingnumber++;
                    }
                    else
                    {

                        int oldworkerid = current_CW_Task[task_to_chase];
                        if (GetIndex_PT(workerid, task_to_chase, current_PT) < GetIndex_PT(oldworkerid, task_to_chase, current_PT))
                        {

                            current_CW_Task[task_to_chase] = workerid;
                            current_CT_Worker[workerid].push_back(task_to_chase);

                            Update_AD1(workerid, task_to_chase, current_Group_workerAD, current_detour_distance);
                            global_workers[current_workerGroup[workerid].Original_Local].ADdis = current_Group_workerAD[workerid];

                            current_CT_Worker[oldworkerid].erase(find(current_CT_Worker[oldworkerid].begin(), current_CT_Worker[oldworkerid].end(), task_to_chase));

                            Update_AD2_WSDA(oldworkerid, task_to_chase, current_Group_workerAD, current_detour_distance);
                            global_workers[current_workerGroup[oldworkerid].Original_Local].ADdis = current_Group_workerAD[oldworkerid];

                            current_MaxDistanceTask[task_to_chase] = (current_taskGroup[task_to_chase].task.Deadline - current_window_endTime) * speed;
                            UpdateTaskDeadline_WSDA(current_taskGroup, true, oldworkerid, workerid, task_to_chase, current_detour_distance, current_CT_Worker, current_poi, current_MaxDistanceTask, current_task_NeedTime);

                            replacematching++;
                        }
                        else
                        {
                        }
                    }
                }
            }
            if (current_num_of_chased_tasks[workerid] >= current_PW[workerid].size())
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
        for (int i = 0; i < current_NextActiveWorker.size(); i++)
            current_ActiveWorker.push_back(current_NextActiveWorker[i]);
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

    ShowCTMatching(current_CT_Worker, current_Number_Worker);
}
void Basic_information::UpdateTaskDeadline_WSDA(vector<CURRENT_TASK_GROUP> &current_taskGroup, bool replace, int replaceWorkid, int workerid, int taskid, vector<double> current_detour_distance[], vector<vector<int>> &CT_Worker, vector<int> poi[], double MaxDistanceTask[], double current_task_NeedTime)
{

    for (int i = 0; i < CT_Worker[workerid].size(); i++)
    {
        int assigntask_id = CT_Worker[workerid][i];
        int assignedpoint = poi[assigntask_id][workerid];
        int detourpoint = poi[taskid][workerid];

        if (assigntask_id == taskid)
        {

            MaxDistanceTask[taskid] = MaxDistanceTask[taskid] - current_task_NeedTime * speed;
            global_MaxDistanceTask[current_taskGroup[taskid].Original_Local] = MaxDistanceTask[taskid];
        }
        else
        {
            if (detourpoint < assignedpoint)
            {

                MaxDistanceTask[assigntask_id] = MaxDistanceTask[assigntask_id] - (2 * current_detour_distance[taskid][workerid]);

                global_MaxDistanceTask[current_taskGroup[assigntask_id].Original_Local] = MaxDistanceTask[assigntask_id];
            }
        }
    }

    if (replace == true)
    {
        for (int i = 0; i < CT_Worker[replaceWorkid].size(); i++)
        {
            int assigntask_id = CT_Worker[replaceWorkid][i];
            int assignedpoint = poi[assigntask_id][replaceWorkid];
            int detourpoint = poi[taskid][replaceWorkid];
            if (assigntask_id == taskid)
            {

                MaxDistanceTask[taskid] = MaxDistanceTask[taskid] - current_task_NeedTime * speed;
                global_MaxDistanceTask[current_taskGroup[taskid].Original_Local] = MaxDistanceTask[taskid];
            }
            else
            {
                if (detourpoint < assignedpoint)
                {

                    MaxDistanceTask[assigntask_id] = MaxDistanceTask[assigntask_id] + (2 * current_detour_distance[taskid][replaceWorkid]);

                    global_MaxDistanceTask[current_taskGroup[assigntask_id].Original_Local] = MaxDistanceTask[assigntask_id];
                }
            }
        }
    }
}

/***
 * 4.7-1
 * 根据混合排序得到已经排好序的（任务、工人）数据时间
 * 逐个初始化每个信息
 * 并按照时间进行匹配计算
 *  计算分为任务（或工人）两种形式
 *
 */

void Basic_information::
    time_random_Framework()
{
    vector<pair<WORKER, TASK>> hybird_datasets;
    sort_hybrid(global_tasks, global_workers, hybird_datasets);

    vector<CURRENT_TASK_GROUP> current_taskGroup;
    vector<CURRENT_WORKERS_GROUP> current_workerGroup;
    vector<CURRENT_WORKER_STATE> current_workerState;
    vector<CURRENT_TASK_STATE> current_taskState;

    double current_time = 0;
    int arrive_global_workerID = -1;
    int arrive_global_taskID = -1;
    CURRENT_TASK_GROUP taskg;
    CURRENT_WORKERS_GROUP workerg;
    CURRENT_WORKER_STATE workerStateg;
    CURRENT_TASK_STATE taskStateg;
    bool sign;

    int nummm = 0;
    workerg.sign = true;
    taskg.sign = true;

    for (auto p : hybird_datasets)
    {

        if (p.first.startTime != 0)
        {
            arrive_global_workerID++;
            workerg.worker = p.first;
            workerg.Original_Local = arrive_global_workerID;
            current_workerGroup.push_back(workerg);
            current_time = p.first.startTime;

            workerStateg.current_alltaskCost = 0;
            workerStateg.current_worker_subTrajectoryDis = global_Worker_subTrajectoryDis[arrive_global_workerID];
            workerStateg.detour_poi.clear();
            workerStateg.detour_poi.resize(0);
            workerStateg.matched_MaxDistanceTask_orig.clear();
            workerStateg.matched_MaxDistanceTask_orig.resize(0);
            workerStateg.matched_task.clear();
            workerStateg.matched_task.resize(0);
            workerStateg.workerAD_orig = (global_workers[arrive_global_workerID].endTime - current_time) * speed - global_Sumdis[arrive_global_workerID];

            current_workerState.push_back(workerStateg);

            sign = false;
        }
        else
        {
            arrive_global_taskID++;
            taskg.task = p.second;
            taskg.Original_Local = arrive_global_taskID;

            current_taskGroup.push_back(taskg);
            current_time = p.second.startTime;

            taskStateg.MaxDistanceTask_orig = (p.second.Deadline - p.second.startTime) * speed;
            taskStateg.poi = -1;
            taskStateg.current_task_detour_distance = 0;

            current_taskState.push_back(taskStateg);
            sign = true;
        }

        if (current_taskGroup.size() > 0 && current_workerGroup.size() > 0)
        {

            erase_Task_worker_Timeout(current_taskGroup, current_workerGroup, current_time, current_workerState, current_taskState);
            time_random_match(current_taskGroup, current_workerGroup, current_time, current_workerState, current_taskState, sign);
        }
    }
}

/***
 * 4.7-2
 * 根据混合排序得到已经排好序的（任务、工人）数据时间
 * 逐个初始化每个信息
 * 并按照时间进行匹配计算
 *  计算分为任务（或工人）两种形式
 *
 */

void Basic_information::whole_Greedy_Framework()
{
    vector<pair<WORKER, TASK>> hybird_datasets;
    sort_hybrid(global_tasks, global_workers, hybird_datasets);

    vector<CURRENT_TASK_GROUP> current_taskGroup;
    vector<CURRENT_WORKERS_GROUP> current_workerGroup;
    vector<CURRENT_WORKER_STATE> current_workerState;
    vector<CURRENT_TASK_STATE> current_taskState;

    double current_time = 0;
    int arrive_global_workerID = -1;
    int arrive_global_taskID = -1;
    CURRENT_TASK_GROUP taskg;
    CURRENT_WORKERS_GROUP workerg;
    CURRENT_WORKER_STATE workerStateg;
    CURRENT_TASK_STATE taskStateg;
    bool sign;

    int nummm = 0;
    for (auto p : hybird_datasets)
    {

        if (p.first.startTime != 0)
        {
            arrive_global_workerID++;
            workerg.worker = p.first;
            workerg.Original_Local = arrive_global_workerID;
            current_workerGroup.push_back(workerg);
            current_time = p.first.startTime;

            workerStateg.current_alltaskCost = 0;
            workerStateg.current_worker_subTrajectoryDis = global_Worker_subTrajectoryDis[arrive_global_workerID];
            workerStateg.workerAD_orig = (global_workers[arrive_global_workerID].endTime - current_time) * speed - global_Sumdis[arrive_global_workerID];

            current_workerState.push_back(workerStateg);

            sign = false;
        }
        else
        {
            arrive_global_taskID++;
            taskg.task = p.second;
            taskg.Original_Local = arrive_global_taskID;
            current_taskGroup.push_back(taskg);
            current_time = p.second.startTime;

            taskStateg.MaxDistanceTask_orig = (p.second.Deadline - current_time) * speed;

            current_taskState.push_back(taskStateg);
            sign = true;
        }

        if (current_taskGroup.size() > 0 && current_workerGroup.size() > 0)
        {

            erase_Task_worker_Timeout(current_taskGroup, current_workerGroup, current_time, current_workerState, current_taskState);
            match_Whole(current_taskGroup, current_workerGroup, current_time, current_workerState, current_taskState, sign);
        }
    }
}

/***
 * 计算并存储当前tasklist对应工人最小的poi点
 * 并返回最小的距离
 */
double Basic_information::Caculate_mindist_whole(int global_workerid, int current_taskid, vector<CURRENT_TASK_GROUP> &task, vector<CURRENT_TASK_STATE> &taskState)
{
    vector<POI> Trajectory;
    Trajectory = global_workers[global_workerid].trajectory;
    double detour_distance, mindis = 1000000;
    int j = 0;
    for (vector<POI>::iterator it = Trajectory.begin(); it != Trajectory.end(); it++)
    {
        detour_distance = GetDistance(task[current_taskid].task.Y, task[current_taskid].task.X, (*it).Y, (*it).X);

        if (detour_distance < mindis)
        {

            mindis = detour_distance;
            taskState[current_taskid].poi = j;
        }
        j++;
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
void Basic_information::time_random_match(vector<CURRENT_TASK_GROUP> &current_taskGroup, vector<CURRENT_WORKERS_GROUP> &current_workerGroup, double current_Time, vector<CURRENT_WORKER_STATE> &workStates, vector<CURRENT_TASK_STATE> &taskStates, bool sign)
{

    int current_Number_Worker = current_workerGroup.size();
    int current_Number_Task = current_taskGroup.size();

    double current_task_NeedTime = 0;

    if (sign == false)
    {
        int tempTid = 0;
        int arrive_global_workerID = current_workerGroup.back().Original_Local;
        for (auto &cu_ts : taskStates)
        {

            if (workStates.back().matched_task.size() < Capacity)
            {
                if (current_taskGroup[tempTid].task.Minscore <= current_workerGroup[current_Number_Worker - 1].worker.score)
                {

                    double dist = Caculate_mindist_whole(arrive_global_workerID, tempTid, current_taskGroup, taskStates);
                    cu_ts.current_task_detour_distance = dist;

                    if (current_workerGroup.back().worker.range >= dist)
                    {

                        if (current_taskGroup[tempTid].task.Reward - (2 * dist * c) > 0)
                        {

                            if (CurrentTask_Satisfy_whole(current_taskGroup, current_workerGroup, tempTid, taskStates, workStates, current_Time, current_Number_Worker - 1, &current_task_NeedTime))
                            {

                                workStates.back().matched_task.push_back(current_taskGroup[tempTid]);
                                workStates.back().current_alltaskCost += 2 * dist;
                                workStates.back().detour_poi.push_back(cu_ts.poi);

                                cu_ts.MaxDistanceTask_orig -= 2 * dist;
                                workStates.back().matched_MaxDistanceTask_orig.push_back(cu_ts.MaxDistanceTask_orig);

                                /**
                                 * 更新
                                 */
                                current_taskGroup[tempTid].sign = false;
                                global_CT_Worker[arrive_global_workerID].push_back(current_taskGroup[tempTid].Original_Local);

                                UpdateTaskDeadline_whole(current_Number_Worker - 1, tempTid, workStates, taskStates, current_task_NeedTime, current_taskGroup, current_workerGroup);
                            }
                        }
                    }
                }
                if (workStates.back().matched_task.size() == Capacity)
                {
                    current_workerGroup.back().sign = false;
                    break;
                }
            }

            tempTid++;
        }
    }
    else
    {

        int tempWid = 0;
        int arrive_global_workerID = 0;
        int arrive_global_taskID = current_taskGroup.back().Original_Local;

        for (auto &cu_ws : workStates)
        {

            if (current_taskGroup.back().task.Minscore <= current_workerGroup[tempWid].worker.score)
            {

                arrive_global_workerID = current_workerGroup[tempWid].Original_Local;

                double dist = Caculate_mindist_whole(arrive_global_workerID, current_Number_Task - 1, current_taskGroup, taskStates);
                taskStates.back().current_task_detour_distance = (dist);

                if (current_workerGroup[tempWid].worker.range >= dist)
                {

                    if (current_taskGroup[current_Number_Task - 1].task.Reward - (2 * dist * c) > 0)
                    {

                        if (CurrentTask_Satisfy_whole(current_taskGroup, current_workerGroup, current_Number_Task - 1, taskStates, workStates, current_Time, tempWid, &current_task_NeedTime))
                        {

                            cu_ws.matched_task.push_back(current_taskGroup[current_Number_Task - 1]);

                            cu_ws.current_alltaskCost += 2 * dist;

                            cu_ws.detour_poi.push_back(taskStates[current_Number_Task - 1].poi);

                            taskStates.back().MaxDistanceTask_orig -= 2 * dist;
                            cu_ws.matched_MaxDistanceTask_orig.push_back(taskStates.back().MaxDistanceTask_orig);

                            /**
                             * 更新
                             */
                            current_taskGroup[current_Number_Task - 1].sign = false;

                            global_CT_Worker[arrive_global_workerID].push_back(current_taskGroup[current_Number_Task - 1].Original_Local);

                            if (cu_ws.matched_task.size() == Capacity || (cu_ws.workerAD_orig - cu_ws.current_alltaskCost - speed * (current_Time - current_workerGroup[tempWid].worker.startTime)) <= 0)
                            {
                                current_workerGroup[tempWid].sign = false;
                            }

                            break;
                        }
                    }
                }
            }

            tempWid++;
        }
    }
}

void Basic_information::match_Whole(vector<CURRENT_TASK_GROUP> &current_taskGroup, vector<CURRENT_WORKERS_GROUP> &current_workerGroup, double current_Time, vector<CURRENT_WORKER_STATE> &workStates, vector<CURRENT_TASK_STATE> &taskStates, bool sign)
{

    int current_Number_Worker = current_workerGroup.size();
    int current_Number_Task = current_taskGroup.size();

    vector<vector<int>> current_CT_Worker(current_Number_Worker);
    double current_task_NeedTime = 0;

    if (sign == false)
    {
        int tempTid = 0;
        int arrive_global_workerID;

        for (auto &cu_ts : taskStates)
        {
            if (workStates.back().matched_task.size() < Capacity)
            {
                if (current_taskGroup[tempTid].task.Minscore <= current_workerGroup[current_Number_Worker - 1].worker.score)
                {

                    arrive_global_workerID = current_workerGroup.back().Original_Local;

                    double dist = Caculate_mindist_whole(arrive_global_workerID, tempTid, current_taskGroup, taskStates);
                    cu_ts.current_task_detour_distance = (dist);

                    if (current_workerGroup[current_Number_Worker - 1].worker.range >= dist)
                    {

                        if (current_taskGroup[tempTid].task.Reward - (2 * dist * c) > 0)
                        {
                            if (CurrentTask_Satisfy_whole(current_taskGroup, current_workerGroup, tempTid, taskStates, workStates, current_Time, current_Number_Worker - 1, &current_task_NeedTime))
                            {

                                workStates.back().matched_task.push_back(current_taskGroup[tempTid]);
                                workStates.back().current_alltaskCost += dist;
                                workStates.back().detour_poi.push_back(cu_ts.poi);

                                workStates.back().matched_MaxDistanceTask_orig.push_back(cu_ts.MaxDistanceTask_orig);
                                taskStates[tempTid].MaxDistanceTask_orig -= 2 * dist;
                                /**
                                 * 更新
                                 */
                                current_taskGroup[tempTid].sign = false;
                                global_CT_Worker[arrive_global_workerID].push_back(current_taskGroup[tempTid].Original_Local);

                                UpdateTaskDeadline_whole(current_Number_Worker - 1, tempTid, workStates, taskStates, current_task_NeedTime, current_taskGroup, current_workerGroup);
                            }
                        }
                    }
                }
            }
            else
            {
                current_workerGroup.back().sign = false;
                break;
            }

            tempTid++;
        }
    }
    else
    {
        int tempWid = 0;
        int arrive_global_workerID;

        for (auto &cu_ws : workStates)
        {

            if (current_taskGroup[current_Number_Task - 1].task.Minscore <= current_workerGroup[tempWid].worker.score)
            {

                arrive_global_workerID = current_workerGroup[tempWid].Original_Local;

                double dist = Caculate_mindist_whole(arrive_global_workerID, current_Number_Task - 1, current_taskGroup, taskStates);
                taskStates.back().current_task_detour_distance = (dist);

                if (current_workerGroup[tempWid].worker.range >= dist)
                {

                    if (current_taskGroup[current_Number_Task - 1].task.Reward - (2 * dist * c) > 0)
                    {
                        if (CurrentTask_Satisfy_whole(current_taskGroup, current_workerGroup, current_Number_Task - 1, taskStates, workStates, current_Time, tempWid, &current_task_NeedTime))
                        {

                            cu_ws.matched_task.push_back(current_taskGroup[current_Number_Task - 1]);
                            cu_ws.current_alltaskCost += dist;
                            cu_ws.detour_poi.push_back(taskStates.back().poi);

                            cu_ws.matched_MaxDistanceTask_orig.push_back(taskStates.back().MaxDistanceTask_orig - 2 * dist);
                            taskStates.back().MaxDistanceTask_orig -= 2 * dist;
                            /**
                             * 更新
                             */
                            current_taskGroup[current_Number_Task - 1].sign = false;
                            global_CT_Worker[current_workerGroup[tempWid].Original_Local].push_back(current_taskGroup[current_Number_Task - 1].Original_Local);
                            if (cu_ws.matched_task.size() == Capacity)
                            {
                                current_workerGroup[tempWid].sign = false;
                            }

                            UpdateTaskDeadline_whole(tempWid, current_Number_Task - 1, workStates, taskStates, current_task_NeedTime, current_taskGroup, current_workerGroup);
                            break;
                        }
                    }
                }
            }

            tempWid++;
        }
    }
}

void Basic_information::UpdateTaskDeadline_whole(int current_workerid, int taskid, vector<CURRENT_WORKER_STATE> &workStates,
                                                 vector<CURRENT_TASK_STATE> &taskStates, double current_task_NeedTime, vector<CURRENT_TASK_GROUP> &current_taskGroup, vector<CURRENT_WORKERS_GROUP> &current_workerGroup)
{
    int current_detourid = taskStates[taskid].poi;
    for (int i = 0; i < workStates[current_workerid].matched_task.size() - 1; i++)
    {

        int assigntask_id = workStates[current_workerid].matched_task[i].Original_Local;
        int detourpoint = workStates[current_workerid].detour_poi[i];

        if (detourpoint > current_detourid)
        {

            workStates[current_workerid].matched_MaxDistanceTask_orig[i] -= (2 * taskStates[taskid].current_task_detour_distance);
            global_MaxDistanceTask[assigntask_id] = workStates[current_workerid].matched_MaxDistanceTask_orig[i];
        }
    }
    taskStates[taskid].MaxDistanceTask_orig -= (2 * taskStates[taskid].current_task_detour_distance);
    global_MaxDistanceTask[current_taskGroup[taskid].Original_Local] = taskStates[taskid].MaxDistanceTask_orig;
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

    int assignedNumber = workStates[current_workerid].matched_task.size();

    double ADD = workStates[current_workerid].workerAD_orig - workStates[current_workerid].current_alltaskCost -
                 (current_Time - current_workerGroup[current_workerid].worker.startTime) * speed -
                 2 * taskStates[current_taskid].current_task_detour_distance;

    if (ADD < 0)
    {

        return false;
    }

    int current_detourid = taskStates[current_taskid].poi;

    double SumdetouDis = 0.0;
    for (int i = 0; i < assignedNumber; i++)
    {
        int assigntask_id = workStates[current_workerid].matched_task[i].Original_Local;
        int detourpoint = workStates[current_workerid].detour_poi[i];
        if ((workStates[current_workerid].matched_MaxDistanceTask_orig[i] -
             (current_Time - global_tasks[assigntask_id].startTime) * speed) < 0)

        {
            current_workerGroup[current_workerid].sign = false;

            return false;
        }
        if (current_detourid >= detourpoint)
        {

            double x = global_tasks[assigntask_id].X;
            double y = global_tasks[assigntask_id].Y;
            double x1 = current_workerGroup[current_workerid].worker.trajectory[detourpoint].X;
            double y1 = current_workerGroup[current_workerid].worker.trajectory[detourpoint].Y;

            SumdetouDis += 2 * GetDistance(y, x, y1, x1);
        }
    }

    double Time_TO_Task;
    Time_TO_Task = (taskStates[current_taskid].current_task_detour_distance +
                    workStates[current_workerid].current_worker_subTrajectoryDis[current_detourid] + SumdetouDis) /
                   speed;

    if (current_taskGroup[current_taskid].task.Deadline > (Time_TO_Task + current_Time))
    {
        *current_task_NeedTime = Time_TO_Task;

        if (assignedNumber == 0)
        {

            return true;
        }
        else
        {
            int noinflu_count = 0;
            for (int i = 0; i < assignedNumber; i++)
            {
                int assigntask_id = workStates[current_workerid].matched_task[i].Original_Local;

                int detourpoint = workStates[current_workerid].detour_poi[i];

                if (current_detourid < detourpoint)
                {

                    double x = global_tasks[assigntask_id].X;
                    double y = global_tasks[assigntask_id].Y;
                    double x1 = current_workerGroup[current_workerid].worker.trajectory[detourpoint].X;
                    double y1 = current_workerGroup[current_workerid].worker.trajectory[detourpoint].Y;

                    if (workStates[current_workerid].matched_MaxDistanceTask_orig[i] - (current_Time - global_tasks[assigntask_id].startTime) * speed <
                        (2 * GetDistance(y, x, y1, x1)))

                    {

                        return false;
                    }
                    else
                    {
                    }
                }

                if (current_detourid > detourpoint)
                {
                    noinflu_count++;
                }
            }
            if (noinflu_count == assignedNumber)
            {

                return true;
            }

            return true;
        }
    }
    else
    {

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

    for (int i = taskList.size() - 1; i >= 0; i--)
    {
        if (taskList[i].task.Deadline < nowTime || taskList[i].sign == false)
        {
            taskList.erase(taskList.begin() + i);
            current_taskState.erase(current_taskState.begin() + i);
        }
    }

    for (int i = workerList.size() - 1; i >= 0; i--)
    {

        if (workerList[i].worker.endTime < nowTime || workerList[i].sign == false)
        {

            workerList.erase(workerList.begin() + i);
            current_workerState.erase(current_workerState.begin() + i);
        }
        else
        {
            for (int j = 0; j < current_workerState[i].matched_task.size(); j++)
            {

                double time = current_workerState[i].matched_MaxDistanceTask_orig[j] - (nowTime - current_workerState[i].matched_task[j].task.startTime) * speed;
                if (current_workerState[i].matched_task[j].task.Deadline < nowTime || time <= 0)
                {
                    workerList.erase(workerList.begin() + i);
                    current_workerState.erase(current_workerState.begin() + i);
                    break;
                }
            }
        }
    }
}

void Basic_information::Grouping_Framework_ReverseDA(vector<TASK> &tasks, vector<WORKER> &workers, double Wmax, int Tmax, vector<double> &Sumdis, vector<vector<double>> &global_Worker_subTrajectoryDis)

{
    vector<CURRENT_TASK_GROUP> current_taskGroup;
    vector<CURRENT_WORKERS_GROUP> current_Group_worker;
    vector<double> current_Group_workerSumdis;
    vector<double> current_Group_workerAD;
    double current_window_endTime = tasks[0].startTime + Wmax;
    double current_window_startTime = tasks[0].startTime;
    int current_workID = global_Current_workID;

    vector<vector<double>> current_Group_worker_subTrajectoryDis;

    int sign = 0;
    int temp = 0;

    for (int i = 0; i < tasks.size(); i++)
    {

        TASK task = tasks[i];
        sign++;

        CURRENT_TASK_GROUP taskg;
        taskg.task = task;
        taskg.Original_Local = i;

        if ((sign - temp == Tmax) || (task.startTime >= current_window_endTime))
        {
            temp = current_taskGroup.size();
            current_workID = global_Current_workID;
            current_window_endTime = task.startTime;

            groupWork_according_TaskGroup(workers, Sumdis, current_Group_worker, current_Group_workerSumdis, current_window_endTime, current_workID, current_Group_workerAD, global_Worker_subTrajectoryDis, current_Group_worker_subTrajectoryDis);
            determine_Window_Task_Timeout(current_taskGroup, current_window_endTime);

            match_WorkerTask_ReverseDA(current_taskGroup, current_Group_worker, current_window_endTime, current_Group_workerAD, current_Group_worker_subTrajectoryDis);

            current_window_startTime = current_window_endTime;
            current_window_endTime += Wmax;
        }
        updata_current_Info(current_taskGroup, current_Group_worker, current_Group_workerAD, current_Group_worker_subTrajectoryDis, current_Group_workerSumdis);

        current_taskGroup.push_back(taskg);
    }

    if (current_taskGroup.size() > 0)
    {

        current_workID = global_Current_workID;
        current_window_startTime = current_taskGroup.back().task.startTime;

        groupWork_according_TaskGroup(workers, Sumdis, current_Group_worker, current_Group_workerSumdis, current_window_endTime, current_workID, current_Group_workerAD, global_Worker_subTrajectoryDis, current_Group_worker_subTrajectoryDis);

        determine_Window_Task_Timeout(current_taskGroup, current_window_endTime);

        match_WorkerTask_ReverseDA(current_taskGroup, current_Group_worker, current_window_endTime, current_Group_workerAD, current_Group_worker_subTrajectoryDis);
    }
}

void Basic_information::match_WorkerTask_ReverseDA(vector<CURRENT_TASK_GROUP> &current_taskGroup, vector<CURRENT_WORKERS_GROUP> &current_workerGroup, double current_window_endTime, vector<double> &current_Group_workerAD, vector<vector<double>> &current_Group_worker_subTrajectoryDis)
{

    int current_Number_Worker = current_workerGroup.size();
    int current_Number_Task = current_taskGroup.size();

    vector<double> current_detour_distance[current_Number_Task];
    vector<int> current_poi[current_Number_Task];
    vector<vector<pair<int, double>>> current_PT(Number_Task);
    vector<vector<pair<int, double>>> current_PW(Number_Worker);

    int current_CW_Task[current_Number_Task];
    int current_num_of_chased_worker[current_Number_Task] = {0};
    vector<int> current_ActiveTask;
    vector<int> current_NextActiveTask(current_ActiveTask);

    vector<vector<int>> current_CT_Worker(current_Number_Worker);
    double current_MaxDistanceTask[current_Number_Task] = {0.0};
    double current_task_NeedTime = 0.0;

    for (int i = 0; i < current_Number_Task; i++)
    {
        current_detour_distance[i] = vector<double>(current_Number_Worker, 0);

        current_poi[i] = vector<int>(current_Number_Worker, 0);
    }

    Compute_PTPW_Group_workerBatch(current_PT, current_PW, current_detour_distance, current_taskGroup, current_workerGroup, current_poi);
    for (int i = 0; i < current_taskGroup.size(); i++)
    {
        if (current_PT[i].size() != 0)
        {
            current_ActiveTask.push_back(i);
        }
        current_CW_Task[i] = -1;
    }
    if (current_ActiveTask.empty())
    {
        return;
    }

    computeMaxDitanceTask(current_MaxDistanceTask, current_taskGroup, current_window_endTime);

    /**
     * 迭代匹配
     * 工人批量时，工人不可用或者任务不可用停止循环匹配
     * 未匹配或匹配失败的批次工人优先匹配
     * 偏好列表ORDER
     */

    while (!current_ActiveTask.empty())
    {

        current_NextActiveTask.clear();

        for (int i = 0; i < current_ActiveTask.size(); i++)
            current_NextActiveTask.push_back(current_ActiveTask[i]);

        int matchingnumber = 0, replacematching = 0;
        for (int i = 0; i < current_ActiveTask.size(); i++)
        {
            int taskid = current_ActiveTask[i];
            int order = current_num_of_chased_worker[taskid];
            int worker_to_chase = current_PT[taskid][order].first;

            current_num_of_chased_worker[taskid] = order + 1;

            if (IFTaskExist(worker_to_chase, taskid, current_PW) != current_PW[worker_to_chase].end())
            {

                if (CurrentTask_Satisfy_TSDA(current_taskGroup, worker_to_chase, taskid, current_Group_workerAD, current_detour_distance, current_Group_worker_subTrajectoryDis, current_CT_Worker, current_poi, current_MaxDistanceTask, &current_task_NeedTime, current_window_endTime))

                {

                    current_CW_Task[taskid] = worker_to_chase;
                    current_CT_Worker[worker_to_chase].push_back(taskid);

                    current_NextActiveTask.erase(find(current_NextActiveTask.begin(), current_NextActiveTask.end(), taskid));

                    Update_AD1(worker_to_chase, taskid, current_Group_workerAD, current_detour_distance);

                    global_workers[current_workerGroup[worker_to_chase].Original_Local].ADdis = current_Group_workerAD[worker_to_chase];

                    UpdateTaskDeadline_TSDA(current_taskGroup, false, 0, worker_to_chase, taskid, current_detour_distance, current_CT_Worker, current_poi, current_MaxDistanceTask, current_task_NeedTime);
                    matchingnumber++;
                }
                else
                {

                    int MinReplaceTask = FindReplaceTaskNew_TSDA(worker_to_chase, taskid, current_PW, current_Group_workerAD, current_detour_distance, current_Group_worker_subTrajectoryDis, current_CT_Worker, current_poi, current_MaxDistanceTask, &current_task_NeedTime, current_taskGroup, current_window_endTime);
                    if (MinReplaceTask != -1)
                    {
                        current_CW_Task[taskid] = worker_to_chase;
                        current_CW_Task[MinReplaceTask] = -1;
                        current_CT_Worker[worker_to_chase].push_back(taskid);

                        current_CT_Worker[worker_to_chase].erase(find(current_CT_Worker[worker_to_chase].begin(), current_CT_Worker[worker_to_chase].end(), MinReplaceTask));

                        if (current_num_of_chased_worker[MinReplaceTask] < current_PT[MinReplaceTask].size())
                            current_NextActiveTask.push_back(MinReplaceTask);
                        current_NextActiveTask.erase(find(current_NextActiveTask.begin(), current_NextActiveTask.end(), taskid));

                        Update_AD2(worker_to_chase, taskid, MinReplaceTask, current_Group_workerAD, current_detour_distance);
                        global_workers[current_workerGroup[worker_to_chase].Original_Local].ADdis = current_Group_workerAD[worker_to_chase];

                        UpdateTaskDeadline_TSDA(current_taskGroup, true, MinReplaceTask, worker_to_chase, taskid, current_detour_distance, current_CT_Worker, current_poi, current_MaxDistanceTask, current_task_NeedTime);

                        current_MaxDistanceTask[MinReplaceTask] = current_taskGroup[MinReplaceTask].task.Deadline * speed;
                        global_MaxDistanceTask[current_taskGroup[MinReplaceTask].Original_Local] = current_MaxDistanceTask[MinReplaceTask];

                        replacematching++;
                    }
                    else
                    {
                    }
                }
            }
            if (current_num_of_chased_worker[taskid] == current_PT[taskid].size())
            {

                vector<int>::iterator iter1 = find(current_NextActiveTask.begin(), current_NextActiveTask.end(), taskid);
                if (iter1 != current_NextActiveTask.end())
                {
                    current_NextActiveTask.erase(find(current_NextActiveTask.begin(), current_NextActiveTask.end(), taskid));
                }
            }
        }

        current_ActiveTask.clear();
        for (int i = 0; i < current_NextActiveTask.size(); i++)
            current_ActiveTask.push_back(current_NextActiveTask[i]);
    }

    int current_num_of_chased_tasks[current_Number_Worker] = {0};

    vector<int> current_ActiveWorker;

    vector<int> current_NextActiveWorker(current_ActiveWorker);

    current_task_NeedTime = 0.0;

    for (int i = 0; i < current_workerGroup.size(); i++)
    {
        if (current_PW[i].size() != 0)
        {

            current_ActiveWorker.push_back(i);
        }
    }

    /**
     * 迭代匹配
     * 工人批量时，工人不可用或者任务不可用停止循环匹配
     * 未匹配或匹配失败的批次工人优先匹配
     * 偏好列表ORDER
     */

    int matchingTimes = 0;
    while (!current_ActiveWorker.empty())
    {

        current_NextActiveWorker.clear();

        for (int i = 0; i < current_ActiveWorker.size(); i++)
            current_NextActiveWorker.push_back(current_ActiveWorker[i]);

        int matchingnumber = 0, replacematching = 0;
        for (int i = 0; i < current_ActiveWorker.size(); i++)
        {
            int workerid = current_ActiveWorker[i];
            int order = current_num_of_chased_tasks[workerid];

            int task_to_chase = current_PW[workerid][order].first;

            current_num_of_chased_tasks[workerid] = order + 1;

            if (IFWorkerExist(workerid, task_to_chase, current_PT) != current_PT[task_to_chase].end())
            {

                if (CurrentTask_Satisfy_TSDA(current_taskGroup, workerid, task_to_chase, current_Group_workerAD, current_detour_distance, current_Group_worker_subTrajectoryDis, current_CT_Worker, current_poi, current_MaxDistanceTask, &current_task_NeedTime, current_window_endTime))

                {

                    if (current_CW_Task[task_to_chase] == -1)
                    {

                        current_CW_Task[task_to_chase] = workerid;
                        current_CT_Worker[workerid].push_back(task_to_chase);

                        Update_AD1(workerid, task_to_chase, current_Group_workerAD, current_detour_distance);

                        global_workers[current_workerGroup[workerid].Original_Local].ADdis = current_Group_workerAD[workerid];

                        UpdateTaskDeadline_WSDA(current_taskGroup, false, 0, workerid, task_to_chase, current_detour_distance, current_CT_Worker, current_poi, current_MaxDistanceTask, current_task_NeedTime);
                        matchingnumber++;
                    }
                    else
                    {

                        int oldworkerid = current_CW_Task[task_to_chase];
                        if (GetIndex_PT(workerid, task_to_chase, current_PT) < GetIndex_PT(oldworkerid, task_to_chase, current_PT))
                        {

                            current_CW_Task[task_to_chase] = workerid;
                            current_CT_Worker[workerid].push_back(task_to_chase);

                            Update_AD1(workerid, task_to_chase, current_Group_workerAD, current_detour_distance);
                            global_workers[current_workerGroup[workerid].Original_Local].ADdis = current_Group_workerAD[workerid];

                            current_CT_Worker[oldworkerid].erase(find(current_CT_Worker[oldworkerid].begin(), current_CT_Worker[oldworkerid].end(), task_to_chase));

                            Update_AD2_WSDA(oldworkerid, task_to_chase, current_Group_workerAD, current_detour_distance);
                            global_workers[current_workerGroup[oldworkerid].Original_Local].ADdis = current_Group_workerAD[oldworkerid];

                            current_MaxDistanceTask[task_to_chase] = (current_taskGroup[task_to_chase].task.Deadline - current_window_endTime) * speed;
                            UpdateTaskDeadline_WSDA(current_taskGroup, true, oldworkerid, workerid, task_to_chase, current_detour_distance, current_CT_Worker, current_poi, current_MaxDistanceTask, current_task_NeedTime);

                            replacematching++;
                        }
                    }
                }
            }
            if (current_num_of_chased_tasks[workerid] >= current_PW[workerid].size())
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
        for (int i = 0; i < current_NextActiveWorker.size(); i++)
            current_ActiveWorker.push_back(current_NextActiveWorker[i]);
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

    ShowCTMatching(current_CT_Worker, current_Number_Worker);
}

void Basic_information::Grouping_Framework_AlternateDA(vector<TASK> &tasks, vector<WORKER> &workers, double Wmax, int Tmax, vector<double> &Sumdis, vector<vector<double>> &global_Worker_subTrajectoryDis)

{
    vector<CURRENT_TASK_GROUP> current_taskGroup;
    vector<CURRENT_WORKERS_GROUP> current_Group_worker;
    vector<double> current_Group_workerSumdis;
    vector<double> current_Group_workerAD;
    double current_window_endTime = tasks[0].startTime + Wmax;
    double current_window_startTime = tasks[0].startTime;
    int current_workID = global_Current_workID;

    vector<vector<double>> current_Group_worker_subTrajectoryDis;

    int sign = 0;
    int temp = 0;

    for (int i = 0; i < tasks.size(); i++)
    {

        TASK task = tasks[i];
        sign++;

        CURRENT_TASK_GROUP taskg;
        taskg.task = task;
        taskg.Original_Local = i;

        if ((sign - temp == Tmax) || (task.startTime >= current_window_endTime))
        {
            temp = current_taskGroup.size();
            current_workID = global_Current_workID;
            current_window_endTime = task.startTime;

            groupWork_according_TaskGroup(workers, Sumdis, current_Group_worker, current_Group_workerSumdis, current_window_endTime, current_workID, current_Group_workerAD, global_Worker_subTrajectoryDis, current_Group_worker_subTrajectoryDis);
            determine_Window_Task_Timeout(current_taskGroup, current_window_endTime);

            match_WorkerTask_AlternateDA(current_taskGroup, current_Group_worker, current_window_endTime, current_Group_workerAD, current_Group_worker_subTrajectoryDis);

            current_window_startTime = current_window_endTime;
            current_window_endTime += Wmax;

            updata_current_Info(current_taskGroup, current_Group_worker, current_Group_workerAD, current_Group_worker_subTrajectoryDis, current_Group_workerSumdis);
        }

        current_taskGroup.push_back(taskg);
    }

    if (current_taskGroup.size() > 0)
    {

        current_workID = global_Current_workID;
        current_window_startTime = current_taskGroup.back().task.startTime;

        groupWork_according_TaskGroup(workers, Sumdis, current_Group_worker, current_Group_workerSumdis, current_window_endTime, current_workID, current_Group_workerAD, global_Worker_subTrajectoryDis, current_Group_worker_subTrajectoryDis);

        determine_Window_Task_Timeout(current_taskGroup, current_window_endTime);

        match_WorkerTask_AlternateDA(current_taskGroup, current_Group_worker, current_window_endTime, current_Group_workerAD, current_Group_worker_subTrajectoryDis);
    }
}

void Basic_information::match_WorkerTask_AlternateDA(vector<CURRENT_TASK_GROUP> &current_taskGroup, vector<CURRENT_WORKERS_GROUP> &current_workerGroup, double current_window_endTime, vector<double> &current_Group_workerAD, vector<vector<double>> &current_Group_worker_subTrajectoryDis)
{
    int current_Number_Worker = current_workerGroup.size();
    int current_Number_Task = current_taskGroup.size();

    vector<double> current_detour_distance[current_Number_Task];
    vector<int> current_poi[current_Number_Task];
    vector<vector<pair<int, double>>> current_PT(Number_Task);
    vector<vector<pair<int, double>>> current_PW(Number_Worker);

    int current_CW_Task[current_Number_Task];

    int current_num_of_chased_worker[current_Number_Task] = {0};
    vector<int> current_ActiveTask;
    vector<int> current_NextActiveTask(current_ActiveTask);

    int current_num_of_chased_tasks[current_Number_Worker] = {0};
    vector<int> current_ActiveWorker;
    vector<int> current_NextActiveWorker(current_ActiveWorker);

    vector<vector<int>> current_CT_Worker(current_Number_Worker);
    double current_MaxDistanceTask[current_Number_Task] = {0.0};
    double current_task_NeedTime = 0.0;

    for (int i = 0; i < current_Number_Task; i++)
    {
        current_detour_distance[i] = vector<double>(current_Number_Worker, 0);

        current_poi[i] = vector<int>(current_Number_Worker, 0);
        current_CW_Task[i] = -1;
        /**
         * 这里与TSDA结合
         */
        if (current_PT[i].size() != 0)
        {
            current_ActiveTask.push_back(i);
        }
    }

    Compute_PTPW_Group_workerBatch(current_PT, current_PW, current_detour_distance, current_taskGroup, current_workerGroup, current_poi);

    for (int i = 0; i < current_workerGroup.size(); i++)
    {
        if (current_PW[i].size() != 0)
        {

            current_ActiveWorker.push_back(i);
        }
    }

    if (current_ActiveWorker.empty() && current_ActiveTask.empty())
    {
        return;
    }

    computeMaxDitanceTask(current_MaxDistanceTask, current_taskGroup, current_window_endTime);

    /**
     * 迭代匹配
     * 工人批量时，工人不可用或者任务不可用停止循环匹配
     * 未匹配或匹配失败的批次工人优先匹配
     * 偏好列表ORDER
     */

    int matchingTimes = 0;
    while (!current_ActiveWorker.empty() || !current_ActiveTask.empty())
    {

        while (!current_ActiveTask.empty())
        {

            current_NextActiveTask.clear();

            for (int i = 0; i < current_ActiveTask.size(); i++)
                current_NextActiveTask.push_back(current_ActiveTask[i]);

            int matchingnumber = 0, replacematching = 0;
            for (int i = 0; i < current_ActiveTask.size(); i++)
            {
                int taskid = current_ActiveTask[i];
                int order = current_num_of_chased_worker[taskid];
                int worker_to_chase = current_PT[taskid][order].first;

                current_num_of_chased_worker[taskid] = order + 1;

                if (IFTaskExist(worker_to_chase, taskid, current_PW) != current_PW[worker_to_chase].end())
                {

                    if (CurrentTask_Satisfy_TSDA(current_taskGroup, worker_to_chase, taskid, current_Group_workerAD, current_detour_distance, current_Group_worker_subTrajectoryDis, current_CT_Worker, current_poi, current_MaxDistanceTask, &current_task_NeedTime, current_window_endTime))

                    {

                        current_CW_Task[taskid] = worker_to_chase;
                        current_CT_Worker[worker_to_chase].push_back(taskid);

                        current_NextActiveTask.erase(find(current_NextActiveTask.begin(), current_NextActiveTask.end(), taskid));

                        Update_AD1(worker_to_chase, taskid, current_Group_workerAD, current_detour_distance);

                        global_workers[current_workerGroup[worker_to_chase].Original_Local].ADdis = current_Group_workerAD[worker_to_chase];

                        UpdateTaskDeadline_TSDA(current_taskGroup, false, 0, worker_to_chase, taskid, current_detour_distance, current_CT_Worker, current_poi, current_MaxDistanceTask, current_task_NeedTime);
                        matchingnumber++;
                    }
                    else
                    {

                        int MinReplaceTask = FindReplaceTaskNew_TSDA(worker_to_chase, taskid, current_PW, current_Group_workerAD, current_detour_distance, current_Group_worker_subTrajectoryDis, current_CT_Worker, current_poi, current_MaxDistanceTask, &current_task_NeedTime, current_taskGroup, current_window_endTime);
                        if (MinReplaceTask != -1)
                        {
                            current_CW_Task[taskid] = worker_to_chase;
                            current_CW_Task[MinReplaceTask] = -1;
                            current_CT_Worker[worker_to_chase].push_back(taskid);

                            current_CT_Worker[worker_to_chase].erase(find(current_CT_Worker[worker_to_chase].begin(), current_CT_Worker[worker_to_chase].end(), MinReplaceTask));

                            if (current_num_of_chased_worker[MinReplaceTask] < current_PT[MinReplaceTask].size())
                                current_NextActiveTask.push_back(MinReplaceTask);
                            current_NextActiveTask.erase(find(current_NextActiveTask.begin(), current_NextActiveTask.end(), taskid));

                            Update_AD2(worker_to_chase, taskid, MinReplaceTask, current_Group_workerAD, current_detour_distance);

                            global_workers[current_workerGroup[worker_to_chase].Original_Local].ADdis = current_Group_workerAD[worker_to_chase];

                            UpdateTaskDeadline_TSDA(current_taskGroup, true, MinReplaceTask, worker_to_chase, taskid, current_detour_distance, current_CT_Worker, current_poi, current_MaxDistanceTask, current_task_NeedTime);

                            current_MaxDistanceTask[MinReplaceTask] = current_taskGroup[MinReplaceTask].task.Deadline * speed;
                            global_MaxDistanceTask[current_taskGroup[MinReplaceTask].Original_Local] = current_MaxDistanceTask[MinReplaceTask];

                            replacematching++;
                        }
                        else
                        {
                        }
                    }
                }
                if (current_num_of_chased_worker[taskid] == current_PT[taskid].size())
                {

                    vector<int>::iterator iter1 = find(current_NextActiveTask.begin(), current_NextActiveTask.end(), taskid);
                    if (iter1 != current_NextActiveTask.end())
                    {
                        current_NextActiveTask.erase(find(current_NextActiveTask.begin(), current_NextActiveTask.end(), taskid));
                    }
                    vector<int>::iterator iter2 = find(current_NextActiveTask.begin(), current_NextActiveTask.end(), taskid);
                    if (iter2 == current_NextActiveTask.end())
                    {
                    }
                }
            }

            current_ActiveTask.clear();
            for (int i = 0; i < current_NextActiveTask.size(); i++)
                current_ActiveTask.push_back(current_NextActiveTask[i]);

            break;
        }

        while (!current_ActiveWorker.empty())
        {

            current_NextActiveWorker.clear();

            for (int i = 0; i < current_ActiveWorker.size(); i++)
                current_NextActiveWorker.push_back(current_ActiveWorker[i]);

            int matchingnumber = 0, replacematching = 0;
            for (int i = 0; i < current_ActiveWorker.size(); i++)
            {
                int workerid = current_ActiveWorker[i];
                int order = current_num_of_chased_tasks[workerid];

                int task_to_chase = current_PW[workerid][order].first;

                current_num_of_chased_tasks[workerid] = order + 1;

                if (IFWorkerExist(workerid, task_to_chase, current_PT) != current_PT[task_to_chase].end())
                {

                    if (CurrentTask_Satisfy_TSDA(current_taskGroup, workerid, task_to_chase, current_Group_workerAD, current_detour_distance, current_Group_worker_subTrajectoryDis, current_CT_Worker, current_poi, current_MaxDistanceTask, &current_task_NeedTime, current_window_endTime))

                    {

                        if (current_CW_Task[task_to_chase] == -1)
                        {

                            current_CW_Task[task_to_chase] = workerid;
                            current_CT_Worker[workerid].push_back(task_to_chase);

                            Update_AD1(workerid, task_to_chase, current_Group_workerAD, current_detour_distance);

                            global_workers[current_workerGroup[workerid].Original_Local].ADdis = current_Group_workerAD[workerid];

                            UpdateTaskDeadline_WSDA(current_taskGroup, false, 0, workerid, task_to_chase, current_detour_distance, current_CT_Worker, current_poi, current_MaxDistanceTask, current_task_NeedTime);
                            matchingnumber++;
                        }
                        else
                        {

                            int oldworkerid = current_CW_Task[task_to_chase];
                            if (GetIndex_PT(workerid, task_to_chase, current_PT) < GetIndex_PT(oldworkerid, task_to_chase, current_PT))
                            {

                                current_CW_Task[task_to_chase] = workerid;
                                current_CT_Worker[workerid].push_back(task_to_chase);

                                Update_AD1(workerid, task_to_chase, current_Group_workerAD, current_detour_distance);

                                current_CT_Worker[oldworkerid].erase(find(current_CT_Worker[oldworkerid].begin(), current_CT_Worker[oldworkerid].end(), task_to_chase));

                                Update_AD2_WSDA(oldworkerid, task_to_chase, current_Group_workerAD, current_detour_distance);

                                current_MaxDistanceTask[task_to_chase] = (current_taskGroup[task_to_chase].task.Deadline - current_window_endTime) * speed;
                                UpdateTaskDeadline_WSDA(current_taskGroup, true, oldworkerid, workerid, task_to_chase, current_detour_distance, current_CT_Worker, current_poi, current_MaxDistanceTask, current_task_NeedTime);
                                global_workers[current_workerGroup[oldworkerid].Original_Local].ADdis = current_Group_workerAD[oldworkerid];
                                global_workers[current_workerGroup[workerid].Original_Local].ADdis = current_Group_workerAD[workerid];
                                replacematching++;
                            }
                            else
                            {
                            }
                        }
                    }
                }
                if (current_num_of_chased_tasks[workerid] >= current_PW[workerid].size())
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
            for (int i = 0; i < current_NextActiveWorker.size(); i++)
                current_ActiveWorker.push_back(current_NextActiveWorker[i]);

            break;
        }
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

    ShowCTMatching(current_CT_Worker, current_Number_Worker);
}

/***
 * 读取全文需要的数据
 * 从本地获取数据
 * 以及对没用的数据进行生产
 * dataOption
 *      1：Berlin，   2：G-mission   3:T-drive
 */
bool optionDataset(bool distributionOption, int dataOption, Basic_information &info, vector<vector<double>> &global_Worker_subTrajectoryDis, vector<double> &Sumdis, double workEndtimeX, double taskEndtimeX, double rangeX, double scoreX)
{

    auto start1 = std::chrono::system_clock::now();
    std::time_t tt;
    tt = std::chrono::system_clock::to_time_t(start1);
    string t = ctime(&tt);

    if (dataOption == 1)
    {

        int Number_BusStop = 4346;

        DataManager datasets(distributionOption, workEndtimeX, taskEndtimeX, rangeX, scoreX, info.speed);

        datasets.ReadLocationForTask(info.global_tasks);

        datasets.Prodece_Task_Reward_Minscore_Deadline(info.global_tasks);

        datasets.Get_Trajectory_locations(info.global_workers, Number_BusStop);

        info.Caculate_Sumdist_Trajectory(Sumdis, global_Worker_subTrajectoryDis, info.global_workers);

        datasets.Prodece_Worker_endTime_range_score(info.global_workers, Sumdis);
    }
    else if (dataOption == 2)
    {
        if (info.Number_Worker > 200 || info.Number_Task > 713)
        {
            cout << "本数据集的工人数量最多为：200；任务数量为713" << endl;
            return false;
        }

        double rangeX = 0.3;

        DataManage_G_mission datasets(distributionOption, workEndtimeX, taskEndtimeX, rangeX, scoreX, info.speed);

        datasets.ReadLocationForTask(info.global_tasks);
        datasets.Prodece_Task_Reward_Minscore_Deadline(info.global_tasks);
        datasets.Get_Trajectory_locations(info.global_workers);
        info.Caculate_Sumdist_Trajectory(Sumdis, global_Worker_subTrajectoryDis, info.global_workers);
        datasets.Prodece_Worker_endTime_range_score(info.global_workers, Sumdis);
    }
    else if (dataOption == 3)
    {
        int Worker_Record = 5823;
        int Number_Trajectory_Point = 40;
        if (info.Number_Worker > Worker_Record)
        {
            cout << "工人数量最多为：5823" << endl;
            return false;
        }

        DataManager_T_Drive datasets(distributionOption, workEndtimeX, taskEndtimeX, rangeX, scoreX, info.speed, Worker_Record, info.Number_Worker, Number_Trajectory_Point);

        datasets.Get_Trajectory_locations(info.global_workers);

        datasets.Caculate_Sumdist_Trajectory(info.global_workers, Sumdis, global_Worker_subTrajectoryDis);

        datasets.Prodece_Worker_endTime_range_score(info.global_workers, Sumdis);

        datasets.Produce_Random_Task_ForTrajectoryPoint(info.Number_Task, Number_Trajectory_Point, info.global_workers, info.global_tasks);

        datasets.Prodece_Task_Reward_Minscore_Deadline(info.global_tasks);
    }
    else
    {

        return false;
    }

    return true;
}

/***
 *
 */

/***
 * 下面是多余的函数
 */

void Basic_information::Initialize_group_workNext(vector<double> &AD, vector<double> &Sumdis, vector<vector<double>> &current_Group_worker_subTrajectoryDis, int current_workID, double nowTime, vector<CURRENT_WORKERS_GROUP> &current_workerGroup, int ctwSize)
{

    for (int j = ctwSize; j < current_workerGroup.size(); j++)
    {

        if (Sumdis[j] < 0.01)
        {
            continue;
        }
        double cuAD = (current_workerGroup[j].worker.endTime - nowTime) * speed - Sumdis[j];

        if (current_workerGroup[j].worker.endTime <= nowTime || cuAD <= 0)
        {

            current_workerGroup.erase(current_workerGroup.begin() + j);
            Sumdis.erase(Sumdis.begin() + j);
            worknextMaxDistanceTask.erase(worknextMaxDistanceTask.begin() + j);
            current_Group_worker_subTrajectoryDis.erase(current_Group_worker_subTrajectoryDis.begin() + j);

            j--;
            continue;
        }

        AD.push_back(cuAD);

        if (worknextMaxDistanceTask[j].size() != global_CT_Worker[current_workerGroup[j].Original_Local].size())
        {
            cout << worknextMaxDistanceTask[j].size() << "hhh" << global_CT_Worker[current_workerGroup[j].Original_Local].size() << endl;

            cout << "这里有问题22" << endl;
            return;
        }
    }
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
int Basic_information::FindLatestWorkerNew_Greedy_workNext(int taskid, vector<int> &current_Group_worker_AW, vector<double> &current_Group_workerAD, vector<double> current_detour_distance[], vector<vector<double>> &current_Group_worker_subTrajectoryDis, vector<int> CT_Worker[], double MaxDistanceTask[], double *current_task_NeedTime, vector<CURRENT_TASK_GROUP> &current_taskGroup, vector<CURRENT_WORKERS_GROUP> &current_workerGroup, vector<int> poi[], double nowtime)
{

    int latest_workerid = -1;
    double mindist = __DBL_MAX__;
    for (int j = 0; j < current_Group_worker_AW.size(); j++)
    {
        int workerid = current_Group_worker_AW[j];

        if (current_taskGroup[taskid].task.Minscore <= current_workerGroup[workerid].worker.score)
        {
            double dist = Caculate_mindist(workerid, taskid, poi, current_taskGroup, current_workerGroup);

            current_detour_distance[taskid][workerid] = dist;
            if (current_workerGroup[workerid].worker.range >= dist)
            {

                if (current_taskGroup[taskid].task.Reward - (2 * dist * c) > 0)
                {

                    if (CurrentTask_Satisfy_workNext(current_taskGroup, current_workerGroup, workerid, taskid, current_Group_workerAD, current_detour_distance, current_Group_worker_subTrajectoryDis, CT_Worker, poi, MaxDistanceTask, current_task_NeedTime, nowtime))

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
bool Basic_information::IfReplace_workNext(int workerid, int taskid, int assignedtaskid, vector<double> &AD, vector<double> detour_distance[], vector<vector<double>> &Worker_subTrajectoryDis, vector<vector<int>> &CT_Worker, vector<int> poi[], double MaxDistanceTask[], double *current_task_NeedTime, double beforesumDis, vector<CURRENT_TASK_GROUP> &current_taskGroup, vector<CURRENT_WORKERS_GROUP> &current_workerGroup, double nowtime)
{

    int isEx = worknextMaxDistanceTask[workerid].size();
    if (worknextMaxDistanceTask.size() != current_workerGroup.size())
    {
        cout << "大小不一样" << endl;
    }
    int gisEX = global_CT_Worker[current_workerGroup[workerid].Original_Local].size();
    if (gisEX < isEx)
    {
        cout << "不应该出现" << endl;
    }

    int assignedNumber = CT_Worker[workerid].size() + isEx;
    if (assignedNumber == 0)
        return false;

    double ADD = AD[workerid] + 2 * detour_distance[assignedtaskid][workerid] - 2 * detour_distance[taskid][workerid];

    if (ADD < 0)
    {

        return false;
    }

    int current_detourid = poi[taskid][workerid];

    bool flag_first = false;

    int replacepoi = poi[assignedtaskid][workerid];
    if (replacepoi <= current_detourid)
        beforesumDis = beforesumDis - 2 * detour_distance[assignedtaskid][workerid];

    double Time_TO_Task;
    Time_TO_Task = (detour_distance[taskid][workerid] + Worker_subTrajectoryDis[workerid][current_detourid] + beforesumDis) / speed;
    *current_task_NeedTime = Time_TO_Task;

    if (current_taskGroup[taskid].task.Deadline > Time_TO_Task + nowtime)
    {

        int noinflu_count = 0;
        for (int i = 0; i < CT_Worker[workerid].size(); i++)
        {
            int assigntask_id = CT_Worker[workerid][i];
            int detourpoint = poi[assigntask_id][workerid];
            double temporalMaxd = 0;

            if (replacepoi < detourpoint && assigntask_id != assignedtaskid)
            {

                temporalMaxd = MaxDistanceTask[assigntask_id] + (2 * detour_distance[assignedtaskid][workerid]);
            }

            if (current_detourid < detourpoint && assigntask_id != assignedtaskid)
            {

                if (temporalMaxd < (2 * detour_distance[taskid][workerid]))
                {

                    return false;
                }
                else
                {
                }
            }

            if (current_detourid > detourpoint)
            {
                noinflu_count++;
            }
        }
        int globalwid = current_workerGroup[workerid].Original_Local;
        for (int i = 0; i < isEx; i++)
        {
            int assigntask_id = global_CT_Worker[globalwid][i];
            int detourpoint = global_POI[assigntask_id][globalwid];
            double temporalMaxd = 0;

            int global_assignedtaskid = current_taskGroup[assignedtaskid].Original_Local;
            if (replacepoi < detourpoint && assigntask_id != global_assignedtaskid)
            {

                temporalMaxd = global_MaxDistanceTask[assigntask_id] + (2 * detour_distance[assignedtaskid][workerid]);
            }

            if (current_detourid < detourpoint && assigntask_id != current_taskGroup[assignedtaskid].Original_Local)
            {

                if (temporalMaxd < (2 * detour_distance[taskid][workerid]))
                {

                    return false;
                }
                else
                {
                }
            }

            if (current_detourid > detourpoint)
            {
                noinflu_count++;
            }
        }

        if (noinflu_count == assignedNumber)
        {

            return true;
        }

        return true;
    }
    else
    {

        return false;
    }
}

bool Basic_information::CurrentTask_Satisfy_TSDA_workNext(vector<CURRENT_TASK_GROUP> &current_taskGroup, vector<CURRENT_WORKERS_GROUP> &current_workerGroup, int workerid, int taskid, vector<double> &AD, vector<double> current_detour_distance[], vector<vector<double>> &current_Worker_subTrajectoryDis, vector<vector<int>> &CT_Worker, vector<int> poi[], double MaxDistanceTask[], double *current_task_NeedTime, double nowtime)
{
    int isEx = worknextMaxDistanceTask[workerid].size();
    int gisEX = global_CT_Worker[current_workerGroup[workerid].Original_Local].size();
    if (worknextMaxDistanceTask.size() != current_workerGroup.size())
    {
        cout << "大小不一样" << endl;
    }

    if (gisEX < isEx)
    {
        cout << "不应该出现" << endl;
    }

    int assignedNumber = CT_Worker[workerid].size() + isEx;
    if (assignedNumber == Capacity)
    {
        return false;
    }

    double ADD = AD[workerid] - 2 * current_detour_distance[taskid][workerid];

    if (ADD < 0)
    {
        return false;
    }

    int current_detourid = poi[taskid][workerid];

    bool flag_first = false;
    double SumdetouDis = 0.0;
    for (int i = 0; i < CT_Worker[workerid].size(); i++)
    {
        int assigntask_id = CT_Worker[workerid][i];
        int detourpoint = poi[assigntask_id][workerid];

        if (current_detourid >= detourpoint)
        {
            SumdetouDis = SumdetouDis + 2 * current_detour_distance[assigntask_id][workerid];
        }
        if (detourpoint == 0)
        {
            flag_first = true;
        }
    }

    for (int i = 0; i < isEx; i++)
    {

        int assigntask_id = global_CT_Worker[current_workerGroup[workerid].Original_Local][i];
        int detourpoint = global_POI[assigntask_id][current_workerGroup[workerid].Original_Local];

        if (current_detourid >= detourpoint)
        {

            SumdetouDis = SumdetouDis + 2 * global_detour_distance[assigntask_id][current_workerGroup[workerid].Original_Local];
        }
        if (detourpoint == 0)
        {
            flag_first = true;
        }
    }

    double Time_TO_Task;

    Time_TO_Task = (current_detour_distance[taskid][workerid] + current_Worker_subTrajectoryDis[workerid][current_detourid] + SumdetouDis) / speed;
    *current_task_NeedTime = Time_TO_Task;

    if (current_taskGroup[taskid].task.Deadline > Time_TO_Task + nowtime)
    {

        if (assignedNumber == 0)
        {

            return true;
        }
        else
        {
            int noinflu_count = 0;
            for (int i = 0; i < CT_Worker[workerid].size(); i++)
            {
                int assigntask_id = CT_Worker[workerid][i];
                int detourpoint = poi[assigntask_id][workerid];

                if (current_detourid < detourpoint)
                {

                    if (MaxDistanceTask[assigntask_id] < (2 * current_detour_distance[taskid][workerid]))
                    {

                        return false;
                    }
                }

                if (current_detourid > detourpoint)
                {
                    noinflu_count++;
                }
            }
            int isEx = worknextMaxDistanceTask[workerid].size();
            int gisEX = global_CT_Worker[current_workerGroup[workerid].Original_Local].size();
            if (worknextMaxDistanceTask.size() != current_workerGroup.size())
            {
                cout << "大小不一样" << endl;
            }
            if (gisEX < isEx)
            {
                cout << "不应该出现" << endl;
            }

            for (int i = 0; i < isEx; i++)
            {
                int assigntask_id = global_CT_Worker[current_workerGroup[workerid].Original_Local][i];
                int detourpoint = global_POI[assigntask_id][current_workerGroup[workerid].Original_Local];

                if (current_detourid < detourpoint)
                {

                    if (global_MaxDistanceTask[assigntask_id] < (2 * current_detour_distance[taskid][workerid]))
                    {

                        return false;
                    }
                    else
                    {
                    }
                }

                if (current_detourid > detourpoint)
                {
                    noinflu_count++;
                }
            }

            if (noinflu_count == assignedNumber)
            {

                return true;
            }

            return true;
        }
    }
    else
    {

        return false;
    }
}

bool Basic_information::CurrentTask_Satisfy_workNext(vector<CURRENT_TASK_GROUP> &current_taskGroup, vector<CURRENT_WORKERS_GROUP> &current_workerGroup, int workerid, int taskid, vector<double> &AD, vector<double> current_detour_distance[], vector<vector<double>> &current_Worker_subTrajectoryDis, vector<int> CT_Worker[], vector<int> poi[], double MaxDistanceTask[], double *current_task_NeedTime, double nowtime)
{
    int isEx = worknextMaxDistanceTask[workerid].size();
    int gisEX = global_CT_Worker[current_workerGroup[workerid].Original_Local].size();
    if (worknextMaxDistanceTask.size() != current_workerGroup.size())
    {
        cout << "大小不一样" << endl;
    }
    if (gisEX < isEx)
    {
        cout << "不应该出现" << endl;
    }

    int assignedNumber = CT_Worker[workerid].size() + isEx;
    if (assignedNumber == Capacity)
    {
        return false;
    }

    double ADD = AD[workerid] - 2 * current_detour_distance[taskid][workerid];

    if (ADD < 0)
    {

        return false;
    }

    int current_detourid = poi[taskid][workerid];

    bool flag_first = false;
    double SumdetouDis = 0.0;
    for (int i = 0; i < CT_Worker[workerid].size(); i++)
    {

        int assigntask_id = CT_Worker[workerid][i];
        int detourpoint = poi[assigntask_id][workerid];

        if (current_detourid >= detourpoint)
        {
            SumdetouDis = SumdetouDis + 2 * current_detour_distance[assigntask_id][workerid];
        }
        if (detourpoint == 0)
        {
            flag_first = true;
        }
    }

    for (int i = 0; i < isEx; i++)
    {

        int assigntask_id = global_CT_Worker[current_workerGroup[workerid].Original_Local][i];

        int detourpoint = global_POI[assigntask_id][current_workerGroup[workerid].Original_Local];

        if (current_detourid >= detourpoint)
        {

            SumdetouDis = SumdetouDis + 2 * global_detour_distance[assigntask_id][current_workerGroup[workerid].Original_Local];
        }
        if (detourpoint == 0)
        {
            flag_first = true;
        }
    }

    double Time_TO_Task;

    Time_TO_Task = (current_detour_distance[taskid][workerid] + current_Worker_subTrajectoryDis[workerid][current_detourid] + SumdetouDis) / speed;

    *current_task_NeedTime = Time_TO_Task;

    if (current_taskGroup[taskid].task.Deadline > Time_TO_Task + nowtime)
    {

        if (assignedNumber == 0)
        {

            return true;
        }
        else
        {
            int noinflu_count = 0;
            for (int i = 0; i < isEx; i++)
            {
                int assigntask_id = global_CT_Worker[current_workerGroup[workerid].Original_Local][i];
                int detourpoint = global_POI[assigntask_id][current_workerGroup[workerid].Original_Local];

                if (current_detourid < detourpoint)
                {

                    if (global_MaxDistanceTask[assigntask_id] < (2 * current_detour_distance[taskid][workerid]))
                    {

                        return false;
                    }
                }

                if (current_detourid > detourpoint)
                {
                    noinflu_count++;
                }
            }

            for (int i = 0; i < CT_Worker[workerid].size(); i++)
            {
                int assigntask_id = CT_Worker[workerid][i];
                int detourpoint = poi[assigntask_id][workerid];

                if (current_detourid < detourpoint)
                {

                    if (MaxDistanceTask[assigntask_id] < (2 * current_detour_distance[taskid][workerid]))
                    {

                        return false;
                    }
                }

                if (current_detourid > detourpoint)
                {
                    noinflu_count++;
                }
            }
            if (noinflu_count == assignedNumber)
            {

                return true;
            }

            return true;
        }
    }
    else
    {

        return false;
    }
}

void Basic_information::UpdateTaskDeadline_workNext(vector<CURRENT_TASK_GROUP> &current_taskGroup, vector<CURRENT_WORKERS_GROUP> &current_workerGroup, int workerid, int taskid, vector<double> current_detour_distance[], vector<int> CT_Worker[], vector<int> poi[], double MaxDistanceTask[], double current_task_NeedTime)
{
    int isEx = worknextMaxDistanceTask[workerid].size();
    if (worknextMaxDistanceTask.size() != current_workerGroup.size())
    {
        cout << "大小不一样" << endl;
    }
    int gisEX = global_CT_Worker[current_workerGroup[workerid].Original_Local].size();
    if (gisEX < isEx)
    {
        cout << "不应该出现" << endl;
    }

    int detourpoint = poi[taskid][workerid];
    for (int i = 0; i < CT_Worker[workerid].size(); i++)
    {
        int assigntask_id = CT_Worker[workerid][i];
        int assignedpoint = poi[assigntask_id][workerid];

        if (assigntask_id == taskid)
        {

            MaxDistanceTask[taskid] = MaxDistanceTask[taskid] - current_task_NeedTime * speed;
        }
        else
        {

            if (detourpoint < assignedpoint)
            {

                MaxDistanceTask[assigntask_id] = MaxDistanceTask[assigntask_id] - (2 * current_detour_distance[taskid][workerid]);
            }
        }
    }

    for (int i = 0; i < isEx; i++)
    {

        int assigntask_id = global_CT_Worker[current_workerGroup[workerid].Original_Local][i];
        int assignedpoint = global_POI[assigntask_id][current_workerGroup[workerid].Original_Local];

        if (assigntask_id == current_taskGroup[taskid].Original_Local)
        {

            global_MaxDistanceTask[assigntask_id] -= current_task_NeedTime * speed;
        }
        else
        {

            if (detourpoint < assignedpoint)
            {

                global_MaxDistanceTask[assigntask_id] -= (2 * current_detour_distance[taskid][workerid]);
            }
        }
    }
}

void Basic_information::updata_current_Info_workNext(vector<CURRENT_TASK_GROUP> &current_taskGroup, vector<CURRENT_WORKERS_GROUP> &current_workerGroup, vector<double> &current_Group_workerAD, vector<vector<double>> &current_Group_worker_subTrajectoryDis, vector<double> &current_Group_workerSumdis)
{
    vector<CURRENT_WORKERS_GROUP> temp_work_group;
    vector<double> current_Group_workerAD1;
    int i = 0;
    for (vector<CURRENT_WORKERS_GROUP>::iterator it_curret_Worker = current_workerGroup.begin(); it_curret_Worker != current_workerGroup.end(); ++it_curret_Worker)
    {

        if ((*it_curret_Worker).sign)
        {
            temp_work_group.push_back(*it_curret_Worker);
            current_Group_workerAD1.push_back(*(current_Group_workerAD.begin() + i));
        }
        i++;
    }
    current_workerGroup.clear();
    current_workerGroup = temp_work_group;
    temp_work_group.clear();
    current_Group_workerAD.clear();
    current_Group_workerAD = current_Group_workerAD1;
    current_Group_workerAD1.clear();

    vector<CURRENT_TASK_GROUP> temp_task_group;

    for (vector<CURRENT_TASK_GROUP>::iterator it_curret_task = current_taskGroup.begin(); it_curret_task != current_taskGroup.end(); ++it_curret_task)
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
void Basic_information::Grouping_Framework_Greedy_workNext(vector<TASK> &tasks, vector<WORKER> &workers, double Wmax, int Tmax, vector<double> &Sumdis, vector<vector<double>> &global_Worker_subTrajectoryDis)

{
    vector<CURRENT_TASK_GROUP> current_taskGroup;
    vector<CURRENT_WORKERS_GROUP> current_Group_worker;
    vector<double> current_Group_workerSumdis;
    vector<double> current_Group_workerAD;
    double current_window_endTime = tasks[0].startTime + Wmax;
    double current_window_startTime = tasks[0].startTime;
    int current_workID = global_Current_workID;

    vector<vector<double>> current_Group_worker_subTrajectoryDis;

    int sign = 0;
    int temp = 0;

    for (int i = 0; i < tasks.size(); i++)
    {

        TASK task = tasks[i];
        sign++;

        CURRENT_TASK_GROUP taskg;
        taskg.task = task;
        taskg.Original_Local = i;
        current_taskGroup.push_back(taskg);

        if ((sign - temp == Tmax) || (task.startTime >= current_window_endTime))
        {
            temp = current_taskGroup.size();
            current_workID = global_Current_workID;

            current_window_endTime = task.startTime;

            groupWork_according_TaskGroup_workNext(workers, Sumdis, current_Group_worker, current_Group_workerSumdis, current_window_startTime, current_window_endTime, current_workID, current_Group_workerAD, global_Worker_subTrajectoryDis, current_Group_worker_subTrajectoryDis);
            determine_Window_Task_Timeout(current_taskGroup, current_window_endTime);

            match_WorkerTask_Greedy_workNext(current_taskGroup, current_Group_worker, current_window_endTime, current_Group_workerAD, current_Group_worker_subTrajectoryDis);

            current_window_startTime = current_window_endTime;
            current_window_endTime += Wmax;

            updata_current_Info_workNext(current_taskGroup, current_Group_worker, current_Group_workerAD, current_Group_worker_subTrajectoryDis, current_Group_workerSumdis);
        }
    }

    if (current_taskGroup.size() > 0)
    {

        current_workID = global_Current_workID;
        current_window_startTime = current_taskGroup.back().task.startTime;

        groupWork_according_TaskGroup_workNext(workers, Sumdis, current_Group_worker, current_Group_workerSumdis, current_window_startTime, current_window_endTime, current_workID, current_Group_workerAD, global_Worker_subTrajectoryDis, current_Group_worker_subTrajectoryDis);

        determine_Window_Task_Timeout(current_taskGroup, current_window_endTime);

        match_WorkerTask_Greedy_workNext(current_taskGroup, current_Group_worker, current_window_endTime, current_Group_workerAD, current_Group_worker_subTrajectoryDis);
        updata_current_Info_workNext(current_taskGroup, current_Group_worker, current_Group_workerAD, current_Group_worker_subTrajectoryDis, current_Group_workerSumdis);
    }
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

void Basic_information::groupWork_according_TaskGroup_workNext(vector<WORKER> &workers, vector<double> &Sumdis, vector<CURRENT_WORKERS_GROUP> &current_workerGroup,
                                                               vector<double> &current_Group_workerSumdis, double current_window_startTime, double nowTime, int current_workID, vector<double> &current_Group_workerAD,
                                                               vector<vector<double>> &global_Worker_subTrajectoryDis, vector<vector<double>> &current_Group_worker_subTrajectoryDis)
{
    CURRENT_WORKERS_GROUP workerg;

    int ttt = current_workID;

    for (int j = 0; j < current_workerGroup.size(); j++)
    {
        current_Group_workerAD[j] -= (nowTime - current_window_startTime) * speed;

        if (current_workerGroup[j].worker.endTime <= nowTime || current_Group_workerAD[j] <= 0 || current_workerGroup[j].sign == false)
        {
            current_workerGroup.erase(current_workerGroup.begin() + j);
            current_Group_workerAD.erase(current_Group_workerAD.begin() + j);
            current_Group_workerSumdis.erase(current_Group_workerSumdis.begin() + j);
            current_Group_worker_subTrajectoryDis.erase(current_Group_worker_subTrajectoryDis.begin() + j);

            j--;
            continue;
        }
        for (int m = 0; m < global_CT_Worker[current_workerGroup[j].Original_Local].size(); m++)
        {
            int tempGlobalTaskId = global_CT_Worker[current_workerGroup[j].Original_Local][m];
            global_MaxDistanceTask[tempGlobalTaskId] -= (nowTime - current_window_startTime) * speed;
            if (global_tasks[tempGlobalTaskId].Deadline <= nowTime || global_MaxDistanceTask[tempGlobalTaskId] <= 0)
            {
                current_workerGroup.erase(current_workerGroup.begin() + j);
                current_Group_workerAD.erase(current_Group_workerAD.begin() + j);
                current_Group_workerSumdis.erase(current_Group_workerSumdis.begin() + j);
                current_Group_worker_subTrajectoryDis.erase(current_Group_worker_subTrajectoryDis.begin() + j);
                j--;

                break;
            }
        }
    }
    worknextMaxDistanceTask.clear();
    worknextMaxDistanceTask.resize(0);
    nextWorkSet.clear();
    nextWorkSet.resize(0);

    for (int j = 0; j < current_workerGroup.size(); j++)
    {

        vector<double> wtask(0);

        for (int m = 0; m < global_CT_Worker[current_workerGroup[j].Original_Local].size(); m++)
        {
            int tempGlobalTaskId = global_CT_Worker[current_workerGroup[j].Original_Local][m];

            wtask.push_back(global_MaxDistanceTask[tempGlobalTaskId]);
        }

        worknextMaxDistanceTask.push_back(wtask);
        if (global_CT_Worker[current_workerGroup[j].Original_Local].size() > 0)
            nextWorkSet.push_back(current_workerGroup[j].Original_Local);
    }
    if (worknextMaxDistanceTask.size() != current_workerGroup.size())
    {
        cout << "大小从第一阶段就不一样" << worknextMaxDistanceTask.size() << "\t" << current_workerGroup.size() << endl;
    }

    int ctwSize = current_workerGroup.size();

    for (auto it = (workers.begin() + current_workID); it != workers.end();)
    {
        if (it->startTime < nowTime && it->endTime > nowTime)
        {

            workerg.worker = *it;
            workerg.Original_Local = global_Current_workID;
            current_workerGroup.push_back(workerg);
            current_Group_workerSumdis.push_back(Sumdis[global_Current_workID]);

            current_Group_worker_subTrajectoryDis.push_back(global_Worker_subTrajectoryDis[global_Current_workID]);
            worknextMaxDistanceTask.push_back(vector<double>(0));
        }
        ttt++;

        if (it->startTime >= nowTime)
        {

            Initialize_group_workNext(current_Group_workerAD, current_Group_workerSumdis, current_Group_worker_subTrajectoryDis, current_workID, nowTime, current_workerGroup, ctwSize);

            break;
        }
        ++global_Current_workID;

        ++it;
    }

    if (worknextMaxDistanceTask.size() != current_workerGroup.size())
    {
        cout << "大小从一开始就不一样" << worknextMaxDistanceTask.size() << "\t" << current_workerGroup.size() << endl;
    }
}

/***
 * 对任务和工人执行配对（这里嵌套诗婷的代码）
 * 将原本诗婷写在main函数的内容嵌套进来
 * 对于每个任务，计算任务的可用工人：、
 *      首先，满足range约束，满足最小声誉分数约束；
 *      其次，在匹配时再判断是否满足deadline约束，容量约束
 *      接着，然后计算与每个可用工人的距离，选择最近的工人
 */
void Basic_information::match_WorkerTask_Greedy_workNext(vector<CURRENT_TASK_GROUP> &current_taskGroup, vector<CURRENT_WORKERS_GROUP> &current_workerGroup, double current_window_endTime, vector<double> &current_Group_workerAD, vector<vector<double>> &current_Group_worker_subTrajectoryDis)
{
    int current_Number_Worker = current_workerGroup.size();
    int current_Number_Task = current_taskGroup.size();
    vector<double> current_detour_distance[current_Number_Task];
    vector<int> current_poi[current_Number_Task];
    for (int i = 0; i < current_Number_Task; i++)
    {
        current_detour_distance[i] = vector<double>(current_Number_Worker, 0);

        current_poi[i] = vector<int>(current_Number_Worker, 0);
    }

    vector<int> current_CT_Worker[current_Number_Worker];
    double current_MaxDistanceTask[current_Number_Task];
    double current_task_NeedTime = 0.0;

    computeMaxDitanceTask(current_MaxDistanceTask, current_taskGroup, current_window_endTime);

    vector<int> current_Group_worker_AW;

    for (int i = 0; i < current_Number_Worker; i++)
    {
        current_Group_worker_AW.push_back(i);
    }

    for (int i = 0; i < current_Number_Task; i++)
    {

        if (current_Group_worker_AW.size() != 0)
        {
            int workerid = -1;

            workerid = FindLatestWorkerNew_Greedy_workNext(i, current_Group_worker_AW, current_Group_workerAD, current_detour_distance, current_Group_worker_subTrajectoryDis, current_CT_Worker, current_MaxDistanceTask, &current_task_NeedTime, current_taskGroup, current_workerGroup, current_poi, current_window_endTime);

            if (workerid != -1)
            {

                current_taskGroup[i].sign = false;
                global_MaxDistanceTask[current_taskGroup[i].Original_Local] = current_MaxDistanceTask[i];
                current_CT_Worker[workerid].push_back(i);
                global_CT_Worker[current_workerGroup[workerid].Original_Local].push_back(current_taskGroup[i].Original_Local);

                current_Group_workerAD[workerid] = current_Group_workerAD[workerid] - 2 * current_detour_distance[i][workerid];
                global_workers[current_workerGroup[workerid].Original_Local].ADdis = current_Group_workerAD[workerid];

                UpdateTaskDeadline_workNext(current_taskGroup, current_workerGroup, workerid, i, current_detour_distance, current_CT_Worker, current_poi, current_MaxDistanceTask, current_task_NeedTime);
                int isEx = worknextMaxDistanceTask[workerid].size();
                if (worknextMaxDistanceTask.size() != current_workerGroup.size())
                {
                    cout << "大小不一样" << endl;
                }
                int gisEX = global_CT_Worker[current_workerGroup[workerid].Original_Local].size();
                if (gisEX < isEx)
                {
                    cout << "不应该出现" << endl;
                }

                if (current_CT_Worker[workerid].size() == Capacity - isEx)
                {
                    current_workerGroup[workerid].sign = false;
                    for (vector<int>::iterator it = current_Group_worker_AW.begin(); it != current_Group_worker_AW.end(); ++it)
                    {
                        if (*it == workerid)
                        {
                            it = current_Group_worker_AW.erase(it);

                            break;
                        }
                    }
                }
            }
            else
            {
            }
        }
        else
            break;
    }

    ShowCTMatching(current_CT_Worker, current_Number_Worker);
}

/***
 *
 * 开始workerBatch的代码
 */
void Basic_information::Grouping_Framework_WorkerBatch_workNext(vector<TASK> &tasks, vector<WORKER> &workers, double Wmax, int Tmax, vector<double> &Sumdis, vector<vector<double>> &global_Worker_subTrajectoryDis)

{
    vector<CURRENT_TASK_GROUP> current_taskGroup;
    vector<CURRENT_WORKERS_GROUP> current_Group_worker;
    vector<double> current_Group_workerSumdis;
    vector<double> current_Group_workerAD;
    double current_window_endTime = tasks[0].startTime + Wmax;
    double current_window_startTime = tasks[0].startTime;
    int current_workID = global_Current_workID;

    vector<vector<double>> current_Group_worker_subTrajectoryDis;

    int sign = 0;
    int temp = 0;

    for (int i = 0; i < tasks.size(); i++)
    {

        TASK task = tasks[i];
        sign++;

        CURRENT_TASK_GROUP taskg;
        taskg.task = task;
        taskg.Original_Local = i;
        current_taskGroup.push_back(taskg);

        if ((sign - temp == Tmax) || (task.startTime >= current_window_endTime))
        {
            temp = current_taskGroup.size();
            current_workID = global_Current_workID;
            current_window_endTime = task.startTime;

            groupWork_according_TaskGroup_workNext(workers, Sumdis, current_Group_worker, current_Group_workerSumdis, current_window_startTime, current_window_endTime, current_workID, current_Group_workerAD, global_Worker_subTrajectoryDis, current_Group_worker_subTrajectoryDis);
            determine_Window_Task_Timeout(current_taskGroup, current_window_endTime);

            match_WorkerTask_workerBatch_workNext(current_taskGroup, current_Group_worker, current_window_endTime, current_Group_workerAD, current_Group_worker_subTrajectoryDis);

            current_window_startTime = current_window_endTime;
            current_window_endTime += Wmax;

            updata_current_Info_workNext(current_taskGroup, current_Group_worker, current_Group_workerAD, current_Group_worker_subTrajectoryDis, current_Group_workerSumdis);
        }
    }

    if (current_taskGroup.size() > 0)
    {

        current_workID = global_Current_workID;
        current_window_startTime = current_taskGroup.back().task.startTime;

        groupWork_according_TaskGroup_workNext(workers, Sumdis, current_Group_worker, current_Group_workerSumdis, current_window_startTime, current_window_endTime, current_workID, current_Group_workerAD, global_Worker_subTrajectoryDis, current_Group_worker_subTrajectoryDis);

        determine_Window_Task_Timeout(current_taskGroup, current_window_endTime);

        match_WorkerTask_workerBatch_workNext(current_taskGroup, current_Group_worker, current_window_endTime, current_Group_workerAD, current_Group_worker_subTrajectoryDis);
    }
}

/**
 * 求解分组内工人和任务的偏好
 */
void Basic_information::Compute_PTPW_Group_workerBatch_workNext(vector<vector<pair<int, double>>> &current_PT, vector<vector<pair<int, double>>> &current_PW, vector<double> current_detour_distance[], vector<CURRENT_TASK_GROUP> &current_taskGroup, vector<CURRENT_WORKERS_GROUP> &current_workerGroup, vector<int> current_poi[])
{

    int Number_current_taskGroup = current_taskGroup.size();
    int Number_current_workerGroup = current_workerGroup.size();
    for (int i = 0; i < Number_current_taskGroup; i++)
    {
        for (int j = 0; j < Number_current_workerGroup; j++)
        {
            if (!(current_taskGroup[i].task.Deadline < current_workerGroup[j].worker.startTime || current_taskGroup[i].task.startTime > current_workerGroup[j].worker.endTime))
            {
                if (current_taskGroup[i].task.Minscore <= current_workerGroup[j].worker.score)
                {

                    current_detour_distance[i][j] = Caculate_mindist(j, i, current_poi, current_taskGroup, current_workerGroup);

                    if (current_detour_distance[i][j] <= current_workerGroup[j].worker.range)
                    {

                        double preference1 = current_taskGroup[i].task.Reward - 2 * current_detour_distance[i][j] * c;
                        if (preference1 > 0)
                        {
                            current_PW[j].push_back(make_pair(i, preference1));
                        }

                        double preference2 = current_workerGroup[j].worker.score;
                        if (current_taskGroup[i].task.Minscore <= preference2)
                        {
                            current_PT[i].push_back(make_pair(j, preference2));
                        }
                    }
                }
            }
        }
    }

    for (int i = 0; i < current_workerGroup.size(); i++)
    {

        sort(current_PW[i].begin(), current_PW[i].end(), cmp);
    }
}

void Basic_information::match_WorkerTask_workerBatch_workNext(vector<CURRENT_TASK_GROUP> &current_taskGroup, vector<CURRENT_WORKERS_GROUP> &current_workerGroup, double current_window_endTime, vector<double> &current_Group_workerAD, vector<vector<double>> &current_Group_worker_subTrajectoryDis)
{
    int current_Number_Worker = current_workerGroup.size();
    int current_Number_Task = current_taskGroup.size();

    vector<double> current_detour_distance[current_Number_Task];
    vector<int> current_poi[current_Number_Task];
    vector<vector<pair<int, double>>> current_PT(current_Number_Task);
    vector<vector<pair<int, double>>> current_PW(current_Number_Worker);
    for (int i = 0; i < current_Number_Task; i++)
    {
        current_detour_distance[i] = vector<double>(current_Number_Worker, 0);

        current_poi[i] = vector<int>(current_Number_Worker, 0);
    }
    Compute_PTPW_Group_workerBatch_workNext(current_PT, current_PW, current_detour_distance, current_taskGroup, current_workerGroup, current_poi);

    vector<int> current_CT_Worker[current_Number_Worker];
    double current_MaxDistanceTask[current_Number_Task];
    double current_task_NeedTime = 0.0;

    computeMaxDitanceTask(current_MaxDistanceTask, current_taskGroup, current_window_endTime);

    /**
     * 迭代匹配
     * 工人批量时，工人不可用或者任务不可用停止循环匹配
     * 未匹配或匹配失败的批次工人优先匹配
     * 偏好列表ORDER
     */

    iterator_Match_WorkBatch_workNext(current_workerGroup, current_taskGroup, current_PW, current_PT, current_Group_workerAD, current_detour_distance, current_Group_worker_subTrajectoryDis,
                                      current_CT_Worker, current_MaxDistanceTask, &current_task_NeedTime, current_poi, current_window_endTime);

    ShowCTMatching(current_CT_Worker, current_Number_Worker);
}

void Basic_information::iterator_Match_WorkBatch_workNext(vector<CURRENT_WORKERS_GROUP> &current_workerGroup, vector<CURRENT_TASK_GROUP> &current_taskGroup, vector<vector<pair<int, double>>> &current_PW, vector<vector<pair<int, double>>> &current_PT,
                                                          vector<double> &current_Group_workerAD, vector<double> current_detour_distance[], vector<vector<double>> &current_Group_worker_subTrajectoryDis,
                                                          vector<int> current_CT_Worker[], double MaxDistanceTask[], double *current_task_NeedTime, vector<int> current_poi[], double nowtime)
{
    int Number_Worker = current_workerGroup.size();
    int Number_Task = current_taskGroup.size();
    int Worker_Available[Number_Worker] = {0};
    int Task_Available[Number_Task] = {0};
    int count_NAWorker = 0;
    int count_NATask = 0;
    int CurrentTaskInPW[Number_Worker] = {0};
    int unmatchC = 0;
    for (int i = 0; i < current_workerGroup.size(); i++)
    {
        if (current_PW[i].size() == 0)
        {
            Worker_Available[i] = 3;
            unmatchC++;
        }
        else
        {
        }
    }
    int diedai = 0;
    while ((count_NAWorker + unmatchC) < Number_Worker && count_NATask < Number_Task)
    {

        diedai++;

        vector<int> updateObj;

        for (int flag = 0; flag < 2; flag++)
        {
            for (int i = 0; i < Number_Worker; i++)

            {

                if (Worker_Available[i] == flag)
                {

                    int orderinPW = CurrentTaskInPW[i];

                    int current_task_id = current_PW[i][orderinPW].first;

                    CurrentTaskInPW[i]++;
                    if (Task_Available[current_task_id] == 0)
                    /*
                    && (find_if(current_PT[current_task_id].begin(), current_PT[current_task_id].end(), [i](const std::pair<int, double> &p)
                                                                         { return p.second == i; }) != current_PT[current_task_id].end())

                    */
                    {

                        if (CurrentTask_Satisfy_workNext(current_taskGroup, current_workerGroup, i, current_task_id, current_Group_workerAD, current_detour_distance, current_Group_worker_subTrajectoryDis, current_CT_Worker, current_poi, MaxDistanceTask, current_task_NeedTime, nowtime))
                        {

                            Task_Available[current_task_id] = 1;
                            count_NATask++;
                            current_CT_Worker[i].push_back(current_task_id);

                            current_taskGroup[current_task_id].sign = false;
                            global_CT_Worker[current_workerGroup[i].Original_Local].push_back(current_taskGroup[current_task_id].Original_Local);
                            global_MaxDistanceTask[current_taskGroup[current_task_id].Original_Local] = MaxDistanceTask[current_task_id];

                            current_Group_workerAD[i] = current_Group_workerAD[i] - 2 * current_detour_distance[current_task_id][i];

                            UpdateTaskDeadline_workNext(current_taskGroup, current_workerGroup, i, current_task_id, current_detour_distance, current_CT_Worker, current_poi, MaxDistanceTask, *current_task_NeedTime);

                            int isIsEx = worknextMaxDistanceTask[i].size();
                            if (worknextMaxDistanceTask.size() != current_workerGroup.size())
                            {
                                cout << "大小不一样" << endl;
                            }
                            int gisEX = global_CT_Worker[current_workerGroup[i].Original_Local].size();
                            if (gisEX < isIsEx)
                            {
                                cout << "不应该出现" << endl;
                            }
                            if (current_CT_Worker[i].size() == Capacity - isIsEx)

                            {
                                current_workerGroup[i].sign = false;
                                count_NAWorker++;
                                Worker_Available[i] = 3;
                            }

                            else if (flag == 0)
                            {
                                updateObj.push_back(i);
                            }
                        }
                        else if (flag == 0)
                        {
                        }
                        else if (flag == 1)
                        {
                            Worker_Available[i] = 0;
                        }
                    }
                    else if (flag == 1)
                    {
                        Worker_Available[i] = 0;
                    }

                    if (CurrentTaskInPW[i] == current_PW[i].size())
                    {
                        if (Worker_Available[i] != 3)
                        {
                            count_NAWorker++;
                            Worker_Available[i] = 3;
                        }
                    }
                    else
                    {
                    }
                }
            }
        }

        for (int i = 0; i < updateObj.size(); i++)
        {
            int updateworkerid = updateObj[i];
            if (Worker_Available[updateworkerid] != 3)
            {
                Worker_Available[updateworkerid] = 1;
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
void Basic_information::Grouping_Framework_TPPG_workNext(vector<TASK> &tasks, vector<WORKER> &workers, double Wmax, int Tmax, vector<double> &Sumdis, vector<vector<double>> &global_Worker_subTrajectoryDis)

{
    vector<CURRENT_TASK_GROUP> current_taskGroup;
    vector<CURRENT_WORKERS_GROUP> current_Group_worker;
    vector<double> current_Group_workerSumdis;
    vector<double> current_Group_workerAD;
    double current_window_endTime = tasks[0].startTime + Wmax;
    double current_window_startTime = tasks[0].startTime;
    int current_workID = global_Current_workID;

    vector<vector<double>> current_Group_worker_subTrajectoryDis;

    int sign = 0;
    int temp = 0;

    for (int i = 0; i < tasks.size(); i++)
    {

        TASK task = tasks[i];
        sign++;

        CURRENT_TASK_GROUP taskg;
        taskg.task = task;
        taskg.Original_Local = i;
        current_taskGroup.push_back(taskg);

        if ((sign - temp == Tmax) || (task.startTime >= current_window_endTime))
        {
            temp = current_taskGroup.size();

            current_workID = global_Current_workID;

            current_window_endTime = task.startTime;

            groupWork_according_TaskGroup_workNext(workers, Sumdis, current_Group_worker, current_Group_workerSumdis, current_window_startTime, current_window_endTime, current_workID, current_Group_workerAD, global_Worker_subTrajectoryDis, current_Group_worker_subTrajectoryDis);
            determine_Window_Task_Timeout(current_taskGroup, current_window_endTime);

            match_WorkerTask_TPPG_workNext(current_taskGroup, current_Group_worker, current_window_endTime, current_Group_workerAD, current_Group_worker_subTrajectoryDis);

            current_window_startTime = current_window_endTime;
            current_window_endTime += Wmax;
        }
        updata_current_Info_workNext(current_taskGroup, current_Group_worker, current_Group_workerAD, current_Group_worker_subTrajectoryDis, current_Group_workerSumdis);
    }

    if (current_taskGroup.size() > 0)
    {

        current_workID = global_Current_workID;
        current_window_startTime = current_taskGroup.back().task.startTime;

        groupWork_according_TaskGroup_workNext(workers, Sumdis, current_Group_worker, current_Group_workerSumdis, current_window_startTime, current_window_endTime, current_workID, current_Group_workerAD, global_Worker_subTrajectoryDis, current_Group_worker_subTrajectoryDis);

        determine_Window_Task_Timeout(current_taskGroup, current_window_endTime);

        match_WorkerTask_TPPG_workNext(current_taskGroup, current_Group_worker, current_window_endTime, current_Group_workerAD, current_Group_worker_subTrajectoryDis);
    }
}

void Basic_information::match_WorkerTask_TPPG_workNext(vector<CURRENT_TASK_GROUP> &current_taskGroup, vector<CURRENT_WORKERS_GROUP> &current_workerGroup, double current_window_endTime, vector<double> &current_Group_workerAD, vector<vector<double>> &current_Group_worker_subTrajectoryDis)
{
    int current_Number_Worker = current_workerGroup.size();
    int current_Number_Task = current_taskGroup.size();

    vector<double> current_detour_distance[current_Number_Task];
    vector<int> poi[current_Number_Task];
    for (int i = 0; i < current_Number_Task; i++)
    {
        current_detour_distance[i] = vector<double>(current_Number_Worker, 0);

        poi[i] = vector<int>(current_Number_Worker, 0);
    }

    vector<int> current_CT_Worker[current_Number_Worker];
    double current_MaxDistanceTask[current_Number_Task];
    double current_task_NeedTime = 0.0;

    computeMaxDitanceTask(current_MaxDistanceTask, current_taskGroup, current_window_endTime);

    vector<int> current_Group_worker_AW;

    for (int i = 0; i < current_Number_Worker; i++)
    {
        current_Group_worker_AW.push_back(i);
    }

    for (int i = 0; i < current_Number_Task; i++)
    {

        if (current_Group_worker_AW.size() != 0)
        {
            int workerid = -1;

            workerid = FindPreferedWorkerNew_TPPG_workNext(i, current_Group_worker_AW, current_Group_workerAD, current_detour_distance, current_Group_worker_subTrajectoryDis, current_CT_Worker, current_MaxDistanceTask, &current_task_NeedTime, current_taskGroup, current_workerGroup, poi, current_window_endTime);

            if (workerid != -1)
            {

                current_taskGroup[i].sign = false;
                current_CT_Worker[workerid].push_back(i);
                global_MaxDistanceTask[current_taskGroup[i].Original_Local] = current_MaxDistanceTask[i];

                global_CT_Worker[current_workerGroup[workerid].Original_Local].push_back(current_taskGroup[i].Original_Local);

                current_Group_workerAD[workerid] = current_Group_workerAD[workerid] - 2 * current_detour_distance[i][workerid];
                global_workers[current_workerGroup[workerid].Original_Local].ADdis = current_Group_workerAD[workerid];

                UpdateTaskDeadline_workNext(current_taskGroup, current_workerGroup, workerid, i, current_detour_distance, current_CT_Worker, poi, current_MaxDistanceTask, current_task_NeedTime);

                int isEx = worknextMaxDistanceTask[workerid].size();
                if (worknextMaxDistanceTask.size() != current_workerGroup.size())
                {
                    cout << "大小不一样" << endl;
                }
                int gisEX = global_CT_Worker[current_workerGroup[workerid].Original_Local].size();
                if (gisEX < isEx)
                {
                    cout << "不应该出现" << endl;
                }

                if (current_CT_Worker[workerid].size() == Capacity - isEx)
                {
                    current_workerGroup[workerid].sign = false;
                    for (vector<int>::iterator it = current_Group_worker_AW.begin(); it != current_Group_worker_AW.end(); ++it)
                    {
                        if (*it == workerid)
                        {
                            it = current_Group_worker_AW.erase(it);

                            break;
                        }
                    }
                }
            }
            else
            {
            }
        }
        else
            break;
    }
    ShowCTMatching(current_CT_Worker, current_Number_Worker);
}

int Basic_information::FindPreferedWorkerNew_TPPG_workNext(int taskid, vector<int> &current_Group_worker_AW, vector<double> &current_Group_workerAD, vector<double> current_detour_distance[], vector<vector<double>> &current_Group_worker_subTrajectoryDis, vector<int> CT_Worker[], double MaxDistanceTask[], double *current_task_NeedTime, vector<CURRENT_TASK_GROUP> &current_taskGroup, vector<CURRENT_WORKERS_GROUP> &current_workerGroup, vector<int> poi[], double nowtime)
{

    int best_workerid = -1;
    double maxpre = -__DBL_MAX__;
    double best_distance = 0;
    for (int j = 0; j < current_Group_worker_AW.size(); j++)
    {
        int workerid = current_Group_worker_AW[j];
        if (current_taskGroup[taskid].task.Minscore <= current_workerGroup[workerid].worker.score)
        {
            double preference = current_workerGroup[workerid].worker.score;
            if (preference > maxpre)
            {
                double dist = Caculate_mindist(workerid, taskid, poi, current_taskGroup, current_workerGroup);

                current_detour_distance[taskid][workerid] = dist;

                if (current_workerGroup[workerid].worker.range >= dist)
                {

                    if (current_taskGroup[taskid].task.Reward - (2 * dist * c) > 0)
                    {
                        if (CurrentTask_Satisfy_workNext(current_taskGroup, current_workerGroup, workerid, taskid, current_Group_workerAD, current_detour_distance, current_Group_worker_subTrajectoryDis, CT_Worker, poi, MaxDistanceTask, current_task_NeedTime, nowtime))

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

/***
TPPG_batch算法
    * /

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
void Basic_information::Grouping_Framework_TPPG_Batch_workNext(vector<TASK> &tasks, vector<WORKER> &workers, double Wmax, int Tmax, vector<double> &Sumdis, vector<vector<double>> &global_Worker_subTrajectoryDis)

{
    vector<CURRENT_TASK_GROUP> current_taskGroup;
    vector<CURRENT_WORKERS_GROUP> current_Group_worker;
    vector<double> current_Group_workerSumdis;
    vector<double> current_Group_workerAD;
    double current_window_endTime = tasks[0].startTime + Wmax;
    double current_window_startTime = tasks[0].startTime;
    int current_workID = global_Current_workID;

    vector<vector<double>> current_Group_worker_subTrajectoryDis;

    int sign = 0;
    int temp = 0;

    for (int i = 0; i < tasks.size(); i++)
    {

        TASK task = tasks[i];
        sign++;

        CURRENT_TASK_GROUP taskg;
        taskg.task = task;
        taskg.Original_Local = i;
        current_taskGroup.push_back(taskg);

        if ((sign - temp == Tmax) || (task.startTime >= current_window_endTime))
        {
            temp = current_taskGroup.size();
            current_workID = global_Current_workID;

            current_window_endTime = task.startTime;

            groupWork_according_TaskGroup_workNext(workers, Sumdis, current_Group_worker, current_Group_workerSumdis, current_window_startTime, current_window_endTime, current_workID, current_Group_workerAD, global_Worker_subTrajectoryDis, current_Group_worker_subTrajectoryDis);
            determine_Window_Task_Timeout(current_taskGroup, current_window_endTime);

            match_WorkerTask_TPPG_Batch_workNext(current_taskGroup, current_Group_worker, current_window_endTime, current_Group_workerAD, current_Group_worker_subTrajectoryDis);

            current_window_startTime = current_window_endTime;
            current_window_endTime += Wmax;
        }
        updata_current_Info_workNext(current_taskGroup, current_Group_worker, current_Group_workerAD, current_Group_worker_subTrajectoryDis, current_Group_workerSumdis);
    }

    if (current_taskGroup.size() > 0)
    {

        current_workID = global_Current_workID;
        current_window_startTime = current_taskGroup.back().task.startTime;

        groupWork_according_TaskGroup_workNext(workers, Sumdis, current_Group_worker, current_Group_workerSumdis, current_window_startTime, current_window_endTime, current_workID, current_Group_workerAD, global_Worker_subTrajectoryDis, current_Group_worker_subTrajectoryDis);

        determine_Window_Task_Timeout(current_taskGroup, current_window_endTime);

        match_WorkerTask_workerBatch_workNext(current_taskGroup, current_Group_worker, current_window_endTime, current_Group_workerAD, current_Group_worker_subTrajectoryDis);
    }
}

void Basic_information::match_WorkerTask_TPPG_Batch_workNext(vector<CURRENT_TASK_GROUP> &current_taskGroup, vector<CURRENT_WORKERS_GROUP> &current_workerGroup, double current_window_endTime, vector<double> &current_Group_workerAD, vector<vector<double>> &current_Group_worker_subTrajectoryDis)
{
    int current_Number_Worker = current_workerGroup.size();
    int current_Number_Task = current_taskGroup.size();

    vector<double> current_detour_distance[current_Number_Task];
    vector<int> current_poi[current_Number_Task];
    vector<vector<pair<int, double>>> current_PT(Number_Task);
    vector<vector<pair<int, double>>> current_PW(Number_Worker);

    int current_CW_Task[current_Number_Task];
    int current_num_of_chased_worker[current_Number_Task] = {0};
    vector<int> current_ActiveTask;
    vector<int> current_NextActiveTask;

    vector<int> current_CT_Worker[current_Number_Worker];
    double current_MaxDistanceTask[current_Number_Task] = {0.0};
    double current_task_NeedTime = 0.0;

    for (int i = 0; i < current_Number_Task; i++)
    {
        current_detour_distance[i] = vector<double>(current_Number_Worker, 0);

        current_poi[i] = vector<int>(current_Number_Worker, 0);
    }
    Compute_PTPW_Group_workerBatch_workNext(current_PT, current_PW, current_detour_distance, current_taskGroup, current_workerGroup, current_poi);

    for (int i = 0; i < current_taskGroup.size(); i++)
    {
        if (current_PT[i].size() != 0)
        {
            current_ActiveTask.push_back(i);
            current_NextActiveTask.push_back(i);
        }
        current_CW_Task[i] = -1;
    }
    if (current_ActiveTask.empty())
    {
        return;
    }

    vector<vector<int>> workerWasRequest(Number_Worker, vector<int>(0));
    vector<int> workerRID;

    computeMaxDitanceTask(current_MaxDistanceTask, current_taskGroup, current_window_endTime);
    int current_chaseNum = 0;
    int times = 0;
    while (!current_ActiveTask.empty())
    {
        ++times;

        /**
         * 每个任务开始进行匹配请求
         */

        for (int i = 0; i < current_ActiveTask.size(); i++)
        {
            int taskid = current_ActiveTask[i];
            if ((current_chaseNum) >= current_PT[taskid].size())
            {

                current_NextActiveTask.erase(find(current_NextActiveTask.begin(), current_NextActiveTask.end(), current_ActiveTask[i]));
                continue;
            }
            int workid = current_PT[taskid][current_chaseNum].first;

            if (current_CT_Worker[workid].size() >= Capacity)
            {
                continue;
            }

            if (workerWasRequest[workid].empty())
            {
                workerRID.push_back(workid);
            }

            workerWasRequest[workid].push_back(taskid);
        }

        for (int j = 0; j < workerRID.size(); j++)
        {
            int workerid = workerRID[j];

            /**
             * 1
             * 下面根据工人的偏好列表
             * 对发出请求的工人进行排序
             * workerWasRequest最后得到的是排好序的任务请求列表
             */

            std::unordered_map<int, int> index_map;
            for (int i = 0; i < current_PW[workerid].size(); ++i)
            {
                index_map[current_PW[workerid][i].first] = i;
            }

            std::sort(workerWasRequest[workerid].begin(), workerWasRequest[workerid].end(),
                      [&index_map](const int &left, const int &right)
                      {
                          return index_map[left] < index_map[right];
                      });
            /**
             * 2
             * 拍完顺序开始匹配判断是否满足约束条件
             * 若满足，则task删除。
             * 若工人匹配出超过容量，则删除
             */

            for (int k = 0; k < workerWasRequest[workerid].size(); k++)
            {

                int taskIIID = workerWasRequest[workerid][k];

                if (current_taskGroup[taskIIID].task.Minscore <= current_workerGroup[workerid].worker.score)
                {
                    double preference = current_workerGroup[workerid].worker.score;

                    if (current_CT_Worker[workerid].size() >= Capacity)
                    {
                        break;
                    }

                    if (CurrentTask_Satisfy_workNext(current_taskGroup, current_workerGroup, workerid, taskIIID, current_Group_workerAD, current_detour_distance, current_Group_worker_subTrajectoryDis, current_CT_Worker, current_poi, current_MaxDistanceTask, &current_task_NeedTime, current_window_endTime))

                    {

                        current_workerGroup[workerid].sign = false;
                        current_taskGroup[taskIIID].sign = false;

                        current_CT_Worker[workerid].push_back(taskIIID);
                        global_CT_Worker[current_workerGroup[workerid].Original_Local].push_back(current_taskGroup[taskIIID].Original_Local);

                        global_MaxDistanceTask[current_taskGroup[taskIIID].Original_Local] = current_MaxDistanceTask[taskIIID];

                        current_NextActiveTask.erase(find(current_NextActiveTask.begin(), current_NextActiveTask.end(), taskIIID));

                        Update_AD1(workerid, taskIIID, current_Group_workerAD, current_detour_distance);

                        UpdateTaskDeadline_workNext(current_taskGroup, current_workerGroup, workerid, taskIIID, current_detour_distance, current_CT_Worker, current_poi, current_MaxDistanceTask, current_task_NeedTime);
                    }
                }
            }
            workerWasRequest[workerid].clear();
            workerWasRequest[workerid].resize(0);
        }
        current_ActiveTask.clear();
        current_ActiveTask.resize(current_NextActiveTask.size());

        for (int i = 0; i < current_NextActiveTask.size(); i++)
            current_ActiveTask[i] = current_NextActiveTask[i];

        workerWasRequest.clear();
        workerWasRequest.resize(current_Number_Worker, vector<int>(0));

        workerRID.clear();
        workerRID.resize(0);
        current_chaseNum++;
    }

    ShowCTMatching(current_CT_Worker, current_Number_Worker);
}

/**
 * 4.4TSDA
 */

void Basic_information::Grouping_Framework_TSDA_workNext(vector<TASK> &tasks, vector<WORKER> &workers, double Wmax, int Tmax, vector<double> &Sumdis, vector<vector<double>> &global_Worker_subTrajectoryDis)

{
    vector<CURRENT_TASK_GROUP> current_taskGroup;
    vector<CURRENT_WORKERS_GROUP> current_Group_worker;
    vector<double> current_Group_workerSumdis;
    vector<double> current_Group_workerAD;
    double current_window_endTime = tasks[0].startTime + Wmax;
    double current_window_startTime = tasks[0].startTime;
    int current_workID = global_Current_workID;

    vector<vector<double>> current_Group_worker_subTrajectoryDis;

    int sign = 0;
    int temp = 0;

    for (int i = 0; i < tasks.size(); i++)
    {

        TASK task = tasks[i];
        sign++;

        CURRENT_TASK_GROUP taskg;
        taskg.task = task;
        taskg.Original_Local = i;
        current_taskGroup.push_back(taskg);

        if ((sign - temp == Tmax) || (task.startTime >= current_window_endTime))
        {
            temp = current_taskGroup.size();
            current_workID = global_Current_workID;
            current_window_endTime = task.startTime;

            groupWork_according_TaskGroup_workNext(workers, Sumdis, current_Group_worker, current_Group_workerSumdis, current_window_startTime, current_window_endTime, current_workID, current_Group_workerAD, global_Worker_subTrajectoryDis, current_Group_worker_subTrajectoryDis);
            determine_Window_Task_Timeout(current_taskGroup, current_window_endTime);

            match_WorkerTask_TSDA_workNext(current_taskGroup, current_Group_worker, current_window_endTime, current_Group_workerAD, current_Group_worker_subTrajectoryDis);

            current_window_startTime = current_window_endTime;
            current_window_endTime += Wmax;

            updata_current_Info_workNext(current_taskGroup, current_Group_worker, current_Group_workerAD, current_Group_worker_subTrajectoryDis, current_Group_workerSumdis);
        }
    }

    if (current_taskGroup.size() > 0)
    {

        current_workID = global_Current_workID;
        current_window_startTime = current_taskGroup.back().task.startTime;

        groupWork_according_TaskGroup_workNext(workers, Sumdis, current_Group_worker, current_Group_workerSumdis, current_window_startTime, current_window_endTime, current_workID, current_Group_workerAD, global_Worker_subTrajectoryDis, current_Group_worker_subTrajectoryDis);

        determine_Window_Task_Timeout(current_taskGroup, current_window_endTime);

        match_WorkerTask_TSDA_workNext(current_taskGroup, current_Group_worker, current_window_endTime, current_Group_workerAD, current_Group_worker_subTrajectoryDis);
    }
}

void Basic_information::match_WorkerTask_TSDA_workNext(vector<CURRENT_TASK_GROUP> &current_taskGroup, vector<CURRENT_WORKERS_GROUP> &current_workerGroup, double current_window_endTime, vector<double> &current_Group_workerAD, vector<vector<double>> &current_Group_worker_subTrajectoryDis)
{
    int current_Number_Worker = current_workerGroup.size();
    int current_Number_Task = current_taskGroup.size();

    vector<double> current_detour_distance[current_Number_Task];
    vector<int> current_poi[current_Number_Task];
    vector<vector<pair<int, double>>> current_PT(Number_Task);
    vector<vector<pair<int, double>>> current_PW(Number_Worker);

    int current_CW_Task[current_Number_Task];
    int current_num_of_chased_worker[current_Number_Task] = {0};
    vector<int> current_ActiveTask;
    vector<int> current_NextActiveTask(current_ActiveTask);

    vector<vector<int>> current_CT_Worker(current_Number_Worker);
    double current_MaxDistanceTask[current_Number_Task] = {0.0};
    double current_task_NeedTime = 0.0;

    for (int i = 0; i < current_Number_Task; i++)
    {
        current_detour_distance[i] = vector<double>(current_Number_Worker, 0);

        current_poi[i] = vector<int>(current_Number_Worker, 0);
    }

    Compute_PTPW_Group_workerBatch_workNext(current_PT, current_PW, current_detour_distance, current_taskGroup, current_workerGroup, current_poi);
    for (int i = 0; i < current_taskGroup.size(); i++)
    {
        if (current_PT[i].size() != 0)
        {
            current_ActiveTask.push_back(i);
        }
        current_CW_Task[i] = -1;
    }
    if (current_ActiveTask.empty())
    {
        return;
    }

    computeMaxDitanceTask(current_MaxDistanceTask, current_taskGroup, current_window_endTime);

    /**
     * 迭代匹配
     * 工人批量时，工人不可用或者任务不可用停止循环匹配
     * 未匹配或匹配失败的批次工人优先匹配
     * 偏好列表ORDER
     */

    while (!current_ActiveTask.empty())
    {

        current_NextActiveTask.clear();

        for (int i = 0; i < current_ActiveTask.size(); i++)
            current_NextActiveTask.push_back(current_ActiveTask[i]);

        int matchingnumber = 0, replacematching = 0;
        for (int i = 0; i < current_ActiveTask.size(); i++)
        {
            int taskid = current_ActiveTask[i];
            int order = current_num_of_chased_worker[taskid];
            int worker_to_chase = current_PT[taskid][order].first;

            current_num_of_chased_worker[taskid] = order + 1;

            if (IFTaskExist(worker_to_chase, taskid, current_PW) != current_PW[worker_to_chase].end())
            {

                if (CurrentTask_Satisfy_TSDA_workNext(current_taskGroup, current_workerGroup, worker_to_chase, taskid, current_Group_workerAD, current_detour_distance, current_Group_worker_subTrajectoryDis, current_CT_Worker, current_poi, current_MaxDistanceTask, &current_task_NeedTime, current_window_endTime))

                {

                    current_CW_Task[taskid] = worker_to_chase;
                    current_CT_Worker[worker_to_chase].push_back(taskid);

                    current_NextActiveTask.erase(find(current_NextActiveTask.begin(), current_NextActiveTask.end(), taskid));

                    Update_AD1(worker_to_chase, taskid, current_Group_workerAD, current_detour_distance);

                    UpdateTaskDeadline_TSDA_workNext(current_taskGroup, current_workerGroup, false, 0, worker_to_chase, taskid, current_detour_distance, current_CT_Worker, current_poi, current_MaxDistanceTask, current_task_NeedTime);
                    matchingnumber++;
                }
                else
                {

                    int MinReplaceTask = FindReplaceTaskNew_TSDA_workNext(worker_to_chase, taskid, current_PW, current_Group_workerAD, current_detour_distance, current_Group_worker_subTrajectoryDis, current_CT_Worker, current_poi, current_MaxDistanceTask, &current_task_NeedTime, current_taskGroup, current_workerGroup, current_window_endTime);
                    if (MinReplaceTask != -1)
                    {
                        current_CW_Task[taskid] = worker_to_chase;
                        current_CW_Task[MinReplaceTask] = -1;
                        current_CT_Worker[worker_to_chase].push_back(taskid);

                        current_CT_Worker[worker_to_chase].erase(find(current_CT_Worker[worker_to_chase].begin(), current_CT_Worker[worker_to_chase].end(), MinReplaceTask));

                        if (current_num_of_chased_worker[MinReplaceTask] < current_PT[MinReplaceTask].size())
                            current_NextActiveTask.push_back(MinReplaceTask);
                        current_NextActiveTask.erase(find(current_NextActiveTask.begin(), current_NextActiveTask.end(), taskid));

                        Update_AD2(worker_to_chase, taskid, MinReplaceTask, current_Group_workerAD, current_detour_distance);

                        UpdateTaskDeadline_TSDA_workNext(current_taskGroup, current_workerGroup, true, MinReplaceTask, worker_to_chase, taskid, current_detour_distance, current_CT_Worker, current_poi, current_MaxDistanceTask, current_task_NeedTime);

                        current_MaxDistanceTask[MinReplaceTask] = current_taskGroup[MinReplaceTask].task.Deadline * speed;

                        replacematching++;
                    }
                }
            }
            if (current_num_of_chased_worker[taskid] == current_PT[taskid].size())
            {

                vector<int>::iterator iter1 = find(current_NextActiveTask.begin(), current_NextActiveTask.end(), taskid);
                if (iter1 != current_NextActiveTask.end())
                {
                    current_NextActiveTask.erase(find(current_NextActiveTask.begin(), current_NextActiveTask.end(), taskid));
                }
            }
        }

        current_ActiveTask.clear();
        for (int i = 0; i < current_NextActiveTask.size(); i++)
            current_ActiveTask.push_back(current_NextActiveTask[i]);
    }

    for (int i = 0; i < current_Number_Worker; i++)
    {

        if (current_CT_Worker[i].size() + global_CT_Worker[current_workerGroup[i].Original_Local].size() == Capacity)
        {
            current_workerGroup[i].sign = false;
            for (auto m : current_CT_Worker[i])
            {
                global_CT_Worker[current_workerGroup[i].Original_Local].push_back(current_taskGroup[m].Original_Local);
                current_taskGroup[m].sign = false;
            }
        }
        else
        {
            for (auto m : current_CT_Worker[i])
            {
                global_CT_Worker[current_workerGroup[i].Original_Local].push_back(current_taskGroup[m].Original_Local);
                global_MaxDistanceTask[current_taskGroup[m].Original_Local] = current_MaxDistanceTask[m];
                current_taskGroup[m].sign = false;
            }
        }
    }

    ShowCTMatching(current_CT_Worker, current_Number_Worker);
}

void Basic_information::UpdateTaskDeadline_TSDA_workNext(vector<CURRENT_TASK_GROUP> &current_taskGroup, vector<CURRENT_WORKERS_GROUP> &current_workerGroup, bool replace, int replaceid, int workerid, int taskid, vector<double> current_detour_distance[], vector<vector<int>> &CT_Worker, vector<int> poi[], double MaxDistanceTask[], double current_task_NeedTime)
{
    int replacepoi = 0;
    if (replace)
    {
        replacepoi = poi[replaceid][workerid];
    }
    int detourpoint = poi[taskid][workerid];
    for (int i = 0; i < CT_Worker[workerid].size(); i++)
    {
        int assigntask_id = CT_Worker[workerid][i];
        int assignedpoint = poi[assigntask_id][workerid];

        if (assigntask_id == taskid)
        {

            MaxDistanceTask[taskid] = MaxDistanceTask[taskid] - current_task_NeedTime * speed;
        }
        else
        {
            if (detourpoint < assignedpoint)
            {

                MaxDistanceTask[assigntask_id] = MaxDistanceTask[assigntask_id] - (2 * current_detour_distance[taskid][workerid]);
            }
            if (replace == true && replacepoi < assignedpoint)
            {

                MaxDistanceTask[assigntask_id] = MaxDistanceTask[assigntask_id] + (2 * current_detour_distance[replaceid][workerid]);
            }
        }
    }
    int isEx = worknextMaxDistanceTask[workerid].size();
    if (worknextMaxDistanceTask.size() != current_workerGroup.size())
    {
        cout << "大小不一样" << endl;
    }
    int gisEX = global_CT_Worker[current_workerGroup[workerid].Original_Local].size();
    if (gisEX < isEx)
    {
        cout << "不应该出现" << endl;
    }

    for (int i = 0; i < isEx; i++)
    {
        int assigntask_id = global_CT_Worker[current_workerGroup[workerid].Original_Local][i];
        int assignedpoint = global_POI[assigntask_id][current_workerGroup[workerid].Original_Local];

        if (assigntask_id == current_taskGroup[taskid].Original_Local)
        {

            global_MaxDistanceTask[assigntask_id] -= current_task_NeedTime * speed;
        }
        else
        {
            if (detourpoint < assignedpoint)
            {

                global_MaxDistanceTask[assigntask_id] -= (2 * current_detour_distance[replaceid][workerid]);
            }
            if (replace == true && replacepoi < assignedpoint)
            {

                global_MaxDistanceTask[assigntask_id] += (2 * current_detour_distance[replaceid][workerid]);
            }
        }
    }
}

int Basic_information::FindReplaceTaskNew_TSDA_workNext(int workerid, int taskid, vector<vector<pair<int, double>>> &PW, vector<double> &AD, vector<double> detour_distance[], vector<vector<double>> &Worker_subTrajectoryDis, vector<vector<int>> &CT_Worker, vector<int> poi[], double MaxDistanceTask[], double *current_task_NeedTime, vector<CURRENT_TASK_GROUP> &current_taskGroup, vector<CURRENT_WORKERS_GROUP> &current_workerGroup, double nowtime)
{
    int reTaskindex = -1;
    int replacetaskid = -1;
    double minProfit = __DBL_MAX__;
    int t1 = GetIndex_PW(workerid, taskid, PW);

    double SumdetouDis = 0.0;
    int detourpoint = poi[taskid][workerid];
    for (vector<int>::iterator it = CT_Worker[workerid].begin(); it != CT_Worker[workerid].end(); it++)
    {
        int assignpoi = poi[*it][workerid];
        if (assignpoi <= detourpoint)
        {
            SumdetouDis = SumdetouDis + 2 * detour_distance[*it][workerid];
        }
    }
    int global_wid = current_workerGroup[workerid].Original_Local;
    for (vector<int>::iterator it = global_CT_Worker[global_wid].begin(); it != global_CT_Worker[global_wid].end(); it++)
    {
        int assignpoi = global_POI[*it][global_wid];
        if (assignpoi <= detourpoint)
        {

            SumdetouDis = SumdetouDis + 2 * global_detour_distance[*it][global_wid];
        }
    }

    for (vector<int>::iterator it = CT_Worker[workerid].begin(); it != CT_Worker[workerid].end(); it++)
    {
        int t2 = GetIndex_PW(workerid, *it, PW);
        if (t1 < t2)
        {
            if (IfReplace_workNext(workerid, taskid, *it, AD, detour_distance, Worker_subTrajectoryDis, CT_Worker, poi, MaxDistanceTask, current_task_NeedTime, SumdetouDis, current_taskGroup, current_workerGroup, nowtime))

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

void Basic_information::Grouping_Framework_WPPG_workNext(vector<TASK> &tasks, vector<WORKER> &workers, double Wmax, int Tmax, vector<double> &Sumdis, vector<vector<double>> &global_Worker_subTrajectoryDis)

{
    vector<CURRENT_TASK_GROUP> current_taskGroup;
    vector<CURRENT_WORKERS_GROUP> current_Group_worker;
    vector<double> current_Group_workerSumdis;
    vector<double> current_Group_workerAD;
    double current_window_endTime = tasks[0].startTime + Wmax;
    double current_window_startTime = tasks[0].startTime;
    int current_workID = global_Current_workID;

    vector<vector<double>> current_Group_worker_subTrajectoryDis;

    int sign = 0;
    int temp = 0;

    for (int i = 0; i < tasks.size(); i++)
    {

        TASK task = tasks[i];
        sign++;

        CURRENT_TASK_GROUP taskg;
        taskg.task = task;
        taskg.Original_Local = i;
        current_taskGroup.push_back(taskg);

        if ((sign - temp == Tmax) || (task.startTime >= current_window_endTime))
        {
            temp = current_taskGroup.size();
            current_workID = global_Current_workID;

            current_window_endTime = task.startTime;

            groupWork_according_TaskGroup_workNext(workers, Sumdis, current_Group_worker, current_Group_workerSumdis, current_window_startTime, current_window_endTime, current_workID, current_Group_workerAD, global_Worker_subTrajectoryDis, current_Group_worker_subTrajectoryDis);
            determine_Window_Task_Timeout(current_taskGroup, current_window_endTime);

            match_WorkerTask_WPPG_workNext(current_taskGroup, current_Group_worker, current_window_endTime, current_Group_workerAD, current_Group_worker_subTrajectoryDis);

            current_window_startTime = current_window_endTime;
            current_window_endTime += Wmax;
        }
        updata_current_Info_workNext(current_taskGroup, current_Group_worker, current_Group_workerAD, current_Group_worker_subTrajectoryDis, current_Group_workerSumdis);
    }

    if (current_taskGroup.size() > 0)
    {

        current_workID = global_Current_workID;
        current_window_startTime = current_taskGroup.back().task.startTime;

        groupWork_according_TaskGroup_workNext(workers, Sumdis, current_Group_worker, current_Group_workerSumdis, current_window_startTime, current_window_endTime, current_workID, current_Group_workerAD, global_Worker_subTrajectoryDis, current_Group_worker_subTrajectoryDis);

        determine_Window_Task_Timeout(current_taskGroup, current_window_endTime);

        match_WorkerTask_WPPG_workNext(current_taskGroup, current_Group_worker, current_window_endTime, current_Group_workerAD, current_Group_worker_subTrajectoryDis);
    }
}

void Basic_information::match_WorkerTask_WPPG_workNext(vector<CURRENT_TASK_GROUP> &current_taskGroup, vector<CURRENT_WORKERS_GROUP> &current_workerGroup, double current_window_endTime, vector<double> &current_Group_workerAD, vector<vector<double>> &current_Group_worker_subTrajectoryDis)
{
    int current_Number_Worker = current_workerGroup.size();
    int current_Number_Task = current_taskGroup.size();

    vector<double> current_detour_distance[current_Number_Task];
    vector<int> current_poi[current_Number_Task];
    vector<vector<pair<int, double>>> current_PT(current_Number_Task);
    vector<vector<pair<int, double>>> current_PW(current_Number_Worker);

    for (int i = 0; i < current_Number_Task; i++)
    {
        current_detour_distance[i] = vector<double>(current_Number_Worker, 0);

        current_poi[i] = vector<int>(current_Number_Worker, 0);
    }

    vector<int> current_CT_Worker[current_Number_Worker];
    double current_MaxDistanceTask[current_Number_Task];
    double current_task_NeedTime = 0.0;

    computeMaxDitanceTask(current_MaxDistanceTask, current_taskGroup, current_window_endTime);

    vector<int> current_Group_worker_AT;
    for (int i = 0; i < current_Number_Task; i++)
    {
        current_Group_worker_AT.push_back(i);
    }

    for (int i = 0; i < current_Number_Worker; i++)
    {
        int workerid = i;

        if (current_Group_worker_AT.size() != 0)
        {

            ComputePWforAT_WPPG_workNext(workerid, current_PW, current_Group_worker_AT, current_detour_distance, current_poi, current_workerGroup, current_taskGroup);
            sort(current_PW[workerid].begin(), current_PW[workerid].end(), cmp);
            if (current_PW[workerid].size() > 0)
            {
                int assigned_task = 0;
                for (int j = 0; j < current_PW[workerid].size(); j++)
                {
                    int ct_task_id = current_PW[workerid][j].first;
                    if (current_taskGroup[ct_task_id].sign)
                    {
                        int isEx = worknextMaxDistanceTask[workerid].size();
                        if (worknextMaxDistanceTask.size() != current_workerGroup.size())
                        {
                            cout << "大小不一样" << endl;
                        }
                        int gisEX = global_CT_Worker[current_workerGroup[workerid].Original_Local].size();
                        if (gisEX < isEx)
                        {
                            cout << "不应该出现" << endl;
                        }

                        if (assigned_task < Capacity - isEx)
                        {
                            if (find(current_Group_worker_AT.begin(), current_Group_worker_AT.end(), current_PW[workerid][j].first) != current_Group_worker_AT.end())
                            {
                                if (CurrentTask_Satisfy_workNext(current_taskGroup, current_workerGroup, workerid, current_PW[workerid][j].first, current_Group_workerAD, current_detour_distance, current_Group_worker_subTrajectoryDis, current_CT_Worker, current_poi, current_MaxDistanceTask, &current_task_NeedTime, current_window_endTime))

                                {
                                    current_CT_Worker[workerid].push_back(current_PW[workerid][j].first);

                                    current_taskGroup[ct_task_id].sign = false;

                                    global_CT_Worker[current_workerGroup[workerid].Original_Local].push_back(current_taskGroup[ct_task_id].Original_Local);
                                    global_MaxDistanceTask[current_taskGroup[ct_task_id].Original_Local] = current_MaxDistanceTask[ct_task_id];

                                    current_Group_workerAD[workerid] = current_Group_workerAD[workerid] - 2 * current_detour_distance[current_PW[workerid][j].first][workerid];
                                    global_workers[current_workerGroup[workerid].Original_Local].ADdis = current_Group_workerAD[workerid];
                                    UpdateTaskDeadline_workNext(current_taskGroup, current_workerGroup, workerid, current_PW[workerid][j].first, current_detour_distance, current_CT_Worker, current_poi, current_MaxDistanceTask, current_task_NeedTime);

                                    assigned_task++;
                                    for (vector<int>::iterator it = current_Group_worker_AT.begin(); it != current_Group_worker_AT.end(); ++it)
                                    {
                                        if (*it == current_PW[workerid][j].first)
                                        {

                                            vector<int>::iterator itt = current_Group_worker_AT.erase(it);

                                            break;
                                        }
                                    }
                                }
                            }
                        }
                        else
                            break;
                    }
                }
            }
        }
        else
            break;
    }
    for (int i = 0; i < current_Number_Worker; i++)
    {
        int isIsEx = worknextMaxDistanceTask[i].size();
        if (worknextMaxDistanceTask.size() != current_workerGroup.size())
        {
            cout << "大小不一样" << endl;
        }
        int gisEX = global_CT_Worker[current_workerGroup[i].Original_Local].size();
        if (gisEX < isIsEx)
        {
            cout << "不应该出现" << endl;
        }
        if (current_CT_Worker[i].size() + isIsEx == Capacity)
        {
            current_workerGroup[i].sign = false;
        }
    }
    ShowCTMatching(current_CT_Worker, current_Number_Worker);
}

void Basic_information::ComputePWforAT_WPPG_workNext(int workerid, vector<vector<pair<int, double>>> &PW, vector<int> &AT, vector<double> current_detour_distance[], vector<int> current_poi[], vector<CURRENT_WORKERS_GROUP> &current_workerGroup, vector<CURRENT_TASK_GROUP> &current_taskGroup)
{

    for (int i = 0; i < AT.size(); i++)
    {
        int j = AT[i];
        if (current_taskGroup[j].task.Minscore <= current_workerGroup[workerid].worker.score)
        {

            current_detour_distance[j][workerid] = Caculate_mindist(workerid, j, current_poi, current_taskGroup, current_workerGroup);

            if (current_detour_distance[j][workerid] <= current_workerGroup[workerid].worker.range)
            {
                double preference = current_taskGroup[j].task.Reward - (2 * current_detour_distance[j][workerid] * c);
                if (preference > 0)
                {

                    {
                        PW[workerid].push_back(make_pair(j, preference));
                    }
                }
            }
        }
    }
}

void Basic_information::Grouping_Framework_WSDA_workNext(vector<TASK> &tasks, vector<WORKER> &workers, double Wmax, int Tmax, vector<double> &Sumdis, vector<vector<double>> &global_Worker_subTrajectoryDis)

{
    vector<CURRENT_TASK_GROUP> current_taskGroup;
    vector<CURRENT_WORKERS_GROUP> current_Group_worker;
    vector<double> current_Group_workerSumdis;
    vector<double> current_Group_workerAD;
    double current_window_endTime = tasks[0].startTime + Wmax;
    double current_window_startTime = tasks[0].startTime;
    int current_workID = global_Current_workID;

    vector<vector<double>> current_Group_worker_subTrajectoryDis;

    int sign = 0;
    int temp = 0;

    for (int i = 0; i < tasks.size(); i++)
    {

        TASK task = tasks[i];
        sign++;

        CURRENT_TASK_GROUP taskg;
        taskg.task = task;
        taskg.Original_Local = i;

        if ((sign - temp == Tmax) || (task.startTime >= current_window_endTime))
        {
            temp = current_taskGroup.size();
            current_workID = global_Current_workID;
            current_window_endTime = task.startTime;

            groupWork_according_TaskGroup_workNext(workers, Sumdis, current_Group_worker, current_Group_workerSumdis, current_window_startTime, current_window_endTime, current_workID, current_Group_workerAD, global_Worker_subTrajectoryDis, current_Group_worker_subTrajectoryDis);
            determine_Window_Task_Timeout(current_taskGroup, current_window_endTime);

            match_WorkerTask_WSDA_workNext(current_taskGroup, current_Group_worker, current_window_endTime, current_Group_workerAD, current_Group_worker_subTrajectoryDis);

            current_window_startTime = current_window_endTime;
            current_window_endTime += Wmax;

            updata_current_Info_workNext(current_taskGroup, current_Group_worker, current_Group_workerAD, current_Group_worker_subTrajectoryDis, current_Group_workerSumdis);
        }

        current_taskGroup.push_back(taskg);
    }

    if (current_taskGroup.size() > 0)
    {

        current_workID = global_Current_workID;
        current_window_startTime = current_taskGroup.back().task.startTime;

        groupWork_according_TaskGroup_workNext(workers, Sumdis, current_Group_worker, current_Group_workerSumdis, current_window_startTime, current_window_endTime, current_workID, current_Group_workerAD, global_Worker_subTrajectoryDis, current_Group_worker_subTrajectoryDis);

        determine_Window_Task_Timeout(current_taskGroup, current_window_endTime);

        match_WorkerTask_WSDA_workNext(current_taskGroup, current_Group_worker, current_window_endTime, current_Group_workerAD, current_Group_worker_subTrajectoryDis);
    }
}

void Basic_information::match_WorkerTask_WSDA_workNext(vector<CURRENT_TASK_GROUP> &current_taskGroup, vector<CURRENT_WORKERS_GROUP> &current_workerGroup, double current_window_endTime, vector<double> &current_Group_workerAD, vector<vector<double>> &current_Group_worker_subTrajectoryDis)
{
    int current_Number_Worker = current_workerGroup.size();
    int current_Number_Task = current_taskGroup.size();

    vector<double> current_detour_distance[current_Number_Task];
    vector<int> current_poi[current_Number_Task];
    vector<vector<pair<int, double>>> current_PT(Number_Task);
    vector<vector<pair<int, double>>> current_PW(Number_Worker);

    int current_CW_Task[current_Number_Task];
    int current_num_of_chased_tasks[current_Number_Worker] = {0};

    vector<int> current_ActiveWorker;

    vector<int> current_NextActiveWorker(current_ActiveWorker);

    vector<vector<int>> current_CT_Worker(current_Number_Worker);
    double current_MaxDistanceTask[current_Number_Task] = {0.0};
    double current_task_NeedTime = 0.0;

    for (int i = 0; i < current_Number_Task; i++)
    {
        current_detour_distance[i] = vector<double>(current_Number_Worker, 0);

        current_poi[i] = vector<int>(current_Number_Worker, 0);
        current_CW_Task[i] = -1;
    }

    Compute_PTPW_Group_workerBatch_workNext(current_PT, current_PW, current_detour_distance, current_taskGroup, current_workerGroup, current_poi);

    for (int i = 0; i < current_workerGroup.size(); i++)
    {
        if (current_PW[i].size() != 0)
        {

            current_ActiveWorker.push_back(i);
        }
    }

    if (current_ActiveWorker.empty())
    {
        return;
    }

    computeMaxDitanceTask(current_MaxDistanceTask, current_taskGroup, current_window_endTime);

    /**
     * 迭代匹配
     * 工人批量时，工人不可用或者任务不可用停止循环匹配
     * 未匹配或匹配失败的批次工人优先匹配
     * 偏好列表ORDER
     */

    int matchingTimes = 0;
    while (!current_ActiveWorker.empty())
    {

        current_NextActiveWorker.clear();

        for (int i = 0; i < current_ActiveWorker.size(); i++)
            current_NextActiveWorker.push_back(current_ActiveWorker[i]);

        int matchingnumber = 0, replacematching = 0;
        for (int i = 0; i < current_ActiveWorker.size(); i++)
        {
            int workerid = current_ActiveWorker[i];
            int order = current_num_of_chased_tasks[workerid];

            int task_to_chase = current_PW[workerid][order].first;

            current_num_of_chased_tasks[workerid] = order + 1;

            if (IFWorkerExist(workerid, task_to_chase, current_PT) != current_PT[task_to_chase].end())
            {

                if (CurrentTask_Satisfy_TSDA_workNext(current_taskGroup, current_workerGroup, workerid, task_to_chase, current_Group_workerAD, current_detour_distance, current_Group_worker_subTrajectoryDis, current_CT_Worker, current_poi, current_MaxDistanceTask, &current_task_NeedTime, current_window_endTime))

                {

                    if (current_CW_Task[task_to_chase] == -1)
                    {

                        current_CW_Task[task_to_chase] = workerid;
                        current_CT_Worker[workerid].push_back(task_to_chase);

                        Update_AD1(workerid, task_to_chase, current_Group_workerAD, current_detour_distance);

                        UpdateTaskDeadline_WSDA_workNext(current_taskGroup, current_workerGroup, false, 0, workerid, task_to_chase, current_detour_distance, current_CT_Worker, current_poi, current_MaxDistanceTask, current_task_NeedTime);
                        matchingnumber++;
                    }
                    else
                    {

                        int oldworkerid = current_CW_Task[task_to_chase];
                        if (GetIndex_PT(workerid, task_to_chase, current_PT) < GetIndex_PT(oldworkerid, task_to_chase, current_PT))
                        {

                            current_CW_Task[task_to_chase] = workerid;
                            current_CT_Worker[workerid].push_back(task_to_chase);

                            Update_AD1(workerid, task_to_chase, current_Group_workerAD, current_detour_distance);

                            current_CT_Worker[oldworkerid].erase(find(current_CT_Worker[oldworkerid].begin(), current_CT_Worker[oldworkerid].end(), task_to_chase));

                            Update_AD2_WSDA(oldworkerid, task_to_chase, current_Group_workerAD, current_detour_distance);

                            current_MaxDistanceTask[task_to_chase] = (current_taskGroup[task_to_chase].task.Deadline - current_window_endTime) * speed;
                            UpdateTaskDeadline_WSDA_workNext(current_taskGroup, current_workerGroup, true, oldworkerid, workerid, task_to_chase, current_detour_distance, current_CT_Worker, current_poi, current_MaxDistanceTask, current_task_NeedTime);

                            replacematching++;
                        }
                    }
                }
            }
            if (current_num_of_chased_tasks[workerid] >= current_PW[workerid].size())
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
        for (int i = 0; i < current_NextActiveWorker.size(); i++)
            current_ActiveWorker.push_back(current_NextActiveWorker[i]);
    }

    for (int i = 0; i < current_Number_Worker; i++)
    {

        if (current_CT_Worker[i].size() + global_CT_Worker[current_workerGroup[i].Original_Local].size() == Capacity)
        {
            current_workerGroup[i].sign = false;
            for (auto m : current_CT_Worker[i])
            {
                global_CT_Worker[current_workerGroup[i].Original_Local].push_back(current_taskGroup[m].Original_Local);
                current_taskGroup[m].sign = false;
            }
        }
        else
        {
            for (auto m : current_CT_Worker[i])
            {
                global_CT_Worker[current_workerGroup[i].Original_Local].push_back(current_taskGroup[m].Original_Local);
                global_MaxDistanceTask[current_taskGroup[m].Original_Local] = current_MaxDistanceTask[m];
                current_taskGroup[m].sign = false;
            }
        }
    }

    ShowCTMatching(current_CT_Worker, current_Number_Worker);
}
void Basic_information::UpdateTaskDeadline_WSDA_workNext(vector<CURRENT_TASK_GROUP> &current_taskGroup, vector<CURRENT_WORKERS_GROUP> &current_workerGroup, bool replace, int replaceWorkid, int workerid, int taskid, vector<double> current_detour_distance[], vector<vector<int>> &CT_Worker, vector<int> poi[], double MaxDistanceTask[], double current_task_NeedTime)
{

    int detourpoint = poi[taskid][workerid];
    for (int i = 0; i < CT_Worker[workerid].size(); i++)
    {
        int assigntask_id = CT_Worker[workerid][i];
        int assignedpoint = poi[assigntask_id][workerid];

        if (assigntask_id == taskid)
        {

            MaxDistanceTask[taskid] = MaxDistanceTask[taskid] - current_task_NeedTime * speed;
        }
        else
        {
            if (detourpoint < assignedpoint)
            {

                MaxDistanceTask[assigntask_id] = MaxDistanceTask[assigntask_id] - (2 * current_detour_distance[taskid][workerid]);
            }
        }
    }
    int isEx = worknextMaxDistanceTask[workerid].size();
    if (worknextMaxDistanceTask.size() != current_workerGroup.size())
    {
        cout << "大小不一样" << endl;
    }
    int gisEX = global_CT_Worker[current_workerGroup[workerid].Original_Local].size();
    if (gisEX < isEx)
    {
        cout << "不应该出现" << endl;
    }

    for (int i = 0; i < isEx; i++)
    {
        int assigntask_id = global_CT_Worker[current_workerGroup[workerid].Original_Local][i];
        int assignedpoint = global_POI[assigntask_id][current_workerGroup[workerid].Original_Local];

        if (assigntask_id == current_taskGroup[taskid].Original_Local)
        {

            global_MaxDistanceTask[assigntask_id] -= current_task_NeedTime * speed;
        }
        else
        {
            if (detourpoint < assignedpoint)
            {

                global_MaxDistanceTask[assigntask_id] -= (2 * current_detour_distance[taskid][workerid]);
            }
        }
    }

    if (replace == true)
    {
        int replacepoi = poi[taskid][replaceWorkid];
        for (int i = 0; i < CT_Worker[replaceWorkid].size(); i++)
        {
            int assigntask_id = CT_Worker[replaceWorkid][i];
            int assignedpoint = poi[assigntask_id][replaceWorkid];

            if (replacepoi < assignedpoint)
            {

                MaxDistanceTask[assigntask_id] = MaxDistanceTask[assigntask_id] + (2 * current_detour_distance[taskid][replaceWorkid]);
            }
        }
        int global_wid = current_workerGroup[replaceWorkid].Original_Local;
        int isIsEx = worknextMaxDistanceTask[replaceWorkid].size();
        if (worknextMaxDistanceTask.size() != current_workerGroup.size())
        {
            cout << "大小不一样" << endl;
        }
        int gisEX = global_CT_Worker[current_workerGroup[replaceWorkid].Original_Local].size();
        if (gisEX < isIsEx)
        {
            cout << "不应该出现" << endl;
        }
        for (int i = 0; i < isIsEx; i++)
        {
            int assigntask_id = global_CT_Worker[global_wid][i];
            int assignedpoint = global_POI[assigntask_id][global_wid];

            if (replacepoi < assignedpoint)
            {

                global_MaxDistanceTask[assigntask_id] = global_MaxDistanceTask[assigntask_id] + (2 * current_detour_distance[taskid][replaceWorkid]);
            }
        }
    }
}

void Basic_information::Grouping_Framework_ReverseDA_workNext(vector<TASK> &tasks, vector<WORKER> &workers, double Wmax, int Tmax, vector<double> &Sumdis, vector<vector<double>> &global_Worker_subTrajectoryDis)

{
    vector<CURRENT_TASK_GROUP> current_taskGroup;
    vector<CURRENT_WORKERS_GROUP> current_Group_worker;
    vector<double> current_Group_workerSumdis;
    vector<double> current_Group_workerAD;
    double current_window_endTime = tasks[0].startTime + Wmax;
    double current_window_startTime = tasks[0].startTime;
    int current_workID = global_Current_workID;

    vector<vector<double>> current_Group_worker_subTrajectoryDis;

    int sign = 0;
    int temp = 0;

    for (int i = 0; i < tasks.size(); i++)
    {

        TASK task = tasks[i];
        sign++;

        CURRENT_TASK_GROUP taskg;
        taskg.task = task;
        taskg.Original_Local = i;

        if ((sign - temp == Tmax) || (task.startTime >= current_window_endTime))
        {
            temp = current_taskGroup.size();
            current_workID = global_Current_workID;
            current_window_endTime = task.startTime;

            groupWork_according_TaskGroup_workNext(workers, Sumdis, current_Group_worker, current_Group_workerSumdis, current_window_startTime, current_window_endTime, current_workID, current_Group_workerAD, global_Worker_subTrajectoryDis, current_Group_worker_subTrajectoryDis);
            determine_Window_Task_Timeout(current_taskGroup, current_window_endTime);

            match_WorkerTask_ReverseDA_workNext(current_taskGroup, current_Group_worker, current_window_endTime, current_Group_workerAD, current_Group_worker_subTrajectoryDis);

            current_window_startTime = current_window_endTime;
            current_window_endTime += Wmax;

            updata_current_Info_workNext(current_taskGroup, current_Group_worker, current_Group_workerAD, current_Group_worker_subTrajectoryDis, current_Group_workerSumdis);
        }

        current_taskGroup.push_back(taskg);
    }

    if (current_taskGroup.size() > 0)
    {

        current_workID = global_Current_workID;
        current_window_startTime = current_taskGroup.back().task.startTime;

        groupWork_according_TaskGroup_workNext(workers, Sumdis, current_Group_worker, current_Group_workerSumdis, current_window_startTime, current_window_endTime, current_workID, current_Group_workerAD, global_Worker_subTrajectoryDis, current_Group_worker_subTrajectoryDis);

        determine_Window_Task_Timeout(current_taskGroup, current_window_endTime);

        match_WorkerTask_ReverseDA_workNext(current_taskGroup, current_Group_worker, current_window_endTime, current_Group_workerAD, current_Group_worker_subTrajectoryDis);
    }
}

void Basic_information::match_WorkerTask_ReverseDA_workNext(vector<CURRENT_TASK_GROUP> &current_taskGroup, vector<CURRENT_WORKERS_GROUP> &current_workerGroup, double current_window_endTime, vector<double> &current_Group_workerAD, vector<vector<double>> &current_Group_worker_subTrajectoryDis)
{

    int current_Number_Worker = current_workerGroup.size();
    int current_Number_Task = current_taskGroup.size();

    vector<double> current_detour_distance[current_Number_Task];
    vector<int> current_poi[current_Number_Task];
    vector<vector<pair<int, double>>> current_PT(Number_Task);
    vector<vector<pair<int, double>>> current_PW(Number_Worker);

    int current_CW_Task[current_Number_Task];
    int current_num_of_chased_worker[current_Number_Task] = {0};
    vector<int> current_ActiveTask;
    vector<int> current_NextActiveTask(current_ActiveTask);

    vector<vector<int>> current_CT_Worker(current_Number_Worker);
    double current_MaxDistanceTask[current_Number_Task] = {0.0};
    double current_task_NeedTime = 0.0;

    for (int i = 0; i < current_Number_Task; i++)
    {
        current_detour_distance[i] = vector<double>(current_Number_Worker, 0);

        current_poi[i] = vector<int>(current_Number_Worker, 0);
    }

    Compute_PTPW_Group_workerBatch_workNext(current_PT, current_PW, current_detour_distance, current_taskGroup, current_workerGroup, current_poi);
    for (int i = 0; i < current_taskGroup.size(); i++)
    {
        if (current_PT[i].size() != 0)
        {
            current_ActiveTask.push_back(i);
        }
        current_CW_Task[i] = -1;
    }
    if (current_ActiveTask.empty())
    {
        return;
    }

    computeMaxDitanceTask(current_MaxDistanceTask, current_taskGroup, current_window_endTime);

    /**
     * 迭代匹配
     * 工人批量时，工人不可用或者任务不可用停止循环匹配
     * 未匹配或匹配失败的批次工人优先匹配
     * 偏好列表ORDER
     */

    while (!current_ActiveTask.empty())
    {

        current_NextActiveTask.clear();

        for (int i = 0; i < current_ActiveTask.size(); i++)
            current_NextActiveTask.push_back(current_ActiveTask[i]);

        int matchingnumber = 0, replacematching = 0;
        for (int i = 0; i < current_ActiveTask.size(); i++)
        {
            int taskid = current_ActiveTask[i];
            int order = current_num_of_chased_worker[taskid];
            int worker_to_chase = current_PT[taskid][order].first;

            current_num_of_chased_worker[taskid] = order + 1;

            if (IFTaskExist(worker_to_chase, taskid, current_PW) != current_PW[worker_to_chase].end())
            {

                if (CurrentTask_Satisfy_TSDA_workNext(current_taskGroup, current_workerGroup, worker_to_chase, taskid, current_Group_workerAD, current_detour_distance, current_Group_worker_subTrajectoryDis, current_CT_Worker, current_poi, current_MaxDistanceTask, &current_task_NeedTime, current_window_endTime))

                {

                    current_CW_Task[taskid] = worker_to_chase;
                    current_CT_Worker[worker_to_chase].push_back(taskid);

                    current_NextActiveTask.erase(find(current_NextActiveTask.begin(), current_NextActiveTask.end(), taskid));

                    Update_AD1(worker_to_chase, taskid, current_Group_workerAD, current_detour_distance);

                    UpdateTaskDeadline_TSDA_workNext(current_taskGroup, current_workerGroup, false, 0, worker_to_chase, taskid, current_detour_distance, current_CT_Worker, current_poi, current_MaxDistanceTask, current_task_NeedTime);
                    matchingnumber++;
                }
                else
                {

                    int MinReplaceTask = FindReplaceTaskNew_TSDA_workNext(worker_to_chase, taskid, current_PW, current_Group_workerAD, current_detour_distance, current_Group_worker_subTrajectoryDis, current_CT_Worker, current_poi, current_MaxDistanceTask, &current_task_NeedTime, current_taskGroup, current_workerGroup, current_window_endTime);
                    if (MinReplaceTask != -1)
                    {
                        current_CW_Task[taskid] = worker_to_chase;
                        current_CW_Task[MinReplaceTask] = -1;
                        current_CT_Worker[worker_to_chase].push_back(taskid);

                        current_CT_Worker[worker_to_chase].erase(find(current_CT_Worker[worker_to_chase].begin(), current_CT_Worker[worker_to_chase].end(), MinReplaceTask));

                        if (current_num_of_chased_worker[MinReplaceTask] < current_PT[MinReplaceTask].size())
                            current_NextActiveTask.push_back(MinReplaceTask);
                        current_NextActiveTask.erase(find(current_NextActiveTask.begin(), current_NextActiveTask.end(), taskid));

                        Update_AD2(worker_to_chase, taskid, MinReplaceTask, current_Group_workerAD, current_detour_distance);

                        UpdateTaskDeadline_TSDA_workNext(current_taskGroup, current_workerGroup, true, MinReplaceTask, worker_to_chase, taskid, current_detour_distance, current_CT_Worker, current_poi, current_MaxDistanceTask, current_task_NeedTime);

                        current_MaxDistanceTask[MinReplaceTask] = current_taskGroup[MinReplaceTask].task.Deadline * speed;

                        replacematching++;
                    }
                }
            }
            if (current_num_of_chased_worker[taskid] == current_PT[taskid].size())
            {

                vector<int>::iterator iter1 = find(current_NextActiveTask.begin(), current_NextActiveTask.end(), taskid);
                if (iter1 != current_NextActiveTask.end())
                {
                    current_NextActiveTask.erase(find(current_NextActiveTask.begin(), current_NextActiveTask.end(), taskid));
                }
            }
        }

        current_ActiveTask.clear();
        for (int i = 0; i < current_NextActiveTask.size(); i++)
            current_ActiveTask.push_back(current_NextActiveTask[i]);
    }

    int current_num_of_chased_tasks[current_Number_Worker] = {0};

    vector<int> current_ActiveWorker;

    vector<int> current_NextActiveWorker(current_ActiveWorker);

    current_task_NeedTime = 0.0;

    for (int i = 0; i < current_workerGroup.size(); i++)
    {
        if (current_PW[i].size() != 0)
        {

            current_ActiveWorker.push_back(i);
        }
    }

    /**
     * 迭代匹配
     * 工人批量时，工人不可用或者任务不可用停止循环匹配
     * 未匹配或匹配失败的批次工人优先匹配
     * 偏好列表ORDER
     */

    int matchingTimes = 0;
    while (!current_ActiveWorker.empty())
    {

        current_NextActiveWorker.clear();

        for (int i = 0; i < current_ActiveWorker.size(); i++)
            current_NextActiveWorker.push_back(current_ActiveWorker[i]);

        int matchingnumber = 0, replacematching = 0;
        for (int i = 0; i < current_ActiveWorker.size(); i++)
        {
            int workerid = current_ActiveWorker[i];
            int order = current_num_of_chased_tasks[workerid];

            int task_to_chase = current_PW[workerid][order].first;

            current_num_of_chased_tasks[workerid] = order + 1;

            if (IFWorkerExist(workerid, task_to_chase, current_PT) != current_PT[task_to_chase].end())
            {

                if (CurrentTask_Satisfy_TSDA_workNext(current_taskGroup, current_workerGroup, workerid, task_to_chase, current_Group_workerAD, current_detour_distance, current_Group_worker_subTrajectoryDis, current_CT_Worker, current_poi, current_MaxDistanceTask, &current_task_NeedTime, current_window_endTime))

                {

                    if (current_CW_Task[task_to_chase] == -1)
                    {

                        current_CW_Task[task_to_chase] = workerid;
                        current_CT_Worker[workerid].push_back(task_to_chase);

                        Update_AD1(workerid, task_to_chase, current_Group_workerAD, current_detour_distance);

                        UpdateTaskDeadline_WSDA_workNext(current_taskGroup, current_workerGroup, false, 0, workerid, task_to_chase, current_detour_distance, current_CT_Worker, current_poi, current_MaxDistanceTask, current_task_NeedTime);
                        matchingnumber++;
                    }
                    else
                    {

                        int oldworkerid = current_CW_Task[task_to_chase];
                        if (GetIndex_PT(workerid, task_to_chase, current_PT) < GetIndex_PT(oldworkerid, task_to_chase, current_PT))
                        {

                            current_CW_Task[task_to_chase] = workerid;
                            current_CT_Worker[workerid].push_back(task_to_chase);

                            Update_AD1(workerid, task_to_chase, current_Group_workerAD, current_detour_distance);

                            current_CT_Worker[oldworkerid].erase(find(current_CT_Worker[oldworkerid].begin(), current_CT_Worker[oldworkerid].end(), task_to_chase));

                            Update_AD2_WSDA(oldworkerid, task_to_chase, current_Group_workerAD, current_detour_distance);

                            current_MaxDistanceTask[task_to_chase] = (current_taskGroup[task_to_chase].task.Deadline - current_window_endTime) * speed;
                            UpdateTaskDeadline_WSDA_workNext(current_taskGroup, current_workerGroup, true, oldworkerid, workerid, task_to_chase, current_detour_distance, current_CT_Worker, current_poi, current_MaxDistanceTask, current_task_NeedTime);

                            replacematching++;
                        }
                    }
                }
            }
            if (current_num_of_chased_tasks[workerid] >= current_PW[workerid].size())
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
        for (int i = 0; i < current_NextActiveWorker.size(); i++)
            current_ActiveWorker.push_back(current_NextActiveWorker[i]);
    }

    for (int i = 0; i < current_Number_Worker; i++)
    {

        if (current_CT_Worker[i].size() + global_CT_Worker[current_workerGroup[i].Original_Local].size() == Capacity)
        {
            current_workerGroup[i].sign = false;
            for (auto m : current_CT_Worker[i])
            {
                global_CT_Worker[current_workerGroup[i].Original_Local].push_back(current_taskGroup[m].Original_Local);
                current_taskGroup[m].sign = false;
            }
        }
        else
        {
            for (auto m : current_CT_Worker[i])
            {
                global_CT_Worker[current_workerGroup[i].Original_Local].push_back(current_taskGroup[m].Original_Local);
                global_MaxDistanceTask[current_taskGroup[m].Original_Local] = current_MaxDistanceTask[m];
                current_taskGroup[m].sign = false;
            }
        }
    }

    ShowCTMatching(current_CT_Worker, current_Number_Worker);
}

void Basic_information::Grouping_Framework_AlternateDA_workNext(vector<TASK> &tasks, vector<WORKER> &workers, double Wmax, int Tmax, vector<double> &Sumdis, vector<vector<double>> &global_Worker_subTrajectoryDis)

{
    vector<CURRENT_TASK_GROUP> current_taskGroup;
    vector<CURRENT_WORKERS_GROUP> current_Group_worker;
    vector<double> current_Group_workerSumdis;
    vector<double> current_Group_workerAD;
    double current_window_endTime = tasks[0].startTime + Wmax;
    double current_window_startTime = tasks[0].startTime;
    int current_workID = global_Current_workID;

    vector<vector<double>> current_Group_worker_subTrajectoryDis;

    int sign = 0;
    int temp = 0;

    for (int i = 0; i < tasks.size(); i++)
    {

        TASK task = tasks[i];
        sign++;

        CURRENT_TASK_GROUP taskg;
        taskg.task = task;
        taskg.Original_Local = i;

        if ((sign - temp == Tmax) || (task.startTime >= current_window_endTime))
        {
            temp = current_taskGroup.size();
            current_workID = global_Current_workID;
            current_window_endTime = task.startTime;
            groupWork_according_TaskGroup_workNext(workers, Sumdis, current_Group_worker, current_Group_workerSumdis, current_window_startTime, current_window_endTime, current_workID, current_Group_workerAD, global_Worker_subTrajectoryDis, current_Group_worker_subTrajectoryDis);

            determine_Window_Task_Timeout(current_taskGroup, current_window_endTime);

            match_WorkerTask_AlternateDA_workNext(current_taskGroup, current_Group_worker, current_window_endTime, current_Group_workerAD, current_Group_worker_subTrajectoryDis);

            current_window_startTime = current_window_endTime;
            current_window_endTime += Wmax;

            updata_current_Info_workNext(current_taskGroup, current_Group_worker, current_Group_workerAD, current_Group_worker_subTrajectoryDis, current_Group_workerSumdis);
        }

        current_taskGroup.push_back(taskg);
    }

    if (current_taskGroup.size() > 0)
    {

        current_workID = global_Current_workID;
        current_window_startTime = current_taskGroup.back().task.startTime;

        groupWork_according_TaskGroup_workNext(workers, Sumdis, current_Group_worker, current_Group_workerSumdis, current_window_startTime, current_window_endTime, current_workID, current_Group_workerAD, global_Worker_subTrajectoryDis, current_Group_worker_subTrajectoryDis);

        determine_Window_Task_Timeout(current_taskGroup, current_window_endTime);

        match_WorkerTask_AlternateDA_workNext(current_taskGroup, current_Group_worker, current_window_endTime, current_Group_workerAD, current_Group_worker_subTrajectoryDis);
    }
}

void Basic_information::match_WorkerTask_AlternateDA_workNext(vector<CURRENT_TASK_GROUP> &current_taskGroup, vector<CURRENT_WORKERS_GROUP> &current_workerGroup, double current_window_endTime, vector<double> &current_Group_workerAD, vector<vector<double>> &current_Group_worker_subTrajectoryDis)
{

    int current_Number_Worker = current_workerGroup.size();
    int current_Number_Task = current_taskGroup.size();

    vector<double> current_detour_distance[current_Number_Task];
    vector<int> current_poi[current_Number_Task];
    vector<vector<pair<int, double>>> current_PT(Number_Task);
    vector<vector<pair<int, double>>> current_PW(Number_Worker);

    int current_CW_Task[current_Number_Task];
    int current_num_of_chased_worker[current_Number_Task] = {0};
    vector<int> current_ActiveTask;
    vector<int> current_NextActiveTask(current_ActiveTask);

    vector<vector<int>> current_CT_Worker(current_Number_Worker);
    double current_MaxDistanceTask[current_Number_Task] = {0.0};
    double current_task_NeedTime = 0.0;

    for (int i = 0; i < current_Number_Task; i++)
    {
        current_detour_distance[i] = vector<double>(current_Number_Worker, 0);

        current_poi[i] = vector<int>(current_Number_Worker, 0);
    }

    Compute_PTPW_Group_workerBatch_workNext(current_PT, current_PW, current_detour_distance, current_taskGroup, current_workerGroup, current_poi);
    /**
     * 任务hy
     */
    for (int i = 0; i < current_taskGroup.size(); i++)
    {
        if (current_PT[i].size() != 0)
        {
            current_ActiveTask.push_back(i);
        }
        current_CW_Task[i] = -1;
    }
    if (current_ActiveTask.empty())
    {
        return;
    }

    /**
     * 工人hy
     */

    int current_num_of_chased_tasks[current_Number_Worker] = {0};

    vector<int> current_ActiveWorker;

    vector<int> current_NextActiveWorker(current_ActiveWorker);

    current_task_NeedTime = 0.0;

    for (int i = 0; i < current_workerGroup.size(); i++)
    {
        if (current_PW[i].size() != 0)
        {

            current_ActiveWorker.push_back(i);
        }
    }

    computeMaxDitanceTask(current_MaxDistanceTask, current_taskGroup, current_window_endTime);

    /**
     * 迭代匹配
     * 工人批量时，工人不可用或者任务不可用停止循环匹配
     * 未匹配或匹配失败的批次工人优先匹配
     * 偏好列表ORDER
     */

    int flagState = 0;
    /***
     * 0:两个都没完成
     * 1:任务完成
     * 2:工人完成
     * 3:两个皆完成
     */
    int matchingTimes = 0;
    while (flagState < 3)
    {
        while (!current_ActiveWorker.empty())
        {
            current_NextActiveWorker.clear();

            for (int i = 0; i < current_ActiveWorker.size(); i++)
                current_NextActiveWorker.push_back(current_ActiveWorker[i]);

            int matchingnumber = 0, replacematching = 0;
            for (int i = 0; i < current_ActiveWorker.size(); i++)
            {
                int workerid = current_ActiveWorker[i];
                int order = current_num_of_chased_tasks[workerid];

                int task_to_chase = current_PW[workerid][order].first;

                current_num_of_chased_tasks[workerid] = order + 1;

                if (IFWorkerExist(workerid, task_to_chase, current_PT) != current_PT[task_to_chase].end())
                {

                    if (CurrentTask_Satisfy_TSDA_workNext(current_taskGroup, current_workerGroup, workerid, task_to_chase, current_Group_workerAD, current_detour_distance, current_Group_worker_subTrajectoryDis, current_CT_Worker, current_poi, current_MaxDistanceTask, &current_task_NeedTime, current_window_endTime))

                    {

                        if (current_CW_Task[task_to_chase] == -1)
                        {

                            current_CW_Task[task_to_chase] = workerid;
                            current_CT_Worker[workerid].push_back(task_to_chase);

                            current_NextActiveTask.erase(std::remove(current_NextActiveTask.begin(), current_NextActiveTask.end(), task_to_chase), current_NextActiveTask.end());

                            Update_AD1(workerid, task_to_chase, current_Group_workerAD, current_detour_distance);

                            UpdateTaskDeadline_WSDA_workNext(current_taskGroup, current_workerGroup, false, 0, workerid, task_to_chase, current_detour_distance, current_CT_Worker, current_poi, current_MaxDistanceTask, current_task_NeedTime);
                            matchingnumber++;
                        }
                        else
                        {

                            int oldworkerid = current_CW_Task[task_to_chase];
                            if (GetIndex_PT(workerid, task_to_chase, current_PT) < GetIndex_PT(oldworkerid, task_to_chase, current_PT))
                            {

                                current_CW_Task[task_to_chase] = workerid;
                                current_CT_Worker[workerid].push_back(task_to_chase);

                                Update_AD1(workerid, task_to_chase, current_Group_workerAD, current_detour_distance);

                                current_CT_Worker[oldworkerid].erase(find(current_CT_Worker[oldworkerid].begin(), current_CT_Worker[oldworkerid].end(), task_to_chase));

                                Update_AD2_WSDA(oldworkerid, task_to_chase, current_Group_workerAD, current_detour_distance);

                                current_MaxDistanceTask[task_to_chase] = (current_taskGroup[task_to_chase].task.Deadline - current_window_endTime) * speed;
                                UpdateTaskDeadline_WSDA_workNext(current_taskGroup, current_workerGroup, true, oldworkerid, workerid, task_to_chase, current_detour_distance, current_CT_Worker, current_poi, current_MaxDistanceTask, current_task_NeedTime);

                                replacematching++;
                            }
                        }
                    }
                }
                if (current_num_of_chased_tasks[workerid] >= current_PW[workerid].size())
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

            break;
        }

        while (!current_ActiveTask.empty())
        {

            current_NextActiveTask.clear();

            for (int i = 0; i < current_ActiveTask.size(); i++)
                current_NextActiveTask.push_back(current_ActiveTask[i]);

            int matchingnumber = 0, replacematching = 0;
            for (int i = 0; i < current_ActiveTask.size(); i++)
            {

                int taskid = current_ActiveTask[i];
                int order = current_num_of_chased_worker[taskid];
                int worker_to_chase = current_PT[taskid][order].first;

                current_num_of_chased_worker[taskid] = order + 1;

                if (IFTaskExist(worker_to_chase, taskid, current_PW) != current_PW[worker_to_chase].end())
                {

                    if (CurrentTask_Satisfy_TSDA_workNext(current_taskGroup, current_workerGroup, worker_to_chase, taskid, current_Group_workerAD, current_detour_distance, current_Group_worker_subTrajectoryDis, current_CT_Worker, current_poi, current_MaxDistanceTask, &current_task_NeedTime, current_window_endTime))

                    {

                        current_CW_Task[taskid] = worker_to_chase;
                        current_CT_Worker[worker_to_chase].push_back(taskid);

                        current_NextActiveTask.erase(find(current_NextActiveTask.begin(), current_NextActiveTask.end(), taskid));

                        Update_AD1(worker_to_chase, taskid, current_Group_workerAD, current_detour_distance);

                        UpdateTaskDeadline_TSDA_workNext(current_taskGroup, current_workerGroup, false, 0, worker_to_chase, taskid, current_detour_distance, current_CT_Worker, current_poi, current_MaxDistanceTask, current_task_NeedTime);
                        matchingnumber++;
                    }
                    else
                    {

                        int MinReplaceTask = FindReplaceTaskNew_TSDA_workNext(worker_to_chase, taskid, current_PW, current_Group_workerAD, current_detour_distance, current_Group_worker_subTrajectoryDis, current_CT_Worker, current_poi, current_MaxDistanceTask, &current_task_NeedTime, current_taskGroup, current_workerGroup, current_window_endTime);
                        if (MinReplaceTask != -1)
                        {
                            current_CW_Task[taskid] = worker_to_chase;
                            current_CW_Task[MinReplaceTask] = -1;

                            current_CT_Worker[worker_to_chase].erase(find(current_CT_Worker[worker_to_chase].begin(), current_CT_Worker[worker_to_chase].end(), MinReplaceTask));
                            current_CT_Worker[worker_to_chase].push_back(taskid);

                            if (current_num_of_chased_worker[MinReplaceTask] < current_PT[MinReplaceTask].size())
                                current_NextActiveTask.push_back(MinReplaceTask);
                            current_NextActiveTask.erase(find(current_NextActiveTask.begin(), current_NextActiveTask.end(), taskid));

                            Update_AD2(worker_to_chase, taskid, MinReplaceTask, current_Group_workerAD, current_detour_distance);

                            UpdateTaskDeadline_TSDA_workNext(current_taskGroup, current_workerGroup, true, MinReplaceTask, worker_to_chase, taskid, current_detour_distance, current_CT_Worker, current_poi, current_MaxDistanceTask, current_task_NeedTime);

                            current_MaxDistanceTask[MinReplaceTask] = current_taskGroup[MinReplaceTask].task.Deadline * speed;

                            replacematching++;
                        }
                        else
                        {
                        }
                    }
                }
                if (current_num_of_chased_worker[taskid] == current_PT[taskid].size())
                {

                    vector<int>::iterator iter1 = find(current_NextActiveTask.begin(), current_NextActiveTask.end(), taskid);
                    if (iter1 != current_NextActiveTask.end())
                    {
                        current_NextActiveTask.erase(find(current_NextActiveTask.begin(), current_NextActiveTask.end(), taskid));
                    }
                }
            }

            break;
        }

        /**
         * 迭代匹配
         * 工人批量时，工人不可用或者任务不可用停止循环匹配
         * 未匹配或匹配失败的批次工人优先匹配
         * 偏好列表ORDER
         */

        current_ActiveTask.clear();
        for (int i = 0; i < current_NextActiveTask.size(); i++)
            current_ActiveTask.push_back(current_NextActiveTask[i]);

        current_ActiveWorker.clear();
        for (int i = 0; i < current_NextActiveWorker.size(); i++)
            current_ActiveWorker.push_back(current_NextActiveWorker[i]);

        if (current_ActiveTask.empty() && current_ActiveWorker.empty())
        {
            flagState = 3;
        }
    }

    for (int i = 0; i < current_Number_Worker; i++)
    {

        if (current_CT_Worker[i].size() + global_CT_Worker[current_workerGroup[i].Original_Local].size() == Capacity)
        {
            current_workerGroup[i].sign = false;
            for (auto m : current_CT_Worker[i])
            {
                global_CT_Worker[current_workerGroup[i].Original_Local].push_back(current_taskGroup[m].Original_Local);
                current_taskGroup[m].sign = false;
            }
        }
        else
        {
            for (auto m : current_CT_Worker[i])
            {
                global_CT_Worker[current_workerGroup[i].Original_Local].push_back(current_taskGroup[m].Original_Local);
                global_MaxDistanceTask[current_taskGroup[m].Original_Local] = current_MaxDistanceTask[m];
                current_taskGroup[m].sign = false;
            }
        }
    }

    ShowCTMatching(current_CT_Worker, current_Number_Worker);
}