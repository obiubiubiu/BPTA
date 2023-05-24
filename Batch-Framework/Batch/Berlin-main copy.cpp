#include "DataManager.h"
#include "Basic_information.h"
#include "Basic_information.cpp"
#include <chrono>
// #include "whole_greedy.cpp"

double run_time;
LARGE_INTEGER time_start; // 开始时间
LARGE_INTEGER time_over;  // 结束时间
double dqFreq;            // 计时器频率
LARGE_INTEGER f;          // 计时器频率
// 需要确定每个里面的东西是否正确，
// 1、与诗婷算法做对比;
// 2、时间是否正确;
// 3、当前匹配结果是否正确;
// 4、当前匹配的是否二次使用;
// 5、当前匹配的是够已经标记为已使用;

int main(int argc, char const *argv[])
{
    double c = 1.0 / 1000;   // 单位距离的工人成本
    double speed = 1000;     // 工人的移动速度1km/min,60km/h,1km/min
    int Number_Task = 400;   // 任务数量
    int Number_Worker = 200; // 工人数量
    int Capacity = 5;        // 工人容量
    int Wmax = 2;            // hy时间窗口
    double Tmax = 5;         // hy任务窗口
    bool sati_state = true;  // 求解满意度的方式，true：平均。false：sum

    /**
     * 获取全局信息
     */
    int dataOption = 1; // 用于选择哪个数据集
    cout << "输入数据集dataOption，其中：1:Berlin\t2:G_mission\t3:T_Drive" << endl;
    // cin >> dataOption;
    // 经纬度
    Basic_information info(c, speed, Number_Task, Number_Worker, Capacity, Wmax, Tmax, sati_state, dataOption);

    // 单位距离的工人成本、单位速度、任务数、工人数、公交站点数、工人容量、窗口时间、窗口数量、求满意度方式(true为agv，false为sum）、数据集选择
    // 欧氏距离
    // Basic_information info(1.0 / 1000, 1000, 20, 50, 4346, 5, 0.01, 5, true, dataOption); // 单位距离的工人成本、单位速度、任务数、工人数、公交站点数、工人容量、窗口时间、窗口数量、求满意度方式、数据集选择

    vector<vector<double>> Worker_subTrajectoryDis(info.Number_Worker); // 记录工人每个轨迹点前的距之和，无需修改
    vector<double> Sumdis(info.Number_Worker, 0);                       // woker的整条轨迹的距离

    optionDataset(dataOption, info, Worker_subTrajectoryDis, Sumdis); // 获取数据,从txt获取

    sort(info.global_tasks.begin(), info.global_tasks.end(), cmp_task_start); // 对任务和工人分别按照开始时间进行升序排序
    sortWork(info.global_workers, Sumdis, info.global_Sumdis, Worker_subTrajectoryDis, info.global_Worker_subTrajectoryDis);
    info.Compute_global_PTPW_Group(info.global_PT, info.global_PW); // 计算全部可用工人\任务及偏好值并加入到偏序列表，用于对比信息
    // info.print_info();

    /***
     * 贪心算法的计时开始
     */
    cout << "贪心算法" << endl;
    // QueryPerformanceFrequency(&f);
    // dqFreq = (double)f.QuadPart;
    // QueryPerformanceCounter(&time_start);
    auto start = std::chrono::high_resolution_clock::now();
    info.Grouping_Framework_Greedy(info.global_tasks, info.global_workers, info.Wmax, info.Tmax, info.global_Sumdis, info.global_Worker_subTrajectoryDis); // hy-- - 增加窗口划分框架
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    // QueryPerformanceCounter(&time_over);                                                                                                            // 计时结束
    // run_time = (time_over.QuadPart - time_start.QuadPart) / dqFreq;                                                                                 // 乘以1000000把单位由秒化为微秒，精度为1000 000/（cpu主频）微秒
    printf("\nSumArrRow run_time：%fs\n", duration.count());
    info.printf_Satisfaction_Results("贪心算法", duration.count());
    cout << endl;

    /*****
     * workerBatch 计时开始
     */

    info.begin_Algorithm("workerBatch算法");
    // QueryPerformanceFrequency(&f);
    // dqFreq = (double)f.QuadPart;
    // QueryPerformanceCounter(&time_start);
    start = std::chrono::high_resolution_clock::now();
    info.Grouping_Framework_WorkerBatch(info.global_tasks, info.global_workers, info.Wmax, info.Tmax, info.global_Sumdis, info.global_Worker_subTrajectoryDis); // hy-- - 增加窗口划分框架

    end = std::chrono::high_resolution_clock::now();
    duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    // QueryPerformanceCounter(&time_over);                                                                                                                 // 计时结束
    // run_time = (time_over.QuadPart - time_start.QuadPart) / dqFreq;                                                                                      // 乘以1000000把单位由秒化为微秒，精度为1000 000/（cpu主频）微秒
    printf("\nSumArrRow run_time：%fs\n", duration.count());
    info.printf_Satisfaction_Results("workerBatch算法", duration.count());

    /*****
     * TPPG算法 计时开始
     */

    info.begin_Algorithm("TPPG算法");
    QueryPerformanceFrequency(&f);
    dqFreq = (double)f.QuadPart;
    QueryPerformanceCounter(&time_start);
    info.Grouping_Framework_TPPG(info.global_tasks, info.global_workers, info.Wmax, info.Tmax, info.global_Sumdis, info.global_Worker_subTrajectoryDis); // hy-- - 增加窗口划分框架
    QueryPerformanceCounter(&time_over);                                                                                                                 // 计时结束
    run_time = (time_over.QuadPart - time_start.QuadPart) / dqFreq;                                                                                      // 乘以1000000把单位由秒化为微秒，精度为1000 000/（cpu主频）微秒
    printf("\nSumArrRow run_time：%fs\n", run_time);
    info.printf_Satisfaction_Results("TPPG算法", run_time);

    /*****
     * TSDA算法 计时开始,,虽然已经完成，但是还有可能存在瑕疵需要修稿
     */

    info.begin_Algorithm("TSDA算法");
    QueryPerformanceFrequency(&f);
    dqFreq = (double)f.QuadPart;
    QueryPerformanceCounter(&time_start);
    info.Grouping_Framework_TSDA(info.global_tasks, info.global_workers, info.Wmax, info.Tmax, info.global_Sumdis, info.global_Worker_subTrajectoryDis); // hy-- - 增加窗口划分框架
    QueryPerformanceCounter(&time_over);                                                                                                                 // 计时结束
    run_time = (time_over.QuadPart - time_start.QuadPart) / dqFreq;                                                                                      // 乘以1000000把单位由秒化为微秒，精度为1000 000/（cpu主频）微秒
    printf("\nSumArrRow run_time：%fs\n", run_time);
    info.printf_Satisfaction_Results("TSDA算法", run_time);

    /*****
     * WPPG算法 计时开始,,虽然已经完成，但是还有可能存在瑕疵需要修稿
     */

    info.begin_Algorithm("WPPG算法");
    QueryPerformanceFrequency(&f);
    dqFreq = (double)f.QuadPart;
    QueryPerformanceCounter(&time_start);
    info.Grouping_Framework_WPPG(info.global_tasks, info.global_workers, info.Wmax, info.Tmax, info.global_Sumdis, info.global_Worker_subTrajectoryDis); // hy-- - 增加窗口划分框架
    QueryPerformanceCounter(&time_over);                                                                                                                 // 计时结束
    run_time = (time_over.QuadPart - time_start.QuadPart) / dqFreq;                                                                                      // 乘以1000000把单位由秒化为微秒，精度为1000 000/（cpu主频）微秒
    printf("\nSumArrRow run_time：%fs\n", run_time);
    // info.printf_Satisfaction_Results("WPPG算法", run_time);

    /*****
     * Dynamic_whole 算法 按照时间先来先匹配
     */
    info.begin_Algorithm("Dynamic_whole算法");
    QueryPerformanceFrequency(&f);
    dqFreq = (double)f.QuadPart;
    QueryPerformanceCounter(&time_start);
    info.whole_Greedy_Framework();
    QueryPerformanceCounter(&time_over);                            // 计时结束
    run_time = (time_over.QuadPart - time_start.QuadPart) / dqFreq; // 乘以1000000把单位由秒化为微秒，精度为1000 000/（cpu主频）微秒
    printf("\nSumArrRow run_time：%fs\n", run_time);
    info.printf_Satisfaction_Results("Dynamic_whole算法", run_time);
    // int num = 0;
    // for (auto i : info.global_POI)
    // {
    //     cout << num << endl;
    //     cout << "\t";
    //     for (int j = 0; j < i.size(); j++)
    //     {
    //         cout << i[j] << " ";
    //     }
    //     num++;
    // }

    return 0;
}
