#include "DataManager.h"
#include "Basic_information.h"
#include "Basic_information.cpp"

// #include "whole_greedy.cpp"
// 对于任务说

int main(int argc, char const *argv[])
{
    double c = 1.0 / 1000; // 单位距离的工人成本
    double speed = 1000;   // 工人的移动速度1km/min,60km/h,1km/min
    double endtimeX = 0.5; // 用于计算工人结束时间的参数，

    int Number_Task = 1000;  // 任务数量（Gmission：713）、（berlin：无限）、（T-drive：无限）
    int Number_Worker = 200; // 工人数量（Gmission：199）、（berlin： 236）、（T-drive： 5823）

    int Capacity = 5;       // 工人容量
    int Wmax = 2;           // hy时间窗口
    double Tmax = 20;       // hy任务窗口
    bool sati_state = true; // 求解满意度的方式，true：平均。false：sum

    /**
     * 获取全局信息
     */
    int dataOption = 1; // 用于选择哪个数据集
    cout << "输入数据集dataOption，其中：\n"
         << "\t1 : Berlin   \n"
         << "\t2 : G_mission \n"
         << "\t3 : T_Drive  " << endl;
    // cin >> dataOption;
    // 经纬度
    Basic_information info(c, speed, Number_Task, Number_Worker, Capacity, Wmax, Tmax, sati_state, dataOption);

    // 欧氏距离
    // Basic_information info(1.0 / 1000, 1000, 20, 50, 4346, 5, 0.01, 5, true, dataOption); // 单位距离的工人成本、单位速度、任务数、工人数、公交站点数、工人容量、窗口时间、窗口数量、求满意度方式、数据集选择

    vector<vector<double>> Worker_subTrajectoryDis(info.Number_Worker); // 记录工人每个轨迹点前的距之和，无需修改
    vector<double> Sumdis(info.Number_Worker, 0);                       // woker的轨迹距离之和

    bool dataset_state = optionDataset(dataOption, info, Worker_subTrajectoryDis, Sumdis, endtimeX); // 获取数据,从txt获取
    if (dataset_state)
    {

        cout << "获取数据结束" << endl;

        sort(info.global_tasks.begin(), info.global_tasks.end(), cmp_task_start); // 对任务和工人分别按照开始时间进行升序排序
        sortWork(info.global_workers, Sumdis, info.global_Sumdis, Worker_subTrajectoryDis, info.global_Worker_subTrajectoryDis);
        info.Compute_global_PTPW_Group(info.global_PT, info.global_PW); // 计算全部可用工人\任务及偏好值并加入到偏序列表，用于对比信息

        /***
         * 贪心算法的计时开始
         */

        info.begin_Algorithm("贪心算法");
        auto start = std::chrono::high_resolution_clock::now();
        info.Grouping_Framework_Greedy(info.global_tasks, info.global_workers, info.Wmax, info.Tmax, info.global_Sumdis, info.global_Worker_subTrajectoryDis); // hy-- - 增加窗口划分框架
        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
        info.printf_Satisfaction_Results("贪心算法", duration.count());
        cout << endl;

        /*****
         * workerBatch 计时开始
         */

        info.begin_Algorithm("workerBatch算法");
        start = std::chrono::high_resolution_clock::now();
        info.Grouping_Framework_WorkerBatch(info.global_tasks, info.global_workers, info.Wmax, info.Tmax, info.global_Sumdis, info.global_Worker_subTrajectoryDis); // hy-- - 增加窗口划分框架
        end = std::chrono::high_resolution_clock::now();
        duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
        info.printf_Satisfaction_Results("workerBatch算法", duration.count());

        /*****
         * TPPG算法 计时开始
         */

        info.begin_Algorithm("TPPG算法");
        start = std::chrono::high_resolution_clock::now();
        info.Grouping_Framework_TPPG(info.global_tasks, info.global_workers, info.Wmax, info.Tmax, info.global_Sumdis, info.global_Worker_subTrajectoryDis); // hy-- - 增加窗口划分框架
        end = std::chrono::high_resolution_clock::now();
        duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
        info.printf_Satisfaction_Results("TPPG算法", duration.count());

        /*****
         * TSDA算法 计时开始,,虽然已经完成，但是还有可能存在瑕疵需要修稿
         */

        info.begin_Algorithm("TSDA算法");
        start = std::chrono::high_resolution_clock::now();
        info.Grouping_Framework_TSDA(info.global_tasks, info.global_workers, info.Wmax, info.Tmax, info.global_Sumdis, info.global_Worker_subTrajectoryDis); // hy-- - 增加窗口划分框架
        end = std::chrono::high_resolution_clock::now();
        duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
        info.printf_Satisfaction_Results("TSDA算法", duration.count());

        /*****
         * WPPG算法 计时开始,,虽然已经完成，但是还有可能存在瑕疵需要修稿
         */

        info.begin_Algorithm("WPPG算法");
        start = std::chrono::high_resolution_clock::now();
        info.Grouping_Framework_WPPG(info.global_tasks, info.global_workers, info.Wmax, info.Tmax, info.global_Sumdis, info.global_Worker_subTrajectoryDis); // hy-- - 增加窗口划分框架
        end = std::chrono::high_resolution_clock::now();
        duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
        info.printf_Satisfaction_Results("WPPG算法", duration.count());

        /*****
         * WSDA算法 计时开始,,虽然已经完成，但是还有可能存在瑕疵需要修稿
         */

        info.begin_Algorithm("WSDA算法");
        start = std::chrono::high_resolution_clock::now();
        info.Grouping_Framework_WSDA(info.global_tasks, info.global_workers, info.Wmax, info.Tmax, info.global_Sumdis, info.global_Worker_subTrajectoryDis); // hy-- - 增加窗口划分框架
        end = std::chrono::high_resolution_clock::now();
        duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
        info.printf_Satisfaction_Results("WSDA算法", duration.count());

        /*****
         * Dynamic_whole 算法 按照时间先来先匹配
         */
        info.begin_Algorithm("Dynamic_whole算法");
        start = std::chrono::high_resolution_clock::now();
        info.whole_Greedy_Framework();
        end = std::chrono::high_resolution_clock::now();
        duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
        info.printf_Satisfaction_Results("Dynamic_whole算法", duration.count());
    }
    else
    {
        cout << "输入错误" << endl;
    }
    return 0;
}
