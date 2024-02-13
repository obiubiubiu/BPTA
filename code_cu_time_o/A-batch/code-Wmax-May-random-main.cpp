#include "DataManager.h"
#include "Basic_information.h"
#include "Basic_information.cpp"
#include <string>

int main(int argc, char const *argv[])
{

    {
        double c = 1.0 / 1000;
        double speed = 1000;
        double workEndtimeX = 0.5;
        double taskEndtimeX = 0.3;
        double rangeX = 1500;
        double scoreX = 1;

        int Number_Task = 5000;
        int Number_Worker = 3000;

        int Capacity = 5;

        double Tmax = 200;
        int Wmax = 20;
        bool sati_state = true;

        int ntask[6];
        int nworker[5];
        int nrange[5] = {500, 1000, 1500, 2000, 2500};
        int nCapacity[7] = {1, 2, 3, 4, 5, 6, 7};
        int nTmax[8] = {20, 40, 60, 80, 100, 120, 140, 160};
        int nWmax[8] = {2, 5, 10, 20, 50, 100, 200};

        /**
         * 获取全局信息
         */

        bool distributionOption = false;
        int dataOption = 1;
        cout << "输入数据集dataOption，其中：\n"
             << "\t1 : Berlin   \n"
             << "\t2 : G_mission \n"
             << "\t3 : T_Drive  " << endl;

        if (dataOption == 1)
        {

            Number_Task = 2000;
            Number_Worker = 500;
            int ntask1[7] = {100, 500, 1000, 1500, 2000, 2500, 3000};
            int nworker1[5] = {100, 250, 500, 750, 1000};
            std::copy(std::begin(ntask1), std::end(ntask1), std::begin(ntask));
            std::copy(std::begin(nworker1), std::end(nworker1), std::begin(nworker));
        }
        else if (dataOption == 2)
        {

            Number_Task = 2000;
            Number_Worker = 100;
            int ntask1[7] = {50, 150, 250, 350, 450, 550, 650};
            int nworker1[5] = {25, 50, 100, 150, 199};
            std::copy(std::begin(ntask1), std::end(ntask1), std::begin(ntask));
            std::copy(std::begin(nworker1), std::end(nworker1), std::begin(nworker));
        }
        else if (dataOption == 3)
        {
            Number_Task = 10000;
            Number_Worker = 3000;
            int ntask1[7] = {500, 1000, 4000, 5000, 10000, 15000, 20000};
            int nworker1[5] = {1000, 2000, 3000, 4000, 5000};
            std::copy(std::begin(ntask1), std::end(ntask1), std::begin(ntask));
            std::copy(std::begin(nworker1), std::end(nworker1), std::begin(nworker));
        }

        {
            for (int algorithmOption = 0; algorithmOption <= 9; algorithmOption++)
            {
                for (int test_taskNum = 1; test_taskNum <= 1; test_taskNum++)
                {
                    for (int test_workerNum = 1; test_workerNum <= 1; test_workerNum++)
                    {
                        for (int test_rangeNum = 1; test_rangeNum <= 1; test_rangeNum++)
                        {
                            for (int test_Capacity = 1; test_Capacity <= 1; test_Capacity++)
                            {
                                for (int test_Tmax = 1; test_Tmax <= 1; test_Tmax++)
                                {
                                    for (int test_Wmax = 0; test_Wmax <= 6; test_Wmax++)
                                    {

                                        cout << endl;
                                        avg_task_sati = 0;
                                        avg_work_sati = 0;
                                        avg_work_match_num = 0;
                                        avg_task_match_num = 0;
                                        avg_runtime = 0;
                                        sum_repeat = 0;

                                        Basic_information info(c, speed, Number_Task * test_taskNum, Number_Worker * test_workerNum, Capacity * test_Capacity, nWmax[test_Wmax], Tmax * test_Tmax, sati_state, dataOption);

                                        vector<vector<double>> Worker_subTrajectoryDis(info.Number_Worker);
                                        vector<double> Sumdis(info.Number_Worker, 0);
                                        bool dataset_state = optionDataset(distributionOption, dataOption, info, Worker_subTrajectoryDis, Sumdis, workEndtimeX, taskEndtimeX, nrange[test_rangeNum], scoreX);
                                        cout << "获取数据结束" << endl;

                                        sort(info.global_tasks.begin(), info.global_tasks.end(), cmp_task_start);
                                        sortWork(info.global_workers, Sumdis, info.global_Sumdis, Worker_subTrajectoryDis, info.global_Worker_subTrajectoryDis);
                                        int allmacth = info.Compute_global_PTPW_Group(info.global_PT, info.global_PW);
                                        string alg_name;
                                        for (int repeat_Num = 0; repeat_Num < 5; repeat_Num++)
                                        {
                                            sum_repeat++;
                                            if (dataset_state)
                                            {
                                                if (algorithmOption == 0)
                                                {
                                                    /***
                                                     * 贪心算法的计时开始
                                                     */

                                                    alg_name = "greedy-Algorithm";
                                                    info.begin_Algorithm(alg_name);
                                                    auto start = std::chrono::high_resolution_clock::now();
                                                    info.Grouping_Framework_Greedy(info.global_tasks, info.global_workers, info.Wmax, info.Tmax, info.global_Sumdis, info.global_Worker_subTrajectoryDis);

                                                    auto end = std::chrono::high_resolution_clock::now();
                                                    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
                                                    info.printf_Satisfaction_Results(alg_name, duration.count());
                                                }

                                                else if (algorithmOption == 1)
                                                {
                                                    /*****
                                                     * TPPG算法 计时开始
                                                     */
                                                    alg_name = "TPPG-Algorithm";
                                                    info.begin_Algorithm(alg_name);
                                                    auto start = std::chrono::high_resolution_clock::now();
                                                    info.Grouping_Framework_TPPG(info.global_tasks, info.global_workers, info.Wmax, info.Tmax, info.global_Sumdis, info.global_Worker_subTrajectoryDis);
                                                    auto end = std::chrono::high_resolution_clock::now();
                                                    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
                                                    info.printf_Satisfaction_Results(alg_name, duration.count());
                                                }

                                                else if (algorithmOption == 2)
                                                {
                                                    /*****
                                                     * WPPG算法 计时开始,
                                                     */
                                                    alg_name = "WPPG-Algorithm";
                                                    info.begin_Algorithm(alg_name);
                                                    auto start = std::chrono::high_resolution_clock::now();
                                                    info.Grouping_Framework_WPPG(info.global_tasks, info.global_workers, info.Wmax, info.Tmax, info.global_Sumdis, info.global_Worker_subTrajectoryDis);
                                                    auto end = std::chrono::high_resolution_clock::now();
                                                    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
                                                    info.printf_Satisfaction_Results(alg_name, duration.count());
                                                }

                                                else if (algorithmOption == 3)
                                                {
                                                    /*****
                                                     * TPPGbatch算法 
                                                     */
                                                    alg_name = "TPPG-batch-Algorithm";
                                                    info.begin_Algorithm(alg_name);
                                                    auto start = std::chrono::high_resolution_clock::now();
                                                    info.Grouping_Framework_TPPG_Batch(info.global_tasks, info.global_workers, info.Wmax, info.Tmax, info.global_Sumdis, info.global_Worker_subTrajectoryDis);
                                                    auto end = std::chrono::high_resolution_clock::now();
                                                    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

                                                    info.printf_Satisfaction_Results(alg_name, duration.count());
                                                }

                                                else if (algorithmOption == 4)
                                                {
                                                    /*****
                                                     * workerBatch 计时开始
                                                     */
                                                    alg_name = "workerBatch-Algorithm";
                                                    info.begin_Algorithm(alg_name);
                                                    auto start = std::chrono::high_resolution_clock::now();
                                                    info.Grouping_Framework_WorkerBatch(info.global_tasks, info.global_workers, info.Wmax, info.Tmax, info.global_Sumdis, info.global_Worker_subTrajectoryDis);
                                                    auto end = std::chrono::high_resolution_clock::now();
                                                    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
                                                    info.printf_Satisfaction_Results(alg_name, duration.count());
                                                }
                                                else if (algorithmOption == 5)
                                                {
                                                    /*****
                                                     * TSDA算法 计时开始,
                                                     */
                                                    alg_name = "TSDA-Algorithm";
                                                    info.begin_Algorithm(alg_name);
                                                    auto start = std::chrono::high_resolution_clock::now();
                                                    info.Grouping_Framework_TSDA(info.global_tasks, info.global_workers, info.Wmax, info.Tmax, info.global_Sumdis, info.global_Worker_subTrajectoryDis);
                                                    auto end = std::chrono::high_resolution_clock::now();
                                                    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
                                                    info.printf_Satisfaction_Results(alg_name, duration.count());
                                                }
                                                else if (algorithmOption == 6)
                                                {
                                                    /*****
                                                     * WSDA算法 计时开始,
                                                     */
                                                    alg_name = "WSDA-Algorithm";
                                                    info.begin_Algorithm(alg_name);
                                                    auto start = std::chrono::high_resolution_clock::now();
                                                    info.Grouping_Framework_WSDA(info.global_tasks, info.global_workers, info.Wmax, info.Tmax, info.global_Sumdis, info.global_Worker_subTrajectoryDis);
                                                    auto end = std::chrono::high_resolution_clock::now();
                                                    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
                                                    info.printf_Satisfaction_Results(alg_name, duration.count());
                                                }

                                                else if (algorithmOption == 7)
                                                {
                                                    /*****
                                                     * 反向DA匹配。先工人后任务。
                                                     */

                                                    alg_name = "ReverseDA-Algorithm";
                                                    info.begin_Algorithm(alg_name);
                                                    auto start = std::chrono::high_resolution_clock::now();
                                                    info.Grouping_Framework_ReverseDA(info.global_tasks, info.global_workers, info.Wmax, info.Tmax, info.global_Sumdis, info.global_Worker_subTrajectoryDis);
                                                    auto end = std::chrono::high_resolution_clock::now();
                                                    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
                                                    info.printf_Satisfaction_Results(alg_name, duration.count());
                                                }

                                                else if (algorithmOption == 8)
                                                {
                                                    /*****
                                                     * 交替DA匹配。先工人后任务。
                                                     */

                                                    alg_name = "AlternateDA-Algorithm";
                                                    info.begin_Algorithm(alg_name);
                                                    auto start = std::chrono::high_resolution_clock::now();
                                                    info.Grouping_Framework_AlternateDA(info.global_tasks, info.global_workers, info.Wmax, info.Tmax, info.global_Sumdis, info.global_Worker_subTrajectoryDis);
                                                    auto end = std::chrono::high_resolution_clock::now();
                                                    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
                                                    info.printf_Satisfaction_Results(alg_name, duration.count());
                                                }
                                               
                                            }
                                            else
                                            {
                                                cout << "输入错误" << endl;
                                            }
                                        }

                                        cout << alg_name + "\t 窗口中时间大小：" << info.Wmax << endl;

                                        cout << "\t任务数：" << info.global_tasks.size() << "  工人数：" << info.global_workers.size() << "  工人容量:" << info.Capacity
                                             << "  工人成本/单位" << info.c << "  移动速度:" << info.speed << "  时间窗口:" << info.Wmax << "  任务窗口:" << info.Tmax << "  半径:" << rangeX * test_rangeNum << endl;

                                        cout
                                            << "任务的平均满意度：\t" << avg_task_sati / sum_repeat << "\n"
                                            << " 工人的平均满意度：\t " << avg_work_sati / sum_repeat << "\n"
                                            << "任务匹配对数：\t" << avg_task_match_num / sum_repeat << "\n"
                                            << "工人匹配对数：\t" << avg_work_match_num / sum_repeat << "\n"
                                            << "运行时间(ms):\t" << avg_runtime / sum_repeat << endl;
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    return 0;
}
