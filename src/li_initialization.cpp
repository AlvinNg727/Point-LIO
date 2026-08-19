#include "li_initialization.h"
bool data_accum_finished = false, data_accum_start = false, online_calib_finish = false, refine_print = false;
int frame_num_init = 0;
double time_lag_IMU_wtr_lidar = 0.0, move_start_time = 0.0, online_calib_starts_time = 0.0; //, mean_acc_norm = 9.81;
double imu_first_time = 0.0;
bool lose_lid = false;
double timediff_imu_wrt_lidar = 0.0;
bool timediff_set_flg = false;
V3D gravity_lio = V3D::Zero();
mutex mtx_buffer;
sensor_msgs::Imu imu_last, imu_next;
// sensor_msgs::Imu::ConstPtr imu_last_ptr;
PointCloudXYZI::Ptr ptr_con(new PointCloudXYZI());
double T1[MAXN], s_plot[MAXN], s_plot2[MAXN], s_plot3[MAXN], s_plot11[MAXN];

condition_variable sig_buffer;
int scan_count = 0;
int frame_ct = 0, wait_num = 0;
std::mutex m_time;
// Wall-clock instant each subscriber last had a message handed to it, and how many of those
// messages it threw away on the monotonicity guard below. The stall watchdog reads all four:
// together they separate "the driver stopped" (ages climb, drops flat) from "data is arriving
// but this node is refusing it" (ages flat, drops climb) from "data is arriving and being
// accepted, so the stall is downstream of the callbacks" (ages flat, drops flat).
double last_lidar_cbk_walltime = -1.0, last_imu_cbk_walltime = -1.0;
uint64_t lidar_loopback_drops = 0, imu_loopback_drops = 0;
bool lidar_pushed = false, imu_pushed = false;
std::deque<PointCloudXYZI::Ptr> lidar_buffer;
std::deque<double> time_buffer;
std::deque<sensor_msgs::Imu::Ptr> imu_deque;

void standard_pcl_cbk(const sensor_msgs::PointCloud2::ConstPtr &msg) {
    // mtx_buffer.lock();
    scan_count++;
    last_lidar_cbk_walltime = ros::WallTime::now().toSec();
    double preprocess_start_time = omp_get_wtime();
    if (msg->header.stamp.toSec() < last_timestamp_lidar) {
        // A single message stamped in the future latches last_timestamp_lidar there and
        // every later scan is dropped here for good -- the driver keeps publishing, this
        // node never sees another point, and the pose stream stops. Throttled and counted
        // so that shows up as a climbing drop count in the watchdog instead of log spam.
        ++lidar_loopback_drops;
        ROS_ERROR_THROTTLE(2.0, "Point-LIO: lidar loop back -- dropping scan (t=%.6f <= latched %.6f, %lu dropped)",
                           msg->header.stamp.toSec(), last_timestamp_lidar,
                           static_cast<unsigned long>(lidar_loopback_drops));
        // lidar_buffer.shrink_to_fit();

        // mtx_buffer.unlock();
        // sig_buffer.notify_all();
        return;
    }

    last_timestamp_lidar = msg->header.stamp.toSec();
    // printf("check lidar time %f\n", last_timestamp_lidar);
    // if (abs(last_timestamp_imu - last_timestamp_lidar) > 1.0 && !timediff_set_flg && !imu_deque.empty()) {
    //     timediff_set_flg = true;
    //     timediff_imu_wrt_lidar = last_timestamp_imu - last_timestamp_lidar;
    //     printf("Self sync IMU and LiDAR, HARD time lag is %.10lf \n \n", timediff_imu_wrt_lidar);
    // }

    if ((lidar_type == VELO16 || lidar_type == OUST64 || lidar_type == HESAIxt32) && cut_frame_init) {
        deque<PointCloudXYZI::Ptr> ptr;
        deque<double> timestamp_lidar;
        p_pre->process_cut_frame_pcl2(msg, ptr, timestamp_lidar, cut_frame_num, scan_count);
        while (!ptr.empty() && !timestamp_lidar.empty()) {
            lidar_buffer.push_back(ptr.front());
            ptr.pop_front();
            time_buffer.push_back(timestamp_lidar.front() / double(1000)); // unit:s
            timestamp_lidar.pop_front();
        }
    } else {
        PointCloudXYZI::Ptr ptr(new PointCloudXYZI(20000, 1));
        p_pre->process(msg, ptr);
        if (con_frame) {
            if (frame_ct == 0) {
                time_con = last_timestamp_lidar; // msg->header.stamp.toSec();
            }
            if (frame_ct < 10) {
                for (int i = 0; i < ptr->size(); i++) {
                    ptr->points[i].curvature += (last_timestamp_lidar - time_con) * 1000;
                    ptr_con->push_back(ptr->points[i]);
                }
                frame_ct++;
            } else {
                PointCloudXYZI::Ptr ptr_con_i(new PointCloudXYZI(10000, 1));
                // cout << "ptr div num:" << ptr_div->size() << endl;
                *ptr_con_i = *ptr_con;
                lidar_buffer.push_back(ptr_con_i);
                double time_con_i = time_con;
                time_buffer.push_back(time_con_i);
                ptr_con->clear();
                frame_ct = 0;
            }
        } else {
            if (ptr->points.size() > 0) {
                lidar_buffer.emplace_back(ptr);
                time_buffer.emplace_back(msg->header.stamp.toSec());
            }
        }
    }
    s_plot11[scan_count] = omp_get_wtime() - preprocess_start_time;
    // mtx_buffer.unlock();
    // sig_buffer.notify_all();
}

void livox_pcl_cbk(const livox_ros_driver2::CustomMsg::ConstPtr &msg) {
    // mtx_buffer.lock();
    double preprocess_start_time = omp_get_wtime();
    scan_count++;
    last_lidar_cbk_walltime = ros::WallTime::now().toSec();
    if (msg->header.stamp.toSec() < last_timestamp_lidar) {
        // See standard_pcl_cbk: one future-stamped scan wedges this guard permanently.
        ++lidar_loopback_drops;
        ROS_ERROR_THROTTLE(2.0, "Point-LIO: lidar loop back -- dropping scan (t=%.6f <= latched %.6f, %lu dropped)",
                           msg->header.stamp.toSec(), last_timestamp_lidar,
                           static_cast<unsigned long>(lidar_loopback_drops));

        // mtx_buffer.unlock();
        // sig_buffer.notify_all();
        return;
        // lidar_buffer.shrink_to_fit();
    }

    last_timestamp_lidar = msg->header.stamp.toSec();
    // if (abs(last_timestamp_imu - last_timestamp_lidar) > 1.0 && !timediff_set_flg && !imu_deque.empty()) {
    //     timediff_set_flg = true;
    //     timediff_imu_wrt_lidar = last_timestamp_imu - last_timestamp_lidar;
    //     printf("Self sync IMU and LiDAR, HARD time lag is %.10lf \n \n", timediff_imu_wrt_lidar);
    // }

    if (cut_frame_init) {
        deque<PointCloudXYZI::Ptr> ptr;
        deque<double> timestamp_lidar;
        p_pre->process_cut_frame_livox(msg, ptr, timestamp_lidar, cut_frame_num, scan_count);

        while (!ptr.empty() && !timestamp_lidar.empty()) {
            lidar_buffer.push_back(ptr.front());
            ptr.pop_front();
            time_buffer.push_back(timestamp_lidar.front() / double(1000)); // unit:s
            timestamp_lidar.pop_front();
        }
    } else {
        PointCloudXYZI::Ptr ptr(new PointCloudXYZI(10000, 1));
        p_pre->process(msg, ptr);
        if (con_frame) {
            if (frame_ct == 0) {
                time_con = last_timestamp_lidar; // msg->header.stamp.toSec();
            }
            if (frame_ct < 10) {
                for (int i = 0; i < ptr->size(); i++) {
                    ptr->points[i].curvature += (last_timestamp_lidar - time_con) * 1000;
                    ptr_con->push_back(ptr->points[i]);
                }
                frame_ct++;
            } else {
                PointCloudXYZI::Ptr ptr_con_i(new PointCloudXYZI(10000, 1));
                // cout << "ptr div num:" << ptr_div->size() << endl;
                *ptr_con_i = *ptr_con;
                double time_con_i = time_con;
                lidar_buffer.push_back(ptr_con_i);
                time_buffer.push_back(time_con_i);
                ptr_con->clear();
                frame_ct = 0;
            }
        } else {
            if (ptr->points.size() > 0) {
                lidar_buffer.emplace_back(ptr);
                time_buffer.emplace_back(msg->header.stamp.toSec());
            }
        }
    }
    s_plot11[scan_count] = omp_get_wtime() - preprocess_start_time;
    // mtx_buffer.unlock();
    // sig_buffer.notify_all();
}

void imu_cbk(const sensor_msgs::Imu::ConstPtr &msg_in) {
    // mtx_buffer.lock();

    // publish_count ++;
    sensor_msgs::Imu::Ptr msg(new sensor_msgs::Imu(*msg_in));

    msg->header.stamp =
        ros::Time().fromSec(msg->header.stamp.toSec() - timediff_imu_wrt_lidar - time_lag_IMU_wtr_lidar);

    double timestamp = msg->header.stamp.toSec();
    last_imu_cbk_walltime = ros::WallTime::now().toSec();
    // printf("time_diff%f, %f, %f\n", last_timestamp_imu - timestamp, last_timestamp_imu, timestamp);

    if (timestamp < last_timestamp_imu) {
        // Same latching hazard as the lidar guard, and worse here: sync_packages will not
        // release a scan until last_timestamp_imu reaches its end time, so once this starts
        // dropping every sample the whole pipeline stops with nothing else logged.
        ++imu_loopback_drops;
        ROS_ERROR_THROTTLE(2.0, "Point-LIO: imu loop back -- dropping sample (t=%.6f <= latched %.6f, %lu dropped)",
                           timestamp, last_timestamp_imu, static_cast<unsigned long>(imu_loopback_drops));
        // imu_deque.shrink_to_fit();
        // cout << "check time:" << timestamp << ";" << last_timestamp_imu << endl;
        // printf("time_diff%f, %f, %f\n", last_timestamp_imu - timestamp, last_timestamp_imu, timestamp);

        // mtx_buffer.unlock();
        // sig_buffer.notify_all();
        return;
    }
    imu_deque.emplace_back(msg);
    last_timestamp_imu = timestamp;
    // mtx_buffer.unlock();
    // sig_buffer.notify_all();
}

bool sync_packages(MeasureGroup &meas) {
    {
        if (!imu_en) {
            if (!lidar_buffer.empty()) {
                if (!lidar_pushed) {
                    meas.lidar = lidar_buffer.front();
                    meas.lidar_beg_time = time_buffer.front();
                    lose_lid = false;
                    if (meas.lidar->points.size() < 1) {
                        cout << "lose lidar" << std::endl;
                        // return false;
                        lose_lid = true;
                    } else {
                        double end_time = meas.lidar->points.back().curvature;
                        for (auto pt : meas.lidar->points) {
                            if (pt.curvature > end_time) {
                                end_time = pt.curvature;
                            }
                        }
                        lidar_end_time = meas.lidar_beg_time + end_time / double(1000);
                        meas.lidar_last_time = lidar_end_time;
                    }
                    lidar_pushed = true;
                }

                time_buffer.pop_front();
                lidar_buffer.pop_front();
                lidar_pushed = false;
                if (!lose_lid) {
                    return true;
                } else {
                    return false;
                }
            }
            return false;
        }

        if (lidar_buffer.empty() || imu_deque.empty()) {
            return false;
        }
        /*** push a lidar scan ***/
        if (!lidar_pushed) {
            lose_lid = false;
            meas.lidar = lidar_buffer.front();
            meas.lidar_beg_time = time_buffer.front();
            if (meas.lidar->points.size() < 1) {
                cout << "lose lidar" << endl;
                lose_lid = true;
                // lidar_buffer.pop_front();
                // time_buffer.pop_front();
                // return false;
            } else {
                double end_time = meas.lidar->points.back().curvature;
                for (auto pt : meas.lidar->points) {
                    if (pt.curvature > end_time) {
                        end_time = pt.curvature;
                    }
                }
                lidar_end_time = meas.lidar_beg_time + end_time / double(1000);
                // cout << "check time lidar:" << end_time << endl;
                meas.lidar_last_time = lidar_end_time;
            }
            lidar_pushed = true;
        }

        if (!lose_lid && (last_timestamp_imu < lidar_end_time)) {
            return false;
        }
        if (lose_lid && last_timestamp_imu < meas.lidar_beg_time + lidar_time_inte) {
            return false;
        }

        if (!lose_lid && !imu_pushed) {
            /*** push imu data, and pop from imu buffer ***/
            if (p_imu->imu_need_init_) {
                double imu_time = imu_deque.front()->header.stamp.toSec();
                imu_next = *(imu_deque.front());
                meas.imu.shrink_to_fit();
                while (imu_time < lidar_end_time) {
                    meas.imu.emplace_back(imu_deque.front());
                    imu_last = imu_next;
                    imu_deque.pop_front();
                    if (imu_deque.empty())
                        break;
                    imu_time = imu_deque.front()->header.stamp.toSec(); // can be changed
                    imu_next = *(imu_deque.front());
                }
            }
            imu_pushed = true;
        }

        if (lose_lid && !imu_pushed) {
            /*** push imu data, and pop from imu buffer ***/
            if (p_imu->imu_need_init_) {
                double imu_time = imu_deque.front()->header.stamp.toSec();
                meas.imu.shrink_to_fit();

                imu_next = *(imu_deque.front());
                while (imu_time < meas.lidar_beg_time + lidar_time_inte) {
                    meas.imu.emplace_back(imu_deque.front());
                    imu_last = imu_next;
                    imu_deque.pop_front();
                    if (imu_deque.empty())
                        break;
                    imu_time = imu_deque.front()->header.stamp.toSec(); // can be changed
                    imu_next = *(imu_deque.front());
                }
            }
            imu_pushed = true;
        }

        lidar_buffer.pop_front();
        time_buffer.pop_front();
        lidar_pushed = false;
        imu_pushed = false;
        return true;
    }
}
