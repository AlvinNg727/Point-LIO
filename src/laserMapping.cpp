// #include <so3_math.h>
#include "li_initialization.h"
#include <malloc.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
// #include <cv_bridge/cv_bridge.h>
// #include "matplotlibcpp.h"
// #include <ros/console.h>

using namespace std;

#define PUBFRAME_PERIOD (20)

const float MOV_THRESHOLD = 1.5f;

string root_dir = ROOT_DIR;

int time_log_counter = 0; //, publish_count = 0;

bool init_map = false, flg_first_scan = true;

// Time Log Variables
double match_time = 0, solve_time = 0, propag_time = 0, update_time = 0;

bool flg_reset = false, flg_exit = false;

// surf feature in map
PointCloudXYZI::Ptr feats_undistort(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_down_body_space(new PointCloudXYZI());
PointCloudXYZI::Ptr init_feats_world(new PointCloudXYZI());
std::deque<PointCloudXYZI::Ptr> depth_feats_world;
pcl::VoxelGrid<PointType> downSizeFilterSurf;
pcl::VoxelGrid<PointType> downSizeFilterMap;

V3D euler_cur;

nav_msgs::Path path;
nav_msgs::Odometry odomAftMapped;
geometry_msgs::PoseStamped msg_body_pose;
geometry_msgs::PoseStamped lio_pose;
geometry_msgs::PoseWithCovarianceStamped lio_pose_cov;

// The estimator stamp of the most recent odometry publish, and the wall-clock instant it
// went out at. The first is how publish_odometry suppresses a repeat of an instant it has
// already sent (distinct from the rate gate's own static, which tracks the last stamp the
// gate *let through* rather than the last one actually published). The second is what the
// stall watchdog in the main loop measures against, and it has to be wall time: a stalled
// node stops producing sensor stamps altogether, so sensor time cannot detect its own
// absence.
double last_published_odom_stamp_ = -1.0;
double last_odom_pub_walltime_ = -1.0;

void SigHandle(int sig) {
    flg_exit = true;
    ROS_WARN("catch sig %d", sig);
    sig_buffer.notify_all();
}

inline void dump_lio_state_to_log(FILE *fp) {
    V3D rot_ang;
    if (!use_imu_as_input) {
        rot_ang = SO3ToEuler(kf_output.x_.rot);
    } else {
        rot_ang = SO3ToEuler(kf_input.x_.rot);
    }

    fprintf(fp, "%lf ", Measures.lidar_beg_time - first_lidar_time);
    fprintf(fp, "%lf %lf %lf ", rot_ang(0), rot_ang(1), rot_ang(2)); // Angle
    if (use_imu_as_input) {
        fprintf(fp, "%lf %lf %lf ", kf_input.x_.pos(0), kf_input.x_.pos(1), kf_input.x_.pos(2));             // Pos
        fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0);                                                          // omega
        fprintf(fp, "%lf %lf %lf ", kf_input.x_.vel(0), kf_input.x_.vel(1), kf_input.x_.vel(2));             // Vel
        fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0);                                                          // Acc
        fprintf(fp, "%lf %lf %lf ", kf_input.x_.bg(0), kf_input.x_.bg(1), kf_input.x_.bg(2));                // Bias_g
        fprintf(fp, "%lf %lf %lf ", kf_input.x_.ba(0), kf_input.x_.ba(1), kf_input.x_.ba(2));                // Bias_a
        fprintf(fp, "%lf %lf %lf ", kf_input.x_.gravity(0), kf_input.x_.gravity(1), kf_input.x_.gravity(2)); // Bias_a
    } else {
        fprintf(fp, "%lf %lf %lf ", kf_output.x_.pos(0), kf_output.x_.pos(1), kf_output.x_.pos(2)); // Pos
        fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0);                                                 // omega
        fprintf(fp, "%lf %lf %lf ", kf_output.x_.vel(0), kf_output.x_.vel(1), kf_output.x_.vel(2)); // Vel
        fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0);                                                 // Acc
        fprintf(fp, "%lf %lf %lf ", kf_output.x_.bg(0), kf_output.x_.bg(1), kf_output.x_.bg(2));    // Bias_g
        fprintf(fp, "%lf %lf %lf ", kf_output.x_.ba(0), kf_output.x_.ba(1), kf_output.x_.ba(2));    // Bias_a
        fprintf(fp, "%lf %lf %lf ", kf_output.x_.gravity(0), kf_output.x_.gravity(1),
                kf_output.x_.gravity(2)); // Bias_a
    }
    fprintf(fp, "\r\n");
    fflush(fp);
}

void pointBodyLidarToIMU(PointType const *const pi, PointType *const po) {
    V3D p_body_lidar(pi->x, pi->y, pi->z);
    V3D p_body_imu;
    if (extrinsic_est_en) {
        if (!use_imu_as_input) {
            p_body_imu = kf_output.x_.offset_R_L_I * p_body_lidar + kf_output.x_.offset_T_L_I;
        } else {
            p_body_imu = kf_input.x_.offset_R_L_I * p_body_lidar + kf_input.x_.offset_T_L_I;
        }
    } else {
        p_body_imu = Lidar_R_wrt_IMU * p_body_lidar + Lidar_T_wrt_IMU;
    }
    po->x = p_body_imu(0);
    po->y = p_body_imu(1);
    po->z = p_body_imu(2);
    po->intensity = pi->intensity;
}

void MapIncremental() {
    PointVector points_to_add;
    int cur_pts = feats_down_world->size();
    points_to_add.reserve(cur_pts);

    for (size_t i = 0; i < cur_pts; ++i) {
        /* decide if need add to map */
        PointType &point_world = feats_down_world->points[i];
        if (!Nearest_Points[i].empty()) {
            const PointVector &points_near = Nearest_Points[i];

            Eigen::Vector3f center =
                ((point_world.getVector3fMap() / filter_size_map_min).array().floor() + 0.5) * filter_size_map_min;
            bool need_add = true;
            for (int readd_i = 0; readd_i < points_near.size(); readd_i++) {
                Eigen::Vector3f dis_2_center = points_near[readd_i].getVector3fMap() - center;
                if (fabs(dis_2_center.x()) < 0.5 * filter_size_map_min &&
                    fabs(dis_2_center.y()) < 0.5 * filter_size_map_min &&
                    fabs(dis_2_center.z()) < 0.5 * filter_size_map_min) {
                    need_add = false;
                    break;
                }
            }
            if (need_add) {
                points_to_add.emplace_back(point_world);
            }
        } else {
            points_to_add.emplace_back(point_world);
        }
    }
    ivox_->AddPoints(points_to_add);
}

void publish_init_map(const ros::Publisher &pubLaserCloudFullRes) {
    int size_init_map = init_feats_world->size();

    sensor_msgs::PointCloud2 laserCloudmsg;

    pcl::toROSMsg(*init_feats_world, laserCloudmsg);

    laserCloudmsg.header.stamp = ros::Time().fromSec(lidar_end_time);
    laserCloudmsg.header.frame_id = map_frame;
    pubLaserCloudFullRes.publish(laserCloudmsg);
}

PointCloudXYZI::Ptr pcl_wait_pub(new PointCloudXYZI(500000, 1));
PointCloudXYZI::Ptr pcl_wait_save(new PointCloudXYZI());
void publish_frame_world(const ros::Publisher &pubLaserCloudFullRes) {
    if (scan_pub_en) {
        PointCloudXYZI::Ptr laserCloudFullRes(feats_down_body);
        int size = laserCloudFullRes->points.size();

        PointCloudXYZI::Ptr laserCloudWorld(new PointCloudXYZI(size, 1));

        for (int i = 0; i < size; i++) {
            laserCloudWorld->points[i].x = feats_down_world->points[i].x;
            laserCloudWorld->points[i].y = feats_down_world->points[i].y;
            laserCloudWorld->points[i].z = feats_down_world->points[i].z;
            laserCloudWorld->points[i].intensity =
                feats_down_world->points[i].intensity; // feats_down_world->points[i].y; //
        }
        sensor_msgs::PointCloud2 laserCloudmsg;
        pcl::toROSMsg(*laserCloudWorld, laserCloudmsg);

        laserCloudmsg.header.stamp = ros::Time().fromSec(lidar_end_time);
        laserCloudmsg.header.frame_id = map_frame;
        pubLaserCloudFullRes.publish(laserCloudmsg);
        // publish_count -= PUBFRAME_PERIOD;
    }

    /**************** save map ****************/
    /* 1. make sure you have enough memories
    /* 2. noted that pcd save will influence the real-time performences **/
    if (pcd_save_en) {
        int size = feats_down_world->points.size();
        PointCloudXYZI::Ptr laserCloudWorld(new PointCloudXYZI(size, 1));

        for (int i = 0; i < size; i++) {
            laserCloudWorld->points[i].x = feats_down_world->points[i].x;
            laserCloudWorld->points[i].y = feats_down_world->points[i].y;
            laserCloudWorld->points[i].z = feats_down_world->points[i].z;
            laserCloudWorld->points[i].intensity = feats_down_world->points[i].intensity;
        }

        *pcl_wait_save += *laserCloudWorld;

        static int scan_wait_num = 0;
        scan_wait_num++;
        if (pcl_wait_save->size() > 0 && scan_wait_num >= pcd_save_interval) {
            pcd_index++;
            string all_points_dir(string(string(ROOT_DIR) + "PCD/scans_") + to_string(pcd_index) + string(".pcd"));
            pcl::PCDWriter pcd_writer;
            cout << "current scan saved to /PCD/" << all_points_dir << endl;
            pcd_writer.writeBinary(all_points_dir, *pcl_wait_save);
            pcl_wait_save->clear();
            scan_wait_num = 0;
        }
    }
}

void publish_frame_body(const ros::Publisher &pubLaserCloudFull_body) {
    int size = feats_undistort->points.size();
    PointCloudXYZI::Ptr laserCloudIMUBody(new PointCloudXYZI(size, 1));

    for (int i = 0; i < size; i++) {
        pointBodyLidarToIMU(&feats_undistort->points[i], &laserCloudIMUBody->points[i]);
    }

    sensor_msgs::PointCloud2 laserCloudmsg;
    pcl::toROSMsg(*laserCloudIMUBody, laserCloudmsg);
    laserCloudmsg.header.stamp = ros::Time().fromSec(lidar_end_time);
    laserCloudmsg.header.frame_id = imu_frame;
    pubLaserCloudFull_body.publish(laserCloudmsg);
    // publish_count -= PUBFRAME_PERIOD;
}

// Orientation of the estimated IMU/body frame in the filter's world frame (map_frame).
inline Eigen::Quaterniond body_quat() {
    return (!use_imu_as_input) ? Eigen::Quaterniond(kf_output.x_.rot) : Eigen::Quaterniond(kf_input.x_.rot);
}

// Position of the estimated IMU/body frame in the filter's world frame (map_frame).
inline V3D body_pos() { return (!use_imu_as_input) ? V3D(kf_output.x_.pos) : V3D(kf_input.x_.pos); }

// Pose of the odom frame in the map frame -- i.e. where base_link was when the filter
// initialised. Latched on the first publish and re-latched after a bag-loop reset.
//
// The filter's world frame (map_frame) is gravity-levelled, but nothing constrains its yaw:
// Set_init only rotates measured gravity onto [0,0,-g], so map_frame inherits whatever the
// IMU happened to be pointing at. With this mount that yaw is not even repeatable -- the
// IMU is inverted (extrinsic_R_base maps base +z to body -z), which makes the two gravity
// vectors antiparallel, and Set_init's rotation axis (g_world x g_body) is then pinned by
// nothing but the residual tilt and noise at startup. Applying extrinsic_R_base to the
// child side alone leaves the trajectory in that frame, so base_link's axes come out
// rotated by an arbitrary angle: lateral motion reads out along x, a diagonal, or anything
// else, differently on each run.
//
// Re-expressing the pose relative to the initial base_link pose cancels both that arbitrary
// yaw and the extrinsic's own, and gives the REP-105 odom frame the ZED also uses: origin
// at the start point, axes along base_link, so the two odometries are directly comparable.
bool odom_origin_latched = false;
Eigen::Quaterniond q_map_odom = Eigen::Quaterniond::Identity(); // R_map_odom
V3D p_map_odom = V3D::Zero();                                   // t_map_odom

// Both frames sharing a name is the legacy single-frame setup: leave the origin at identity
// so the published pose stays in the filter's world frame exactly as before.
inline bool rebase_to_odom_frame() { return odom_frame != map_frame; }

void publish_odom_to_map_tf();

void latch_odom_origin() {
    if (odom_origin_latched || !rebase_to_odom_frame()) {
        return;
    }
    const Eigen::Quaterniond q_body = body_quat();
    q_map_odom = q_body * Eigen::Quaterniond(Base_R_wrt_IMU);
    q_map_odom.normalize();
    p_map_odom = body_pos() + q_body * Base_T_wrt_IMU;
    odom_origin_latched = true;
    publish_odom_to_map_tf();
}

// Pose of base_link in the odom frame. The filter estimates the IMU/body frame in map_frame,
// so the fixed body->base_link extrinsic is applied first:
//   q_map_base = q_body * R_body_base
//   p_map_base = p_body + q_body * t_body_base
// and the result is then re-expressed in the odom frame latched above. With the defaults
// (identity/zero extrinsic, odom_frame == map_frame) this reduces to the original body pose.
template <typename T> void set_posestamp(T &out) {
    const Eigen::Quaterniond q_body = body_quat();

    const V3D p_map_base = body_pos() + q_body * Base_T_wrt_IMU;
    const Eigen::Quaterniond q_map_base = q_body * Eigen::Quaterniond(Base_R_wrt_IMU);

    const Eigen::Quaterniond q_odom_map = q_map_odom.conjugate();
    const V3D p_base = q_odom_map * (p_map_base - p_map_odom);
    Eigen::Quaterniond q_base = q_odom_map * q_map_base;
    q_base.normalize();

    out.position.x = p_base(0);
    out.position.y = p_base(1);
    out.position.z = p_base(2);
    out.orientation.x = q_base.x();
    out.orientation.y = q_base.y();
    out.orientation.z = q_base.z();
    out.orientation.w = q_base.w();
}

// The 6x6 covariance of the pose set_posestamp() just wrote, in ROS order
// [x, y, z, rot_x, rot_y, rot_z], row-major.
//
// The filter's error state is p_map_body = p + dp (additive, map frame) and
// R_map_body = R * Exp(dtheta) -- a right/body perturbation, since MTK's SO3 boxplus is
// *this = *this * exp(vec). Pushing both through the same transform set_posestamp applies:
//
//   dp_base    = R_om * dp - R_ob * [t_bb]x * dtheta      (t_bb = Base_T_wrt_IMU)
//   dtheta_odom = R_ob * dtheta                            (R_ob = R_om * R_map_body)
//
// so J is block upper-triangular and C = J P J'. The rotation block comes out about
// odom_frame's axes, not base_link's: converting the body-frame perturbation to a
// fixed-frame one cancels Base_R_wrt_IMU exactly, which is why the base extrinsic shows up
// only in the lever-arm term above. That is the convention robot_localization assumes when
// it rotates a pose covariance into its own target frame, and it is why the rviz display
// for this topic wants Covariance/Orientation/Frame: Fixed rather than Local.
//
// Caveat worth knowing: P is the filter's *absolute* uncertainty in map_frame, while the
// pose is relative to the odom origin latched from that same state. Strictly the origin's
// own uncertainty and its correlation with the current state should be subtracted; the
// filter does not keep that cross-covariance, so it cannot be. The result over-estimates
// near t=0 -- the pose is exactly the origin, yet C starts at reset_cov_output's 0.01*I --
// which is the safe direction for a downstream EKF, and washes out as the run proceeds.
//
// Templated on the array type for the same reason set_posestamp is templated on the pose
// type: it avoids naming boost::array<double, 36> here.
template <typename T> void set_pose_covariance(T &cov) {
    // kf_input.P_ is 24x24 and kf_output.P_ is 30x30 (esekfom's cov is Matrix<n, n> with
    // n = state::DOF), so the two blocks are unrelated types -- a ternary like the one in
    // body_quat() does not compile here. Assign into a fixed 6x6 instead. pos occupies
    // error-state indices 0-2 and rot 3-5 in both manifolds, which is already ROS order.
    Eigen::Matrix<double, 6, 6> P6;
    if (!use_imu_as_input) {
        P6 = kf_output.P_.block<6, 6>(0, 0);
    } else {
        P6 = kf_input.P_.block<6, 6>(0, 0);
    }

    const M3D R_om = q_map_odom.conjugate().toRotationMatrix();
    const M3D R_ob = R_om * body_quat().toRotationMatrix();

    Eigen::Matrix<double, 6, 6> J = Eigen::Matrix<double, 6, 6>::Zero();
    J.block<3, 3>(0, 0) = R_om;
    J.block<3, 3>(0, 3) = -R_ob * skew_sym_mat(Base_T_wrt_IMU);
    J.block<3, 3>(3, 3) = R_ob;

    Eigen::Matrix<double, 6, 6> C = J * P6 * J.transpose();
    // The filter's update is not in Joseph form, so P_ drifts slightly asymmetric.
    // Symmetrise before publishing; consumers assume it. eval() breaks the aliasing.
    C = 0.5 * (C + C.transpose().eval());

    C *= pose_cov_scale;
    for (int i = 0; i < 6; ++i) {
        C(i, i) = std::max(C(i, i), pose_cov_min[i]);
    }
    for (int i = 0; i < 36; ++i) {
        cov[i] = C(i / 6, i % 6);
    }
}

void publish_odometry(const ros::Publisher &pubOdomAftMapped, const ros::Publisher &pubLioPose,
                      const ros::Publisher &pubLioPoseCov) {
    latch_odom_origin();
    odomAftMapped.header.frame_id = odom_frame;
    odomAftMapped.child_frame_id = base_frame;
    const double stamp = publish_odometry_without_downsample ? time_current : lidar_end_time;

    // time_current is still 0 until the estimator has walked its first point, and the
    // end-of-scan safety-net call can reach here before that has happened on a first scan
    // that carries no usable points. Publishing that would put an epoch-0 stamp on the
    // odometry and the TF, and ros::Time::fromSec throws outright on a negative one.
    if (stamp <= 0.0) {
        return;
    }

    // Re-publishing an estimator instant already sent says nothing new. This only comes up
    // because of the end-of-scan safety-net call in the main loop, which re-enters with an
    // unchanged time_current whenever the point-by-point path already published; dropping
    // the duplicate here is what makes that safety net a no-op in the healthy case.
    if (stamp == last_published_odom_stamp_) {
        return;
    }

    // readParameters guarantees odom_pub_freq > 0 whenever the gate is enabled.
    if (odom_pub_freq_en) {
        static double last_odom_pub_stamp = -1.0;
        const double min_interval = 1.0 / odom_pub_freq;
        // stamp < last => time ran backwards (bag loop / flg_reset): publish and re-latch
        if (last_odom_pub_stamp >= 0.0 && stamp >= last_odom_pub_stamp && stamp - last_odom_pub_stamp < min_interval) {
            return;
        }
        last_odom_pub_stamp = stamp;
    }

    last_published_odom_stamp_ = stamp;
    last_odom_pub_walltime_ = ros::WallTime::now().toSec();

    odomAftMapped.header.stamp = ros::Time().fromSec(stamp);
    set_posestamp(odomAftMapped.pose.pose);

    // nav_msgs/Odometry twist is expressed in child_frame_id, i.e. base_link.
    // In the state, vel is world-frame and omg is body-frame. state_input carries no omg,
    // so that path falls back to the latest bias-corrected gyro sample.
    const Eigen::Quaterniond q_body = body_quat();
    const V3D vel_world = (!use_imu_as_input) ? V3D(kf_output.x_.vel) : V3D(kf_input.x_.vel);
    const V3D omg_body = (!use_imu_as_input) ? V3D(kf_output.x_.omg) : V3D(angvel_avr - kf_input.x_.bg);

    const V3D vel_body = q_body.conjugate() * vel_world;
    const V3D w_base = Base_R_wrt_IMU.transpose() * omg_body;
    // Lever-arm term: a base_link offset from the IMU picks up velocity under rotation.
    const V3D v_base = Base_R_wrt_IMU.transpose() * (vel_body + omg_body.cross(Base_T_wrt_IMU));

    odomAftMapped.twist.twist.linear.x = v_base(0);
    odomAftMapped.twist.twist.linear.y = v_base(1);
    odomAftMapped.twist.twist.linear.z = v_base(2);
    odomAftMapped.twist.twist.angular.x = w_base(0);
    odomAftMapped.twist.twist.angular.y = w_base(1);
    odomAftMapped.twist.twist.angular.z = w_base(2);

    pubOdomAftMapped.publish(odomAftMapped);

    // The same pose again as a bare PoseStamped. Built from odomAftMapped rather than by
    // calling set_posestamp a second time, so the two topics cannot drift apart: identical
    // stamp, identical frame, identical numbers, and it inherits the odom_pub_freq_en gate
    // above. This is the shape the ZED publishes its estimate in
    // (/zed2i/zed_node/pose), so the two are directly comparable for anything that wants a
    // pose without the twist and covariance riding along.
    lio_pose.header = odomAftMapped.header;
    lio_pose.pose = odomAftMapped.pose.pose;
    pubLioPose.publish(lio_pose);

    // And once more with the filter's uncertainty attached -- the shape
    // /zed2i/zed_node/pose_with_covariance comes in, and what robot_localization or a
    // vision-pose bridge needs in order to weight this estimate against another. Same
    // source pose, so it inherits the stamp, the frame and the gate above unchanged;
    // /aft_mapped_to_init deliberately keeps its all-zero covariance so nothing already
    // recorded changes shape.
    lio_pose_cov.header = odomAftMapped.header;
    lio_pose_cov.pose.pose = odomAftMapped.pose.pose;
    set_pose_covariance(lio_pose_cov.pose.covariance);
    pubLioPoseCov.publish(lio_pose_cov);

    static tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::Quaternion q;
    transform.setOrigin(tf::Vector3(odomAftMapped.pose.pose.position.x, odomAftMapped.pose.pose.position.y,
                                    odomAftMapped.pose.pose.position.z));
    q.setW(odomAftMapped.pose.pose.orientation.w);
    q.setX(odomAftMapped.pose.pose.orientation.x);
    q.setY(odomAftMapped.pose.pose.orientation.y);
    q.setZ(odomAftMapped.pose.pose.orientation.z);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, odomAftMapped.header.stamp, odom_frame, base_frame));
}

// One broadcaster for every static transform this node emits. tf2 replaces by
// child_frame_id, so re-sending odom->map after a reset overwrites the stale one, and a
// late subscriber still gets base_link->body from the same latched message.
tf2_ros::StaticTransformBroadcaster &static_tf_broadcaster() {
    // Function-local so construction happens after ros::init, not at static-init time.
    static tf2_ros::StaticTransformBroadcaster br;
    return br;
}

// Declares base_link -> IMU/body on /tf_static, so the flip is described once and both the
// odometry math and the TF tree read it from the same yaml keys. Called once at startup.
void publish_static_tf() {
    // Defaults leave base_frame == imu_frame; emitting that would be a self-loop.
    if (base_frame == imu_frame) {
        return;
    }
    tf2_ros::StaticTransformBroadcaster &static_br = static_tf_broadcaster();

    // The config stores body->base_link; TF needs base_link->body, so invert it.
    const M3D R_base_imu = Base_R_wrt_IMU.transpose();
    const V3D t_base_imu = -R_base_imu * Base_T_wrt_IMU;
    Eigen::Quaterniond q(R_base_imu);
    q.normalize();

    geometry_msgs::TransformStamped tf_msg;
    tf_msg.header.stamp = ros::Time::now();
    tf_msg.header.frame_id = base_frame;
    tf_msg.child_frame_id = imu_frame;
    tf_msg.transform.translation.x = t_base_imu(0);
    tf_msg.transform.translation.y = t_base_imu(1);
    tf_msg.transform.translation.z = t_base_imu(2);
    tf_msg.transform.rotation.x = q.x();
    tf_msg.transform.rotation.y = q.y();
    tf_msg.transform.rotation.z = q.z();
    tf_msg.transform.rotation.w = q.w();
    static_br.sendTransform(tf_msg);
}

// Hangs the filter's world frame off the odom frame, so /cloud_registered and the odometry
// stay in one tree. Cannot go out at startup like the transform above: the origin is only
// known once the filter has a pose, so latch_odom_origin() calls this instead.
void publish_odom_to_map_tf() {
    tf2_ros::StaticTransformBroadcaster &static_br = static_tf_broadcaster();

    // q_map_odom / p_map_odom describe odom in map; TF wants odom -> map, so invert.
    const Eigen::Quaterniond q_odom_map = q_map_odom.conjugate();
    const V3D t_odom_map = q_odom_map * -p_map_odom;

    geometry_msgs::TransformStamped tf_msg;
    tf_msg.header.stamp = ros::Time::now();
    tf_msg.header.frame_id = odom_frame;
    tf_msg.child_frame_id = map_frame;
    tf_msg.transform.translation.x = t_odom_map(0);
    tf_msg.transform.translation.y = t_odom_map(1);
    tf_msg.transform.translation.z = t_odom_map(2);
    tf_msg.transform.rotation.x = q_odom_map.x();
    tf_msg.transform.rotation.y = q_odom_map.y();
    tf_msg.transform.rotation.z = q_odom_map.z();
    tf_msg.transform.rotation.w = q_odom_map.w();
    static_br.sendTransform(tf_msg);
}

void publish_path(const ros::Publisher pubPath) {
    latch_odom_origin();
    set_posestamp(msg_body_pose.pose);
    // msg_body_pose.header.stamp = ros::Time::now();
    msg_body_pose.header.stamp = ros::Time().fromSec(lidar_end_time);
    msg_body_pose.header.frame_id = odom_frame;
    static int jjj = 0;
    jjj++;
    // if (jjj % 2 == 0) // if path is too large, the rvis will crash
    {
        path.poses.emplace_back(msg_body_pose);
        // nav_msgs/Path is cumulative and the whole thing is re-serialised and re-sent on
        // every scan, so an uncapped path makes this publish cost grow linearly with uptime:
        // by the end of an hour at 10 Hz it is ~36k poses going out ten times a second. That
        // is a steady squeeze on a node that has to keep up with the sensors in real time,
        // and it is one more thing that only bites "after running a while". Drop the oldest.
        if (path_max_poses > 0 && static_cast<int>(path.poses.size()) > path_max_poses) {
            const size_t excess = path.poses.size() - static_cast<size_t>(path_max_poses);
            path.poses.erase(path.poses.begin(), path.poses.begin() + excess);
        }
        pubPath.publish(path);
    }
}

// Watchdog for this node's characteristic failure: it stops publishing but stays alive, so
// nothing crashes, roslaunch never notices, and nothing is logged. Driven from the main loop
// on wall time and deliberately outside the sync_packages() gate -- the most opaque stall is
// sync_packages itself returning false for ever, and in that state nothing inside the gate
// runs at all.
//
// The dump is meant to name the stuck stage rather than just report silence:
//   lidar/imu age   seconds since the subscriber last received a message. Climbing means the
//                   sensor or driver stopped. Flat near zero means data is still arriving.
//   dropped         messages the monotonicity guards in the callbacks threw away. Climbing
//                   with a flat age is the latched-future-timestamp wedge: the driver is
//                   publishing, this node is refusing everything.
//   buffered        scans and IMU samples waiting for sync_packages. Both growing without
//                   bound while data arrives and nothing is dropped means sync_packages is
//                   blocked -- read the next line for why.
//   need imu to     sync_packages holds a scan until the IMU stream reaches its end time.
//                   A large positive value here is that wait; a huge one means lidar_end_time
//                   was poisoned by an outlier point offset and will never be reached.
//   state finite    false means the filter diverged to NaN. That is terminal: every later
//                   LiDAR update finds no match, so the state can never be corrected back.
void check_odom_stall() {
    if (odom_stall_warn_sec <= 0.0 || last_odom_pub_walltime_ < 0.0) {
        return; // disabled, or nothing published yet -- still initialising
    }
    const double now = ros::WallTime::now().toSec();
    const double silent_for = now - last_odom_pub_walltime_;
    if (silent_for < odom_stall_warn_sec) {
        return;
    }

    static double last_report = -1.0;
    if (last_report > 0.0 && now - last_report < odom_stall_warn_sec) {
        return;
    }
    last_report = now;

    const V3D p = body_pos();
    const Eigen::Quaterniond q = body_quat();
    const bool finite = p.allFinite() && std::isfinite(q.w()) && std::isfinite(q.x()) && std::isfinite(q.y()) &&
                        std::isfinite(q.z());

    ROS_ERROR("Point-LIO STALL: no odometry for %.2f s | lidar age %.2f s (%lu dropped), "
              "imu age %.2f s (%lu dropped) | buffered %zu scans / %zu imu | "
              "lidar_pushed=%d lose_lid=%d need imu to reach %+.3f s past last | "
              "time_current=%.3f state_finite=%d",
              silent_for, last_lidar_cbk_walltime < 0.0 ? -1.0 : now - last_lidar_cbk_walltime,
              static_cast<unsigned long>(lidar_loopback_drops),
              last_imu_cbk_walltime < 0.0 ? -1.0 : now - last_imu_cbk_walltime,
              static_cast<unsigned long>(imu_loopback_drops), lidar_buffer.size(), imu_deque.size(),
              static_cast<int>(lidar_pushed), static_cast<int>(lose_lid), lidar_end_time - last_timestamp_imu,
              time_current, static_cast<int>(finite));
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "laserMapping");
    ros::NodeHandle nh("~");
    // NO background spinner here, deliberately. Upstream ran `ros::AsyncSpinner spinner(0)`
    // (0 = one thread per core) *and* ros::spinOnce() in the loop below, so the subscriber
    // callbacks executed on the spinner threads while this thread was inside the estimator.
    // Both sides touch lidar_buffer / time_buffer / imu_deque with no synchronisation --
    // mtx_buffer exists but every lock() call site in li_initialization.cpp is commented
    // out -- so imu_cbk's emplace_back raced pop_front in the point-by-point loop. Racing
    // push/pop on a std::deque is undefined: the usual outcome is a torn _M_start/_M_finish,
    // after which the buffer reads back permanently empty, sync_packages returns false for
    // ever, and the node keeps running happily while the pose stream stops dead with no
    // error printed. That is the "output stops after a while" failure.
    //
    // Spinning only from this thread makes the whole node single-threaded, which removes the
    // race by construction rather than by locking ~20 imu_deque call sites in the estimator.
    // Nothing is dropped meanwhile: both subscribers below queue 200000 messages, far more
    // than the ~20 IMU samples that accumulate during one scan's processing.
    readParameters(nh);
    cout << "lidar_type: " << lidar_type << endl;
    ivox_ = std::make_shared<IVoxType>(ivox_options_);

    path.header.stamp = ros::Time().fromSec(lidar_end_time);
    path.header.frame_id = odom_frame;

    /*** variables definition for counting ***/
    int frame_num = 0;
    double aver_time_consu = 0, aver_time_icp = 0, aver_time_match = 0, aver_time_incre = 0, aver_time_solve = 0,
           aver_time_propag = 0;

    memset(point_selected_surf, true, sizeof(point_selected_surf));
    downSizeFilterSurf.setLeafSize(filter_size_surf_min, filter_size_surf_min, filter_size_surf_min);
    downSizeFilterMap.setLeafSize(filter_size_map_min, filter_size_map_min, filter_size_map_min);

    Lidar_T_wrt_IMU << VEC_FROM_ARRAY(extrinT);
    Lidar_R_wrt_IMU << MAT_FROM_ARRAY(extrinR);
    Base_T_wrt_IMU << VEC_FROM_ARRAY(extrinT_base);
    Base_R_wrt_IMU << MAT_FROM_ARRAY(extrinR_base);
    publish_static_tf();

    if (extrinsic_est_en) {
        if (!use_imu_as_input) {
            kf_output.x_.offset_R_L_I = Lidar_R_wrt_IMU;
            kf_output.x_.offset_T_L_I = Lidar_T_wrt_IMU;
        } else {
            kf_input.x_.offset_R_L_I = Lidar_R_wrt_IMU;
            kf_input.x_.offset_T_L_I = Lidar_T_wrt_IMU;
        }
    }

    p_imu->lidar_type = p_pre->lidar_type = lidar_type;
    p_imu->imu_en = imu_en;

    kf_input.init_dyn_share_modified_2h(get_f_input, df_dx_input, h_model_input);
    kf_output.init_dyn_share_modified_3h(get_f_output, df_dx_output, h_model_output, h_model_IMU_output);
    Eigen::Matrix<double, 24, 24> P_init; // = MD(18, 18)::Identity() * 0.1;
    reset_cov(P_init);
    kf_input.change_P(P_init);
    Eigen::Matrix<double, 30, 30> P_init_output; // = MD(24, 24)::Identity() * 0.01;
    reset_cov_output(P_init_output);
    kf_output.change_P(P_init_output);
    Eigen::Matrix<double, 24, 24> Q_input = process_noise_cov_input();
    Eigen::Matrix<double, 30, 30> Q_output = process_noise_cov_output();
    /*** debug record ***/
    FILE *fp;
    string pos_log_dir = root_dir + "/Log/pos_log.txt";
    fp = fopen(pos_log_dir.c_str(), "w");
    open_file();

    /*** ROS subscribe initialization ***/
    ros::Subscriber sub_pcl = p_pre->lidar_type == AVIA ? nh.subscribe(lid_topic, 200000, livox_pcl_cbk)
                                                        : nh.subscribe(lid_topic, 200000, standard_pcl_cbk);
    ros::Subscriber sub_imu = nh.subscribe(imu_topic, 200000, imu_cbk);

    ros::Publisher pubLaserCloudFullRes = nh.advertise<sensor_msgs::PointCloud2>("/cloud_registered", 1000);
    ros::Publisher pubLaserCloudFullRes_body = nh.advertise<sensor_msgs::PointCloud2>("/cloud_registered_body", 1000);
    // ros::Publisher pubLaserCloudEffect  = nh.advertise<sensor_msgs::PointCloud2>
    // ("/cloud_effected", 1000);
    ros::Publisher pubLaserCloudMap = nh.advertise<sensor_msgs::PointCloud2>("/Laser_map", 1000);
    ros::Publisher pubOdomAftMapped = nh.advertise<nav_msgs::Odometry>("/aft_mapped_to_init", 1000);
    // base_frame's current pose in odom_frame, published alongside /aft_mapped_to_init.
    ros::Publisher pubLioPose = nh.advertise<geometry_msgs::PoseStamped>("/lio_pose", 1000);
    // The same pose with the filter's uncertainty attached, for consumers that weight it.
    ros::Publisher pubLioPoseCov =
        nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/lio_pose_with_covariance", 1000);
    ros::Publisher pubPath = nh.advertise<nav_msgs::Path>("/path", 1000);
    // ros::Publisher plane_pub = nh.advertise<visualization_msgs::Marker>
    // ("/planner_normal", 1000);
    //------------------------------------------------------------------------------------------------------
    signal(SIGINT, SigHandle);
    ros::Rate loop_rate(500);
    bool status = ros::ok();
    while (status) {
        if (flg_exit)
            break;
        ros::spinOnce();
        if (sync_packages(Measures)) {
            if (flg_reset) {
                ROS_WARN("reset when rosbag play back");
                p_imu->Reset();
                feats_undistort.reset(new PointCloudXYZI());
                if (use_imu_as_input) {
                    // state_in = kf_input.get_x();
                    state_in = state_input();
                    kf_input.change_P(P_init);
                } else {
                    // state_out = kf_output.get_x();
                    state_out = state_output();
                    kf_output.change_P(P_init_output);
                }
                flg_first_scan = true;
                is_first_frame = true;
                flg_reset = false;
                init_map = false;
                // The state went back to the origin and Set_init will pick a new yaw, so the
                // latched odom origin no longer describes anything. Re-latch on next publish.
                odom_origin_latched = false;

                {
                    ivox_.reset(new IVoxType(ivox_options_));
                }
            }

            if (flg_first_scan) {
                first_lidar_time = Measures.lidar_beg_time;
                flg_first_scan = false;
                if (first_imu_time < 1) {
                    first_imu_time = imu_next.header.stamp.toSec();
                    printf("first imu time: %f\n", first_imu_time);
                }
                time_current = 0.0;
                if (imu_en) {
                    // imu_next = *(imu_deque.front());
                    kf_input.x_.gravity << VEC_FROM_ARRAY(gravity);
                    kf_output.x_.gravity << VEC_FROM_ARRAY(gravity);
                    // kf_output.x_.acc << VEC_FROM_ARRAY(gravity);
                    // kf_output.x_.acc *= -1;

                    {
                        while (Measures.lidar_beg_time >
                               imu_next.header.stamp.toSec()) // if it is needed for the new map?
                        {
                            imu_deque.pop_front();
                            if (imu_deque.empty()) {
                                break;
                            }
                            imu_last = imu_next;
                            imu_next = *(imu_deque.front());
                            // imu_deque.pop();
                        }
                    }
                } else {
                    kf_input.x_.gravity << VEC_FROM_ARRAY(gravity);  // _init);
                    kf_output.x_.gravity << VEC_FROM_ARRAY(gravity); //_init);
                    kf_output.x_.acc << VEC_FROM_ARRAY(gravity);     //_init);
                    kf_output.x_.acc *= -1;
                    p_imu->imu_need_init_ = false;
                    // p_imu->after_imu_init_ = true;
                }
                G_m_s2 = std::sqrt(gravity[0] * gravity[0] + gravity[1] * gravity[1] + gravity[2] * gravity[2]);
            }

            double t0, t1, t2, t3, t4, t5, match_start, solve_start;
            match_time = 0;
            solve_time = 0;
            propag_time = 0;
            update_time = 0;
            t0 = omp_get_wtime();

            /*** downsample the feature points in a scan ***/
            t1 = omp_get_wtime();
            p_imu->Process(Measures, feats_undistort);
            if (space_down_sample) {
                downSizeFilterSurf.setInputCloud(feats_undistort);
                downSizeFilterSurf.filter(*feats_down_body);
                sort(feats_down_body->points.begin(), feats_down_body->points.end(), time_list);
            } else {
                feats_down_body = Measures.lidar;
                sort(feats_down_body->points.begin(), feats_down_body->points.end(), time_list);
            }
            {
                time_seq = time_compressing<int>(feats_down_body);
                feats_down_size = feats_down_body->points.size();
            }

            if (!p_imu->after_imu_init_) // !p_imu->UseLIInit &&
            {
                if (!p_imu->imu_need_init_) {
                    V3D tmp_gravity;
                    if (imu_en) {
                        tmp_gravity = -p_imu->mean_acc / p_imu->mean_acc.norm() * G_m_s2;
                    } else {
                        tmp_gravity << VEC_FROM_ARRAY(gravity_init);
                        p_imu->after_imu_init_ = true;
                    }
                    // V3D tmp_gravity << VEC_FROM_ARRAY(gravity_init);
                    M3D rot_init;
                    p_imu->Set_init(tmp_gravity, rot_init);
                    kf_input.x_.rot = rot_init;
                    kf_output.x_.rot = rot_init;
                    // kf_input.x_.rot; //.normalize();
                    // kf_output.x_.rot; //.normalize();
                    kf_output.x_.acc = -rot_init.transpose() * kf_output.x_.gravity;
                } else {
                    continue;
                }
            }
            /*** initialize the map ***/
            if (!init_map) {
                feats_down_world->resize(feats_undistort->size());
                for (int i = 0; i < feats_undistort->size(); i++) {
                    {
                        pointBodyToWorld(&(feats_undistort->points[i]), &(feats_down_world->points[i]));
                    }
                }
                for (size_t i = 0; i < feats_down_world->size(); i++) {
                    init_feats_world->points.emplace_back(feats_down_world->points[i]);
                }
                if (init_feats_world->size() < init_map_size) {
                    init_map = false;
                } else {
                    ivox_->AddPoints(init_feats_world->points);
                    publish_init_map(pubLaserCloudMap); //(pubLaserCloudFullRes);

                    init_feats_world.reset(new PointCloudXYZI());
                    init_map = true;
                }
                continue;
            }

            /*** ICP and Kalman filter update ***/
            normvec->resize(feats_down_size);
            feats_down_world->resize(feats_down_size);

            Nearest_Points.resize(feats_down_size);

            t2 = omp_get_wtime();

            /*** iterated state estimation ***/
            crossmat_list.reserve(feats_down_size);
            pbody_list.reserve(feats_down_size);
            // pbody_ext_list.reserve(feats_down_size);

            for (size_t i = 0; i < feats_down_body->size(); i++) {
                V3D point_this(feats_down_body->points[i].x, feats_down_body->points[i].y,
                               feats_down_body->points[i].z);
                pbody_list[i] = point_this;
                if (!extrinsic_est_en)
                // {
                //     if (!use_imu_as_input)
                //     {
                //         point_this = kf_output.x_.offset_R_L_I * point_this + kf_output.x_.offset_T_L_I;
                //     }
                //     else
                //     {
                //         point_this = kf_input.x_.offset_R_L_I * point_this + kf_input.x_.offset_T_L_I;
                //     }
                // }
                // else
                {
                    point_this = Lidar_R_wrt_IMU * point_this + Lidar_T_wrt_IMU;
                    M3D point_crossmat;
                    point_crossmat << SKEW_SYM_MATRX(point_this);
                    crossmat_list[i] = point_crossmat;
                }
            }
            if (!use_imu_as_input) {
                bool imu_upda_cov = false;
                effct_feat_num = 0;
                /**** point by point update ****/
                if (time_seq.size() > 0) {
                    double pcl_beg_time = Measures.lidar_beg_time;
                    idx = -1;
                    for (k = 0; k < time_seq.size(); k++) {
                        PointType &point_body = feats_down_body->points[idx + time_seq[k]];

                        time_current = point_body.curvature / 1000.0 + pcl_beg_time;

                        if (is_first_frame) {
                            if (imu_en) {
                                while (time_current > imu_next.header.stamp.toSec()) {
                                    imu_deque.pop_front();
                                    if (imu_deque.empty())
                                        break;
                                    imu_last = imu_next;
                                    imu_next = *(imu_deque.front());
                                }
                                angvel_avr << imu_last.angular_velocity.x, imu_last.angular_velocity.y,
                                    imu_last.angular_velocity.z;
                                acc_avr << imu_last.linear_acceleration.x, imu_last.linear_acceleration.y,
                                    imu_last.linear_acceleration.z;
                            }
                            is_first_frame = false;
                            imu_upda_cov = true;
                            time_update_last = time_current;
                            time_predict_last_const = time_current;
                        }
                        if (imu_en && !imu_deque.empty()) {
                            bool last_imu = imu_next.header.stamp.toSec() == imu_deque.front()->header.stamp.toSec();
                            while (imu_next.header.stamp.toSec() < time_predict_last_const && !imu_deque.empty()) {
                                if (!last_imu) {
                                    imu_last = imu_next;
                                    imu_next = *(imu_deque.front());
                                    break;
                                } else {
                                    imu_deque.pop_front();
                                    if (imu_deque.empty())
                                        break;
                                    imu_last = imu_next;
                                    imu_next = *(imu_deque.front());
                                }
                            }
                            bool imu_comes = time_current > imu_next.header.stamp.toSec();
                            while (imu_comes) {
                                imu_upda_cov = true;
                                angvel_avr << imu_next.angular_velocity.x, imu_next.angular_velocity.y,
                                    imu_next.angular_velocity.z;
                                acc_avr << imu_next.linear_acceleration.x, imu_next.linear_acceleration.y,
                                    imu_next.linear_acceleration.z;

                                /*** covariance update ***/
                                double dt = imu_next.header.stamp.toSec() - time_predict_last_const;
                                kf_output.predict(dt, Q_output, input_in, true, false);
                                time_predict_last_const = imu_next.header.stamp.toSec(); // big problem

                                {
                                    double dt_cov = imu_next.header.stamp.toSec() - time_update_last;

                                    if (dt_cov > 0.0) {
                                        time_update_last = imu_next.header.stamp.toSec();
                                        double propag_imu_start = omp_get_wtime();

                                        kf_output.predict(dt_cov, Q_output, input_in, false, true);

                                        propag_time += omp_get_wtime() - propag_imu_start;
                                        double solve_imu_start = omp_get_wtime();
                                        kf_output.update_iterated_dyn_share_IMU();
                                        solve_time += omp_get_wtime() - solve_imu_start;
                                    }
                                }
                                imu_deque.pop_front();
                                if (imu_deque.empty())
                                    break;
                                imu_last = imu_next;
                                imu_next = *(imu_deque.front());
                                imu_comes = time_current > imu_next.header.stamp.toSec();
                            }
                        }
                        if (flg_reset) {
                            break;
                        }

                        double dt = time_current - time_predict_last_const;
                        double propag_state_start = omp_get_wtime();
                        if (!prop_at_freq_of_imu) {
                            double dt_cov = time_current - time_update_last;
                            if (dt_cov > 0.0) {
                                kf_output.predict(dt_cov, Q_output, input_in, false, true);
                                time_update_last = time_current;
                            }
                        }
                        kf_output.predict(dt, Q_output, input_in, true, false);
                        propag_time += omp_get_wtime() - propag_state_start;
                        time_predict_last_const = time_current;
                        double t_update_start = omp_get_wtime();

                        if (feats_down_size < 1) {
                            ROS_WARN("No point, skip this scan!\n");
                            idx += time_seq[k];
                            continue;
                        }
                        if (!kf_output.update_iterated_dyn_share_modified()) {
                            idx = idx + time_seq[k];
                            continue;
                        }
                        solve_start = omp_get_wtime();

                        if (publish_odometry_without_downsample) {
                            /******* Publish odometry *******/

                            publish_odometry(pubOdomAftMapped, pubLioPose, pubLioPoseCov);
                            if (runtime_pos_log) {
                                euler_cur = SO3ToEuler(kf_output.x_.rot);
                                fout_out << setw(20) << Measures.lidar_beg_time - first_lidar_time << " "
                                         << euler_cur.transpose() << " " << kf_output.x_.pos.transpose() << " "
                                         << kf_output.x_.vel.transpose() << " " << kf_output.x_.omg.transpose() << " "
                                         << kf_output.x_.acc.transpose() << " " << kf_output.x_.gravity.transpose()
                                         << " " << kf_output.x_.bg.transpose() << " " << kf_output.x_.ba.transpose()
                                         << " " << feats_undistort->points.size() << endl;
                            }
                        }

                        for (int j = 0; j < time_seq[k]; j++) {
                            PointType &point_body_j = feats_down_body->points[idx + j + 1];
                            PointType &point_world_j = feats_down_world->points[idx + j + 1];
                            pointBodyToWorld(&point_body_j, &point_world_j);
                        }

                        solve_time += omp_get_wtime() - solve_start;

                        update_time += omp_get_wtime() - t_update_start;
                        idx += time_seq[k];
                        // cout << "pbp output effect feat num:" << effct_feat_num << endl;
                    }
                } else {
                    if (!imu_deque.empty()) {
                        imu_last = imu_next;
                        imu_next = *(imu_deque.front());

                        while (imu_next.header.stamp.toSec() > time_current &&
                               ((imu_next.header.stamp.toSec() < Measures.lidar_beg_time + lidar_time_inte))) { // >= ?
                            if (is_first_frame) {
                                {
                                    {
                                        while (imu_next.header.stamp.toSec() <
                                               Measures.lidar_beg_time + lidar_time_inte) {
                                            // meas.imu.emplace_back(imu_deque.front()); should add to initialization
                                            imu_deque.pop_front();
                                            if (imu_deque.empty())
                                                break;
                                            imu_last = imu_next;
                                            imu_next = *(imu_deque.front());
                                        }
                                    }
                                    break;
                                }
                                angvel_avr << imu_last.angular_velocity.x, imu_last.angular_velocity.y,
                                    imu_last.angular_velocity.z;

                                acc_avr << imu_last.linear_acceleration.x, imu_last.linear_acceleration.y,
                                    imu_last.linear_acceleration.z;

                                imu_upda_cov = true;
                                time_update_last = time_current;
                                time_predict_last_const = time_current;

                                is_first_frame = false;
                            }
                            time_current = imu_next.header.stamp.toSec();

                            if (!is_first_frame) {
                                double dt = time_current - time_predict_last_const;
                                {
                                    double dt_cov = time_current - time_update_last;
                                    if (dt_cov > 0.0) {
                                        kf_output.predict(dt_cov, Q_output, input_in, false, true);
                                        time_update_last = time_current;
                                    }
                                    kf_output.predict(dt, Q_output, input_in, true, false);
                                }

                                time_predict_last_const = time_current;

                                angvel_avr << imu_next.angular_velocity.x, imu_next.angular_velocity.y,
                                    imu_next.angular_velocity.z;
                                acc_avr << imu_next.linear_acceleration.x, imu_next.linear_acceleration.y,
                                    imu_next.linear_acceleration.z;
                                // acc_avr_norm = acc_avr * G_m_s2 / acc_norm;
                                kf_output.update_iterated_dyn_share_IMU();
                                imu_deque.pop_front();
                                if (imu_deque.empty())
                                    break;
                                imu_last = imu_next;
                                imu_next = *(imu_deque.front());
                            } else {
                                imu_deque.pop_front();
                                if (imu_deque.empty())
                                    break;
                                imu_last = imu_next;
                                imu_next = *(imu_deque.front());
                            }
                        }
                    }
                }
            } else {
                bool imu_prop_cov = false;
                effct_feat_num = 0;
                if (time_seq.size() > 0) {
                    double pcl_beg_time = Measures.lidar_beg_time;
                    idx = -1;
                    for (k = 0; k < time_seq.size(); k++) {
                        PointType &point_body = feats_down_body->points[idx + time_seq[k]];
                        time_current = point_body.curvature / 1000.0 + pcl_beg_time;
                        if (is_first_frame) {
                            while (time_current > imu_next.header.stamp.toSec()) {
                                imu_deque.pop_front();
                                if (imu_deque.empty())
                                    break;
                                imu_last = imu_next;
                                imu_next = *(imu_deque.front());
                            }
                            imu_prop_cov = true;

                            is_first_frame = false;
                            t_last = time_current;
                            time_update_last = time_current;
                            {
                                input_in.gyro << imu_last.angular_velocity.x, imu_last.angular_velocity.y,
                                    imu_last.angular_velocity.z;
                                input_in.acc << imu_last.linear_acceleration.x, imu_last.linear_acceleration.y,
                                    imu_last.linear_acceleration.z;
                                input_in.acc = input_in.acc * G_m_s2 / acc_norm;
                            }
                        }

                        while (time_current > imu_next.header.stamp.toSec()) // && !imu_deque.empty())
                        {
                            imu_deque.pop_front();

                            input_in.gyro << imu_last.angular_velocity.x, imu_last.angular_velocity.y,
                                imu_last.angular_velocity.z;
                            input_in.acc << imu_last.linear_acceleration.x, imu_last.linear_acceleration.y,
                                imu_last.linear_acceleration.z;
                            input_in.acc = input_in.acc * G_m_s2 / acc_norm;
                            double dt = imu_last.header.stamp.toSec() - t_last;

                            double dt_cov = imu_last.header.stamp.toSec() - time_update_last;
                            if (dt_cov > 0.0) {
                                kf_input.predict(dt_cov, Q_input, input_in, false, true);
                                time_update_last = imu_last.header.stamp.toSec(); // time_current;
                            }
                            kf_input.predict(dt, Q_input, input_in, true, false);
                            t_last = imu_last.header.stamp.toSec();
                            imu_prop_cov = true;

                            if (imu_deque.empty())
                                break;
                            imu_last = imu_next;
                            imu_next = *(imu_deque.front());
                            // imu_upda_cov = true;
                        }
                        if (flg_reset) {
                            break;
                        }
                        double dt = time_current - t_last;
                        t_last = time_current;
                        double propag_start = omp_get_wtime();

                        if (!prop_at_freq_of_imu) {
                            double dt_cov = time_current - time_update_last;
                            if (dt_cov > 0.0) {
                                kf_input.predict(dt_cov, Q_input, input_in, false, true);
                                time_update_last = time_current;
                            }
                        }
                        kf_input.predict(dt, Q_input, input_in, true, false);

                        propag_time += omp_get_wtime() - propag_start;

                        double t_update_start = omp_get_wtime();

                        if (feats_down_size < 1) {
                            ROS_WARN("No point, skip this scan!\n");

                            idx += time_seq[k];
                            continue;
                        }
                        if (!kf_input.update_iterated_dyn_share_modified()) {
                            idx = idx + time_seq[k];
                            continue;
                        }

                        solve_start = omp_get_wtime();

                        if (publish_odometry_without_downsample) {
                            /******* Publish odometry *******/

                            publish_odometry(pubOdomAftMapped, pubLioPose, pubLioPoseCov);
                            if (runtime_pos_log) {
                                euler_cur = SO3ToEuler(kf_input.x_.rot);
                                fout_out << setw(20) << Measures.lidar_beg_time - first_lidar_time << " "
                                         << euler_cur.transpose() << " " << kf_input.x_.pos.transpose() << " "
                                         << kf_input.x_.vel.transpose() << " " << kf_input.x_.bg.transpose() << " "
                                         << kf_input.x_.ba.transpose() << " " << kf_input.x_.gravity.transpose() << " "
                                         << feats_undistort->points.size() << endl;
                            }
                        }

                        for (int j = 0; j < time_seq[k]; j++) {
                            PointType &point_body_j = feats_down_body->points[idx + j + 1];
                            PointType &point_world_j = feats_down_world->points[idx + j + 1];
                            pointBodyToWorld(&point_body_j, &point_world_j);
                        }
                        solve_time += omp_get_wtime() - solve_start;

                        update_time += omp_get_wtime() - t_update_start;
                        idx = idx + time_seq[k];
                    }
                } else {
                    if (!imu_deque.empty()) {
                        imu_last = imu_next;
                        imu_next = *(imu_deque.front());
                        while (imu_next.header.stamp.toSec() > time_current &&
                               ((imu_next.header.stamp.toSec() < Measures.lidar_beg_time + lidar_time_inte))) { // >= ?
                            if (is_first_frame) {
                                {
                                    {
                                        while (imu_next.header.stamp.toSec() <
                                               Measures.lidar_beg_time + lidar_time_inte) {
                                            imu_deque.pop_front();
                                            if (imu_deque.empty())
                                                break;
                                            imu_last = imu_next;
                                            imu_next = *(imu_deque.front());
                                        }
                                    }

                                    break;
                                }
                                imu_prop_cov = true;

                                t_last = time_current;
                                time_update_last = time_current;
                                input_in.gyro << imu_last.angular_velocity.x, imu_last.angular_velocity.y,
                                    imu_last.angular_velocity.z;
                                input_in.acc << imu_last.linear_acceleration.x, imu_last.linear_acceleration.y,
                                    imu_last.linear_acceleration.z;
                                input_in.acc = input_in.acc * G_m_s2 / acc_norm;

                                is_first_frame = false;
                            }
                            time_current = imu_next.header.stamp.toSec();

                            if (!is_first_frame) {
                                double dt = time_current - t_last;

                                double dt_cov = time_current - time_update_last;
                                if (dt_cov > 0.0) {
                                    // kf_input.predict(dt_cov, Q_input, input_in, false, true);
                                    time_update_last = imu_next.header.stamp.toSec(); // time_current;
                                }
                                // kf_input.predict(dt, Q_input, input_in, true, false);

                                t_last = imu_next.header.stamp.toSec();

                                input_in.gyro << imu_next.angular_velocity.x, imu_next.angular_velocity.y,
                                    imu_next.angular_velocity.z;
                                input_in.acc << imu_next.linear_acceleration.x, imu_next.linear_acceleration.y,
                                    imu_next.linear_acceleration.z;
                                input_in.acc = input_in.acc * G_m_s2 / acc_norm;
                                imu_deque.pop_front();
                                if (imu_deque.empty())
                                    break;
                                imu_last = imu_next;
                                imu_next = *(imu_deque.front());
                            } else {
                                imu_deque.pop_front();
                                if (imu_deque.empty())
                                    break;
                                imu_last = imu_next;
                                imu_next = *(imu_deque.front());
                            }
                        }
                    }
                }
            }
            // M3D rot_cur_lidar;
            // {
            //     rot_cur_lidar = state.rot_end;
            // }
            // euler_cur = RotMtoEuler(rot_cur_lidar);
            // geoQuat = tf::createQuaternionMsgFromRollPitchYaw
            //                     (euler_cur(0), euler_cur(1), euler_cur(2));
            /******* Publish odometry downsample *******/
            // Unconditional, once per scan. When publish_odometry_without_downsample is set,
            // the per-point publishes inside the estimator above sit *after* the
            // `if (!kf.update_iterated_dyn_share_modified()) continue;` guard, so odometry
            // only goes out on point batches whose LiDAR update succeeded. h_model_output
            // fails a batch whenever it finds no plane match (effect_num_k == 0), and a
            // filter that has lost the map fails every batch of every future scan -- there is
            // no path back, because only a successful update could correct the state. The
            // stream then stops permanently while the node stays healthy.
            //
            // This call is the floor: while scans keep arriving, pose keeps going out at
            // least at scan rate, IMU-propagated if LiDAR cannot correct it. In the healthy
            // case it costs nothing -- the point-by-point path already published this
            // time_current, so publish_odometry drops it as a duplicate instant.
            publish_odometry(pubOdomAftMapped, pubLioPose, pubLioPoseCov);

            /*** add the feature points to map ***/
            t3 = omp_get_wtime();

            if (feats_down_size > 4) {
                MapIncremental();
            }

            t5 = omp_get_wtime();
            /******* Publish points *******/
            if (path_en)
                publish_path(pubPath);
            if (scan_pub_en || pcd_save_en)
                publish_frame_world(pubLaserCloudFullRes);
            if (scan_pub_en && scan_body_pub_en)
                publish_frame_body(pubLaserCloudFullRes_body);

            /*** Debug variables Logging ***/
            if (runtime_pos_log) {
                frame_num++;
                aver_time_consu = aver_time_consu * (frame_num - 1) / frame_num + (t5 - t0) / frame_num;
                {
                    aver_time_icp = aver_time_icp * (frame_num - 1) / frame_num + update_time / frame_num;
                }
                aver_time_match = aver_time_match * (frame_num - 1) / frame_num + (match_time) / frame_num;
                aver_time_solve = aver_time_solve * (frame_num - 1) / frame_num + solve_time / frame_num;
                aver_time_propag = aver_time_propag * (frame_num - 1) / frame_num + propag_time / frame_num;
                T1[time_log_counter] = Measures.lidar_beg_time;
                s_plot[time_log_counter] = t5 - t0;
                s_plot2[time_log_counter] = feats_undistort->points.size();
                s_plot3[time_log_counter] = aver_time_consu;
                time_log_counter++;
                printf("[ mapping ]: time: IMU + Map + Input Downsample: %0.6f ave match: %0.6f ave solve: %0.6f  ave "
                       "ICP: %0.6f  map incre: %0.6f ave total: %0.6f icp: %0.6f propogate: %0.6f \n",
                       t1 - t0, aver_time_match, aver_time_solve, t3 - t1, t5 - t3, aver_time_consu, aver_time_icp,
                       aver_time_propag);
                if (!publish_odometry_without_downsample) {
                    if (!use_imu_as_input) {
                        euler_cur = SO3ToEuler(kf_output.x_.rot);
                        fout_out << setw(20) << Measures.lidar_beg_time - first_lidar_time << " "
                                 << euler_cur.transpose() << " " << kf_output.x_.pos.transpose() << " "
                                 << kf_output.x_.vel.transpose() << " " << kf_output.x_.omg.transpose() << " "
                                 << kf_output.x_.acc.transpose() << " " << kf_output.x_.gravity.transpose() << " "
                                 << kf_output.x_.bg.transpose() << " " << kf_output.x_.ba.transpose() << " "
                                 << feats_undistort->points.size() << endl;
                    } else {
                        euler_cur = SO3ToEuler(kf_input.x_.rot);
                        fout_out << setw(20) << Measures.lidar_beg_time - first_lidar_time << " "
                                 << euler_cur.transpose() << " " << kf_input.x_.pos.transpose() << " "
                                 << kf_input.x_.vel.transpose() << " " << kf_input.x_.bg.transpose() << " "
                                 << kf_input.x_.ba.transpose() << " " << kf_input.x_.gravity.transpose() << " "
                                 << feats_undistort->points.size() << endl;
                    }
                }
                dump_lio_state_to_log(fp);
            }
        }
        // Outside the sync_packages() branch on purpose: a stalled sync_packages is exactly
        // the case this has to be able to report on.
        check_odom_stall();
        status = ros::ok();
        loop_rate.sleep();
    }
    //--------------------------save map-----------------------------------
    /* 1. make sure you have enough memories
    /* 2. noted that pcd save will influence the real-time performences **/
    if (pcl_wait_save->size() > 0 && pcd_save_en) {
        string file_name = string("scans.pcd");
        string all_points_dir(string(string(ROOT_DIR) + "PCD/") + file_name);
        pcl::PCDWriter pcd_writer;
        pcd_writer.writeBinary(all_points_dir, *pcl_wait_save);
    }
    fout_out.close();
    fout_imu_pbp.close();
    return 0;
}
