#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/features/normal_3d.h>
#include <pcl/common/pca.h>
#include <Eigen/Dense>
#include <mutex>
#include <string>
#include <vector>
#include <numeric>

class CloudDiagnostics {
public:
    CloudDiagnostics() : nh_("~") {
        nh_.param<std::string>("topic", topic_, "/merged_points");
        nh_.param<int>("skip_frames", skip_frames_, 5);

        sub_ = nh_.subscribe(topic_, 10, &CloudDiagnostics::cloudCallback, this);

        ROS_INFO("CloudDiagnostics started");
        ROS_INFO("  Topic: %s", topic_.c_str());
        ROS_INFO("  Skip frames: %d", skip_frames_);
        ROS_INFO("========================================");
    }

private:
    void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
        static int frame_count = 0;
        frame_count++;

        if (frame_count % skip_frames_ != 0) return;

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud);

        if (cloud->empty()) return;

        ROS_INFO("========================================");
        ROS_INFO("帧 #%d, 时间戳: %.3f", frame_count, msg->header.stamp.toSec());
        ROS_INFO("点云大小: %lu", cloud->size());

        // 1. 基本统计
        analyzeBasicStats(cloud);

        // 2. 高度分布分析
        analyzeHeightDistribution(cloud);

        // 3. 几何特征分析（平面度、线性度）
        analyzeGeometry(cloud);

        // 4. 退化风险评估
        assessDegradationRisk(cloud);

        ROS_INFO("========================================");
    }

    void analyzeBasicStats(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
        float min_x = 1e9, max_x = -1e9;
        float min_y = 1e9, max_y = -1e9;
        float min_z = 1e9, max_z = -1e9;

        for (const auto& p : cloud->points) {
            min_x = std::min(min_x, p.x); max_x = std::max(max_x, p.x);
            min_y = std::min(min_y, p.y); max_y = std::max(max_y, p.y);
            min_z = std::min(min_z, p.z); max_z = std::max(max_z, p.z);
        }

        ROS_INFO("[基本统计]");
        ROS_INFO("  X: [%.2f, %.2f], 范围: %.2f m", min_x, max_x, max_x - min_x);
        ROS_INFO("  Y: [%.2f, %.2f], 范围: %.2f m", min_y, max_y, max_y - min_y);
        ROS_INFO("  Z: [%.2f, %.2f], 范围: %.2f m", min_z, max_z, max_z - min_z);
    }

    void analyzeHeightDistribution(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
        // 计算高度直方图
        std::vector<float> heights;
        heights.reserve(cloud->size());
        for (const auto& p : cloud->points) {
            heights.push_back(p.z);
        }

        // 统计
        float min_z = *std::min_element(heights.begin(), heights.end());
        float max_z = *std::max_element(heights.begin(), heights.end());
        float mean_z = std::accumulate(heights.begin(), heights.end(), 0.0f) / heights.size();

        // 计算方差
        float var_z = 0;
        for (float z : heights) {
            var_z += (z - mean_z) * (z - mean_z);
        }
        var_z /= heights.size();
        float std_z = std::sqrt(var_z);

        // 计算高度分布
        int bins = 10;
        float bin_width = (max_z - min_z) / bins;
        std::vector<int> histogram(bins, 0);
        for (float z : heights) {
            int bin = std::min((int)((z - min_z) / bin_width), bins - 1);
            histogram[bin]++;
        }

        ROS_INFO("[高度分布]");
        ROS_INFO("  均值: %.2f, 标准差: %.4f, 方差: %.4f", mean_z, std_z, var_z);

        // 打印直方图
        ROS_INFO("  直方图 (Z从低到高):");
        for (int i = 0; i < bins; i++) {
            float z_low = min_z + i * bin_width;
            float z_high = z_low + bin_width;
            int count = histogram[i];
            float ratio = (float)count / cloud->size() * 100;
            std::string bar(ratio / 2, '#');
            ROS_INFO("    [%.1f, %.1f]: %4d (%5.1f%%) %s", z_low, z_high, count, ratio, bar.c_str());
        }

        // 计算地面点占比（Z < mean + 0.3m）
        int ground_count = 0;
        for (float z : heights) {
            if (z < mean_z + 0.3) ground_count++;
        }
        float ground_ratio = (float)ground_count / cloud->size();
        ROS_INFO("  地面点占比 (Z < mean+0.3m): %.1f%%", ground_ratio * 100);
    }

    void analyzeGeometry(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
        // 使用 PCA 分析几何特征
        pcl::PCA<pcl::PointXYZ> pca;
        pca.setInputCloud(cloud);
        Eigen::Vector3f eigen_values = pca.getEigenValues();
        Eigen::Matrix3f eigen_vectors = pca.getEigenVectors();

        // 归一化特征值
        float sum = eigen_values.sum();
        Eigen::Vector3f normalized = eigen_values / sum;

        // 几何特征
        float linearity = (normalized[0] - normalized[1]) / normalized[0];
        float planarity = (normalized[1] - normalized[2]) / normalized[0];
        float sphericity = normalized[2] / normalized[0];

        ROS_INFO("[几何特征 (PCA)]");
        ROS_INFO("  特征值: %.4f, %.4f, %.4f", eigen_values[0], eigen_values[1], eigen_values[2]);
        ROS_INFO("  归一化: %.4f, %.4f, %.4f", normalized[0], normalized[1], normalized[2]);
        ROS_INFO("  线性度: %.4f (高=线状结构)", linearity);
        ROS_INFO("  平面度: %.4f (高=平面结构)", planarity);
        ROS_INFO("  球面度: %.4f (高=球状结构)", sphericity);

        // 主方向
        ROS_INFO("  主方向 (第一主成分): (%.4f, %.4f, %.4f)",
                  eigen_vectors(0, 0), eigen_vectors(1, 0), eigen_vectors(2, 0));
        ROS_INFO("  第二方向: (%.4f, %.4f, %.4f)",
                  eigen_vectors(0, 1), eigen_vectors(1, 1), eigen_vectors(2, 1));
        ROS_INFO("  第三方向: (%.4f, %.4f, %.4f)",
                  eigen_vectors(0, 2), eigen_vectors(1, 2), eigen_vectors(2, 2));

        // 判断是否为平面场景
        if (planarity > 0.8) {
            ROS_WARN("  ⚠️  高平面度 (%.4f > 0.8)：这是一个平面场景！", planarity);
        }
    }

    void assessDegradationRisk(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
        // 使用 PCA 分析
        pcl::PCA<pcl::PointXYZ> pca;
        pca.setInputCloud(cloud);
        Eigen::Vector3f eigen_values = pca.getEigenValues();
        Eigen::Vector3f normalized = eigen_values / eigen_values.sum();

        float planarity = (normalized[1] - normalized[2]) / normalized[0];

        // 计算高度范围
        float min_z = 1e9, max_z = -1e9;
        for (const auto& p : cloud->points) {
            min_z = std::min(min_z, p.z);
            max_z = std::max(max_z, p.z);
        }
        float z_range = max_z - min_z;

        // 计算地面点占比
        float mean_z = 0;
        for (const auto& p : cloud->points) mean_z += p.z;
        mean_z /= cloud->size();

        int ground_count = 0;
        for (const auto& p : cloud->points) {
            if (p.z < mean_z + 0.3) ground_count++;
        }
        float ground_ratio = (float)ground_count / cloud->size();

        // 退化风险评估
        ROS_INFO("[退化风险评估]");

        int risk_score = 0;
        std::vector<std::string> risks;

        if (planarity > 0.8) {
            risk_score += 3;
            risks.push_back("高平面度");
            ROS_WARN("  ❌ 高平面度 (%.4f > 0.8)：+3 风险分", planarity);
        } else if (planarity > 0.6) {
            risk_score += 1;
            risks.push_back("中等平面度");
            ROS_WARN("  ⚠️  中等平面度 (%.4f > 0.6)：+1 风险分", planarity);
        }

        if (z_range < 0.5) {
            risk_score += 3;
            risks.push_back("高度范围小");
            ROS_WARN("  ❌ 高度范围小 (%.2fm < 0.5m)：+3 风险分", z_range);
        } else if (z_range < 1.0) {
            risk_score += 1;
            risks.push_back("高度范围较小");
            ROS_WARN("  ⚠️  高度范围较小 (%.2fm < 1.0m)：+1 风险分", z_range);
        }

        if (ground_ratio > 0.7) {
            risk_score += 2;
            risks.push_back("地面点占比高");
            ROS_WARN("  ❌ 地面点占比高 (%.1f%% > 70%%)：+2 风险分", ground_ratio * 100);
        }

        ROS_INFO("  风险总分: %d", risk_score);
        ROS_INFO("  风险因素: ");
        for (const auto& r : risks) {
            ROS_INFO("    - %s", r.c_str());
        }

        if (risk_score >= 5) {
            ROS_ERROR("========================================");
            ROS_ERROR("  ❌❌❌ KISS-ICP 退化风险：高 (%d分)", risk_score);
            ROS_ERROR("  建议：替换为 FastVGICP 或 NDT_OMP");
            ROS_ERROR("========================================");
        } else if (risk_score >= 3) {
            ROS_WARN("========================================");
            ROS_WARN("  ⚠️⚠️⚠️ KISS-ICP 退化风险：中 (%d分)", risk_score);
            ROS_WARN("  建议：监控位姿输出，必要时替换算法");
            ROS_WARN("========================================");
        } else {
            ROS_INFO("========================================");
            ROS_INFO("  ✅ KISS-ICP 退化风险：低 (%d分)", risk_score);
            ROS_INFO("  当前场景适合使用 KISS-ICP");
            ROS_INFO("========================================");
        }
    }

    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    std::string topic_;
    int skip_frames_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "cloud_diagnostics");
    CloudDiagnostics diag;
    ros::spin();
    return 0;
}
