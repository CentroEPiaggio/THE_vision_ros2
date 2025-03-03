#include "fixedeye_calibration/pose_weighted_average.hpp"
#include "rclcpp/rclcpp.hpp"
namespace fixed_eye_calibration
{
    WeightedPoseAverage::WeightedPoseAverage(MeasureType m_type, WeightType w_type):
    measures_(std::list<PoseMeasure>()),
    covariances_(std::list<PoseCovariance>()),
    weights_(std::list<double>()),
    measure_type_(m_type),
    weight_type_(w_type)
    {};

    void WeightedPoseAverage::convert_pose_to_vector(Pose pose, PoseMeasure& vect)
    {
        vect[0] = pose.position.x;
        vect[1] = pose.position.y;
        vect[2] = pose.position.z;
        vect[3] = pose.orientation.w;
        vect[4] = pose.orientation.x;
        vect[5] = pose.orientation.y;
        vect[6] = pose.orientation.z;
    };

    void WeightedPoseAverage::convert_vector_to_pose(Pose& pose, PoseMeasure vect)
    {
        pose.position.set__x(vect[0]);
        pose.position.set__y(vect[1]);
        pose.position.set__z(vect[2]);
        pose.orientation.set__w(vect[3]);
        pose.orientation.set__x(vect[4]);
        pose.orientation.set__y(vect[5]);
        pose.orientation.set__z(vect[6]);
    };

    void WeightedPoseAverage::normalize_quaternion(Quaternion& quat)
    {
        Eigen::Quaterniond quat_eigen;
        tf2::fromMsg(quat,quat_eigen);
        quat_eigen.normalize();
        quat = tf2::toMsg(quat_eigen);
    }

    void WeightedPoseAverage::add_measure(Pose pose, PoseCovariance cov)
    {
        PoseMeasure meas;
        if(measure_type_ == MeasureType::UNDEFINED)
            measure_type_ = MeasureType::POSE_W_COVARIANCE;
        else
        {
            if(measure_type_ != MeasureType::POSE_W_COVARIANCE)
                throw std::runtime_error("Trying to add not POSE_W_COVARIANCE measure");
        }

        convert_pose_to_vector(pose,meas);
        measures_.push_back(meas);
        covariances_.push_back(cov);
        weights_.push_back(1.0);
    };

    void WeightedPoseAverage::add_measure(Pose pose)
    {
        PoseMeasure meas;
        if(measure_type_ == MeasureType::UNDEFINED)
            measure_type_ = MeasureType::POSE;
        else
        {
            if(measure_type_ != MeasureType::POSE)
                throw std::runtime_error("Trying to add not POSE measure");
        }
        convert_pose_to_vector(pose,meas);
        PoseCovariance cov = PoseCovariance::Identity();
        measures_.push_back(meas);
        covariances_.push_back(cov);
        weights_.push_back(1.0);
    }

    void WeightedPoseAverage::add_measure(Quaternion quat)
    {
        PoseMeasure meas;
        if(measure_type_ == MeasureType::UNDEFINED)
            measure_type_ = MeasureType::QUATERNION;
        else
        {
            if(measure_type_ != MeasureType::QUATERNION)
                throw std::runtime_error("Trying to add not QUATERNION measure");
        }

        PoseCovariance cov = PoseCovariance::Identity();
        Pose pose;
        pose.set__orientation(quat);
        convert_pose_to_vector(pose,meas);
        measures_.push_back(meas);
        covariances_.push_back(cov);
        weights_.push_back(1.0);
    }

    void WeightedPoseAverage::add_measure(Point point)
    {
        PoseMeasure meas;
        if(measure_type_ == MeasureType::UNDEFINED)
            measure_type_ = MeasureType::POINT;
        else
        {
            if(measure_type_ != MeasureType::POINT)
                throw std::runtime_error("Trying to add not POINT measure");
        }

        PoseCovariance cov = PoseCovariance::Identity();
        Pose pose;
        pose.set__position(point);
        convert_pose_to_vector(pose,meas);
        measures_.push_back(meas);
        covariances_.push_back(cov);
        weights_.push_back(1.0);
    }

    void WeightedPoseAverage::set_normalize_weight()
    {
        std::list<PoseMeasure>::iterator meas_it = measures_.begin();
        std::list<PoseCovariance>::iterator cov_it = covariances_.begin();
        std::list<double>::iterator wgt_it = weights_.begin();
        double weight_sum = 0;
        while(wgt_it!=weights_.end())
        {
            if(cov_it == covariances_.end() || meas_it == measures_.end())
                throw std::runtime_error("The list has not the same lenght");
            if(weight_type_ == WeightType::TRACE)
                *wgt_it = cov_it->trace();
            else if(weight_type_ == WeightType::MAHALANOBIS)
                *wgt_it = (*meas_it).transpose() * (*cov_it) * (*meas_it);
            weight_sum += *wgt_it;
            wgt_it++;
            cov_it++;
            meas_it++;
        }
        for(wgt_it = weights_.begin() ; wgt_it != weights_.end() ; wgt_it++ )
            *wgt_it /= weight_sum;
    };

    void WeightedPoseAverage::compute_average(Pose& result)
    {
        Eigen::Matrix4d  A_sum = Eigen::Matrix4d::Zero();
        Eigen::Vector3d pos_eigen_sum = Eigen::Vector3d::Zero();
        Eigen::Vector4cd quat_eigen;
        std::list<PoseMeasure>::iterator meas_it = measures_.begin();
        std::list<double>::iterator wgt_it = weights_.begin();

        while(wgt_it != weights_.end())
        {
            auto a = (*meas_it).block<4,1>(3,0).transpose();
            pos_eigen_sum += (*wgt_it)*(*meas_it).block<3,1>(0,0);
            A_sum += (*wgt_it)*(*meas_it).block<4,1>(3,0)*(*meas_it).block<4,1>(3,0).transpose();

            wgt_it++;

            meas_it++;
        }
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d> es(A_sum);

        quat_eigen = es.eigenvectors().col(es.eigenvectors().cols() -1);

        result.position.set__x(pos_eigen_sum[0]);
        result.position.set__y(pos_eigen_sum[1]);
        result.position.set__z(pos_eigen_sum[2]);
        result.orientation.set__w(quat_eigen[0].real());
        result.orientation.set__x(quat_eigen[1].real());
        result.orientation.set__y(quat_eigen[2].real());
        result.orientation.set__z(quat_eigen[3].real());

        normalize_quaternion(result.orientation);     
    };

    void WeightedPoseAverage::compute_covariance(PoseCovariance& result,PoseMeasure mean)
    {
        std::list<PoseMeasure>::iterator meas_it = measures_.begin();
        std::list<double>::iterator wgt_it = weights_.begin();
        Eigen::Quaterniond q_mean=Eigen::Quaterniond(mean[3],mean[4],mean[5],mean[6]),q_act,q_err;
        Eigen::Vector3d v_mean=mean.block<3,1>(0,0),v_act,v_err;
        Eigen::Matrix3d pos_cov = Eigen::Matrix3d::Zero();
        Eigen::Matrix4d quat_cov = Eigen::Matrix4d::Zero();
        Eigen::Vector4d quat_err_vect;
        result = PoseCovariance::Zero();

        while(meas_it!=measures_.end())
        {
            v_act = (*meas_it).block<3,1>(0,0);
            q_act = Eigen::Quaterniond((*meas_it)[3],(*meas_it)[4],(*meas_it)[5],(*meas_it)[6]);
            v_err = v_act-v_mean;
            
            pos_cov +=(*wgt_it)*(v_err * v_err.transpose());
            q_err = q_act * q_mean.inverse();
            quat_err_vect << q_err.w(),q_err.x(),q_err.y(),q_err.z();
            quat_cov += (*wgt_it)*(quat_err_vect * quat_err_vect.transpose());
            meas_it++;
        }
        
        result.block<3,3>(0,0) = pos_cov;
        result.block<4,4>(3,3) = quat_cov;
    }
}