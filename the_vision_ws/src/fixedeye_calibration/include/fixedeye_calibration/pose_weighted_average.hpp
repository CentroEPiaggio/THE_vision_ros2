#include <list>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Eigenvalues>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2_eigen/tf2_eigen.hpp>


namespace fixed_eye_calibration
{
    typedef Eigen::Matrix<double,7,7> PoseCovariance;
    typedef Eigen::Vector<double,7> PoseMeasure;
    using Pose = geometry_msgs::msg::Pose;
    using Quaternion = geometry_msgs::msg::Quaternion;
    using Point = geometry_msgs::msg::Point;

    enum MeasureType {
        POINT,
        QUATERNION,
        POSE,
        POSE_W_COVARIANCE,
        UNDEFINED
    };
    enum WeightType {
        UNIFORM,
        TRACE,
        MAHALANOBIS
    };

    class WeightedPoseAverage
    {
        public:
            WeightedPoseAverage(MeasureType m_type=MeasureType::UNDEFINED, WeightType w_type=WeightType::UNIFORM);        
            // add measure methods
            void add_measure(Pose pose, PoseCovariance cov);
            void add_measure(Pose pose);
            void add_measure(Quaternion quat);
            void add_measure(Point point);
            
            

            void set_measure_type(MeasureType type)
            {
                measure_type_ = type;
            };

            void set_weight_type(WeightType type)
            {
                weight_type_ = type;
            };

            void ave_and_cov_compute(PoseCovariance& cov_res, Pose& mean_res, bool do_reset=true)
            {
                PoseMeasure mean;
                weight_type_ = WeightType::UNIFORM;
                set_normalize_weight();
                compute_average(mean_res);
                convert_pose_to_vector(mean_res,mean);
                compute_covariance(cov_res,mean);
                if(do_reset)
                    reset();
            };

            Pose w_ave_compute(Pose& mean_res,WeightType type=WeightType::MAHALANOBIS, bool do_reset=true)
            {
                Pose err;
                weight_type_=type;
                set_normalize_weight();
                compute_average(mean_res);
                compute_err(err,mean_res);
                if(do_reset)
                    reset();
                return err;
            }

        private:
            void set_normalize_weight();

            void compute_average(Pose& result);

            void compute_covariance(PoseCovariance& result, PoseMeasure mean);
            
            void convert_pose_to_vector(Pose pose,PoseMeasure& vect);

            void convert_vector_to_pose(Pose& pose,PoseMeasure vect);
            
            void normalize_quaternion(Quaternion& quat);

            void compute_err(Pose& err,Pose mean);



            void reset()
            {
                measures_.clear();
                covariances_.clear();
                weights_.clear();
            };


            std::list<PoseMeasure> measures_;

            std::list<PoseCovariance> covariances_;

            std::list<double> weights_;

            MeasureType measure_type_;

            WeightType weight_type_;


            
    };
};
