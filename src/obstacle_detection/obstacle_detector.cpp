/// COMPONENT
#include <generic_obstacle_detection/detector.h>
#include <generic_obstacle_detection/low_pass_filter.h>
#include <generic_obstacle_detection/heat_calculator.h>

/// PROJECT
#include <csapex/model/node.h>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex_point_cloud/point_cloud_message.h>
#include <csapex/profiling/timer.h>
#include <csapex_ros/tf_listener.h>
#include <csapex_point_cloud/indeces_message.h>
#include <csapex_transform/transform_message.h>
#include <csapex/profiling/interlude.hpp>

/// SYSTEM
#include <pcl/PointIndices.h>
#include <pcl/filters/extract_indices.h>

using namespace csapex;
using namespace csapex::connection_types;
using namespace od;

namespace od
{


namespace impl {

template <class PointT>
struct Impl;
}


class ObstacleDetector : public Node
{
public:
    void setup(NodeModifier& modifier) override
    {
        in_cloud_ = modifier.addInput<PointCloudMessage>("Cloud");
        in_transform_ = modifier.addOptionalInput<TransformMessage>("Transform (opt.)");

        output_cloud_heat_ = modifier.addOutput<PointCloudMessage>("Heat Cloud");
        output_cloud_classified_ = modifier.addOutput<PointCloudMessage>("Classified Cloud");
        output_cloud_obstacle_ = modifier.addOutput<PointCloudMessage>("Obstacle Cloud");
        output_cloud_floor_= modifier.addOutput<PointCloudMessage>("Floor Cloud");
        output_cloud_obstacle_original_ = modifier.addOutput<PointCloudMessage>("Obstacle Cloud (as in)");

        output_cloud_obstacle_indices_ = modifier.addOutput<PointIndecesMessage>("Obstacle Indices");
        output_cloud_floor_indices_ = modifier.addOutput<PointIndecesMessage>("Floor Indices");

        output_transform_ = modifier.addOutput<TransformMessage>("Transform");
    }

    void setupParameters(Parameterizable& params) override
    {
        std::function<void(csapex::param::Parameter*)> cb = [this](csapex::param::Parameter*) { updateImpl(); };

        std::map<std::string, int> methods = {
            std::make_pair("Naive", (int) HeatCalculatorMethod::NAIVE),
            std::make_pair("Parts", (int) HeatCalculatorMethod::PARTS),
            std::make_pair("Linear", (int) HeatCalculatorMethod::LINEAR),
            std::make_pair("Approximated", (int) HeatCalculatorMethod::APPROX)
        };
        csapex::param::Parameter::Ptr method = csapex::param::ParameterFactory::declareParameterSet("method", methods, (int) HeatCalculatorMethod::NAIVE);
        params.addParameter(method, cb);


        std::map<std::string, int> types = {
            std::make_pair("Slope", (int) HeatType::SLOPE),
            std::make_pair("Slope Difference", (int) HeatType::SLOPE_DIFF),
            std::make_pair("Curvature", (int) HeatType::CURVATURE),
            std::make_pair("SBC 15", (int) HeatType::SBC15),
            std::make_pair("IROS 2016", (int) HeatType::IROS16),
            std::make_pair("PATSY (old)", (int) HeatType::PATSY_OLD)
        };
        csapex::param::Parameter::Ptr type = csapex::param::ParameterFactory::declareParameterSet("heat_type", types, (int) HeatType::IROS16);
        params.addParameter(type, cb);

        params.addParameter(csapex::param::ParameterFactory::declareBool("transform cloud", false), do_transform_);
        params.addParameter(csapex::param::ParameterFactory::declareAngle("sensor_orientation", 0.0));

        csapex::param::ParameterPtr use_lowpass = csapex::param::ParameterFactory::declareBool("use low pass filter", true);
        params.addParameter(use_lowpass, cb);

        auto condition = [use_lowpass]() { return use_lowpass->as<bool>(); };

        params.addConditionalParameter(csapex::param::ParameterFactory::declareRange("lowpass/height", 0.0, 1.0, 0.1, 0.001),
                                       condition, cb);
        params.addConditionalParameter(csapex::param::ParameterFactory::declareRange("lowpass/max_distance", 0.0, 1.0, 0.1, 0.001),
                                       condition, cb);
        params.addConditionalParameter(csapex::param::ParameterFactory::declareRange("lowpass/zlimit", 0.0, 1.0, 0.1, 0.001),
                                       condition, cb);

        params.addParameter(csapex::param::ParameterFactory::declareRange("heat/scale/height", 0.0, 10000.0, 5000.0, 0.1), cb);
        params.addParameter(csapex::param::ParameterFactory::declareRange("heat/scale/intensity", 0.0, 10000.0, 5000.0, 0.1), cb);
        params.addParameter(csapex::param::ParameterFactory::declareInterval("heat/inverval/depth", -2.0, 2.0, -2.0, 2.0, 0.01), cb);
        params.addParameter(csapex::param::ParameterFactory::declareInterval("heat/inverval/intensity", 0.0, 1024.0, 0.0, 1024.0, 0.1), cb);
        params.addParameter(csapex::param::ParameterFactory::declareRange("heat/zlimit", 0.0, 2.0, 0.5, 0.001), cb);
        params.addParameter(csapex::param::ParameterFactory::declareRange("heat/lookout", 0.01, 1.5, 0.1, 0.01), cb);
        params.addParameter(csapex::param::ParameterFactory::declareRange("heat/min_length", 0.01, 1.0, 0.01, 0.01), cb);
        params.addParameter(csapex::param::ParameterFactory::declareRange("heat/initial_offset", 1, 24, 1, 1), cb);
        params.addParameter(csapex::param::ParameterFactory::declareRange("heat/skip", 1, 24, 1, 1), cb);
        params.addParameter(csapex::param::ParameterFactory::declareRange("heat/step", 1, 24, 1, 1), cb);
        params.addParameter(csapex::param::ParameterFactory::declareRange("heat/max_dist", 1, 512, 128, 1), cb);

        params.addParameter(csapex::param::ParameterFactory::declareRange("heat/threshold", 0.1, 10000.0, 5000.0, 0.1), cb);
    }

    void process()
    {
        PointCloudMessage::ConstPtr msg(msg::getMessage<PointCloudMessage>(in_cloud_));

        boost::apply_visitor (PointCloudMessage::Dispatch<ObstacleDetector>(this, msg), msg->value);
    }

    template <class PointT>
    void inputCloud(typename pcl::PointCloud<PointT>::ConstPtr in)
    {
        double sensor_orientation = readParameter<double>("sensor_orientation");

        if(!do_transform_) {
            impl::Impl<PointT>::inputCloud(this, in, sensor_orientation);

        } else {
            NAMED_INTERLUDE(transform_to_base_link);

            tf::Transform trafo;
            std::string target_frame = "/base_link";
            std::string source_frame = in->header.frame_id;

            if(msg::isConnected(in_transform_)) {
                TransformMessage::ConstPtr trafo_msg = msg::getMessage<TransformMessage>(in_transform_);
                trafo = trafo_msg->value;
                source_frame = trafo_msg->frame_id;
                target_frame = trafo_msg->child_frame;

            } else {
                NAMED_INTERLUDE(lookup);

                LockedTFListener l = TFListener::getLocked();
                apex_assert(l.l);
                auto listener = l.l->tfl;
                apex_assert(listener);
                tf::TransformListener& tfl = *listener;

                ros::Time time;
                time.fromNSec(in->header.stamp * 1000);

                tf::StampedTransform t;

                if(tfl.canTransform(target_frame, source_frame, time)) {
                    tfl.lookupTransform(target_frame, source_frame, time, t);
                    node_modifier_->setNoError();
                } else {
                    if(tfl.canTransform(target_frame, source_frame, ros::Time(0))) {
                        node_modifier_->setWarning("cannot transform, using latest transform");
                        tfl.lookupTransform(target_frame, source_frame, ros::Time(0), t);
                    } else {
                        node_modifier_->setError("cannot transform at all...");
                        return;
                    }
                }

                trafo = t;
            }

            double roll = 0, pitch = 0, yaw = 0;
            tf::Matrix3x3 rot_mat(trafo.getRotation());
            rot_mat.getRPY(roll, pitch, yaw);

            tf::Vector3 z_base = trafo * tf::Vector3(0.0, 0.0, 1.0);

            sensor_orientation = std::atan2(z_base.y(), z_base.x());


            typename pcl::PointCloud<PointT>::Ptr in_transformed(new pcl::PointCloud<PointT>);
            in_transformed->header = in->header;
            in_transformed->width = in->width;
            in_transformed->height = in->height;
            in_transformed->is_dense = in->is_dense;
            in_transformed->header.frame_id = target_frame;

            std::size_t N = in->points.size();
            in_transformed->points.resize(N);

            auto* inP = &in->points[0];
            auto* outP = &in_transformed->points[0];

            {
                NAMED_INTERLUDE(multiply);
                for(std::size_t i = 0; i < N; ++i, ++inP, ++outP) {
                    const PointT& pt_in = *inP;
                    PointT& pt_out = *outP;
                    pt_out = pt_in;

                    tf::Vector3 tfd = trafo * tf::Vector3(pt_in.x, pt_in.y, pt_in.z);
                    pt_out.x = tfd.x();
                    pt_out.y = tfd.y();
                    pt_out.z = tfd.z();
                }
            }

            interlude_transform_to_base_link.reset();

            impl::Impl<PointT>::inputCloud(this, in_transformed, sensor_orientation);

            TransformMessage::Ptr trafo_out = std::make_shared<TransformMessage>(target_frame, source_frame);
            trafo_out->value = trafo;
            msg::publish(output_transform_, trafo_out);
        }
    }

private:
    void updateImpl()
    {
        if(p_impl) {
            p_impl->update(this);
        }
    }

public:
    Input* in_cloud_;
    Input* in_transform_;

    Output* output_cloud_heat_;
    Output* output_cloud_classified_;
    Output* output_cloud_obstacle_;
    Output* output_cloud_floor_;
    Output* output_cloud_obstacle_original_;
    Output* output_cloud_obstacle_indices_;
    Output* output_cloud_floor_indices_;
    Output* output_transform_;

    bool do_transform_;

    struct ImplInterface {
        virtual ~ImplInterface() {}

        virtual void update(od::ObstacleDetector* instance) = 0;
    };

    std::shared_ptr<ImplInterface> p_impl;
};


namespace impl {

template <class PointT>
struct Impl : ObstacleDetector::ImplInterface
{
    virtual void update(od::ObstacleDetector* instance) override
    {
        heat_calculator_.method_ = static_cast<HeatCalculatorMethod>(instance->readParameter<int>("method"));
        heat_calculator_.heat_type_ = static_cast<HeatType>(instance->readParameter<int>("heat_type"));

        use_low_pass_ = instance->readParameter<bool>("use low pass filter");

        low_pass_.alpha_ = instance->readParameter<double>("lowpass/height");
        low_pass_.max_distance_ = instance->readParameter<double>("lowpass/max_distance");
        low_pass_.zlimit_ = instance->readParameter<double>("lowpass/zlimit");

        heat_calculator_.max_dist = instance->readParameter<int>("heat/max_dist");

        heat_calculator_.scale_h_ = instance->readParameter<double>("heat/scale/height");
        heat_calculator_.scale_i_ = instance->readParameter<double>("heat/scale/intensity");
        heat_calculator_.hzlimit_ = instance->readParameter<double>("heat/zlimit");
        heat_calculator_.lookout_ = instance->readParameter<double>("heat/lookout");
        heat_calculator_.min_length_ = instance->readParameter<double>("heat/min_length");
        heat_calculator_.initial_offset_ = instance->readParameter<int>("heat/initial_offset");
        heat_calculator_.skip_ = instance->readParameter<int>("heat/skip");
        heat_calculator_.step_ = instance->readParameter<int>("heat/step");

        auto pair_depth = instance->readParameter<std::pair<double, double>>("heat/inverval/depth");
        heat_calculator_.interval_min_ = pair_depth.first;
        heat_calculator_.interval_max_ = pair_depth.second;
        auto pair_intensity = instance->readParameter<std::pair<double, double>>("heat/inverval/intensity");
        heat_calculator_.intensity_min_ = pair_intensity.first;
        heat_calculator_.intensity_max_ = pair_intensity.second;

        detector_.heat_threshold_ = instance->readParameter<double>("heat/threshold");
    }

    static void inputCloud(od::ObstacleDetector* instance, typename pcl::PointCloud<PointT>::ConstPtr cloud,
                           const double sensor_orientation)
    {
        std::shared_ptr<Impl<PointT>> p_impl = std::dynamic_pointer_cast<Impl<PointT>>(instance->p_impl);
        if(!p_impl) {
            p_impl = std::make_shared<Impl<PointT>>();
            instance->p_impl = p_impl;
            p_impl->update(instance);
        }

        p_impl->process(instance, cloud, sensor_orientation);
    }

    void process(od::ObstacleDetector* instance, typename pcl::PointCloud<PointT>::ConstPtr cloud,
                 const double sensor_orientation)
    {
        NAMED_INTERLUDE_INSTANCE(instance, input);

        const pcl::PointCloud<PointT>& in = *cloud;
        const std::vector<PointT, Eigen::aligned_allocator<PointT> >& points = in.points;

        std::size_t N = points.size();

        std::vector<float> heat(N, 0);
        std::vector<Point> data(N, Point());

        {
            NAMED_INTERLUDE_INSTANCE(instance, transform_data);
            transformData(cloud, data, sensor_orientation);
        }

        apex_assert(data.size() == N);


        if(use_low_pass_){
            NAMED_INTERLUDE_INSTANCE(instance, lowpass);
            low_pass_.lowPass(cloud, data);
        }
        interlude_input.reset();

        apex_assert(data.size() == N);

        {
            NAMED_INTERLUDE_INSTANCE(instance, heat);
            heat_calculator_.calculate(cloud, data, heat);
            //calculateHeatParts(cloud, data, heat);
        }

        apex_assert(data.size() == N);


        ///// CLASSIFY
        pcl::PointIndicesPtr obstacle_indices(new pcl::PointIndices);
        pcl::PointIndicesPtr floor_indices(new pcl::PointIndices);

        {
            NAMED_INTERLUDE_INSTANCE(instance, classify);
            detector_.classify(cloud, data, heat, *obstacle_indices, *floor_indices);
        }

        // publish indices
        PointIndecesMessage::Ptr o_indices_msg(new PointIndecesMessage);
        o_indices_msg->value = obstacle_indices;
        msg::publish(instance->output_cloud_obstacle_indices_, o_indices_msg);

        PointIndecesMessage::Ptr f_indices_msg(new PointIndecesMessage);
        f_indices_msg->value = floor_indices;
        msg::publish(instance->output_cloud_floor_indices_, f_indices_msg);


        NAMED_INTERLUDE_INSTANCE(instance, output);

        if(msg::isConnected(instance->output_cloud_heat_)) {
            NAMED_INTERLUDE_INSTANCE(instance, output_heat);
            PointCloudMessage::Ptr msg_heat(new PointCloudMessage(cloud->header.frame_id, cloud->header.stamp));
            msg_heat->value = detector_.generateHeatCloud(cloud, heat);
            msg::publish(instance->output_cloud_heat_, msg_heat);
        }

        if(msg::isConnected(instance->output_cloud_classified_)) {
            NAMED_INTERLUDE_INSTANCE(instance, output_classified);
            PointCloudMessage::Ptr msg_class(new PointCloudMessage(cloud->header.frame_id, cloud->header.stamp));
            msg_class->value = detector_.generateClassifiedCloud(cloud, *obstacle_indices, *floor_indices);
            msg::publish(instance->output_cloud_classified_, msg_class);
        }

        if(msg::isConnected(instance->output_cloud_obstacle_)){
            NAMED_INTERLUDE_INSTANCE(instance, output_obstacles);

            bool c_labeled = msg::isConnected(instance->output_cloud_obstacle_);
            bool c_original = msg::isConnected(instance->output_cloud_obstacle_original_);

            if(c_original || c_labeled) {
                typename pcl::PointCloud<PointT>::Ptr output_obstacles(new pcl::PointCloud<PointT>);
                output_obstacles->header = cloud->header;

                pcl::ExtractIndices<PointT> extract;
                extract.setInputCloud (cloud);
                extract.setNegative(false);
                extract.setIndices (obstacle_indices);
                extract.filter (*output_obstacles);

                if(c_original) {
                    PointCloudMessage::Ptr msg_obstacles_original(new PointCloudMessage(cloud->header.frame_id, cloud->header.stamp));
                    msg_obstacles_original->value = output_obstacles;
                    msg::publish(instance->output_cloud_obstacle_original_, msg_obstacles_original);
                }

                if(c_labeled) {
                    typename pcl::PointCloud<pcl::PointXYZL>::Ptr output_obstacles_labeled(new pcl::PointCloud<pcl::PointXYZL>);

                    std::size_t n = output_obstacles->points.size();

                    output_obstacles_labeled->header = output_obstacles->header;
                    output_obstacles_labeled->width = output_obstacles->width;
                    output_obstacles_labeled->height = output_obstacles->height;
                    output_obstacles_labeled->points.resize(n);

                    const PointT* src = &output_obstacles->points[0];
                    pcl::PointXYZL* dst = &output_obstacles_labeled->points[0];

                    for(std::size_t i = 0; i < n; ++i,++src,++dst) {
                        for(std::size_t j = 0; j < 3; ++j) {
                            dst->data[j] = src->data[j];
                        }
                        dst->label = 1;
                    }

                    PointCloudMessage::Ptr msg_obstacles_original(new PointCloudMessage(cloud->header.frame_id, cloud->header.stamp));
                    msg_obstacles_original->value = output_obstacles_labeled;
                    msg::publish(instance->output_cloud_obstacle_, msg_obstacles_original);
                }
            }
        }

        if(msg::isConnected(instance->output_cloud_floor_)){
            NAMED_INTERLUDE_INSTANCE(instance, output_floor);


            typename pcl::PointCloud<PointT>::Ptr output_floor(new pcl::PointCloud<PointT>);
            output_floor->header = cloud->header;

            pcl::ExtractIndices<PointT> extract;
            extract.setInputCloud (cloud);
            extract.setIndices (floor_indices);
            extract.filter (*output_floor);

            PointCloudMessage::Ptr msg_floor(new PointCloudMessage(cloud->header.frame_id, cloud->header.stamp));
            msg_floor->value = output_floor;
            msg::publish(instance->output_cloud_floor_, msg_floor);
        }
    }

private:
    void transformData(typename pcl::PointCloud<PointT>::ConstPtr cloud,
                       std::vector<Point>& data,
                       const double sensor_orientation)
    {
        double theta = sensor_orientation;
        while(theta < -M_PI) {
            theta += 2 * M_PI;
        }
        while(theta >= M_PI) {
            theta -= 2 * M_PI;
        }

        if(std::abs(theta) < 1e-3) {
            transformData(cloud, data);
        } else if(std::abs(theta - M_PI) < 1e-1) {
            transformDataFlipX(cloud, data);
        } else if(std::abs(theta + M_PI) < 1e-1) {
            transformDataFlipX(cloud, data);
        } else if(std::abs(theta - M_PI/2) < 1e-1) {
            transformDataLeft(cloud, data);
        } else if(std::abs(theta + M_PI/2) < 1e-1) {
            transformDataRight(cloud, data);
        } else {
            transformDataArbitrary(cloud, data, theta);
        }
    }

    void transformData(typename pcl::PointCloud<PointT>::ConstPtr cloud,
                       std::vector<Point>& data)
    {
        const std::vector<PointT, Eigen::aligned_allocator<PointT> >& points = cloud->points;

        std::size_t rows = cloud->height;
        std::size_t cols = cloud->width;


        auto* dataP = &data[0];
        for(std::size_t col = 0; col < cols; ++col) {
            auto* orig_Ptr = &points[col];

            for(std::size_t row = 0; row < rows; ++row, ++dataP) {
                const auto& pt = *orig_Ptr;

                auto& dat = *dataP;
                assign(dat, pt);
                dat.x = pt.x;
                dat.y = pt.y;
                dat.z = pt.z;

                orig_Ptr += cols;
            }
        }
    }

    void transformDataFlipX(typename pcl::PointCloud<PointT>::ConstPtr cloud,
                            std::vector<Point>& data)
    {
        const std::vector<PointT, Eigen::aligned_allocator<PointT> >& points = cloud->points;

        std::size_t rows = cloud->height;
        std::size_t cols = cloud->width;


        auto* dataP = &data[0];
        for(std::size_t col = 0; col < cols; ++col) {
            auto* orig_Ptr = &points[col];

            for(std::size_t row = 0; row < rows; ++row, ++dataP) {
                const auto& pt = *orig_Ptr;

                auto& dat = *dataP;
                assign(dat, pt);
                dat.x = -pt.x;
                dat.y = -pt.y;
                dat.z = pt.z;

                orig_Ptr += cols;
            }
        }
    }


    void transformDataLeft(typename pcl::PointCloud<PointT>::ConstPtr cloud,
                           std::vector<Point>& data)
    {
        const std::vector<PointT, Eigen::aligned_allocator<PointT> >& points = cloud->points;

        std::size_t rows = cloud->height;
        std::size_t cols = cloud->width;


        auto* dataP = &data[0];
        for(std::size_t col = 0; col < cols; ++col) {
            auto* orig_Ptr = &points[col];

            for(std::size_t row = 0; row < rows; ++row, ++dataP) {
                const auto& pt = *orig_Ptr;

                auto& dat = *dataP;
                assign(dat, pt);
                dat.x = -pt.y;
                dat.y = pt.x;
                dat.z = pt.z;

                orig_Ptr += cols;
            }
        }
    }


    void transformDataRight(typename pcl::PointCloud<PointT>::ConstPtr cloud,
                            std::vector<Point>& data)
    {
        const std::vector<PointT, Eigen::aligned_allocator<PointT> >& points = cloud->points;

        std::size_t rows = cloud->height;
        std::size_t cols = cloud->width;


        auto* dataP = &data[0];
        for(std::size_t col = 0; col < cols; ++col) {
            auto* orig_Ptr = &points[col];

            for(std::size_t row = 0; row < rows; ++row, ++dataP) {
                const auto& pt = *orig_Ptr;

                auto& dat = *dataP;
                assign(dat, pt);
                dat.x = pt.y;
                dat.y = -pt.x;
                dat.z = pt.z;

                orig_Ptr += cols;
            }
        }
    }

    void transformDataArbitrary(typename pcl::PointCloud<PointT>::ConstPtr cloud,
                                std::vector<Point>& data,
                                const double sensor_orientation)
    {
        const std::vector<PointT, Eigen::aligned_allocator<PointT> >& points = cloud->points;

        std::size_t rows = cloud->height;
        std::size_t cols = cloud->width;

        double c = std::cos(-sensor_orientation);
        double s = std::sin(-sensor_orientation);

        auto* dataP = &data[0];
        for(std::size_t col = 0; col < cols; ++col) {
            auto* orig_Ptr = &points[col];

            for(std::size_t row = 0; row < rows; ++row, ++dataP) {
                const auto& pt = *orig_Ptr;

                auto& dat = *dataP;
                assign(dat, pt);
                dat.x = c * pt.x - s * pt.y;
                dat.y = s * pt.x + c * pt.y;
                dat.z = pt.z;

                orig_Ptr += cols;
            }
        }
    }

    template <typename T>
    static void assign(Point& dat, const T& pt)
    {
        dat.intensity = 0;
    }
    static void assign(Point& dat, const pcl::PointXYZI& pt)
    {
        dat.intensity = pt.intensity;
    }

    bool use_low_pass_;
    LowPassFilter<PointT> low_pass_;
    HeatCalculator<PointT> heat_calculator_;
    ObstacleDetector3D<PointT> detector_;
};

}

}

CSAPEX_REGISTER_CLASS(od::ObstacleDetector, Node)
