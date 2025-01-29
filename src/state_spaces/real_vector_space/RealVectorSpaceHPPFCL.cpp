
#include "RealVectorSpaceHPPFCL.h"
#include "RealVectorSpaceConfig.h"
#include "xArm6.h"
// #include <glog/log_severity.h>
// #include <glog/logging.h>

base::RealVectorSpaceHPPFCL::~RealVectorSpaceHPPFCL() {}

base::RealVectorSpaceHPPFCL::RealVectorSpaceHPPFCL(size_t num_dimensions_,
                                                   const std::shared_ptr<robots::AbstractRobot> robot_,
                                                   const std::shared_ptr<env::Environment> env_,
                                                   const MDP::ConfigReader::SceneTask scene_task)
    : RealVectorSpace(num_dimensions_, robot_, env_),
      collision_manager(std::make_shared<MDP::CollisionManager>(scene_task)) {
    this->setStateSpaceType(base::StateSpaceType::RealVectorSpaceHPPFCL);
}

bool base::RealVectorSpaceHPPFCL::isValid(const std::shared_ptr<base::State> q) {
    fcl::DefaultCollisionData<float> collision_data{};
    std::vector<double> pos(q->getCoord().data(), q->getCoord().data() + q->getCoord().rows() * q->getCoord().cols());
    float time = this->env->getTime();
    return !this->collision_manager->check_collision(pos, time);
}

// Return minimal distance from robot in configuration 'q' to obstacles
// Compute minimal distance from each robot's link in configuration 'q' to obstacles, i.e., compute distance profile
// function Moreover, set 'd_c', 'd_c_profile', and corresponding 'nearest_points' for the configuation 'q' If
// 'compute_again' is true, the new distance profile will be computed again!
float base::RealVectorSpaceHPPFCL::computeDistance(const std::shared_ptr<base::State> q, bool compute_again) {
    if (!compute_again && q->getDistance() > 0 && q->getIsRealDistance())
        return q->getDistance();

    float d_c{INFINITY};
    std::vector<float> d_c_profile(this->robot->getNumDOFs(), std::numeric_limits<float>::max());

    bool is_collided = false;
    std::vector<double> pos(q->getCoord().data(), q->getCoord().data() + q->getCoord().rows() * q->getCoord().cols());
    std::vector<MDP::CollisionManager::robot_link_distance_to_obj> distances =
        this->collision_manager->get_distances(pos, this->env->getTime(), is_collided);

    if (is_collided) // The collision occurs
    {
        q->setDistance(0);
        q->setDistanceProfile(d_c_profile);
        q->setIsRealDistance(true);
        q->setNearestPoints(nullptr);
        return 0;
    }

    // std::shared_ptr<std::vector<Eigen::MatrixXf>> nearest_points{
    //     std::make_shared<std::vector<Eigen::MatrixXf>>(std::vector<Eigen::MatrixXf>(
    //         distances[0].neares_point_robot.size(), Eigen::MatrixXf(6, this->robot->getNumLinks())))};
    
    
    for (size_t i = 0; i < distances.size(); i++) {
        // std::cout<<distances[i].min_distance<<std::endl;
        d_c_profile[i+1] = distances[i].min_distance;
        // for (size_t j = 0; j < distances[i].neares_point_robot.size(); j++) {
        //     nearest_points->at(j).col(i+this->robot->getNumLinks()-distances.size()) << distances[i].neares_point_robot[j].cast<float>(),
        //         distances[i].neares_point_obj[j].cast<float>();
        // }
        d_c = std::min(d_c, d_c_profile[i]);
    }
    d_c_profile[distances.size()-1] = d_c_profile[distances.size()-2];//hardcode

    q->setDistance(d_c);
    q->setDistanceProfile(d_c_profile);
    q->setIsRealDistance(true);
    q->setNearestPoints(nullptr);

    return d_c;
}

bool base::RealVectorSpaceHPPFCL::check_robot_selfcollision(const std::shared_ptr<base::State> q1,
                                                            std::shared_ptr<base::State> & q2) {
    return false;
    // std::vector<float> pos2(q2->getCoord().data(),
    //                         q2->getCoord().data() + q2->getCoord().rows() * q2->getCoord().cols());

    float dist{this->getNorm(q1, q2)};
    std::shared_ptr<base::State> q_last_valid = q1;
    for (size_t num_check = 1; num_check <= 10; num_check++) {
        std::shared_ptr<base::State> q_temp = this->interpolateEdge(q1, q2, dist * num_check / 10, dist);
        std::vector<double> pos(q_temp->getCoord().data(),
                               q_temp->getCoord().data() + q_temp->getCoord().rows() * q_temp->getCoord().cols());

        bool is_collision = this->collision_manager->check_robot_selfcollision(pos);
        if (is_collision) {
            q2 = q_last_valid;
            return true;
        }
        q_last_valid = q_temp;
    }
    return false;
}
