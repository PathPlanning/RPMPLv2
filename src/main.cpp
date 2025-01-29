// ./src/main ../../STRRT/scene_task.json ./ ../../STRRT/strrt_config.json
#include "Scenario.h"

#include "ConfigurationReader.h"
#include "DRGBT.h"
#include "RealVectorSpaceHPPFCL.h"
#include "RealVectorSpaceState.h"
#include "config_read_writer/config_read.hpp"
#include "config_read_writer/ResultsWriter.hpp"
#include "xArm6.h"
#include <chrono>
#include <cmath>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iostream>
#include <rapidjson/document.h>
#include <rapidjson/istreamwrapper.h>
#include <rapidjson/stringbuffer.h>
#include <rapidjson/writer.h>
#include <string>
#include <unordered_map>
#include <vector>
#include <Eigen/Core>



auto read_file(std::string path) -> std::string {
    // reads file from path and stores it in string
    constexpr auto read_size = std::size_t(4096);
    auto stream = std::ifstream(path);
    stream.exceptions(std::ios_base::badbit);

    if (not stream) {
        throw std::ios_base::failure("file does not exist");
    }

    auto out = std::string();
    auto buf = std::string(read_size, '\0');
    while (stream.read(&buf[0], read_size)) {
        out.append(buf, 0, stream.gcount());
    }
    out.append(buf, 0, stream.gcount());
    return out;
}

Eigen::VectorXf vectorDoubleToEigenVector(std::vector<double> vec) {
    Eigen::VectorXf eigenVec(vec.size());
    for (size_t i = 0; i < vec.size(); ++i) {
        eigenVec[i] = vec[i];
    }
    return eigenVec;
}

void parse_my_args(int argc, char **argv, std::string &path_to_scene_json, std::string &path_to_result_folder, std::string &path_to_strrt_config_json, int& random_seed)
{
    if (argc < 5)
    {
        std::cerr << "Usage: " << argv[0] << " <path_to_scene_json> <path_to_result_folder> <path_to_strrt_config_json> <random_seed>" << std::endl;
        exit(EXIT_FAILURE);
    }

    path_to_scene_json = argv[1];
    path_to_result_folder = argv[2];
    path_to_strrt_config_json = argv[3];
    random_seed = atoi(argv[4]);
}
void check_args(const std::string &path_to_scene_json, const std::string &path_to_result_folder,
                const std::string &path_to_strrt_config_json) {
    if (!std::ifstream(path_to_scene_json) || !std::ifstream(path_to_strrt_config_json) ||
        !std::filesystem::is_directory(path_to_result_folder)) {
        throw std::runtime_error("Invalid file or directory path");
    }
}

int main(int argc, char **argv) {
    std::chrono::high_resolution_clock::time_point start_time = std::chrono::high_resolution_clock::now();
    std::vector<std::shared_ptr<base::State>> result_path;
    const std::string project_path{"./RPMPLv2"};
    ConfigurationReader::initConfiguration(project_path);

    std::string path_to_scene_json;
    std::string path_to_result_folder;
    std::string path_to_strrt_config_json;
    int random_seed;
    parse_my_args(argc, argv, path_to_scene_json, path_to_result_folder, path_to_strrt_config_json,random_seed);
    check_args(path_to_scene_json, path_to_result_folder, path_to_strrt_config_json);
    std::srand(random_seed);
    
    MDP::ResultsWriter::get_instance().setup(path_to_scene_json, path_to_strrt_config_json, path_to_result_folder, MDP::ResultsWriter::PlannerType::DRGBT);
    MDP::ResultsWriter::get_instance().algorithm_start();

    MDP::ConfigReader SceneTask(path_to_scene_json);

    std::ifstream config_file(path_to_strrt_config_json);
    rapidjson::IStreamWrapper config_wrapper(config_file);
    rapidjson::Document planner_parsed_config;
    planner_parsed_config.ParseStream(config_wrapper);
    
    // TODO: optimise collision manager construction and dont construct in space validity checker, or find another way to get joint limits
    std::shared_ptr<MDP::CollisionManager> collision_manager = std::make_shared<MDP::CollisionManager>(SceneTask.get_scene_task());
    std::vector<std::pair<float, float>> joint_limits = collision_manager->get_planned_robot_limits();
    int low_bound_frame=0;
    // int low_bound_frame  = collision_manager->get_goal_frame_low_bound();
    double low_bound_time =  (double)low_bound_frame/ (double)SceneTask.get_scene_task().fps;

    for (int joint_ind = 0; joint_ind < SceneTask.get_scene_task().robot_joint_count; joint_ind++)
    {
        assert(joint_limits[joint_ind].first != joint_limits[joint_ind].second);
    }


    int max_attempt = 1;
    std::vector<float> alg_times{};
    std::vector<float> iter_times{};
    std::vector<float> path_lengths{};

    DRGBTConfig::MAX_PLANNING_TIME = 1000000000000000;

    if (DRGBTConfig::STATIC_PLANNER_TYPE == planning::PlannerType::RGBMTStar) {
        RGBMTStarConfig::TERMINATE_WHEN_PATH_IS_FOUND = true;
    }

    MDP::ResultsWriter::get_instance().config_end();
    bool result;
    for (int attempt = 0; attempt < max_attempt; attempt++) {
        std::shared_ptr<env::Environment> env = std::make_shared<env::Environment>();
        std::string json = read_file(path_to_scene_json);
        rapidjson::Document document;
        document.Parse(json.c_str());
        // env->parse_json_document(document);
        std::shared_ptr<robots::xArm6> robot =
            std::make_shared<robots::xArm6>(SceneTask.get_scene_task().robot_urdf_path, 0, false);

        std::vector<double> temp_vel(SceneTask.get_scene_task().robot_joint_max_velocity);
        std::vector<float> max_speed_float(temp_vel.begin(), temp_vel.end());
        Eigen::Map<Eigen::VectorXf> max_speed_eigen(max_speed_float.data(),max_speed_float.size());
        robot->setMaxVel(max_speed_eigen);
        env->setRobotMaxVel(robot->getMaxVel(0)); // Only velocity of the first joint matters

        std::vector<double> temp_caps(SceneTask.get_scene_task().robot_capsules_radius);
        std::vector<float> robot_capsules_radius_float(temp_caps.begin(), temp_caps.end());
        robot->setCapsulesRadius(robot_capsules_radius_float);
        // env->setBaseRadius(std::max(ss->robot->getCapsuleRadius(0), ss->robot->getCapsuleRadius(1)));

        std::shared_ptr<base::RealVectorSpaceHPPFCL> ss = std::make_shared<base::RealVectorSpaceHPPFCL>(
            SceneTask.get_scene_task().robot_joint_count, robot, env, SceneTask.get_scene_task());
        std::shared_ptr<base::State> q_start = std::make_shared<base::RealVectorSpaceState>(
            vectorDoubleToEigenVector(SceneTask.get_scene_task().start_configuration));
        std::shared_ptr<base::State> q_goal = std::make_shared<base::RealVectorSpaceState>(
            vectorDoubleToEigenVector(SceneTask.get_scene_task().end_configuration));
        std::unique_ptr<planning::AbstractPlanner> planner =
            std::make_unique<planning::drbt::DRGBT>(ss, q_start, q_goal, low_bound_time);
        

        // LOG(INFO) << "Using scenario: " << "./scene.json";
        // LOG(INFO) << "Environment parts: " << env->getNumObjects();
        // LOG(INFO) << "Number of DOFs: " << ss->num_dimensions;
        // LOG(INFO) << "State space type: " << ss->getStateSpaceType();
        // LOG(INFO) << "Start: " << q_start;
        // LOG(INFO) << "Goal: " << q_goal;

        result = planner->solve();

        // LOG(INFO) << planner->getPlannerType() << " planning finished with " << (result ? "SUCCESS!" : "FAILURE!");
        // LOG(INFO) << "Number of iterations: " << planner->getPlannerInfo()->getNumIterations();
        // LOG(INFO) << "Algorithm time: " << planner->getPlannerInfo()->getPlanningTime() << " [s]";
        // LOG(INFO) << "Task 1 interrupted: " << (planner->getPlannerInfo()->getTask1Interrupted() ? "true" : "false");
        // LOG(INFO) << "Planner data is saved at: "
        //           << "./drgbt_test.log";
        // planner->outputPlannerData("./drgbt_test.log");

        float path_length{0};
        result_path = planner->getPath();
        if (result) {
            break;
        }
    }
    MDP::ResultsWriter::get_instance().solver_end();



    rapidjson::Document data_to_export;

    data_to_export.SetObject();
    rapidjson::Document::AllocatorType &allocator = data_to_export.GetAllocator();
    data_to_export.AddMember("has_result", result, MDP::ResultsWriter::get_instance().get_json_allocator());


    rapidjson::Value path_array(rapidjson::kArrayType);
    
    std::vector<MDP::ResultsWriter::PathState> result_path_pathstate;
    for (const std::shared_ptr<base::State> &state : result_path)
    {
        double point_time = state->getTime();
        std::vector<double> point_array;
        for (double v : state->getCoord()) {
            point_array.push_back(v);
        }
        result_path_pathstate.emplace_back(point_array, point_time);
    }

    rapidjson::Value path_json;
    MDP::ResultsWriter::get_instance().convert_path_to_json(result_path_pathstate,path_json);
    data_to_export.AddMember("final_path", path_json, MDP::ResultsWriter::get_instance().get_json_allocator());
    data_to_export.AddMember("path", path_array, allocator);

    MDP::ResultsWriter::get_instance().save_json(std::to_string(std::chrono::system_clock::to_time_t(std::chrono::system_clock::now())), data_to_export,random_seed);

    return 0;
}
