#include "Environment.h"

env::Environment::Environment(const std::shared_ptr<env::Environment> env)
{
    objects = env->getObjects();
    WS_center = env->getWSCenter();
    WS_radius = env->getWSRadius();
    base_radius = env->getBaseRadius();
    robot_max_vel = env->getRobotMaxVel();
    ground_included = env->getGroundIncluded();
}
#include "rapidjson/document.h"
#include "rapidjson/stringbuffer.h"

env::Environment::Environment(const std::string &config_file_path, const std::string &root_path)
{
    YAML::Node node { YAML::LoadFile(root_path + config_file_path) };
    size_t num_added { 0 };

    try
    {
        for (size_t i = 0; i < node["environment"].size(); i++)
        {
            std::shared_ptr<env::Object> object;
            YAML::Node obstacle = node["environment"][i];
            if (obstacle["box"].IsDefined())
            {
                std::string label = "";
                if (obstacle["box"]["label"].IsDefined())
                    label = obstacle["box"]["label"].as<std::string>();

                if (label == "ground" && node["robot"]["ground_included"].as<size_t>() == 0)
                    continue;

                YAML::Node d = obstacle["box"]["dim"];
                YAML::Node p = obstacle["box"]["pos"];
                YAML::Node r = obstacle["box"]["rot"];
                YAML::Node min_dist_tol = obstacle["box"]["min_dist_tol"];
                
                fcl::Vector3f dim(d[0].as<float>(), d[1].as<float>(), d[2].as<float>());
                fcl::Vector3f pos(p[0].as<float>(), p[1].as<float>(), p[2].as<float>());
                fcl::Quaternionf rot = fcl::Quaternionf::Identity();

                if (r.IsDefined())
                    rot = fcl::Quaternionf(r[3].as<float>(), r[0].as<float>(), r[1].as<float>(), r[2].as<float>());

                object = std::make_shared<env::Box>(dim, pos, rot, label);

                if (min_dist_tol.IsDefined())
                    object->setMinDistTol(min_dist_tol.as<float>());

            }
            else if (obstacle["sphere"].IsDefined())
            {
                // TODO
            }
            else
                throw std::domain_error("Object type is wrong! ");

            objects.emplace_back(object);
            std::cout << "Added " << num_added++ << ". " << object;
        }

        if (node["robot"]["ground_included"].IsDefined())
            ground_included = node["robot"]["ground_included"].as<size_t>();
        else if (node["robot"]["type"].as<std::string>() == "xarm6")
        {
            ground_included = 0;
            throw std::domain_error("It is not defined whether ground is included! Thus, it will not be included. ");
        }

        if (node["robot"]["WS_center"].IsDefined())
        {
            for (size_t i = 0; i < 3; i++)
                WS_center(i) = node["robot"]["WS_center"][i].as<float>();

            WS_radius = node["robot"]["WS_radius"].as<float>();
        }
        else
            throw std::domain_error("Workspace center point is not defined! ");
    }
    catch (std::exception &e)
    {
        std::cout << e.what() << "\n";
    }
}

env::Environment::Environment()
{}
env::Environment::~Environment()
{
    objects.clear();
}

void env::Environment::addObject(const std::shared_ptr<env::Object> object, const fcl::Vector3f &velocity, 
                                 const fcl::Vector3f &acceleration) 
{
    object->setVelocity(velocity);
    object->setAcceleration(acceleration);
    objects.emplace_back(object);
}

// Remove object at 'idx' position
void env::Environment::removeObject(size_t idx)
{
    objects.erase(objects.begin() + idx);
}

// Remove objects from 'start_idx'-th object to 'end_idx'-th object
// If 'end_idx' is not passed, it will be considered as the last index in 'objects'
void env::Environment::removeObjects(int start_idx, int end_idx)
{
    if (end_idx == -1)
        end_idx = objects.size() - 1;
    
    for (int idx = end_idx; idx >= start_idx; idx--)
        objects.erase(objects.begin() + idx);
}

// Remove objects with label 'label' if 'with_label' is true (default)
// Remove objects NOT with label 'label' if 'with_label' is false
void env::Environment::removeObjects(const std::string &label, bool with_label)
{
    if (with_label)
    {
        for (int idx = objects.size()-1; idx >= 0; idx--)
        {
            if (objects[idx]->getLabel() == label)
                objects.erase(objects.begin() + idx);
        }
    }
    else
    {
        for (int idx = objects.size()-1; idx >= 0; idx--)
        {
            if (objects[idx]->getLabel() != label)
                objects.erase(objects.begin() + idx);
        }
    }        
}

// Remove all objects from the environment
void env::Environment::removeAllObjects()
{
    objects.clear();
}

// Check whether an object position 'pos' is valid when the object moves at 'vel' velocity
bool env::Environment::isValid(const Eigen::Vector3f &pos, float vel)
{
    float tol_radius {std::max(vel / robot_max_vel, base_radius)};

    if (ground_included > 0)
    {
        if ((pos - WS_center).norm() > WS_radius || pos.z() < 0 ||              // Out of workspace
            (pos.head(2).norm() < tol_radius && pos.z() < WS_center.z()) ||     // Surrounding of robot base
            (pos - WS_center).norm() < tol_radius)                              // Surrounding of robot base
            return false;
    }
    else
    {
        if ((pos - WS_center).norm() > WS_radius ||                                                     // Out of workspace
            (pos.head(2).norm() < tol_radius && pos.z() < WS_center.z() && pos.z() > -base_radius) ||   // Surrounding of robot base
            (pos - WS_center).norm() < tol_radius)                                                      // Surrounding of robot base
            return false;
    }

    return true;


}

// void env::Environment::updateEnvironment(float delta_time)
// {
//     fcl::Vector3f pos;
//     for (size_t i = 0; i < objects.size(); i++)
//     {
//         pos = objects[i]->getPosition();
//         pos(0) -= delta_time * objects[i]->getMaxVel();    // Move along x-axis
//         objects[i]->setPosition(pos);
//         std::cout << i << ". " << objects[i];
//     }
// }

void env::Environment::updateEnvironment(double delta_time)
{
    elapsed_time += delta_time;
    // for (auto& [key, value] : objects_coords){
    //     int frame = std::floor(elapsed_time*fps)*7;
    //     Eigen::Vector3f pos{value[0+frame],value[1+frame],value[2+frame]};
    //     fcl::Quaternionf q(value[6+frame],value[3+frame],value[4+frame],value[5+frame]);
    //     // uncomment if q is in euler
    //     // std::vector<float> rot(value.begin()+3+frame, value.begin()+5+frame);
    //     // q = Eigen::AngleAxisf(rot[0], Eigen::Vector3f::UnitX())
    //     //     * Eigen::AngleAxisf(rot[1], Eigen::Vector3f::UnitY())
    //     //     * Eigen::AngleAxisf(rot[2], Eigen::Vector3f::UnitZ());
    //     key->setQuatRotation(q);
    //     key->setPosition(pos);
    // }


    // std::cout << "-------------------------------------------------" << std::endl;
}


void env::Environment::parse_json_document(rapidjson::Document& doc){
    // fps  = doc["fps"].GetInt();
	// frame_count  = doc["frame_count"].GetInt();
	// const rapidjson::Value& obstacles_json = doc["obstacles"]; // Using a reference for consecutive access is handy and faster.
	// for (rapidjson::SizeType i = 0; i < obstacles_json.Size(); i++){ // rapidjson uses SizeType instead of size_t.
	// 	std::string label = obstacles_json[i]["name"].GetString();
    //     std::shared_ptr<env::Object> object;

    //     const rapidjson::Value& dimensions_json = obstacles_json[i]["dimensions"]; 
        
    //     fcl::Vector3f dim(dimensions_json[0].GetFloat(), dimensions_json[1].GetFloat() ,dimensions_json[2].GetFloat());


        

    //     std::vector<float> pos_rot_in_time;

    //     for (auto& array_of_pos : obstacles_json[i]["positions"].GetArray()){
    //         for (auto& coords : array_of_pos.GetArray()){
    //             pos_rot_in_time.push_back(coords.GetFloat());
    //         }
    //     }
        
    //     Eigen::Vector3f pos{pos_rot_in_time[0],pos_rot_in_time[1],pos_rot_in_time[2]};
    //     std::vector<float> rot(pos_rot_in_time.begin()+3, pos_rot_in_time.begin()+pos_rot_in_time.size()-3);
    //     fcl::Quaternionf q(rot[3],rot[0],rot[1],rot[2]);
    //     // uncomment if q is in euler
    //     // q = Eigen::AngleAxisf(rot[0], Eigen::Vector3f::UnitX()) 
    //     //     * Eigen::AngleAxisf(rot[1], Eigen::Vector3f::UnitY())
    //     //     * Eigen::AngleAxisf(rot[2], Eigen::Vector3f::UnitZ());
    //     object = std::make_shared<env::Box>(dim, pos, q, label);
        
    //     objects_coords[object] = pos_rot_in_time;
    //     objects.emplace_back(object);
    // }

}
