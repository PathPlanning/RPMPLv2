#pragma once
#include "RealVectorSpace.h"
#include <CollisionManager/CollisionManager.hpp>
#include <config_read_writer/config_read.hpp>

namespace base
{
	class RealVectorSpaceHPPFCL : public base::RealVectorSpace
	{
	public:
		std::shared_ptr<MDP::CollisionManager> collision_manager;

		RealVectorSpaceHPPFCL(size_t num_dimensions_, const std::shared_ptr<robots::AbstractRobot> robot_, 
						   const std::shared_ptr<env::Environment> env_,const MDP::ConfigReader::SceneTask scene_task);
		~RealVectorSpaceHPPFCL();

		
		bool isValid(const std::shared_ptr<base::State> q) override;
		float computeDistance(const std::shared_ptr<base::State> q, bool compute_again) override;
		bool check_robot_selfcollision(const std::shared_ptr<base::State> q1,std::shared_ptr<base::State> & q2) override;

	};
}
