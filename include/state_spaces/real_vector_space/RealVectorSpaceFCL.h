//
// Created by dinko on 14.2.22.
//

#ifndef RPMPL_REALVECTORSPACEFCL_H
#define RPMPL_REALVECTORSPACEFCL_H

#include "RealVectorSpace.h"
#include "RealVectorSpaceConfig.h"
#include "xArm6.h"
#include <cassert>
namespace base
{
	class RealVectorSpaceFCL : public base::RealVectorSpace
	{
	public:
		std::shared_ptr<fcl::BroadPhaseCollisionManagerf> collision_manager_robot;
		std::shared_ptr<fcl::BroadPhaseCollisionManagerf> collision_manager_env;

		RealVectorSpaceFCL(size_t num_dimensions_, const std::shared_ptr<robots::AbstractRobot> robot_, 
						   const std::shared_ptr<env::Environment> env_);
		~RealVectorSpaceFCL();
		bool check_robot_selfcollision(const std::shared_ptr<base::State> q1,std::shared_ptr<base::State> & q2){assert(false);return false;};
		std::shared_ptr<fcl::BroadPhaseCollisionManagerf> getCollisionManagerRobot() const { return collision_manager_robot; }
		std::shared_ptr<fcl::BroadPhaseCollisionManagerf> getCollisionManagerEnv() const { return collision_manager_env; }
		
		bool isValid(const std::shared_ptr<base::State> q) override;
		float computeDistance(const std::shared_ptr<base::State> q, bool compute_again) override;
	};
}

#endif //RPMPL_REALVECTORSPACE_H