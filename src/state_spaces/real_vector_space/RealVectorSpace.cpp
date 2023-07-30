//
// Created by dinko on 07.03.21.
// Modified by nermin on 07.03.22.
//
#include "RealVectorSpace.h"
#include "RealVectorSpaceState.h"
#include <ostream>
#include <Eigen/Dense>
#include "ConfigurationReader.h"
// #include <QuadProg++.hh>
// #include <glog/log_severity.h>
// #include <glog/logging.h>

base::RealVectorSpace::RealVectorSpace(int dimensions_) : dimensions(dimensions_)
{
	setStateSpaceType(StateSpaceType::RealVectorSpace);
}

base::RealVectorSpace::RealVectorSpace(int dimensions_, const std::shared_ptr<robots::AbstractRobot> robot_, 
									   const std::shared_ptr<env::Environment> env_) : dimensions(dimensions_)
{
	srand((unsigned int) time(0));
	setStateSpaceType(StateSpaceType::RealVectorSpace);
	robot = robot_;
	env = env_;
}

base::RealVectorSpace::~RealVectorSpace() {}

std::ostream &base::operator<<(std::ostream &os, const base::RealVectorSpace &space)
{
	os << " dimensions: " << space.dimensions;
	return os;
}

// Get a random state with uniform distribution, which is centered around 'q_center' and limited with robot joint limits
std::shared_ptr<base::State> base::RealVectorSpace::randomState(std::shared_ptr<base::State> q_center)
{
	std::shared_ptr<base::State> state = std::make_shared<base::RealVectorSpaceState>(dimensions);
	Eigen::VectorXf rand = Eigen::VectorXf::Random(dimensions);
	std::vector<std::vector<float>> limits = robot->getLimits();
	for (size_t i = 0; i < dimensions; ++i)
		rand(i) = ((limits[i][1] - limits[i][0]) * rand(i) + limits[i][0] + limits[i][1]) / 2;

	if (q_center == nullptr)
		state->setCoord(rand);
	else
		state->setCoord(rand + q_center->getCoord());

	//std::cout << "Random coord: " << state->getCoord().transpose();
	return state;
}

// Make a copy of 'state'
std::shared_ptr<base::State> base::RealVectorSpace::newState(std::shared_ptr<base::State> state)
{
	std::shared_ptr<base::State> q = std::make_shared<base::RealVectorSpaceState>(state);
	return q;
}

// Make completely a new state with same coordinates as 'state'
std::shared_ptr<base::State> base::RealVectorSpace::newState(const Eigen::VectorXf &state)
{
	std::shared_ptr<base::State> q = std::make_shared<base::RealVectorSpaceState>(state);
	return q;
}

// Check if two states are equal
bool base::RealVectorSpace::isEqual(const std::shared_ptr<base::State> q1, const std::shared_ptr<base::State> q2)
{
	float d = (q1->getCoord() - q2->getCoord()).norm();
	if (d < RealVectorSpaceConfig::EQUALITY_THRESHOLD)
		return true;
	return false;
}

// Interpolate from 'q1' to 'q2' for step 'step'
// 'D' (optional parameter) is the distance between q1 and q2
// Return status of interpolation (Advanced, Trapped or Reached) and new state
std::tuple<base::State::Status, std::shared_ptr<base::State>> base::RealVectorSpace::interpolate
	(const std::shared_ptr<base::State> q1, const std::shared_ptr<base::State> q2, float step, float D)
{
	std::shared_ptr<base::State> q_new = std::make_shared<base::RealVectorSpaceState>(dimensions);
	base::State::Status status;

	if (D < 0) 	// D = -1 is the default value
		D = (q2->getCoord() - q1->getCoord()).norm();
	
	if (step + RealVectorSpaceConfig::EQUALITY_THRESHOLD < D)
	{
		q_new->setCoord(q1->getCoord() + step * (q2->getCoord() - q1->getCoord()) / D);
		status = base::State::Status::Advanced;
	}
	else
	{
		q_new->setCoord(q2->getCoord());
		status = base::State::Status::Reached;
	}
	
	if (isValid(q_new))
		return {status, q_new};
	else
		return {base::State::Status::Trapped, nullptr};
}

bool base::RealVectorSpace::isValid(const std::shared_ptr<base::State> q1, const std::shared_ptr<base::State> q2)
{
	int num_checks = RealVectorSpaceConfig::NUM_INTERPOLATION_VALIDITY_CHECKS;
	float D = (q2->getCoord() - q1->getCoord()).norm();
	for (float i = 1; i <= num_checks; i++)
	{
		if (std::get<0>(interpolate(q1, q2, i / num_checks * D, D)) == base::State::Status::Trapped)
			return false;
	}
	return true;
}

bool base::RealVectorSpace::isValid(const std::shared_ptr<base::State> q)
{
	bool collision;
	std::shared_ptr<Eigen::MatrixXf> XYZ = robot->computeSkeleton(q);
	int i0 = (robot->getType() == "xarm6") ? 1 : 0;
	
	for (int i = i0; i < robot->getParts().size(); i++)
	{
    	for (int j = 0; j < env->getParts().size(); j++)
		{
            if (env->getParts()[j]->getNodeType() == fcl::NODE_TYPE::GEOM_BOX)
			{
				// std::cout << "(i,j) = (" <<i<<","<<j<<")" << std::endl;
				// std::cout << "r(i): " << robot->getRadius(i) << std::endl;
				// std::cout << "XYZ(i):   " << XYZ->col(i).transpose() << std::endl;
				// std::cout << "XYZ(i+1): " << XYZ->col(i+1).transpose() << std::endl;
				fcl::AABB AABB = env->getParts()[j]->getAABB();
				Eigen::VectorXf obs(6);
				obs << AABB.min_[0], AABB.min_[1], AABB.min_[2], AABB.max_[0], AABB.max_[1], AABB.max_[2];
				if (collisionCapsuleToBox(XYZ->col(i), XYZ->col(i+1), robot->getRadius(i), obs))
					return false;
            }
			// else if (env->getParts()[j]->getNodeType() == fcl::NODE_TYPE::GEOM_SPHERE)
			// {
            //     if (collisionCapsuleToSphere(XYZ->col(i), XYZ->col(i+1), robot->getRadius(i), obs))
			// 		return false;
            // }
        }
    }
    return true;
}

// Check collision between capsule (determined with line segment AB and 'radius') and box (determined with 'obs = (x_min, y_min, z_min, x_max, y_max, z_max)')
bool base::RealVectorSpace::collisionCapsuleToBox(const Eigen::Vector3f &A, const Eigen::Vector3f &B, float radius, Eigen::VectorXf &obs)
{
    bool collision = false;
    float r_new = radius * sqrt(3) / 3;

	if (A(0) > obs(0) - r_new && A(1) > obs(1) - r_new && A(2) > obs(2) - r_new &&
		A(0) < obs(3) + r_new && A(1) < obs(4) + r_new && A(2) < obs(5) + r_new ||
		B(0) > obs(0) - r_new && B(1) > obs(1) - r_new && B(2) > obs(2) - r_new &&
		B(0) < obs(3) + r_new && B(1) < obs(4) + r_new && B(2) < obs(5) + r_new)
		return true;
    else if (A(0) < obs(0) - radius && B(0) < obs(0) - radius || A(0) > obs(3) + radius && B(0) > obs(3) + radius ||
           	 A(1) < obs(1) - radius && B(1) < obs(1) - radius || A(1) > obs(4) + radius && B(1) > obs(4) + radius ||
           	 A(2) < obs(2) - radius && B(2) < obs(2) - radius || A(2) > obs(5) + radius && B(2) > obs(5) + radius)
		return false;
    
    if (A(0) < obs(0))      				// < x_min
        collision = collisionCapsuleToRectangle(A, B, radius, obs, 0);
    else if (A(0) > obs(3))  				// > x_max
        collision = collisionCapsuleToRectangle(A, B, radius, obs, 3);
    
	if (!collision)
	{
		if (A(1) < obs(1))     				// < y_min
			collision = collisionCapsuleToRectangle(A, B, radius, obs, 1);
		else if (A(1) > obs(4)) 			// > y_max
			collision = collisionCapsuleToRectangle(A, B, radius, obs, 4);
		
		if (!collision)
		{
			if (A(2) < obs(2))     			// < z_min
				collision = collisionCapsuleToRectangle(A, B, radius, obs, 2);
			else if (A(2) > obs(5)) 		// > z_max
				collision = collisionCapsuleToRectangle(A, B, radius, obs, 5);
		}
	}
    return collision;
}

// Check collision between capsule (determined with line segment AB and 'radius') and rectangle (determined with 'obs',
// where 'coord' determines which coordinate is constant: {0,1,2,3,4,5} = {x_min, y_min, z_min, x_max, y_max, z_max}
bool base::RealVectorSpace::collisionCapsuleToRectangle(const Eigen::Vector3f &A, const Eigen::Vector3f &B, float radius, 
														Eigen::VectorXf &obs, int coord)
{
	float obs_coord = obs(coord);
    if (coord > 2)
        coord -= 3;
    
	Eigen::Vector4f rec; rec << obs.head(coord), obs.segment(coord+1, 2-coord), obs.segment(3, coord), obs.tail(2-coord);
	Eigen::Vector2f A_rec; A_rec << A.head(coord), A.tail(2-coord);
	Eigen::Vector2f B_rec; B_rec << B.head(coord), B.tail(2-coord);
    float t = (B(coord) - A(coord)) != 0 ? (obs_coord - A(coord)) / (B(coord) - A(coord)) : INFINITY;
	Eigen::Vector2f M = A_rec + t * (B_rec - A_rec);
	Eigen::Vector3f A_proj = get3DPoint(A_rec, obs_coord, coord);
	Eigen::Vector3f B_proj = get3DPoint(B_rec, obs_coord, coord);
    if (t > 0 && t < 1)		// Line segment AB intersects the plane 'obs(coord)'
	{
		if (M(0) > rec(0) - radius && M(0) < rec(2) + radius && 
			M(1) > rec(1) - radius && M(1) < rec(3) + radius) 	// Whether point lies on the oversized rectangle
		{
			if (M(0) > rec(0) - radius && M(0) < rec(2) + radius && M(1) > rec(1) && M(1) < rec(3) || 
				M(1) > rec(1) - radius && M(1) < rec(3) + radius && M(0) > rec(0) && M(0) < rec(2) ||
				M(0) < rec(0) && M(1) < rec(2) && (M - Eigen::Vector2f(rec(0), rec(2))).norm() < radius ||
				M(0) < rec(0) && M(1) > rec(3) && (M - Eigen::Vector2f(rec(0), rec(3))).norm() < radius ||
				M(0) > rec(1) && M(1) < rec(2) && (M - Eigen::Vector2f(rec(1), rec(2))).norm() < radius ||
				M(0) > rec(1) && M(1) > rec(3) && (M - Eigen::Vector2f(rec(1), rec(3))).norm() < radius)
				return true;
		}
	}
	else if (std::min((A - A_proj).norm(), (B - B_proj).norm()) > radius)
		return false;

    // Considering collision between capsule and rectangle
	if (radius > 0)
	{
		if (A_rec(0) > rec(0) && A_rec(0) < rec(2) && A_rec(1) > rec(1) && A_rec(1) < rec(3))
		{
			if (B_rec(0) > rec(0) && B_rec(0) < rec(2) && B_rec(1) > rec(1) && B_rec(1) < rec(3)) 	// Both projections
			{
				if (std::min((A - A_proj).norm(), (B - B_proj).norm()) < radius)
					return true;
			}
			else																					// Only projection of point A
			{
				if (std::min(checkCases(A, B, rec, B_rec, obs_coord, coord), (A - A_proj).norm()) < radius)
					return true;
			}
		}
		else if (B_rec(0) > rec(0) && B_rec(0) < rec(2) && B_rec(1) > rec(1) && B_rec(1) < rec(3))	// Only projection of point B
		{
			if (std::min(checkCases(A, B, rec, A_rec, obs_coord, coord), (B- B_proj).norm()) < radius)
				return true;
		}
		else																						// No projections						
		{
			if (checkCases(A, B, rec, A_rec, obs_coord, coord) < radius)
				return true;
				
			if (checkCases(A, B, rec, B_rec, obs_coord, coord) < radius)
				return true;
		}
	}
	
	return false;
}

float base::RealVectorSpace::checkCases(const Eigen::Vector3f &A, const Eigen::Vector3f &B, Eigen::Vector4f &rec, 
										Eigen::Vector2f &point, float obs_coord, int coord)
{
	float d_c1 = INFINITY;
	float d_c2 = INFINITY;
	if (point(0) < rec(0))
	{
		Eigen::Vector3f C = get3DPoint(Eigen::Vector2f(rec(0), rec(1)), obs_coord, coord);
		Eigen::Vector3f D = get3DPoint(Eigen::Vector2f(rec(0), rec(3)), obs_coord, coord);
		d_c1 = std::get<0>(distanceLineSegToLineSeg(A, B, C, D));
	}
	else if (point(0) > rec(2))
	{
		Eigen::Vector3f C = get3DPoint(Eigen::Vector2f(rec(2), rec(1)), obs_coord, coord);
		Eigen::Vector3f D = get3DPoint(Eigen::Vector2f(rec(2), rec(3)), obs_coord, coord);
		d_c1 = std::get<0>(distanceLineSegToLineSeg(A, B, C, D));
	}
	
	if (d_c1 > 0 && point(1) < rec(1))
	{
		Eigen::Vector3f C = get3DPoint(Eigen::Vector2f(rec(0), rec(1)), obs_coord, coord);
		Eigen::Vector3f D = get3DPoint(Eigen::Vector2f(rec(2), rec(1)), obs_coord, coord);
		d_c2 = std::get<0>(distanceLineSegToLineSeg(A, B, C, D));
	}
	else if (d_c1 > 0 && point(1) > rec(3))
	{
		Eigen::Vector3f C = get3DPoint(Eigen::Vector2f(rec(0), rec(3)), obs_coord, coord);
		Eigen::Vector3f D = get3DPoint(Eigen::Vector2f(rec(2), rec(3)), obs_coord, coord);
		d_c2 = std::get<0>(distanceLineSegToLineSeg(A, B, C, D));
	}
	return std::min(d_c1, d_c2);
}

const Eigen::Vector3f base::RealVectorSpace::get3DPoint(const Eigen::Vector2f &point, float coord_value, int coord)
{
	Eigen::Vector3f point_new;
	point_new << point.head(coord), coord_value, point.tail(2-coord);
	return point_new;
}

// Check collision between two line segments, AB and CD
bool base::RealVectorSpace::collisionLineSegToLineSeg(const Eigen::Vector3f &A, const Eigen::Vector3f &B, Eigen::Vector3f &C, Eigen::Vector3f &D)
{
    Eigen::Vector3f P1, P2;
    double alpha1 = (B - A).squaredNorm();
    double alpha2 = (B - A).dot(D - C);
    double beta1 = (C - D).dot(B - A);
    double beta2 = (C - D).dot(D - C);
    double gamma1 = (A - C).dot(A - B);
    double gamma2 = (A - C).dot(C - D);
    double s = (alpha1 * gamma2 - alpha2 * gamma1) / (alpha1 * beta2 - alpha2 * beta1);
    double t = (gamma1 - beta1 * s) / alpha1;

    if (t > 0 && t < 1 && s > 0 && s < 1)
	{
        P1 = A + t * (B - A);
        P2 = C + s * (D - C);
        if ((P2 - P1).norm() < RealVectorSpaceConfig::EQUALITY_THRESHOLD) 	// The collision occurs
            return true;
    }
    return false;
}

// Check collision between capsule (determined with line segment AB and 'radius') and sphere (determined with 'obs = (x_c, y_c, z_c, r)')
bool base::RealVectorSpace::collisionCapsuleToSphere(const Eigen::Vector3f &A, const Eigen::Vector3f &B, float radius, Eigen::VectorXf &obs)
{
	radius += obs(3);
    if ((A - obs.head(3)).norm() < radius || (B - obs.head(3)).norm() < radius)
        return true;    // The collision occurs

    else
	{
        double a = (B - A).squaredNorm();
        double b = 2 * (A.dot(B) + (A - B).dot(obs.head(3)) - A.squaredNorm());
        double c = A.squaredNorm() + obs.head(3).squaredNorm() - 2 * A.dot(obs.head(3)) - radius * radius;
        double D = b * b - 4 * a * c;
        if (D >= 0)
		{
            double t1 = (-b + sqrt(D)) / (2 * a);
            double t2 = (-b - sqrt(D)) / (2 * a);
            if (t1 > 0 && t1 < 1 || t2 > 0 && t2 < 1)
                return true;
        }
    }
    return false;
}

// ------------------------------------------------------------------------------------------------------------------------------- //

float base::RealVectorSpace::computeDistance(const std::shared_ptr<base::State> q)
{
	return std::get<0>(computeDistanceAndPlanes(q));
}

std::tuple<float, std::shared_ptr<std::vector<Eigen::MatrixXf>>> base::RealVectorSpace::computeDistanceAndPlanes(const std::shared_ptr<base::State> q)
{
    Eigen::MatrixXf distances(robot->getParts().size(), env->getParts().size());
	std::shared_ptr<std::vector<Eigen::MatrixXf>> planes = std::make_shared<std::vector<Eigen::MatrixXf>>
		(std::vector<Eigen::MatrixXf>(env->getParts().size(), Eigen::MatrixXf(6, robot->getParts().size())));
	std::shared_ptr<Eigen::MatrixXf> nearest_pts = nullptr;
	std::shared_ptr<Eigen::MatrixXf> XYZ = robot->computeSkeleton(q);
	int i0 = (robot->getType() == "xarm6") ? 1 : 0;

	for (int i = i0; i < robot->getParts().size(); i++)
	{
    	for (int j = 0; j < env->getParts().size(); j++)
		{
            if (env->getParts()[j]->getNodeType() == fcl::NODE_TYPE::GEOM_BOX)
			{
				fcl::AABB AABB = env->getParts()[j]->getAABB();
				Eigen::VectorXf obs(6);
				obs << AABB.min_[0], AABB.min_[1], AABB.min_[2], AABB.max_[0], AABB.max_[1], AABB.max_[2];
                tie(distances(i, j), nearest_pts) = distanceCapsuleToBox(XYZ->col(i), XYZ->col(i+1), robot->getRadius(i), obs);

				// std::cout << "(i, j) = (" <<i<<", "<<j<<"). " << std::endl;
				// std::cout << "Distance:    " << distances(i, j) << std::endl;
				// // float dQP;
				// // std::shared_ptr<Eigen::MatrixXf> nearest_ptsQP;
                // // tie(dQP, nearest_ptsQP) = distanceCapsuleToBoxQP(XYZ->col(i), XYZ->col(i+1), robot->getRadius(i), obs);
				// // std::cout << "Distance QP: " << dQP << std::endl;
				// // if (std::abs(distances(i, j) - dQP) > 1e-3)
				// // 	std::cout << "****************************** DIFFERENT *************************************" << std::endl;
				// if (nearest_pts != nullptr)
				// {
				// 	std::cout << "Nearest point link:    " << nearest_pts->col(0).transpose() << std::endl;
				// 	// std::cout << "Nearest point link QP: " << nearest_ptsQP->col(0).transpose() << std::endl;
				// 	std::cout << "Nearest point obs:     " << nearest_pts->col(1).transpose() << std::endl;
				// 	// std::cout << "Nearest point obs QP:  " << nearest_ptsQP->col(1).transpose() << std::endl;
				// }
				// std::cout << "r(i): " << robot->getRadius(i) << std::endl;
				// std::cout << "XYZ(i):   " << XYZ->col(i).transpose() << std::endl;
				// std::cout << "XYZ(i+1): " << XYZ->col(i+1).transpose() << std::endl;
				// std::cout << "-------------------------------------------------------------" << std::endl;
            }
			// else if (env->getParts()[j]->getNodeType() == fcl::NODE_TYPE::GEOM_SPHERE)
			// {
            //     tie(distances(i, j), nearest_pts) = distanceCapsuleToSphere(XYZ->col(i), XYZ->col(i+1), robot->getRadius(i), obs);
            // }

            if (distances(i, j) <= 0)		// The collision occurs
                return {0, nullptr};
			
			planes->at(j).col(i) << nearest_pts->col(1), nearest_pts->col(0) - nearest_pts->col(1);
        }
    }
	return {distances.minCoeff(), planes};
}

// Get distance (and nearest points) between capsule (determined with line segment AB and 'radius') 
// and box (determined with 'obs = (x_min, y_min, z_min, x_max, y_max, z_max)')
std::tuple<float, std::shared_ptr<Eigen::MatrixXf>> base::RealVectorSpace::distanceCapsuleToBox
	(const Eigen::Vector3f &A, const Eigen::Vector3f &B, float radius, Eigen::VectorXf &obs)
{
	Capsule_Box capsule_box(A, B, radius, obs);
	capsule_box.compute();
	return {capsule_box.getDistance(), capsule_box.getNearestPoints()};
}

// Get distance (and nearest points) between two line segments, AB and CD
std::tuple<float, std::shared_ptr<Eigen::MatrixXf>> base::RealVectorSpace::distanceLineSegToLineSeg
	(const Eigen::Vector3f &A, const Eigen::Vector3f &B, const Eigen::Vector3f &C, const Eigen::Vector3f &D)
{
    float d_c = INFINITY;
    std::shared_ptr<Eigen::MatrixXf> nearest_pts = std::make_shared<Eigen::MatrixXf>(3, 2);
    std::shared_ptr<Eigen::MatrixXf> nearest_pts_temp = std::make_shared<Eigen::MatrixXf>(3, 2);
    double alpha1 = (B - A).squaredNorm();
    double alpha2 = (B - A).dot(D - C);
    double beta1  = (C - D).dot(B - A);
    double beta2  = (C - D).dot(D - C);
    double gamma1 = (A - C).dot(A - B);
    double gamma2 = (A - C).dot(C - D);
    double s = (alpha1 * gamma2 - alpha2 * gamma1) / (alpha1 * beta2 - alpha2 * beta1);
    double t = (gamma1 - beta1 * s) / alpha1;
	
	if (t > 0 && t < 1 && s > 0 && s < 1)
	{
        nearest_pts->col(0) = A + t * (B - A);
        nearest_pts->col(1) = C + s * (D - C);
		d_c = (nearest_pts->col(1) - nearest_pts->col(0)).norm();
        if (d_c < RealVectorSpaceConfig::EQUALITY_THRESHOLD) 	// The collision occurs
            return {0, nullptr};
    }
    else
	{
		float d_c_temp;
        float alpha3 = (C - D).squaredNorm();
        Eigen::Vector4f opt((A - C).dot(A - B) / alpha1,	// s = 0
							(A - D).dot(A - B) / alpha1,	// s = 1
							(A - C).dot(D - C) / alpha3,	// t = 0
							(B - C).dot(D - C) / alpha3);	// t = 1
        for (int i = 0; i < 4; i++)
		{
            if (opt(i) < 0)
			{
				if (i == 0 || i == 2)     	// s = 0, t = 0
				{
					nearest_pts_temp->col(0) = A;
					nearest_pts_temp->col(1) = C; 
				}
                else if (i == 1)      		// s = 1, t = 0
				{
					nearest_pts_temp->col(0) = A;
                    nearest_pts_temp->col(1) = D; 
				}
                else                      	// t = 1, s = 0
				{
					nearest_pts_temp->col(0) = B;
                    nearest_pts_temp->col(1) = C; 
				}
			}
            else if (opt(i) > 1)
			{
				if (i == 1 || i == 3)    	// s = 1, t = 1
				{
					nearest_pts_temp->col(0) = B;
					nearest_pts_temp->col(1) = D; 
				}
                else if (i == 0)        	// s = 0, t = 1
				{
					nearest_pts_temp->col(0) = B;
					nearest_pts_temp->col(1) = C; 
				}                    
                else                    	// t = 0, s = 1
				{
					nearest_pts_temp->col(0) = A;
					nearest_pts_temp->col(1) = D; 
				}
			}
            else
			{
				if (i == 0)                	// s = 0, t € [0, 1]
				{
					nearest_pts_temp->col(0) = A + opt(i) * (B - A);
					nearest_pts_temp->col(1) = C; 
				}                    
                else if (i == 1)       		// s = 1, t € [0, 1]
				{
					nearest_pts_temp->col(0) = A + opt(i) * (B - A);
                    nearest_pts_temp->col(1) = D; 
				}
                else if (i == 2)           	// t = 0, s € [0, 1]
				{
					nearest_pts_temp->col(0) = A;
                    nearest_pts_temp->col(1) = C + opt(i) * (D - C); 
				}
                else                       	// t = 1, s € [0, 1]
				{
					nearest_pts_temp->col(0) = B;
                    nearest_pts_temp->col(1) = C + opt(i) * (D - C); 
				}
			}
            
            d_c_temp = (nearest_pts_temp->col(1) - nearest_pts_temp->col(0)).norm();
            if (d_c_temp < d_c)
			{
                d_c = d_c_temp;
				*nearest_pts = *nearest_pts_temp;
			}
        }
    }
	return {d_c, nearest_pts};
}

// Get distance (and nearest points) between line segment AB and point C
std::tuple<float, std::shared_ptr<Eigen::MatrixXf>> base::RealVectorSpace::distanceLineSegToPoint
	(const Eigen::Vector3f &A, const Eigen::Vector3f &B, const Eigen::Vector3f &C)
{
    std::shared_ptr<Eigen::MatrixXf> nearest_pts = std::make_shared<Eigen::MatrixXf>(3, 2);
    nearest_pts->col(1) = C;
    float t_opt = (C - A).dot(B - A) / (B - A).squaredNorm();
    if (t_opt < 0)
		nearest_pts->col(0) = A;
    else if (t_opt > 1)
        nearest_pts->col(0) = B;
    else
		nearest_pts->col(0) = A + t_opt * (B - A);
	
	float d_c = (nearest_pts->col(1) - nearest_pts->col(0)).norm();
	if (d_c < RealVectorSpaceConfig::EQUALITY_THRESHOLD)
		return {0, nullptr};

	return {d_c, nearest_pts};
}

// Get distance (and nearest points) between capsule (determined with line segment AB and 'radius') 
// and sphere (determined with 'obs = (x_c, y_c, z_c, r)')
std::tuple<float, std::shared_ptr<Eigen::MatrixXf>> base::RealVectorSpace::distanceCapsuleToSphere
	(const Eigen::Vector3f &A, const Eigen::Vector3f &B, float radius, Eigen::VectorXf &obs)
{
    std::shared_ptr<Eigen::MatrixXf> nearest_pts = std::make_shared<Eigen::MatrixXf>(3, 2);
	double AO = (A - obs.head(3)).norm();
    double d_c = AO - obs(3);
    if (d_c < radius)	// The collision occurs
        return {0, nullptr};
    
    double BO = (B - obs.head(3)).norm();
    double d_c_temp = BO - obs(3);
    if (d_c_temp < radius) 	// The collision occurs
        return {0, nullptr};    
    
    if (d_c_temp < d_c)
        d_c = d_c_temp; 

    double AB = (A - B).norm();
    double s = (AB + AO + BO) / 2;
    double alpha = acos((AO * AO + AB * AB - BO * BO) / (2 * AO * AB));
    d_c_temp = 2 * sqrt(s * (s - AB) * (s - AO) * (s - BO)) / AB - obs(3);     // h = 2 * P / AB; d_c_temp = h - obs(3);
    if (alpha < M_PI / 2)
	{
        double beta = acos((BO * BO + AB * AB - AO * AO) / (2 * BO * AB));
        if (beta < M_PI / 2) 	// Acute triangle
		{    
            d_c = d_c_temp;
            if (d_c_temp < radius)	// The collision occurs
			    return {0, nullptr};    
            
            nearest_pts->col(0) = A + AO * cos(alpha) / AB * (B - A);
            nearest_pts->col(1) = nearest_pts->col(0) + d_c / (obs.head(3) - nearest_pts->col(0)).norm() * (obs.head(3) - nearest_pts->col(0));
        }
        else
		{
            nearest_pts->col(1) = B + d_c / BO * (obs.head(3) - B);  
            nearest_pts->col(0) = B;
        }
    }
    else
	{
        nearest_pts->col(1) = A + d_c / AO * (obs.head(3) - A);  
        nearest_pts->col(0) = A;
    }
    return {d_c, nearest_pts};
}

// ------------------------------------------------ Class Capsule_Box -------------------------------------------------------//
base::RealVectorSpace::Capsule_Box::Capsule_Box(const Eigen::Vector3f &A_, const Eigen::Vector3f &B_, float radius_, Eigen::VectorXf &obs_)
{
	A = A_;
	B = B_;
	AB = Eigen::MatrixXf(3, 2);
	AB << A_, B_;
	radius = radius_;
	obs = obs_;
	d_c = INFINITY;
    nearest_pts = std::make_shared<Eigen::MatrixXf>(3, 2);
	projections = Eigen::MatrixXi::Zero(6, 2);
	dist_AB_obs = Eigen::Vector2f(INFINITY, INFINITY);
}

void base::RealVectorSpace::Capsule_Box::compute()
{
	projectionLineSegOnSide(1, 2, 0, 4, 5, 3);   // Projection on x_min or x_max
	projectionLineSegOnSide(0, 2, 1, 3, 5, 4);   // Projection on y_min or y_max
	projectionLineSegOnSide(0, 1, 2, 3, 4, 5);   // Projection on z_min or z_max     
	if (d_c == 0)
	{
		nearest_pts = nullptr;
		return;
	}

	int num_proj = (projections.col(0) + projections.col(1)).maxCoeff();
	if (num_proj > 0) 					// Projection of one or two points exists
	{
		int idx_point = (dist_AB_obs(0) < dist_AB_obs(1)) ? 0 : 1;
		d_c = dist_AB_obs.minCoeff();
		nearest_pts->col(0) = AB.col(idx_point);
		Eigen::Index idx_coord;
		projections.col(idx_point).maxCoeff(&idx_coord);
		if (idx_coord == 0 || idx_coord == 3)
			nearest_pts->col(1) << obs(idx_coord), AB.col(idx_point).tail(2);
		else if (idx_coord == 1 || idx_coord == 4)
			nearest_pts->col(1) << AB(0, idx_point), obs(idx_coord), AB(2, idx_point);
		else if (idx_coord == 2 || idx_coord == 5)
			nearest_pts->col(1) << AB.col(idx_point).head(2), obs(idx_coord);
		
		if (num_proj == 1)
		{
			if (idx_point == 0)   		// Projection of 'A' exists, but projection of 'B' does not exist)
				checkEdges(B, idx_point);
			else 						// Projection of 'B' exists, but projection of 'A' does not exist
				checkEdges(A, idx_point);
		}		
	}
	else								// There is no projection of any point
		checkOtherCases();

	d_c -= radius;
}

void base::RealVectorSpace::Capsule_Box::projectionLineSegOnSide(int min1, int min2, int min3, int max1, int max2, int max3)
{
	// 'min3' and 'max3' corresponds to the coordinate which is constant
	for (int i = 0; i < 2; i++)
	{
		if (AB(min1, i) >= obs(min1) && AB(min1, i) <= obs(max1) && AB(min2, i) >= obs(min2) && AB(min2, i) <= obs(max2))
		{
			if (AB(min3,i) > obs(min3) && AB(min3,i) < obs(max3))
			{
				d_c = 0;
				return;
			}
			else if (AB(min3, i) <= obs(min3))
			{
				projections(min3, i) = 1;
				dist_AB_obs(i) = obs(min3) - AB(min3, i);
			}
			else if (AB(min3, i) >= obs(max3))
			{
				projections(max3, i) = 1;
				dist_AB_obs(i) = AB(min3, i) - obs(max3);
			}
		}
	}
}

void base::RealVectorSpace::Capsule_Box::checkEdges(Eigen::Vector3f &point, int idx)
{
	std::shared_ptr<Eigen::MatrixXf> line_segments;
	if (projections(0, idx))  		// Projection on x_min
	{
		if (!collisionCapsuleToRectangle(A, B, 0, obs, 0))
			line_segments = getLineSegments(Eigen::Vector2f(point(1), point(2)), obs(1), obs(2), obs(4), obs(5), obs(0), 0);
		else
		{
			d_c = 0;
			return;
		}
	}				
	else if (projections(3, idx))  	// Projection on x_max
	{
		if (!collisionCapsuleToRectangle(A, B, 0, obs, 3))           
			line_segments = getLineSegments(Eigen::Vector2f(point(1), point(2)), obs(1), obs(2), obs(4), obs(5), obs(3), 0);
		else
		{
			d_c = 0;
			return;
		}
	}				
	else if (projections(1, idx))  	// Projection on y_min
	{
		if (!collisionCapsuleToRectangle(A, B, 0, obs, 1))
			line_segments = getLineSegments(Eigen::Vector2f(point(0), point(2)), obs(0), obs(2), obs(3), obs(5), obs(1), 1);
		else
		{
			d_c = 0;
			return;
		}
	}
	else if (projections(4, idx))  	// Projection on y_max
	{
		if (!collisionCapsuleToRectangle(A, B, 0, obs, 4))           
			line_segments = getLineSegments(Eigen::Vector2f(point(0), point(2)), obs(0), obs(2), obs(3), obs(5), obs(4), 1);
		else
		{
			d_c = 0;
			return;
		}
	}
	else if (projections(2, idx))  	// Projection on z_min
	{
		if (!collisionCapsuleToRectangle(A, B, 0, obs, 2))
			line_segments = getLineSegments(Eigen::Vector2f(point(0), point(1)), obs(0), obs(1), obs(3), obs(4), obs(2), 2);
		else
		{
			d_c = 0;
			return;
		}
	}
	else if (projections(5, idx))  	// Projection on z_max 
	{
		if (!collisionCapsuleToRectangle(A, B, 0, obs, 5))            
			line_segments = getLineSegments(Eigen::Vector2f(point(0), point(1)), obs(0), obs(1), obs(3), obs(4), obs(5), 2);
		else
		{
			d_c = 0;
			return;
		}
	}
	distanceToMoreLineSegments(*line_segments);
}

std::shared_ptr<Eigen::MatrixXf> base::RealVectorSpace::Capsule_Box::getLineSegments
	(const Eigen::Vector2f &point, float min1, float min2, float max1, float max2, float coord_value, int coord)
{
	std::shared_ptr<Eigen::MatrixXf> line_segments = std::make_shared<Eigen::MatrixXf>(3, 2);
	int num = 0;
	if (point(0) < min1)
	{
		line_segments->col(0) = get3DPoint(Eigen::Vector2f(min1, min2), coord_value, coord);
		line_segments->col(1) = get3DPoint(Eigen::Vector2f(min1, max2), coord_value, coord);
		num += 2;
	}
	else if (point(0) > max1)
	{
		line_segments->col(0) = get3DPoint(Eigen::Vector2f(max1, min2), coord_value, coord);
		line_segments->col(1) = get3DPoint(Eigen::Vector2f(max1, max2), coord_value, coord);
		num += 2;
	}
				
	if (point(1) < min2)
	{
		line_segments->conservativeResize(3, 2 + num);
		line_segments->col(num) 	= get3DPoint(Eigen::Vector2f(min1, min2), coord_value, coord);
		line_segments->col(num + 1) = get3DPoint(Eigen::Vector2f(max1, min2), coord_value, coord);
	}
	else if (point(1) > max2)
	{
		line_segments->conservativeResize(3, 2 + num);
		line_segments->col(num) 	= get3DPoint(Eigen::Vector2f(min1, max2), coord_value, coord);
		line_segments->col(num + 1) = get3DPoint(Eigen::Vector2f(max1, max2), coord_value, coord);
	}
	return line_segments;	
}
	
void base::RealVectorSpace::Capsule_Box::distanceToMoreLineSegments(const Eigen::MatrixXf &line_segments)
{
	float d_c_temp;
	std::shared_ptr<Eigen::MatrixXf> nearest_pts_temp;
	for (int k = 0; k < line_segments.cols(); k += 2)
	{
		tie(d_c_temp, nearest_pts_temp) = distanceLineSegToLineSeg(A, B, line_segments.col(k), line_segments.col(k+1));
		if (d_c_temp <= 0)
		{
			d_c = 0;
			return;
		}
		else if (d_c_temp < d_c)
		{
			d_c = d_c_temp;
			*nearest_pts = *nearest_pts_temp;
		}
	}
}

void base::RealVectorSpace::Capsule_Box::checkOtherCases()
{
	if (A(0) < obs(0) && B(0) < obs(0))
	{
		if (A(1) < obs(1) && B(1) < obs(1))
		{
			if (A(2) < obs(2) && B(2) < obs(2)) 		// < x_min, < y_min, < z_min
				tie(d_c, nearest_pts) = distanceLineSegToPoint(A, B, Eigen::Vector3f(obs(0), obs(1), obs(2)));
			else if (A(2) > obs(5) && B(2) > obs(5)) 	// < x_min, < y_min, > z_max
				tie(d_c, nearest_pts) = distanceLineSegToPoint(A, B, Eigen::Vector3f(obs(0), obs(1), obs(5)));
			else    									// < x_min, < y_min
				tie(d_c, nearest_pts) = distanceLineSegToLineSeg(A, B, Eigen::Vector3f(obs(0), obs(1), obs(2)), 
																 	   Eigen::Vector3f(obs(0), obs(1), obs(5)));
		}
		else if (A(1) > obs(4) && B(1) > obs(4))
		{
			if (A(2) < obs(2) && B(2) < obs(2)) 		// < x_min, > y_max, < z_min
				tie(d_c, nearest_pts) = distanceLineSegToPoint(A, B, Eigen::Vector3f(obs(0), obs(4), obs(2)));                        
			else if (A(2) > obs(5) && B(2) > obs(5)) 	// < x_min, > y_max, > z_max
				tie(d_c, nearest_pts) = distanceLineSegToPoint(A, B, Eigen::Vector3f(obs(0), obs(4), obs(5)));
			else    									// < x_min, > y_max
				tie(d_c, nearest_pts) = distanceLineSegToLineSeg(A, B, Eigen::Vector3f(obs(0), obs(4), obs(2)), 
																 	   Eigen::Vector3f(obs(0), obs(4), obs(5)));
		}
		else
		{
			if (A(2) < obs(2) && B(2) < obs(2)) 		// < x_min, < z_min
				tie(d_c, nearest_pts) = distanceLineSegToLineSeg(A, B, Eigen::Vector3f(obs(0), obs(1), obs(2)), 
																 	   Eigen::Vector3f(obs(0), obs(4), obs(2)));
			else if (A(2) > obs(5) && B(2) > obs(5)) 	// < x_min, > z_max
				tie(d_c, nearest_pts) = distanceLineSegToLineSeg(A, B, Eigen::Vector3f(obs(0), obs(1), obs(5)), 
																 	   Eigen::Vector3f(obs(0), obs(4), obs(5)));
			else    									// < x_min
			{
				Eigen::MatrixXf line_segments(3, 8);
				line_segments << obs(0), obs(0), obs(0), obs(0), obs(0), obs(0), obs(0), obs(0), 
								 obs(1), obs(4), obs(4), obs(4), obs(4), obs(1), obs(1), obs(1), 
								 obs(2), obs(2), obs(2), obs(5), obs(5), obs(5), obs(5), obs(2);
				distanceToMoreLineSegments(line_segments);
			}
		}
	}
	////////////////////////////////////////
	else if (A(0) > obs(3) && B(0) > obs(3))
	{
		if (A(1) < obs(1) && B(1) < obs(1))
		{
			if (A(2) < obs(2) && B(2) < obs(2)) 		// > x_max, < y_min, < z_min
				tie(d_c, nearest_pts) = distanceLineSegToPoint(A, B, Eigen::Vector3f(obs(3), obs(1), obs(2)));
			else if (A(2) > obs(5) && B(2) > obs(5)) 	// > x_max, < y_min, > z_max
				tie(d_c, nearest_pts) = distanceLineSegToPoint(A, B, Eigen::Vector3f(obs(3), obs(1), obs(5)));
			else    									// > x_max, < y_min
				tie(d_c, nearest_pts) = distanceLineSegToLineSeg(A, B, Eigen::Vector3f(obs(3), obs(1), obs(2)),
																 	   Eigen::Vector3f(obs(3), obs(1), obs(5)));                    
		}
		else if (A(1) > obs(4) && B(1) > obs(4))
		{
			if (A(2) < obs(2) && B(2) < obs(2)) 		// > x_max, > y_max, < z_min
				tie(d_c, nearest_pts) = distanceLineSegToPoint(A, B, Eigen::Vector3f(obs(3), obs(4), obs(2)));
			else if (A(2) > obs(5) && B(2) > obs(5)) 	// > x_max, > y_max, > z_max
				tie(d_c, nearest_pts) = distanceLineSegToPoint(A, B, Eigen::Vector3f(obs(3), obs(4), obs(5)));
			else    									// > x_max, > y_max
				tie(d_c, nearest_pts) = distanceLineSegToLineSeg(A, B, Eigen::Vector3f(obs(3), obs(4), obs(2)), 
																 	   Eigen::Vector3f(obs(3), obs(4), obs(5)));                     
		}
		else
		{
			if (A(2) < obs(2) && B(2) < obs(2)) 		// > x_max, < z_min
				tie(d_c, nearest_pts) = distanceLineSegToLineSeg(A, B, Eigen::Vector3f(obs(3), obs(1), obs(2)),
																 	   Eigen::Vector3f(obs(3), obs(4), obs(2)));
			else if (A(2) > obs(5) && B(2) > obs(5)) 	// > x_max, > z_max
				tie(d_c, nearest_pts) = distanceLineSegToLineSeg(A, B, Eigen::Vector3f(obs(3), obs(1), obs(5)), 
																 	   Eigen::Vector3f(obs(3), obs(4), obs(5)));
			else    									// > x_max
			{
				Eigen::MatrixXf line_segments(3, 8);
				line_segments << obs(3), obs(3), obs(3), obs(3), obs(3), obs(3), obs(3), obs(3), 
								 obs(1), obs(4), obs(4), obs(4), obs(4), obs(1), obs(1), obs(1), 
								 obs(2), obs(2), obs(2), obs(5), obs(5), obs(5), obs(5), obs(2);
				distanceToMoreLineSegments(line_segments);
			}
		}
	}
	///////////////////////////////////////
	else
	{
		if (A(1) < obs(1) && B(1) < obs(1))
		{
			if (A(2) < obs(2) && B(2) < obs(2)) 		// < y_min, < z_min
				tie(d_c, nearest_pts) = distanceLineSegToLineSeg(A, B, Eigen::Vector3f(obs(0), obs(1), obs(2)), 
																 	   Eigen::Vector3f(obs(3), obs(1), obs(2))); 
			else if (A(2) > obs(5) && B(2) > obs(5)) 	// < y_min, > z_max
				tie(d_c, nearest_pts) = distanceLineSegToLineSeg(A, B, Eigen::Vector3f(obs(0), obs(1), obs(5)), 
																 	   Eigen::Vector3f(obs(3), obs(1), obs(5))); 
			else    									// < y_min
			{
				Eigen::MatrixXf line_segments(3, 8);
				line_segments << obs(0), obs(3), obs(3), obs(3), obs(3), obs(0), obs(0), obs(0),
								 obs(1), obs(1), obs(1), obs(1), obs(1), obs(1), obs(1), obs(1),
								 obs(2), obs(2), obs(2), obs(5), obs(5), obs(5), obs(5), obs(2);
				distanceToMoreLineSegments(line_segments);                        
			}
		}
		else if (A(1) > obs(4) && B(1) > obs(4))
		{
			if (A(2) < obs(2) && B(2) < obs(2)) 		// > y_max, < z_min
				tie(d_c, nearest_pts) = distanceLineSegToLineSeg(A, B, Eigen::Vector3f(obs(0), obs(4), obs(2)), 
																 	   Eigen::Vector3f(obs(3), obs(4), obs(2)));                        
			else if (A(2) > obs(5) && B(2) > obs(5)) 	// > y_max, > z_max
				tie(d_c, nearest_pts) = distanceLineSegToLineSeg(A, B, Eigen::Vector3f(obs(0), obs(4), obs(5)),
																 	   Eigen::Vector3f(obs(3), obs(4), obs(5)));                             
			else    									// > y_max
			{
				Eigen::MatrixXf line_segments(3, 8);
				line_segments << obs(0), obs(3), obs(3), obs(3), obs(3), obs(0), obs(0), obs(0),
								 obs(4), obs(4), obs(4), obs(4), obs(4), obs(4), obs(4), obs(4),
								 obs(2), obs(2), obs(2), obs(5), obs(5), obs(5), obs(5), obs(2);
				distanceToMoreLineSegments(line_segments);  
			}
		}					                    
		else
		{
			if (A(2) < obs(2) && B(2) < obs(2)) 		// < z_min
			{
				Eigen::MatrixXf line_segments(3, 8);
				line_segments << obs(0), obs(3), obs(3), obs(3), obs(3), obs(0), obs(0), obs(0), 
								 obs(1), obs(1), obs(1), obs(4), obs(4), obs(4), obs(4), obs(1), 
								 obs(2), obs(2), obs(2), obs(2), obs(2), obs(2), obs(2), obs(2);
				distanceToMoreLineSegments(line_segments);
			}
											
			else if (A(2) > obs(5) && B(2) > obs(5)) 	// > z_max
			{
				Eigen::MatrixXf line_segments(3, 8);
				line_segments << obs(0), obs(3), obs(3), obs(3), obs(3), obs(0), obs(0), obs(0), 
								 obs(1), obs(1), obs(1), obs(4), obs(4), obs(4), obs(4), obs(1), 
								 obs(5), obs(5), obs(5), obs(5), obs(5), obs(5), obs(5), obs(5);
				distanceToMoreLineSegments(line_segments);  
			}
					
			else
			{
				for (int kk = 0; kk < 6; kk++)    		// Check collision with all sides
				{
					if (collisionCapsuleToRectangle(A, B, 0, obs, kk))
					{
						d_c = 0;
						return;
					}
				}
				Eigen::MatrixXf line_segments(3, 24);
				line_segments << obs(0), obs(3), obs(3), obs(3), obs(3), obs(0), obs(0), obs(0), obs(0), obs(3), obs(3), obs(3), obs(3), obs(0), obs(0), obs(0), obs(0), obs(0), obs(3), obs(3), obs(3), obs(3), obs(0), obs(0),
								 obs(1), obs(1), obs(1), obs(4), obs(4), obs(4), obs(4), obs(1), obs(1), obs(1), obs(1), obs(4), obs(4), obs(4), obs(4), obs(1), obs(1), obs(1), obs(1), obs(1), obs(4), obs(4), obs(4), obs(4),
								 obs(2), obs(2), obs(2), obs(2), obs(2), obs(2), obs(2), obs(2), obs(5), obs(5), obs(5), obs(5), obs(5), obs(5), obs(5), obs(5), obs(2), obs(5), obs(2), obs(5), obs(2), obs(5), obs(2), obs(5);
				distanceToMoreLineSegments(line_segments);
			}      
		}
	}
}
// -----------------------------------------------------------------------------------------------------------------------------//

// Get distance (and nearest points) between capsule (determined with line segment AB and 'radius') 
// and box (determined with 'obs = (x_min, y_min, z_min, x_max, y_max, z_max)') using QuadProgPP
std::tuple<float, std::shared_ptr<Eigen::MatrixXf>> base::RealVectorSpace::distanceCapsuleToBoxQP
	(const Eigen::Vector3f &A, const Eigen::Vector3f &B, float radius, Eigen::VectorXf &obs)
{
    // std::shared_ptr<Eigen::MatrixXf> nearest_pts = std::make_shared<Eigen::MatrixXf>(3, 2);
    // Eigen::Vector3f AB = B - A;
    // Eigen::Matrix4f G_temp(4, 4); 	G_temp << 	2,       	0,   		0,   		-2*AB(0),
	// 								 			0,       	2,   		0,  		-2*AB(1),
	// 								 			0,       	0,  		2,   		-2*AB(2),
	// 								 			-2*AB(0), 	-2*AB(1), 	-2*AB(2), 	2*AB.squaredNorm();
    // Eigen::Vector4f g0_temp; 		g0_temp << -2 * A, 2 * A.dot(AB);
	// Eigen::MatrixXf CI_temp(8, 4); 	CI_temp << Eigen::MatrixXf::Identity(4, 4), -1 * Eigen::MatrixXf::Identity(4, 4);
	// Eigen::VectorXf ci0_temp(8); 	ci0_temp << -1 * obs.head(3), 0, obs.tail(3), 1;
	// Eigen::Vector4f sol_temp; 		sol_temp << (obs.head(3) + obs.tail(3)) / 2, 0.5;

	// quadprogpp::Matrix<double> G(4, 4), CE(4, 0), CI(4, 8);
  	// quadprogpp::Vector<double> g0(4), ce0(0), ci0(8), sol(4);
	// for (int i = 0; i < 4; i++)
	// 	for (int j = 0; j < 4; j++)
	// 		G[i][j] = G_temp(i, j);

	// for (int i = 0; i < 8; i++)
	// 	for (int j = 0; j < 4; j++)
	// 		CI[j][i] = CI_temp(i, j);

	// for (int i = 0; i < 4; i++)
	// 	g0[i] = g0_temp(i);

	// for (int i = 0; i < 8; i++)
	// 	ci0[i] = ci0_temp(i);

	// for (int i = 0; i < 4; i++)
	// 	sol[i] = sol_temp(i);

	// // 	minimizes 1/2 x^T G x + x^T g0
	// //  s.t.
	// //  CE^T * x + ce0 =  0
	// //  CI^T * x + ci0 >= 0
	// double sol_val = quadprogpp::solve_quadprog(G, g0, CE, ce0, CI, ci0, sol);

    // float d_c = sqrt(sol_val + A.squaredNorm());
    // if (d_c < RealVectorSpaceConfig::EQUALITY_THRESHOLD)
	// 	return {0, nullptr};
	
	// nearest_pts->col(0) = A + AB * sol[3];
	// nearest_pts->col(1) << sol[0], sol[1], sol[2];
	// return {d_c - radius, nearest_pts};
}
