/**
 * Software License Agreement (BSD 3-Clause License)
 *
 *  Copyright (c) 2022, Bielefeld University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *
 *   * Neither the name of Bielefeld University nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

/* Authors: Florian Patzelt*/

#pragma once

#include <chrono>
#include <pluginlib/class_loader.h>

#include <mujoco_ros/mujoco_sim.h>
#include <mujoco_ros/plugin_utils.h>

#include <mujoco/mujoco.h>

namespace mujoco_screw_plugin {

using namespace MujocoSim;

const std::string PREFIX = "screw_plugin::";

class MujocoScrewPlugin : public MujocoSim::MujocoPlugin
{
public:
	~MujocoScrewPlugin() = default;

	// Overlead entry point
	bool load(mjModelPtr m, mjDataPtr d) override;

	void passiveCallback(mjModelPtr model, mjDataPtr data) override;
	// void renderCallback(mjModelPtr model, mjDataPtr data, mjvScene *scene) override;

	// Called on reset
	void reset() override;

	int collision_cb(const mjModel *m, const mjData *d, mjContact *con, int g1, int g2, mjtNum margin);

protected:
	// Mujoco model and data pointers
	mjModelPtr m_;
	mjDataPtr d_;

private:
	bool insert_contacts = false;

	double **lock_angle;
	double **last_angle;
	double **last_contact_time;

	int n_nuts   = 0;
	int n_screws = 0;

	std::vector<int> nut_ids;
	std::vector<int> screw_ids;

	int *geom2screw;
	int *geom2nut;

	std::vector<int> screw_locks;
	std::vector<int> nut_locks;

	std::vector<int> screw_sites;
	std::vector<int> nut_sites;

	std::vector<std::vector<int>> nut_joints;
	std::vector<std::vector<double>> nut_joint_offsets;
	std::vector<int> screw_joints;
	std::vector<double> screw_joint_offsets;
	std::vector<int> screw_joint_constraints;
	std::vector<int> screw_body_constraints;

	void initCollisionFunction();
	void parseBodies();
	void parseROSParam();
	bool handleScrewing(const mjModel *m, const mjData *d, int nidx, int sidx);
};

} // namespace mujoco_screw_plugin
