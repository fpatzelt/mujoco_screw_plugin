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

#include <mujoco_ros/mujoco_env.h>
#include <mujoco_ros/plugin_utils.h>

#include <mujoco/mujoco.h>

namespace mujoco_screw_plugin {

using namespace mujoco_ros;

const std::string PREFIX = "screw_plugin::";

class MujocoScrewPlugin : public mujoco_ros::MujocoPlugin
{
public:
	~MujocoScrewPlugin();

	// Overlead entry point
	bool load(const mjModel *m, mjData *d) override;

	void passiveCallback(const mjModel *model, mjData *data) override;

	// Called on reset
	void reset() override;

	int collision_cb(const mjModel *m, const mjData *d, mjContact *con, int g1, int g2, mjtNum margin);

	// 0: not screwed, 1: locked, 2: tight
	int getScrewingStatus(int n, int s);

protected:
	// Mujoco model and data pointers
	const mjModel *m_;
	mjData *d_;

private:
	bool insert_contacts = false;

	double **acc_angle;
	double **last_angle;
	double **last_contact_time;
	double **lock_scales;

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

	std::vector<double> nut_joint_offsets;
	std::vector<double> screw_joint_offsets;
	std::vector<int> screw_body_constraints;

	void initCollisionFunction();
	void parseBodies();
	void parseROSParam();
	bool handleScrewing(const mjModel *m, const mjData *d, int nidx, int sidx);
};

} // namespace mujoco_screw_plugin
