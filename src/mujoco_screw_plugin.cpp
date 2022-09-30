/**
 *
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

#include <mujoco_screw_plugin/mujoco_screw_plugin.h>

#include <pluginlib/class_list_macros.h>

namespace mujoco_screw_plugin {

namespace {

mjtNum *rot2euler(mjtNum *R)
{
	mjtNum sy = sqrt(R[0] * R[0] + R[1] * R[1]);

	bool singular = sy < 1e-6; // If

	mjtNum *eulers = new mjtNum[3];

	if (!singular) {
		eulers[0] = atan2(R[5], R[8]);
		eulers[1] = atan2(-R[2], sy);
		eulers[2] = atan2(R[1], R[0]);
	} else {
		eulers[0] = atan2(-R[7], R[4]);
		eulers[1] = atan2(-R[2], sy);
		eulers[2] = 0;
	}

	return eulers;
}

std::map<const mjData *, MujocoScrewPlugin *> instance_map;
mjfCollision defaultCollisionFunctions[mjNGEOMTYPES][mjNGEOMTYPES];
int collision_cb_wrapper(const mjModel *m, const mjData *d, mjContact *con, int g1, int g2, mjtNum margin)
{
	return instance_map[d]->collision_cb(m, d, con, g1, g2, margin);
}

double constrainAngle(double x)
{
	x = std::fmod(x + M_PI, 2 * M_PI);
	if (x < 0)
		x += 2 * M_PI;
	return x - M_PI;
}

} // namespace

MujocoScrewPlugin::~MujocoScrewPlugin()
{
	delete this;
}

bool MujocoScrewPlugin::load(mjModelPtr m, mjDataPtr d)
{
	ROS_INFO_STREAM_NAMED("mujoco_screw_plugin", "Loading mujoco_screw_plugin ...");
	d_                    = d;
	m_                    = m;
	instance_map[d.get()] = this;
	if (instance_map.size() == 1) {
		initCollisionFunction();
	}
	parseBodies();
	ROS_INFO_NAMED("mujoco_screw_plugin", "Loaded mujoco_screw_plugin");
	return true;
}

void MujocoScrewPlugin::parseBodies()
{
	geom2nut   = new int[m_->ngeom];
	geom2screw = new int[m_->ngeom];
	for (int i = 0; i < m_->ngeom; ++i) {
		geom2nut[i]   = -1;
		geom2screw[i] = -1;
	}

	if (m_->nuser_body < 4) {
		ROS_INFO_NAMED("mujoco_screw_plugin", "No screws found, no body has enough userparams.");
		return;
	}

	for (int i = 0; i < m_->nbody; ++i) {
		int adr          = m_->name_bodyadr[i];
		std::string name = "";
		for (int j = 0; m_->names[adr + j] != 0; ++j) {
			name += m_->names[adr + j];
			if (name.find("screw") != std::string::npos) {
				// find site belonging to screw body
				int site_id = -1;
				for (int k = 0; k < m_->nsite; ++k) {
					if (m_->site_bodyid[k] == i &&
					    std::string(mj_id2name(m_.get(), mjOBJ_SITE, k)).find("tip") != std::string::npos) {
						site_id = k;
					}
				}
				std::vector<int> joints;

				std::vector<int> constraints;

				for (auto type : { mjJNT_SLIDE, mjJNT_HINGE }) {
					for (int a = 0; a < 3; ++a) {
						for (int k = 0; k < m_->body_jntnum[i]; ++k) {
							int jnt_id = m_->body_jntadr[i] + k;
							if (m_->jnt_axis[3 * jnt_id + a] == 1.0 && m_->jnt_type[jnt_id] == type) {
								joints.push_back(jnt_id);
								if (!(type == mjJNT_HINGE && a == 2))
									for (int c_id = 0; c_id < m_->neq; ++c_id) {
										if (m_->eq_type[c_id] == mjEQ_JOINT && m_->eq_obj1id[c_id] == jnt_id) {
											constraints.push_back(c_id);
										}
									}
							}
						}
					}
				}

				if (site_id >= 0 && joints.size() == 6 && constraints.size() == 5) {
					ROS_INFO_STREAM_NAMED("mujoco_screw_plugin", "Found screw: " << name);
					std::vector<double> joint_offsets;
					for (int k = 0; k < 3; ++k) {
						joint_offsets.push_back(m_->body_pos[3 * i + k]);
					}
					mjtNum *R = new mjtNum[9];
					mju_quat2Mat(R, m_->body_quat + 4 * i);
					mjtNum *euler = rot2euler(R);
					for (int k = 0; k < 3; ++k) {
						joint_offsets.push_back(euler[k]);
					}
					for (int k = 0; k < m_->body_geomnum[i]; ++k) {
						int geom_id         = m_->body_geomadr[i] + k;
						geom2screw[geom_id] = screw_ids.size();
					}
					screw_ids.push_back(i);
					screw_locks.push_back(-1);
					screw_sites.push_back(site_id);
					screw_joints.push_back(joints);
					screw_joint_constraints.push_back(constraints);
					screw_joint_offsets.push_back(joint_offsets);
					++n_screws;
				} else {
					ROS_WARN_STREAM_NAMED(
					    "mujoco_screw_plugin",
					    "Found screw without defined tip site or with a wrong number of joints/joint constraints: " << name);
				}

			} else if (name.find("nut") != std::string::npos) {
				int site_id = -1;
				for (int k = 0; k < m_->nsite; ++k) {
					if (m_->site_bodyid[k] == i &&
					    std::string(mj_id2name(m_.get(), mjOBJ_SITE, k)).find("peg") != std::string::npos) {
						site_id = k;
					}
				}

				std::vector<int> joints;

				for (auto type : { mjJNT_SLIDE, mjJNT_HINGE }) {
					for (int a = 0; a < 3; ++a) {
						for (int k = 0; k < m_->body_jntnum[i]; ++k) {
							int jnt_id = m_->body_jntadr[i] + k;
							if (m_->jnt_axis[3 * jnt_id + a] == 1.0 && m_->jnt_type[jnt_id] == type) {
								joints.push_back(jnt_id);
							}
						}
					}
				}

				if (site_id >= 0 && (joints.size() == 0 || joints.size() == 6)) {
					ROS_INFO_STREAM_NAMED("mujoco_screw_plugin", "Found nut: " << name << " Num joints: " << joints.size());
					std::vector<double> joint_offsets;
					for (int k = 0; k < 3; ++k) {
						joint_offsets.push_back(m_->body_pos[3 * i + k]);
					}
					mjtNum *R = new mjtNum[9];
					mju_quat2Mat(R, m_->body_quat + 4 * i);
					mjtNum *euler = rot2euler(R);
					for (int k = 0; k < 3; ++k) {
						joint_offsets.push_back(euler[k]);
					}
					for (int k = 0; k < m_->body_geomnum[i]; ++k) {
						int geom_id       = m_->body_geomadr[i] + k;
						geom2nut[geom_id] = nut_ids.size();
					}

					nut_ids.push_back(i);
					nut_locks.push_back(-1);
					nut_sites.push_back(site_id);
					nut_joints.push_back(joints);
					nut_joint_offsets.push_back(joint_offsets);
					++n_nuts;
				} else {
					ROS_WARN_STREAM_NAMED("mujoco_screw_plugin",
					                      "Found nut without defined peg site or with a wrong number of joints: " << name);
				}
			}
		}
	}

	// init arrays between screws and nuts
	last_angle        = new double *[n_nuts];
	lock_angle        = new double *[n_nuts];
	last_contact_time = new double *[n_nuts];
	for (int i = 0; i < n_nuts; ++i) {
		last_angle[i]        = new double[n_screws];
		lock_angle[i]        = new double[n_screws];
		last_contact_time[i] = new double[n_screws];
		for (int j = 0; j < n_screws; ++j) {
			last_angle[i][j]        = 0;
			last_contact_time[i][j] = -1;
		}
	}
}

void MujocoScrewPlugin::initCollisionFunction()
{
	// #define SP std::placeholders
	// 	auto collision_function = std::bind(&MujocoContactSurfacesPlugin::collision_cb, this, SP::_1, SP::_2,
	// SP::_3, SP::_4, SP::_5, SP::_6);
	for (int i = 0; i < mjNGEOMTYPES; ++i) {
		for (int j = 0; j < mjNGEOMTYPES; ++j) {
			defaultCollisionFunctions[i][j] = mjCOLLISIONFUNC[i][j];
			// registerCollisionFunc(i, j, collision_function);
			registerCollisionFunc(i, j, collision_cb_wrapper);
		}
	}
}

// return whether collisions should be computed between the bodies
bool MujocoScrewPlugin::handleScrewing(const mjModel *m, const mjData *d, int nidx, int sidx)
{
	// ROS_INFO_STREAM_NAMED("mujoco_screw_plugin", "Handle screwing " << sidx << " " << nidx);
	if (nidx < 0 || sidx < 0) {
		// ROS_INFO_STREAM_NAMED("mujoco_screw_plugin", "Screws dont match");
		return true;
	}
	// ROS_INFO_STREAM_NAMED("mujoco_screw_plugin", "Collision: screw " << sidx << " and nut " << nidx);
	if (screw_locks[sidx] == nidx && nut_locks[nidx] == sidx) {
		// ROS_INFO_STREAM_NAMED("mujoco_screw_plugin", "Screws locked");
		// locked
		double current_angle =
		    -nut_joint_offsets[nidx][5] + screw_joint_offsets[sidx][5] + d->qpos[m->jnt_qposadr[screw_joints[sidx][5]]];
		if (current_angle > lock_angle[nidx][sidx]) {
			screw_locks[sidx] = -1;
			nut_locks[nidx]   = -1;
			ROS_INFO_STREAM_NAMED("mujoco_screw_plugin", "UNLOCK");
			for (int cid : screw_joint_constraints[sidx]) {
				m_->eq_active[cid] = 0;
			}
			m->dof_frictionloss[m->body_dofadr[screw_ids[sidx]] + 5] = 0;
			m->jnt_limited[m->body_jntadr[screw_ids[sidx]] + 5]      = 0;
		} else {
			int id_s    = screw_sites[sidx];
			int id_n    = nut_sites[nidx];
			mjtNum *p_n = d->site_xpos + 3 * id_n;
			m->eq_data[screw_joint_constraints[sidx][2] * mjNEQDATA] =
			    m->body_user[m->nuser_body * screw_ids[sidx] + 1] / 2. / M_PI * (current_angle - lock_angle[nidx][sidx]) -
			    screw_joint_offsets[sidx][2] + p_n[2] - m->site_pos[3 * id_s + 2];
		}
		return false;
	} else {
		if (screw_locks[sidx] >= 0 || nut_locks[nidx] >= 0) {
			// either screw or nut are locked with a different screw/nut
			return true;
		} else if (m->body_user[m->nuser_body * screw_ids[sidx]] != m->body_user[m->nuser_body * nut_ids[nidx]] ||
		           m->body_user[m->nuser_body * screw_ids[sidx] + 1] != m->body_user[m->nuser_body * nut_ids[nidx] + 1]) {
			// screw and nur are not compatible
			return true;
		} else {
			// check if screw and nut should be locked
			int id_n = nut_sites[nidx];
			int id_s = screw_sites[sidx];

			mjtNum *p_n = d->site_xpos + 3 * id_n;
			mjtNum *p_s = d->site_xpos + 3 * id_s;

			if (mju_dist3(p_n, p_s) < 0.002) {
				mjtNum *R_n = d->site_xmat + 9 * id_n;
				mjtNum *R_s = d->site_xmat + 9 * id_s;
				// // p0 = R_n^T * p_n
				// mjtNum *p0 = new mjtNum[3];
				// mju_mulMatTVec(p0, R_n, p_n, 3, 3);
				// // p1 = R_n^T * p_s
				// mjtNum *p1 = new mjtNum[3];
				// mju_mulMatTVec(p1, R_n, p_s, 3, 3);
				// // p2 = p1 - p0
				// mjtNum *p2 = new mjtNum[3];
				// mju_sub(p2, p1, p0, 3);

				// R0 = R_n*T * R_s
				mjtNum *R0 = new mjtNum[9];
				mju_mulMatTMat(R0, R_n, R_s, 3, 3, 3);

				mjtNum *eulers = rot2euler(R0);
				if (std::abs(eulers[0]) < M_PI / 9. && std::abs(eulers[0]) < M_PI / 9.) {
					if (d->time - last_contact_time[nidx][sidx] < 2 * m->opt.timestep) {
						if (nut_joints[nidx].size() == 0) {
							double current_angle = -nut_joint_offsets[nidx][5] + screw_joint_offsets[sidx][5] +
							                       d->qpos[m->jnt_qposadr[screw_joints[sidx][5]]];
							if (last_angle[nidx][sidx] > current_angle) {
								double fm = std::fmod(current_angle, 2 * M_PI);
								double la;
								la = current_angle > 0 ? current_angle - fm + 2. * M_PI : current_angle - fm;
								if (la < last_angle[nidx][sidx]) {
									// lock screw and nut
									ROS_INFO_STREAM_NAMED("mujoco_screw_plugin",
									                      "LOCK: c: " << current_angle << " last: " << last_angle[nidx][sidx]
									                                  << " lock: " << la);

									screw_locks[sidx]      = nidx;
									nut_locks[nidx]        = sidx;
									lock_angle[nidx][sidx] = la;

									// activate constraints
									for (int i = 0; i < 5; ++i) {
										int cid           = screw_joint_constraints[sidx][i];
										m->eq_active[cid] = 1;
										if (i < 3) {
											m->eq_data[cid * mjNEQDATA] =
											    -screw_joint_offsets[sidx][i] + p_n[i] -
											    m->site_pos[3 * id_s + i]; // + nut_joint_offsets[nidx][i]
											// m->eq_data[screw_joint_constraints[sidx][2] * mjNEQDATA] =
											//     -screw_joint_offsets[sidx][2] + p_n[2] - m->site_pos[3 * id_s + 2];
										} else {
											double target = nut_joint_offsets[nidx][i] - screw_joint_offsets[sidx][i];
											double ja     = d->qpos[m->jnt_qposadr[screw_joints[sidx][i]]];
											while (target + M_PI < ja) {
												target += 2 * M_PI;
											}
											while (target - M_PI > ja) {
												target -= 2 * M_PI;
											}
											// ROS_INFO_STREAM_NAMED("mujoco_screw_plugin", "Target: " << target);
											m->eq_data[cid * mjNEQDATA] = target;
										}
									}

									// activate friction
									// TODO this is not clean, the axis could be wrong
									m->dof_frictionloss[m->body_dofadr[screw_ids[sidx]] + 5] =
									    m->body_user[m->nuser_body * screw_ids[sidx] + 3];

									m->jnt_limited[m->body_jntadr[screw_ids[sidx]] + 5] = 1;
									m->jnt_range[(m->body_jntadr[screw_ids[sidx]] + 5) * 2] =
									    la - (m->body_user[m->nuser_body * screw_ids[sidx] + 2] /
									          m->body_user[m->nuser_body * screw_ids[sidx] + 1]) *
									             2 * M_PI;
									m->jnt_range[(m->body_jntadr[screw_ids[sidx]] + 5) * 2 + 1] =
									    la + 2 * M_PI; // this can never be reached
								}
							}

							last_angle[nidx][sidx] = current_angle;

						} else {
							// ROS_INFO_STREAM_NAMED("mujoco_screw_plugin", "Screwing not implemented for free nuts yet");
							double current_angle = d->qpos[m->jnt_qposadr[screw_joints[sidx][5]]] +
							                       screw_joint_offsets[sidx][5] - nut_joint_offsets[nidx][5] -
							                       d->qpos[m->jnt_qposadr[nut_joints[nidx][5]]];
							if (last_angle[nidx][sidx] > current_angle) {
								double fm = std::fmod(current_angle, 2 * M_PI);
								double la;
								la = current_angle > 0 ? current_angle - fm + 2. * M_PI : current_angle - fm;
								if (la < last_angle[nidx][sidx]) {
									// lock screw and nut
									ROS_INFO_STREAM_NAMED("mujoco_screw_plugin",
									                      "LOCK: c: " << current_angle << " last: " << last_angle[nidx][sidx]
									                                  << " lock: " << la);

									screw_locks[sidx]      = nidx;
									nut_locks[nidx]        = sidx;
									lock_angle[nidx][sidx] = la;

									// activate constraints
									for (int i = 0; i < 5; ++i) {
										int cid           = screw_joint_constraints[sidx][i];
										m->eq_active[cid] = 1;
										m->eq_obj2id[cid] = nut_joints[nidx][i];
										m->eq_data[cid * mjNEQDATA + 1] = 1;
										if (i < 3) {
											
											m->eq_data[cid * mjNEQDATA] =
											    -screw_joint_offsets[sidx][i] + p_n[i] -
											    m->site_pos[3 * id_s + i]; // + nut_joint_offsets[nidx][i]
											// m->eq_data[screw_joint_constraints[sidx][2] * mjNEQDATA] =
											//     -screw_joint_offsets[sidx][2] + p_n[2] - m->site_pos[3 * id_s + 2];
										} else {
											double target = nut_joint_offsets[nidx][i] - screw_joint_offsets[sidx][i];
											double ja     = d->qpos[m->jnt_qposadr[screw_joints[sidx][i]]];
											while (target + M_PI < ja) {
												target += 2 * M_PI;
											}
											while (target - M_PI > ja) {
												target -= 2 * M_PI;
											}
											// ROS_INFO_STREAM_NAMED("mujoco_screw_plugin", "Target: " << target);
											m->eq_data[cid * mjNEQDATA] = target;
										}
									}

									// activate friction
									// TODO this is not clean, the axis could be wrong
									// m->dof_frictionloss[m->body_dofadr[screw_ids[sidx]] + 5] =
									//     m->body_user[m->nuser_body * screw_ids[sidx] + 3];

									// m->jnt_limited[m->body_jntadr[screw_ids[sidx]] + 5] = 1;
									// m->jnt_range[(m->body_jntadr[screw_ids[sidx]] + 5) * 2] =
									//     la - (m->body_user[m->nuser_body * screw_ids[sidx] + 2] /
									//           m->body_user[m->nuser_body * screw_ids[sidx] + 1]) *
									//              2 * M_PI;
									// m->jnt_range[(m->body_jntadr[screw_ids[sidx]] + 5) * 2 + 1] =
									//     la + 2 * M_PI; // this can never be reached
								}
							}

							last_angle[nidx][sidx] = current_angle;
						}
					}
				}
			}
			last_contact_time[nidx][sidx] = d->time;
			return true;
		}
	}
}

int MujocoScrewPlugin::collision_cb(const mjModel *m, const mjData *d, mjContact *con, int g1, int g2, mjtNum margin)
{
	int sidx = -1;
	int nidx = -1;
	if (geom2nut[g1] >= 0 && geom2screw[g2] >= 0) {
		nidx = geom2nut[g1];
		sidx = geom2screw[g2];
	} else if (geom2nut[g2] >= 0 && geom2screw[g1] >= 0) {
		nidx = geom2nut[g2];
		sidx = geom2screw[g1];
	}
	bool col = handleScrewing(m, d, nidx, sidx);
	if (col) {
		int t1 = m->geom_type[g1];
		int t2 = m->geom_type[g2];

		return defaultCollisionFunctions[t1][t2](m, d, con, g1, g2, margin);
	} else {
		return 0;
	}
}

void MujocoScrewPlugin::passiveCallback(mjModelPtr m, mjDataPtr d) {}

void MujocoScrewPlugin::reset()
{
	for (int i = 0; i < n_nuts; ++i) {
		for (int j = 0; j < n_screws; ++j) {
			last_angle[i][j]        = 0;
			last_contact_time[i][j] = -1;
		}
		nut_locks[i] = -1;
	}
	for (int i = 0; i < n_screws; ++i) {
		screw_locks[i] = -1;
		for (int cid : screw_joint_constraints[i]) {
			m_->eq_active[cid] = 0;
		}
	}
}

} // namespace mujoco_screw_plugin
PLUGINLIB_EXPORT_CLASS(mujoco_screw_plugin::MujocoScrewPlugin, MujocoSim::MujocoPlugin)