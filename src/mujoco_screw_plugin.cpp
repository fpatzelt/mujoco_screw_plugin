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

bool MujocoScrewPlugin::load(mjModelPtr m, mjDataPtr d)
{
	ROS_INFO_NAMED("mujoco_screw_plugin", "Loading mujoco_screw_plugin ...");
	d_                    = d;
	m_                    = m;
	instance_map[d.get()] = this;
	if (instance_map.size() == 1) {
		initCollisionFunction();
	}
	parseBodies();
	parseROSParam();
	ROS_INFO_STREAM_NAMED("mujoco_screw_plugin", "Insert screw contacts: " << insert_contacts);
	ROS_INFO_NAMED("mujoco_screw_plugin", "Loaded mujoco_screw_plugin");
	return true;
}

void MujocoScrewPlugin::parseROSParam()
{
	if (rosparam_config_.hasMember("insert_contacts")) {
		insert_contacts = static_cast<bool>(rosparam_config_["insert_contacts"]);
	}
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
				if (site_id < 0) {
					ROS_INFO_STREAM_NAMED("mujoco_screw_plugin", "No site");
					continue;
				}

				int body_constraint = -1;

				for (int c_id = 0; c_id < m_->neq; ++c_id) {
					if (m_->eq_type[c_id] == mjEQ_WELD && m_->eq_obj1id[c_id] == i) {
						body_constraint = c_id;
					}
				}

				if (body_constraint < 0) {
					ROS_INFO_STREAM_NAMED("mujoco_screw_plugin", "Constraint missing");
					continue;
				}

				ROS_INFO_STREAM_NAMED("mujoco_screw_plugin", "Found screw: " << name);
				for (int k = 0; k < m_->body_geomnum[i]; ++k) {
					int geom_id         = m_->body_geomadr[i] + k;
					geom2screw[geom_id] = screw_ids.size();
				}
				screw_ids.push_back(i);
				screw_locks.push_back(-1);
				screw_sites.push_back(site_id);
				screw_body_constraints.push_back(body_constraint);
				mjtNum *R = new mjtNum[9];
				mju_quat2Mat(R, m_->body_quat + 4 * i);
				mjtNum *euler = rot2euler(R);
				screw_joint_offsets.push_back(euler[2]);
				++n_screws;

			} else if (name.find("nut") != std::string::npos) {
				int site_id = -1;
				for (int k = 0; k < m_->nsite; ++k) {
					if (m_->site_bodyid[k] == i &&
					    std::string(mj_id2name(m_.get(), mjOBJ_SITE, k)).find("peg") != std::string::npos) {
						site_id = k;
					}
				}

				if (site_id < 0) {
					continue;
				}

				if (m_->body_dofnum[i] > 0) {
					ROS_INFO_STREAM_NAMED("mujoco_screw_plugin", "Found nut: " << name);
					for (int k = 0; k < m_->body_geomnum[i]; ++k) {
						int geom_id       = m_->body_geomadr[i] + k;
						geom2nut[geom_id] = nut_ids.size();
					}
					nut_ids.push_back(i);
					nut_locks.push_back(-1);
					nut_sites.push_back(site_id);
					// nut_joints.push_back({ jnt_id });

					mjtNum *R = new mjtNum[9];
					mju_quat2Mat(R, m_->body_quat + 4 * i);
					mjtNum *euler = rot2euler(R);
					nut_joint_offsets.push_back(euler[2]);
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
	acc_angle         = new double *[n_nuts];
	last_contact_time = new double *[n_nuts];
	lock_scales       = new double *[n_nuts];
	for (int i = 0; i < n_nuts; ++i) {
		last_angle[i]        = new double[n_screws];
		acc_angle[i]         = new double[n_screws];
		last_contact_time[i] = new double[n_screws];
		lock_scales[i]       = new double[n_screws];
		for (int j = 0; j < n_screws; ++j) {
			last_angle[i][j]        = 0;
			last_contact_time[i][j] = -1;
			lock_scales[i][j]       = 0;
		}
	}
}

void MujocoScrewPlugin::initCollisionFunction()
{
	for (int i = 0; i < mjNGEOMTYPES; ++i) {
		for (int j = 0; j < mjNGEOMTYPES; ++j) {
			defaultCollisionFunctions[i][j] = mjCOLLISIONFUNC[i][j];
			registerCollisionFunc(i, j, collision_cb_wrapper);
		}
	}
}

// return whether collisions should be computed between the bodies
bool MujocoScrewPlugin::handleScrewing(const mjModel *m, const mjData *d, int nidx, int sidx)
{
	if (nidx < 0 || sidx < 0) {
		return true;
	}
	if (screw_locks[sidx] == nidx && nut_locks[nidx] == sidx) {
		// locked
		mjtNum *RB   = new mjtNum[9];
		mjtNum *R_nB = d->xmat + 9 * nut_ids[nidx];
		mjtNum *R_sB = d->xmat + 9 * screw_ids[sidx];
		mju_mulMatTMat(RB, R_nB, R_sB, 3, 3, 3);
		mjtNum *eulersB      = rot2euler(RB);
		double current_angle = eulersB[2];
		// update accumulated relative angle
		acc_angle[nidx][sidx] += constrainAngle(current_angle - last_angle[nidx][sidx]);

		if (acc_angle[nidx][sidx] < 0) {
			// unlock condition met
			screw_locks[sidx] = -1;
			nut_locks[nidx]   = -1;
			ROS_INFO_STREAM_NAMED("mujoco_screw_plugin", "UNLOCK");
			m_->eq_active[screw_body_constraints[sidx]]              = 0;
			m->dof_frictionloss[m->body_dofadr[screw_ids[sidx]] + 5] = 0;
			m->jnt_limited[m->body_jntadr[screw_ids[sidx]] + 5]      = 0;
		} else {
			// screw still locked
			int id_s = screw_sites[sidx];
			int id_n = nut_sites[nidx];
			m->eq_data[screw_body_constraints[sidx] * mjNEQDATA + 5] =
			    m->body_user[m->nuser_body * screw_ids[sidx] + 1] / 2. / M_PI * acc_angle[nidx][sidx] +
			    m->site_pos[3 * id_s + 2] - m->site_pos[3 * id_n + 2];
			double max_rotations =
			    m->body_user[m->nuser_body * screw_ids[sidx] + 2] / m->body_user[m->nuser_body * screw_ids[sidx] + 1];
			double rot_threshold     = max_rotations - 1;
			double current_rotations = acc_angle[nidx][sidx] / (2 * M_PI);
			lock_scales[nidx][sidx] =
			    std::max(0.0, std::min(1.0, (current_rotations - rot_threshold) / (max_rotations - rot_threshold)));
			// torquescale is not touched
			// m->eq_data[screw_body_constraints[sidx] * mjNEQDATA + 10] = 1 + 100 * lock_scales[nidx][sidx];
			if (current_rotations < max_rotations) {
				// eq+data[... + 6] is begin of quaternion
				const mjtNum zaxis[3] = { 0., 0., 1. };
				mju_axisAngle2Quat(m_->eq_data + screw_body_constraints[sidx] * mjNEQDATA + 6, zaxis, current_angle);
			}
		}
		last_contact_time[nidx][sidx] = d->time;
		last_angle[nidx][sidx]        = current_angle;
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
				// sites of screw and nut are close
				mjtNum *R_n = d->site_xmat + 9 * id_n;
				mjtNum *R_s = d->site_xmat + 9 * id_s;
				mjtNum *R0  = new mjtNum[9];
				mju_mulMatTMat(R0, R_n, R_s, 3, 3, 3);
				mjtNum *eulers = rot2euler(R0);

				mjtNum *RB   = new mjtNum[9];
				mjtNum *R_nB = d->xmat + 9 * nut_ids[nidx];
				mjtNum *R_sB = d->xmat + 9 * screw_ids[sidx];
				mju_mulMatTMat(RB, R_nB, R_sB, 3, 3, 3);
				mjtNum *eulersB = rot2euler(RB);
				if (std::abs(eulers[0]) < M_PI / 9. && std::abs(eulers[1]) < M_PI / 9.) {
					// screw and nut are aligned
					if (d->time - last_contact_time[nidx][sidx] < 2 * m->opt.timestep) {
						// screw and nut are aligned for multiple consecutive timesteps
						mjtNum nut_angle     = d->qpos[3 + m->jnt_qposadr[m->body_jntadr[nut_ids[nidx]]]];
						mjtNum screw_angle   = d->qpos[3 + m->jnt_qposadr[m->body_jntadr[screw_ids[sidx]]]];
						double current_angle = eulersB[2];
						if (last_angle[nidx][sidx] < 0 && current_angle > 0 &&
						    current_angle - last_angle[nidx][sidx] < M_PI) {
							// screw went past the "locking angle": lock screw and nut
							ROS_INFO_STREAM_NAMED("mujoco_screw_plugin", "LOCK: c: " << current_angle
							                                                         << " last: " << last_angle[nidx][sidx]
							                                                         << " z-angle: " << eulersB[2]);

							screw_locks[sidx]     = nidx;
							nut_locks[nidx]       = sidx;
							acc_angle[nidx][sidx] = current_angle;

							// activate constraints
							int bc           = screw_body_constraints[sidx];
							m->eq_obj2id[bc] = nut_ids[nidx];
							for (int i = 0; i < 3; ++i) {
								m->eq_data[bc * mjNEQDATA + i + 3] = m->site_pos[3 * id_s + i] - m->site_pos[3 * id_n + i];
								m->eq_data[bc * mjNEQDATA + i]     = 0;
							}
							const mjtNum zaxis[3] = { 0., 0., 1. };
							mju_axisAngle2Quat(m_->eq_data + bc * mjNEQDATA + 6, zaxis, current_angle);
							// do not touch torquescale
							// m_->eq_data[bc * mjNEQDATA + 10] = 1;
							m->eq_active[bc] = 1;
						}
						last_angle[nidx][sidx] = current_angle;
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
	int t1   = m->geom_type[g1];
	int t2   = m->geom_type[g2];
	if (col) {
		return defaultCollisionFunctions[t1][t2](m, d, con, g1, g2, margin);
	} else {
		if (insert_contacts) {
			int n = defaultCollisionFunctions[t1][t2](m, d, con, g1, g2, margin);
			for (int i = 0; i < n; ++i) {
				con[i].dist = mjMAXVAL;
			}
			return n;
		}
		return 0;
	}
}

// compute impedance and derivative for one constraint
static void getimpedance(const mjtNum *solimp, mjtNum pos, mjtNum margin, mjtNum *imp, mjtNum *impP)
{
	// flat function
	if (solimp[0] == solimp[1] || solimp[2] <= mjMINVAL) {
		*imp  = 0.5 * (solimp[0] + solimp[1]);
		*impP = 0;
		return;
	}

	// x = abs((pos-margin) / width)
	mjtNum x   = (pos - margin) / solimp[2];
	mjtNum sgn = 1;
	if (x < 0) {
		x   = -x;
		sgn = -1;
	}

	// fully saturated
	if (x >= 1 || x <= 0) {
		*imp  = (x >= 1 ? solimp[1] : solimp[0]);
		*impP = 0;
		return;
	}

	// linear
	mjtNum y, yP;
	if (solimp[4] == 1) {
		y  = x;
		yP = 1;
	}

	// y(x) = a*x^p if x<=midpoint
	else if (x <= solimp[3]) {
		mjtNum a = 1 / mju_pow(solimp[3], solimp[4] - 1);
		y        = a * mju_pow(x, solimp[4]);
		yP       = solimp[4] * a * mju_pow(x, solimp[4] - 1);
	}

	// y(x) = 1-b*(1-x)^p is x>midpoint
	else {
		mjtNum b = 1 / mju_pow(1 - solimp[3], solimp[4] - 1);
		y        = 1 - b * mju_pow(1 - x, solimp[4]);
		yP       = solimp[4] * b * mju_pow(1 - x, solimp[4] - 1);
	}

	// scale
	*imp  = solimp[0] + y * (solimp[1] - solimp[0]);
	*impP = yP * sgn * (solimp[1] - solimp[0]) / solimp[2];
}

void MujocoScrewPlugin::passiveCallback(mjModelPtr m, mjDataPtr d)
{
	int dim, nefc = d->nefc;
	mjtNum *R = d->efc_R, *KBIP = d->efc_KBIP;
	mjtNum pos, imp, impP, Rpy;
	mjtNum solref[mjNREF] = { 0.2, 1 };
	mjtNum solimp[mjNIMP] = { 0.8, 0.95, 1, 0.5, 2 };

	// look for locked screws/nuts
	for (int sidx = 0; sidx < n_screws; ++sidx) {
		for (int nidx = 0; nidx < n_nuts; ++nidx) {
			if (nut_locks[nidx] == sidx && screw_locks[sidx] == nidx) {
				int id = screw_body_constraints[sidx];
				int i  = 0;
				// find constraints for nut/screw pair
				while (i < d->nefc && !(d->efc_type[i] == mjCNSTR_EQUALITY && d->efc_id[i] == id)) {
					++i;
				}
				if (i < d->nefc) {
					// scale works similar to torquescale but just around z-axis and determines how hard it is to turn the
					// screw
					double scale = 0.1 + 10 * lock_scales[nidx][sidx];
					int j        = i + 5;
					if (!mj_isSparse(m.get())) {
						mju_scl(d->efc_J + j * m->nv, d->efc_J + j * m->nv, scale, m->nv);
					} else {
						mju_scl(d->efc_J + d->efc_J_rowadr[j], d->efc_J + d->efc_J_rowadr[j], scale, m->nv);
					}

					mjtNum efc_pos[6];

					mju_copy(efc_pos, d->efc_pos + i, 6);

					// multiply z-orientation by scale
					efc_pos[5] *= scale;
					dim = 6;
					pos = mju_norm(efc_pos, 6);

					// get imp and impP
					getimpedance(solimp, pos, d->efc_margin[i], &imp, &impP);

					R[j] = mju_max(mjMINVAL, (1 - imp) * d->efc_diagApprox[j] / imp);

					// friction: K = 0
					int tp = d->efc_type[j];

					// standard: K = 1 / (dmax^2 * timeconst^2 * dampratio^2)
					if (solref[0] > 0)
						KBIP[4 * j] =
						    1 / mju_max(mjMINVAL, solimp[1] * solimp[1] * solref[0] * solref[0] * solref[1] * solref[1]);

					// direct: K = -solref[0] / dmax^2
					else {
						KBIP[4 * j] = -solref[0] / mju_max(mjMINVAL, solimp[1] * solimp[1]);
					}

					// standard: B = 2 / (dmax*timeconst)
					if (solref[1] > 0) {
						KBIP[4 * j + 1] = 2 / mju_max(mjMINVAL, solimp[1] * solref[0]);
					}

					// direct: B = -solref[1] / dmax
					else {
						KBIP[4 * j + 1] = -solref[1] / mju_max(mjMINVAL, solimp[1]);
					}

					// I = imp, P = imp'
					KBIP[4 * j + 2]      = imp;
					KBIP[4 * j + 3]      = impP;
					d->efc_D[j]          = 1 / R[j];
					d->efc_diagApprox[j] = R[j] * KBIP[4 * j + 2] / (1 - KBIP[4 * j + 2]);
					mj_projectConstraint(m.get(), d.get());
				}
			}
		}
	}
}

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
		m_->eq_active[screw_body_constraints[i]] = 0;
	}
}

} // namespace mujoco_screw_plugin
PLUGINLIB_EXPORT_CLASS(mujoco_screw_plugin::MujocoScrewPlugin, MujocoSim::MujocoPlugin)
