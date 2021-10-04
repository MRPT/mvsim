/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2020  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/img/CImage.h>
#include <mrpt/opengl/CMesh.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/tfest/TMatchingPair.h>
#include <mvsim/WorldElements/WorldElementBase.h>

namespace mvsim
{
class ElevationMap : public WorldElementBase
{
	DECLARES_REGISTER_WORLD_ELEMENT(ElevationMap)
   public:
	ElevationMap(World* parent, const rapidxml::xml_node<char>* root);
	virtual ~ElevationMap();

	virtual void loadConfigFrom(const rapidxml::xml_node<char>* root) override;

	virtual void simul_pre_timestep(const TSimulContext& context) override;
	virtual void simul_post_timestep(const TSimulContext& context) override;

	bool getElevationAt(
		double x, double y, float& z) const;  //!< return false if out of bounds
	void poses_mutex_lock() override {}
	void poses_mutex_unlock() override {}

   protected:
	virtual void internalGuiUpdate(
		mrpt::opengl::COpenGLScene& viz, mrpt::opengl::COpenGLScene& physical,
		bool childrenOnly) override;

	/** This object holds both, the mesh data, and is in charge of 3D rendering.
	 */
	mrpt::opengl::CMesh::Ptr m_gl_mesh;
	bool m_first_scene_rendering = true;
	float m_resolution = 1.0f;
	/** A copy of elevation data in m_gl_mesh. Coordinate order is (y,x) */
	mrpt::math::CMatrixFloat m_mesh_z_cache;

   private:
	// temp vars (declared here to avoid reallocs):
	mrpt::tfest::TMatchingPairList corrs;
	mrpt::poses::CPose3D m_optimal_transf;
};
}  // namespace mvsim
