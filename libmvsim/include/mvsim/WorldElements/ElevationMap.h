/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014  Jose Luis Blanco Claraco (University of Almeria)    |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under GNU General Public License version 3                  |
  |   See <http://www.gnu.org/licenses/>                                    |
  +-------------------------------------------------------------------------+ */

#pragma once

#include <mvsim/WorldElements/WorldElementBase.h>
#include <mrpt/opengl/CMesh.h>
#include <mrpt/poses/CPose3D.h>

#include <mrpt/version.h>
#if MRPT_VERSION>=0x199
#include <mrpt/tfest/TMatchingPair.h>
#include <mrpt/img/CImage.h>
using mrpt::tfest::TMatchingPairList;
using mrpt::tfest::TMatchingPair;
using mrpt::img::CImage;
#else
#include <mrpt/utils/CImage.h>
#include <mrpt/utils/TMatchingPair.h>
using mrpt::utils::TMatchingPairList;
using mrpt::utils::TMatchingPair;
using mrpt::utils::CImage;
#endif


namespace mvsim
{
class ElevationMap : public WorldElementBase
{
	DECLARES_REGISTER_WORLD_ELEMENT(ElevationMap)
   public:
	ElevationMap(World* parent, const rapidxml::xml_node<char>* root);
	virtual ~ElevationMap();

	virtual void loadConfigFrom(
		const rapidxml::xml_node<char>* root);  //!< See docs in base class
	virtual void gui_update(
		mrpt::opengl::COpenGLScene& scene);  //!< See docs in base class

	virtual void simul_pre_timestep(
		const TSimulContext& context);  //!< See docs in base class
	virtual void simul_post_timestep(
		const TSimulContext& context);  //!< See docs in base class

	bool getElevationAt(
		double x, double y, float& z) const;  //!< return false if out of bounds

   protected:
	/** This object holds both, the mesh data, and is in charge of 3D rendering.
	 */
	mrpt::opengl::CMesh::Ptr m_gl_mesh;
	bool m_first_scene_rendering;
	double m_resolution;
	mrpt::math::CMatrixFloat m_mesh_z_cache;  //!< A copy of elevation data in
											  //!m_gl_mesh. Coordinate order is
											  //!(x,y)

   private:
	// temp vars (declared here to avoid reallocs):
	TMatchingPairList corrs;
	mrpt::poses::CPose3D m_optimal_transf;
};
}
