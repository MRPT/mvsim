/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2024  Jose Luis Blanco Claraco                       |
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

	bool getElevationAt(double x, double y, float& z) const;  //!< return false if out of bounds

   protected:
	virtual void internalGuiUpdate(
		const mrpt::optional_ref<mrpt::opengl::COpenGLScene>& viz,
		const mrpt::optional_ref<mrpt::opengl::COpenGLScene>& physical, bool childrenOnly) override;

	/** This object holds both, the mesh data, and is in charge of 3D rendering.
	 */
	mrpt::opengl::CMesh::Ptr gl_mesh_;
	std::shared_ptr<mrpt::opengl::CPointCloud> gl_debugWheelsContactPoints_;
	bool firstSceneRendering_ = true;
	float resolution_ = 1.0f;

	float textureExtensionX_ = 0;  //!< 0=auto
	float textureExtensionY_ = 0;  //!< 0=auto

	/** A copy of elevation data in gl_mesh_. Coordinate order is (x,y) */
	mrpt::math::CMatrixFloat meshCacheZ_;

	bool debugShowContactPoints_ = false;

   private:
	// temp vars (declared here to avoid reallocs):
	mrpt::tfest::TMatchingPairList corrs_;
	mrpt::poses::CPose3D optimalTf_;
};
}  // namespace mvsim
