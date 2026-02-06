/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2026  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/img/CImage.h>
#include <mrpt/opengl/CMesh.h>
#include <mvsim/WorldElements/WorldElementBase.h>

#include <vector>

namespace mvsim
{
/** Simulates a Digital Elevation Model (DEM) for a given terrain.
 *
 * Input data can be given from a fixed elevation matrix from the XML file,
 * an external image, etc. Refer to examples under mvsim_tutorials.
 *
 * \ingroup world_elements_module
 */
class ElevationMap : public WorldElementBase
{
	DECLARES_REGISTER_WORLD_ELEMENT(ElevationMap)
   public:
	ElevationMap(World* parent, const rapidxml::xml_node<char>* root);
	virtual ~ElevationMap();

	ElevationMap(const ElevationMap&) = delete;
	ElevationMap& operator=(const ElevationMap&) = delete;
	ElevationMap(ElevationMap&&) = delete;
	ElevationMap& operator=(ElevationMap&&) = delete;

	virtual void loadConfigFrom(const rapidxml::xml_node<char>* root) override;

	virtual void simul_pre_timestep(const TSimulContext& context) override;
	virtual void simul_post_timestep(const TSimulContext& context) override;

	std::optional<float> getElevationAt(const mrpt::math::TPoint2D& pt) const override;

   protected:
	virtual void internalGuiUpdate(
		const mrpt::optional_ref<mrpt::opengl::COpenGLScene>& viz,
		const mrpt::optional_ref<mrpt::opengl::COpenGLScene>& physical, bool childrenOnly) override;

	/** This object holds both, the mesh data, and is in charge of 3D rendering.
	 */
	std::vector<mrpt::opengl::CMesh::Ptr> gl_meshes_;
	bool firstSceneRendering_ = true;
	double resolution_ = 1.0f;

	double textureExtensionX_ = 0;	//!< 0=auto
	double textureExtensionY_ = 0;	//!< 0=auto

	/** A copy of elevation data in gl_mesh_. Coordinate order is (x,y) */
	mrpt::math::CMatrixDouble meshCacheZ_;
	double meshMinX_ = 0, meshMaxX_ = 0, meshMinY_ = 0, meshMaxY_ = 0;

	/// If enabled (>0), the mesh will be split into NxM smaller meshes with a max size of this
	/// value, to help correctly render semitransparent objects (e.g. trees).
	double model_split_size_ = .0f;
};
}  // namespace mvsim
