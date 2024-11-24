/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2024  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#include <mrpt/opengl/COpenGLScene.h>
#include <mrpt/version.h>
#include <mvsim/VehicleBase.h>
#include <mvsim/World.h>
#include <mvsim/WorldElements/ElevationMap.h>

#include <limits>
#include <rapidxml.hpp>

#include "xml_utils.h"

using namespace rapidxml;
using namespace mvsim;
using namespace std;

ElevationMap::ElevationMap(World* parent, const rapidxml::xml_node<char>* root)
	: WorldElementBase(parent)
{
	ElevationMap::loadConfigFrom(root);
}

ElevationMap::~ElevationMap() {}
void ElevationMap::loadConfigFrom(const rapidxml::xml_node<char>* root)
{
	// Other general params:
	TParameterDefinitions params;

	std::string sElevationImgFile;
	params["elevation_image"] = TParamEntry("%s", &sElevationImgFile);
	std::string sTextureImgFile;
	params["texture_image"] = TParamEntry("%s", &sTextureImgFile);
	int texture_rotate = 0;
	params["texture_image_rotate"] = TParamEntry("%i", &texture_rotate);

	std::string sElevationMatrixData;
	params["elevation_data_matrix"] = TParamEntry("%s", &sElevationMatrixData);

	double img_min_z = 0.0, img_max_z = 5.0;
	params["elevation_image_min_z"] = TParamEntry("%lf", &img_min_z);
	params["elevation_image_max_z"] = TParamEntry("%lf", &img_max_z);

	double corner_min_x = std::numeric_limits<double>::max();
	double corner_min_y = std::numeric_limits<double>::max();

	params["corner_min_x"] = TParamEntry("%lf", &corner_min_x);
	params["corner_min_y"] = TParamEntry("%lf", &corner_min_y);

	mrpt::img::TColor mesh_color(0xa0, 0xe0, 0xa0);
	params["mesh_color"] = TParamEntry("%color", &mesh_color);

	params["resolution"] = TParamEntry("%f", &resolution_);
	params["texture_extension_x"] = TParamEntry("%f", &textureExtensionX_);
	params["texture_extension_y"] = TParamEntry("%f", &textureExtensionY_);

	params["model_split_size"] = TParamEntry("%f", &model_split_size_);

	parse_xmlnode_children_as_param(*root, params, world_->user_defined_variables());

	// Load elevation data:
	mrpt::math::CMatrixFloat elevation_data;
	if (!sElevationImgFile.empty())
	{
		sElevationImgFile = world_->local_to_abs_path(sElevationImgFile);

		mrpt::img::CImage imgElev;
		if (!imgElev.loadFromFile(sElevationImgFile, 0 /*force load grayscale*/))
			throw std::runtime_error(mrpt::format(
				"[ElevationMap] ERROR: Cannot read elevation image '%s'",
				sElevationImgFile.c_str()));

		// Scale: [0,1] => [min_z,max_z]
		// Get image normalized in range [0,1]
		imgElev.getAsMatrix(elevation_data);
		ASSERT_(img_min_z != img_max_z);

		const double vmin = elevation_data.minCoeff();
		const double vmax = elevation_data.maxCoeff();
		mrpt::math::CMatrixFloat f = elevation_data;
		f -= vmin;
		f *= (img_max_z - img_min_z) / (vmax - vmin);
		mrpt::math::CMatrixFloat m(elevation_data.rows(), elevation_data.cols());
		m.setConstant(img_min_z);
		f += m;
		elevation_data = std::move(f);
	}
	else
	{
		ASSERTMSG_(
			!sElevationMatrixData.empty(),
			"Either <elevation_image> or <elevation_data_matrix> must be provided");

		sElevationMatrixData = mrpt::system::trim(sElevationMatrixData);

		std::stringstream sErrors;
		if (!elevation_data.fromMatlabStringFormat(sElevationMatrixData, sErrors))
		{
			THROW_EXCEPTION_FMT("Error parsing <elevation_data_matrix>: %s", sErrors.str().c_str());
		}
	}

	// Load texture (optional):
	mrpt::img::CImage mesh_image;
	bool has_mesh_image = false;
	if (!sTextureImgFile.empty())
	{
		sTextureImgFile = world_->xmlPathToActualPath(sTextureImgFile);

		if (!mesh_image.loadFromFile(sTextureImgFile))
			throw std::runtime_error(mrpt::format(
				"[ElevationMap] ERROR: Cannot read texture image '%s'", sTextureImgFile.c_str()));
		has_mesh_image = true;

		// Apply rotation:
		switch (texture_rotate)
		{
			case 0:
				break;
			case 90:
			case -90:
			case 180:
			case -180:
			{
				mrpt::img::CImage im;
				mesh_image.rotateImage(
					im, mrpt::DEG2RAD(texture_rotate), mesh_image.getWidth() / 2,
					mesh_image.getHeight() / 2);
				mesh_image = std::move(im);
			}
			break;
			default:
				THROW_EXCEPTION("texture_image_rotate can only be: 0, 90, -90, 180");
		}
	}

	// Extension: X,Y
	const double LX = (elevation_data.rows() - 1) * resolution_;
	const double LY = (elevation_data.cols() - 1) * resolution_;

	if (corner_min_x == std::numeric_limits<double>::max()) corner_min_x = -0.5 * LX;
	if (corner_min_y == std::numeric_limits<double>::max()) corner_min_y = -0.5 * LY;

	// Save copy for calcs:
	meshCacheZ_ = elevation_data;
	meshMinX_ = corner_min_x;
	meshMinY_ = corner_min_y;
	meshMaxX_ = corner_min_x + LX;
	meshMaxY_ = corner_min_y + LY;

	// Build mesh:
	ASSERT_GE_(model_split_size_, .0f);
	if (model_split_size_ == 0)
	{
		// One single mesh:
		auto gl_mesh = mrpt::opengl::CMesh::Create();
		gl_meshes_.push_back(gl_mesh);

		gl_mesh->enableTransparency(false);

		if (has_mesh_image)
		{
			gl_mesh->assignImageAndZ(mesh_image, elevation_data);
			gl_mesh->setMeshTextureExtension(textureExtensionX_, textureExtensionY_);
		}
		else
		{
			gl_mesh->setZ(elevation_data);
			gl_mesh->setColor_u8(mesh_color);
		}

		// Important: the yMin/yMax in the next line are swapped to handle
		// the "+y" different direction in image and map coordinates, it is not
		// a bug:
		gl_mesh->setGridLimits(corner_min_x, corner_min_x + LX, corner_min_y, corner_min_y + LY);

		// hint for rendering z-order:
		gl_mesh->setLocalRepresentativePoint(
			mrpt::math::TPoint3Df(corner_min_x + 0.5 * LX, corner_min_y + 0.5 * LY, .0f));
	}
	else
	{
		// Split in smaller meshes:
		const int M = static_cast<int>(std::ceil(model_split_size_ / resolution_));
		const double subSize = M * resolution_;
		const size_t NX = static_cast<size_t>(std::ceil(LX / subSize));
		const size_t NY = static_cast<size_t>(std::ceil(LY / subSize));
		for (size_t iX = 0; iX < NX; iX++)
		{
			// (recall: rows=X, cols=Y)
			// M+1: we need to duplicate the elevation data from border cells to neighboring
			// blocks to ensure continuity.

			const size_t startIx = iX * M;
			const size_t lenIx_p = std::min<size_t>(M, elevation_data.rows() - startIx);
			const size_t lenIx = std::min<size_t>(M + 1, elevation_data.rows() - startIx);

			for (size_t iY = 0; iY < NY; iY++)
			{
				const size_t startIy = iY * M;
				const size_t lenIy_p = std::min<size_t>(M, elevation_data.cols() - startIy);
				const size_t lenIy = std::min<size_t>(M + 1, elevation_data.cols() - startIy);

				// Extract sub-matrix for elevation data:
				const auto subEle = elevation_data.extractMatrix(lenIx, lenIy, startIx, startIy);

				// One sub-mesh:
				auto gl_mesh = mrpt::opengl::CMesh::Create();
				gl_meshes_.push_back(gl_mesh);

				gl_mesh->enableTransparency(false);

				if (has_mesh_image)
				{
					gl_mesh->assignImageAndZ(mesh_image, subEle);
					gl_mesh->setMeshTextureExtension(textureExtensionX_, textureExtensionY_);
				}
				else
				{
					gl_mesh->setZ(subEle);
					gl_mesh->setColor_u8(mesh_color);
				}

				// Important: the yMin/yMax in the next line are swapped to handle
				// the "+y" different direction in image and map coordinates, it is not
				// a bug:
				gl_mesh->setGridLimits(
					corner_min_x + iX * subSize,
					corner_min_x + iX * subSize + lenIx_p * resolution_,
					corner_min_y + iY * subSize,
					corner_min_y + iY * subSize + lenIy_p * resolution_);

				// hint for rendering z-order:
				gl_mesh->setLocalRepresentativePoint(mrpt::math::TPoint3Df(
					corner_min_x + (iX + 0.5) * subSize, corner_min_y + (iY + 0.5) * subSize,
					subEle(0, 0)));
			}
		}
	}
}

void ElevationMap::internalGuiUpdate(
	const mrpt::optional_ref<mrpt::opengl::COpenGLScene>& viz,
	const mrpt::optional_ref<mrpt::opengl::COpenGLScene>& physical,
	[[maybe_unused]] bool childrenOnly)
{
	ASSERTMSG_(
		!gl_meshes_.empty(),
		"ERROR: Can't render Mesh before loading it! Have you called "
		"loadConfigFrom() first?");

	// 1st time call?? -> Create objects
	if (firstSceneRendering_ && viz && physical)
	{
		firstSceneRendering_ = false;
		for (const auto& glMesh : gl_meshes_)
		{
			viz->get().insert(glMesh);
			physical->get().insert(glMesh);
		}
	}
}

void ElevationMap::simul_pre_timestep([[maybe_unused]] const TSimulContext& context)
{
	// Nothing special to do.
	// Since Sep-2024, this functionality has moved to
	// World::internal_simul_pre_step_terrain_elevation()
}

void ElevationMap::simul_post_timestep(const TSimulContext& context)
{
	Simulable::simul_post_timestep(context);

	// TODO: Think if this is still applicable:
	// Save all elements positions in prestep, then here scale their
	// movements * cos(angle)
}

namespace
{
float calcz(
	const mrpt::math::TPoint3Df& p1, const mrpt::math::TPoint3Df& p2,
	const mrpt::math::TPoint3Df& p3, float x, float y)
{
	const float det = (p2.x - p3.x) * (p1.y - p3.y) +  //
					  (p3.y - p2.y) * (p1.x - p3.x);
	ASSERT_(det != 0.0f);

	const float l1 = ((p2.x - p3.x) * (y - p3.y) + (p3.y - p2.y) * (x - p3.x)) / det;
	const float l2 = ((p3.x - p1.x) * (y - p3.y) + (p1.y - p3.y) * (x - p3.x)) / det;
	const float l3 = 1.0f - l1 - l2;

	return l1 * p1.z + l2 * p2.z + l3 * p3.z;
}
}  // namespace

std::optional<float> ElevationMap::getElevationAt(const mrpt::math::TPoint2Df& pt) const
{
	// mesh->getxMin();
	const float x0 = meshMinX_;
	const float y0 = meshMinY_;
	const float x1 = meshMaxX_;
	const float y1 = meshMaxY_;

	const size_t nCellsX = meshCacheZ_.rows();
	const size_t nCellsY = meshCacheZ_.cols();

	const float sCellX = (x1 - x0) / (nCellsX - 1);
	const float sCellY = (y1 - y0) / (nCellsY - 1);

	// Discretize:
	const int cx00 = ::floor((pt.x - x0) / sCellX);
	const int cy00 = ::floor((pt.y - y0) / sCellY);

	if (cx00 < 0 || cx00 >= int(nCellsX - 1) || cy00 < 0 || cy00 >= int(nCellsY - 1))  //
		return {};	// out of bounds!

	// Linear interpolation:
	const float z00 = meshCacheZ_(cx00, cy00);
	const float z01 = meshCacheZ_(cx00, cy00 + 1);
	const float z10 = meshCacheZ_(cx00 + 1, cy00);
	const float z11 = meshCacheZ_(cx00 + 1, cy00 + 1);

	//
	//   p01 ---- p11
	//    |        |
	//   p00 ---- p10
	//
	const mrpt::math::TPoint3Df p00(.0f, .0f, z00);
	const mrpt::math::TPoint3Df p01(.0f, sCellY, z01);
	const mrpt::math::TPoint3Df p10(sCellX, .0f, z10);
	const mrpt::math::TPoint3Df p11(sCellX, sCellY, z11);

	const float lx = pt.x - (x0 + cx00 * sCellX);
	const float ly = pt.y - (y0 + cy00 * sCellY);

	if (ly >= lx)
		return calcz(p00, p01, p11, lx, ly);
	else
		return calcz(p00, p10, p11, lx, ly);
}
