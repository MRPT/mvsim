/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2024  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/opengl/COpenGLScene.h>
#include <mrpt/version.h>
#include <mvsim/VehicleBase.h>
#include <mvsim/World.h>
#include <mvsim/WorldElements/ElevationMap.h>

#include <limits>
#include <rapidxml.hpp>

#include "../parse_utils.h"
#include "xml_utils.h"

using namespace rapidxml;
using namespace mvsim;
using namespace std;

namespace
{
mrpt::math::CMatrixFloat applyConvolution(
	const mrpt::math::CMatrixFloat& data, const mrpt::math::CMatrixDouble& kernel)
{
	// Dimensions of the data and kernel
	const size_t rows = data.rows();
	const size_t cols = data.cols();
	const size_t kernelSize = kernel.rows();
	const size_t kernelRadius = kernelSize / 2;

	// Ensure kernel is square and normalized
	ASSERT_EQUAL_(kernelSize, static_cast<size_t>(kernel.cols()));

	ASSERTMSG_(std::abs(kernel.sum() - 1.0) < 5e-3, "Kernel must be normalized (sum to 1)");

	// Output matrix
	mrpt::math::CMatrixFloat result(rows, cols);

	// Apply convolution
	for (size_t i = 0; i < rows; ++i)
	{
		for (size_t j = 0; j < cols; ++j)
		{
			double sum = 0.0;

			// Convolution loop over kernel
			for (int ki = -kernelRadius; ki <= static_cast<int>(kernelRadius); ++ki)
			{
				for (int kj = -kernelRadius; kj <= static_cast<int>(kernelRadius); ++kj)
				{
					int ni = i + ki;
					int nj = j + kj;

					// Boundary check
					if (ni >= 0 && ni < static_cast<int>(rows) && nj >= 0 &&
						nj < static_cast<int>(cols))
					{
						sum += data(ni, nj) * kernel(ki + kernelRadius, kj + kernelRadius);
					}
				}
			}
			result(i, j) = sum;
		}
	}
	return result;
}
}  // namespace

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

	std::string sDemTextFile;
	params["dem_xyzrgb_file"] = TParamEntry("%s", &sDemTextFile);

	double img_min_z = 0.0, img_max_z = 5.0;
	params["elevation_image_min_z"] = TParamEntry("%lf", &img_min_z);
	params["elevation_image_max_z"] = TParamEntry("%lf", &img_max_z);

	double corner_min_x = std::numeric_limits<double>::max();
	double corner_min_y = std::numeric_limits<double>::max();

	params["corner_min_x"] = TParamEntry("%lf", &corner_min_x);
	params["corner_min_y"] = TParamEntry("%lf", &corner_min_y);

	mrpt::img::TColor mesh_color(0xa0, 0xe0, 0xa0);
	params["mesh_color"] = TParamEntry("%color", &mesh_color);

	params["resolution"] = TParamEntry("%lf", &resolution_);
	params["texture_extension_x"] = TParamEntry("%lf", &textureExtensionX_);
	params["texture_extension_y"] = TParamEntry("%lf", &textureExtensionY_);

	params["model_split_size"] = TParamEntry("%lf", &model_split_size_);

	std::string convolution_kernel_str;
	params["apply_kernel"] = TParamEntry("%s", &convolution_kernel_str);

	parse_xmlnode_children_as_param(*root, params, world_->user_defined_variables());

	// Load elevation data & (optional) image data:
	mrpt::math::CMatrixFloat elevation_data;
	std::optional<mrpt::img::CImage> mesh_image;

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
	else if (!sElevationMatrixData.empty())
	{
		sElevationMatrixData = mrpt::system::trim(sElevationMatrixData);

		std::stringstream sErrors;
		if (!elevation_data.fromMatlabStringFormat(sElevationMatrixData, sErrors))
		{
			THROW_EXCEPTION_FMT("Error parsing <elevation_data_matrix>: %s", sErrors.str().c_str());
		}
	}
	else
	{
		ASSERTMSG_(
			!sDemTextFile.empty(),
			"Either <elevation_image>, <elevation_data_matrix> or <dem_xyzrgb_file> must be "
			"provided");

		sDemTextFile = world_->local_to_abs_path(sDemTextFile);

		mrpt::math::CMatrixDouble data;
		data.loadFromTextFile(sDemTextFile);
		ASSERTMSG_(data.cols() == 6, "DEM txt file format error: expected 6 columns (x,y,z,r,g,b)");

		// Points from DEM geographic sources are not sorted, not even uniformly sampled.
		// Let's re-sample them:
		const double minx = data.col(0).minCoeff();
		const double maxx = data.col(0).maxCoeff();
		const double miny = data.col(1).minCoeff();
		const double maxy = data.col(1).maxCoeff();

		corner_min_x = minx;
		corner_min_y = miny;

		const auto nx = static_cast<unsigned int>(std::ceil((maxx - minx) / resolution_));
		const auto ny = static_cast<unsigned int>(std::ceil((maxy - miny) / resolution_));

		parent()->logFmt(
			mrpt::system::LVL_DEBUG,
			"[ElevationMap] Loaded %u points, min_corner=(%lf,%lf), max_corner=(%lf,%lf), "
			"cells=(%u,%u)",
			static_cast<unsigned>(data.rows()), minx, miny, maxx, maxy, nx, ny);

		// Store points in a map for using it as a KD-tree:
		// (this could be avoided writing a custom adaptor for nanoflann, but I don't
		//  have time for it now)
		mrpt::maps::CSimplePointsMap pts;
		pts.reserve(data.rows());
		// Insert points wrt the min. corner, to ensure accuracy with float's instead of double's:
		for (int i = 0; i < data.rows(); i++)
			pts.insertPoint(data(i, 0) - minx, data(i, 1) - miny, data(i, 2));

		pts.kdTreeEnsureIndexBuilt2D();	 // 2D queries, not 3D!
		elevation_data.resize(nx, ny);

		// Image data: rows=>+X in the world; cols=>+Y in the world
		// So we access image like: mesh_image(col,row)=>(cy,cx)
		mesh_image.emplace();
		mesh_image->resize(ny, nx, mrpt::img::CH_RGB);

		for (unsigned int cx = 0; cx < nx; cx++)
		{
			const float lx = (0.5f + cx) * resolution_;
			for (unsigned int cy = 0; cy < ny; cy++)
			{
				const float ly = (0.5f + cy) * resolution_;
				float closestSqrErr = 0;
				const auto idxPt = pts.kdTreeClosestPoint2D(lx, ly, closestSqrErr);
				// Store data in the cell:
				elevation_data(cx, cy) = data(idxPt, 2 /*z*/);
				const uint8_t R = data(idxPt, 3);
				const uint8_t G = data(idxPt, 4);
				const uint8_t B = data(idxPt, 5);
				// mesh_image->setPixel(cy, cx, mrpt::img::TColor(R, G, B));
				auto* dest = &mesh_image->ptrLine<uint8_t>(cx)[3 * cy];
				// Copy the color:
				*dest++ = B;
				*dest++ = G;
				*dest++ = R;
			}
		}

	}  // end resample DEM geographic data

	// Load texture (if not defined already above):
	if (!mesh_image && !sTextureImgFile.empty())
	{
		sTextureImgFile = world_->xmlPathToActualPath(sTextureImgFile);
		mesh_image.emplace();

		if (!mesh_image->loadFromFile(sTextureImgFile))
			throw std::runtime_error(mrpt::format(
				"[ElevationMap] ERROR: Cannot read texture image '%s'", sTextureImgFile.c_str()));

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
				mesh_image->rotateImage(
					im, mrpt::DEG2RAD(texture_rotate), mesh_image->getWidth() / 2,
					mesh_image->getHeight() / 2);
				mesh_image = std::move(im);
			}
			break;
			default:
				THROW_EXCEPTION("texture_image_rotate can only be: 0, 90, -90, 180");
		}
	}

	// Optional height filtering:
	if (!convolution_kernel_str.empty())
	{
		mrpt::math::CMatrixDouble kernel;
		std::stringstream ss(mvsim::trim(convolution_kernel_str));
		try
		{
			kernel.loadFromTextFile(ss);
			ASSERT_(kernel.cols() == kernel.rows());
			ASSERT_(kernel.cols() > 1);
		}
		catch (const std::exception& e)
		{
			THROW_EXCEPTION_FMT(
				"Error parsing kernel as matrix: '%s'.\nError: %s", convolution_kernel_str.c_str(),
				e.what());
		}

		parent()->logFmt(
			mrpt::system::LVL_DEBUG, "[ElevationMap] Applying filtering convolution filter %ux%u",
			static_cast<unsigned>(kernel.rows()), static_cast<unsigned>(kernel.cols()));

		elevation_data = applyConvolution(elevation_data, kernel);

	}  // end apply convolution kernel

	// Extension: X,Y
	const double LX = (elevation_data.rows() - 1) * resolution_;
	const double LY = (elevation_data.cols() - 1) * resolution_;

	if (corner_min_x == std::numeric_limits<double>::max()) corner_min_x = -0.5 * LX;
	if (corner_min_y == std::numeric_limits<double>::max()) corner_min_y = -0.5 * LY;

	// Propose to the "world" to use this coordinates as reference
	// for opengl to work with very large coordinates (e.g. UTM)
	parent()->worldRenderOffsetPropose({-corner_min_x, -corner_min_y, .0});

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

		if (mesh_image)
		{
			gl_mesh->assignImageAndZ(*mesh_image, elevation_data);
			gl_mesh->setMeshTextureExtension(textureExtensionX_, textureExtensionY_);
		}
		else
		{
			gl_mesh->setZ(elevation_data);
			gl_mesh->setColor_u8(mesh_color);
		}

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

				if (mesh_image)
				{
					gl_mesh->assignImageAndZ(*mesh_image, subEle);
					gl_mesh->setMeshTextureExtension(textureExtensionX_, textureExtensionY_);
				}
				else
				{
					gl_mesh->setZ(subEle);
					gl_mesh->setColor_u8(mesh_color);
				}

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
			glMesh->setPose(parent()->applyWorldRenderOffset(mrpt::poses::CPose3D::Identity()));

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
double calcz(
	const mrpt::math::TPoint3D& p1, const mrpt::math::TPoint3D& p2, const mrpt::math::TPoint3D& p3,
	double x, double y)
{
	const double det = (p2.x - p3.x) * (p1.y - p3.y) +	//
					   (p3.y - p2.y) * (p1.x - p3.x);
	ASSERT_(det != 0.0);

	const double l1 = ((p2.x - p3.x) * (y - p3.y) + (p3.y - p2.y) * (x - p3.x)) / det;
	const double l2 = ((p3.x - p1.x) * (y - p3.y) + (p1.y - p3.y) * (x - p3.x)) / det;
	const double l3 = 1.0 - l1 - l2;

	return l1 * p1.z + l2 * p2.z + l3 * p3.z;
}
}  // namespace

std::optional<float> ElevationMap::getElevationAt(const mrpt::math::TPoint2D& pt) const
{
	// mesh->getxMin();
	const double x0 = meshMinX_;
	const double y0 = meshMinY_;
	const double x1 = meshMaxX_;
	const double y1 = meshMaxY_;

	const size_t nCellsX = meshCacheZ_.rows();
	const size_t nCellsY = meshCacheZ_.cols();

	const double sCellX = (x1 - x0) / (nCellsX - 1);
	const double sCellY = (y1 - y0) / (nCellsY - 1);

	// Discretize:
	const int cx00 = ::floor((pt.x - x0) / sCellX);
	const int cy00 = ::floor((pt.y - y0) / sCellY);

	if (cx00 < 0 || cx00 >= int(nCellsX - 1) || cy00 < 0 || cy00 >= int(nCellsY - 1))  //
		return {};	// out of bounds!

	// Linear interpolation:
	const double z00 = meshCacheZ_(cx00, cy00);
	const double z01 = meshCacheZ_(cx00, cy00 + 1);
	const double z10 = meshCacheZ_(cx00 + 1, cy00);
	const double z11 = meshCacheZ_(cx00 + 1, cy00 + 1);

	//
	//   p01 ---- p11
	//    |        |
	//   p00 ---- p10
	//
	const mrpt::math::TPoint3D p00(.0, .0, z00);
	const mrpt::math::TPoint3D p01(.0, sCellY, z01);
	const mrpt::math::TPoint3D p10(sCellX, .0, z10);
	const mrpt::math::TPoint3D p11(sCellX, sCellY, z11);

	const double lx = pt.x - (x0 + cx00 * sCellX);
	const double ly = pt.y - (y0 + cy00 * sCellY);

	if (ly >= lx)
		return calcz(p00, p01, p11, lx, ly);
	else
		return calcz(p00, p10, p11, lx, ly);
}
