/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2026  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#include <box2d/b2_settings.h>	// b2_maxPolygonVertices
#include <mrpt/containers/yaml.h>
#include <mrpt/core/get_env.h>
#include <mrpt/opengl/CAssimpModel.h>
#include <mrpt/opengl/CBox.h>
#include <mrpt/opengl/CCylinder.h>
#include <mrpt/opengl/CSphere.h>
#include <mrpt/version.h>
#include <mvsim/CollisionShapeCache.h>

using namespace mvsim;

CollisionShapeCache& CollisionShapeCache::Instance()
{
	static CollisionShapeCache o;
	return o;
}

Shape2p5 CollisionShapeCache::get(
	mrpt::opengl::CRenderizable& obj, float zMin, float zMax, const mrpt::poses::CPose3D& modelPose,
	const float modelScale, const std::optional<std::string>& modelFile)
{
	// already cached?
	if (modelFile)
	{
		if (auto it = cache.find(modelFile.value()); it != cache.end())
		{
			return it->second.shape;
		}
	}

	// No, it's a new model path, create its placeholder:
	Shape2p5 retVal;
	Shape2p5& ret = modelFile ? cache[*modelFile].shape : retVal;

	// Now, decide whether it's a simple geometry, or an arbitrary model:
	const auto simpleGeom = processSimpleGeometries(obj, zMin, zMax, modelPose, modelScale);

	if (simpleGeom)
	{
		ret = simpleGeom.value();
	}
	else
	{
		ret = processGenericGeometry(obj, zMin, zMax, modelPose, modelScale);
	}

	const auto vol = ret.volume();

	const thread_local bool MVSIM_COLLISION_SHAPE_CACHE_VERBOSE =
		mrpt::get_env<bool>("MVSIM_COLLISION_SHAPE_CACHE_VERBOSE");
	if (MVSIM_COLLISION_SHAPE_CACHE_VERBOSE)
	{
		std::cout << "shape2.5 for [" << (modelFile.has_value() ? *modelFile : "none")
				  << "] glClass=" << obj.GetRuntimeClass()->className
				  << " shape=" << ret.getContour().size() << " pts, "
				  << " volume=" << vol << " zMin=" << ret.zMin() << " zMax=" << ret.zMax()
				  << " modelScale= " << modelScale
				  << " was simpleGeom=" << (simpleGeom ? "yes" : "no") << "\n"
				  << ret.getContour().asYAML() << "\n\n";
	}

	if (vol < 1e-8)
	{
		THROW_EXCEPTION_FMT(
			"Error: Collision volume for visual model ('%s') has almost null "
			"volume (=%g m³). A possible cause, if this is a <block>, is not "
			"enough vertices within the given range [zmin,zmax]",
			modelFile.has_value() ? modelFile->c_str() : "none", vol);
	}

	return ret;
}

std::optional<Shape2p5> CollisionShapeCache::processSimpleGeometries(
	const mrpt::opengl::CRenderizable& obj, float zMin, float zMax,
	const mrpt::poses::CPose3D& modelPose, const float modelScale)
{
	using mrpt::literals::operator""_deg;

	if (auto oCyl = dynamic_cast<const mrpt::opengl::CCylinder*>(&obj); oCyl)
	{
		// ===============================
		// Cylinder
		// ===============================
		// If the cylinder is not upright, skip and go for the generic algorithm
		if (std::abs(modelPose.pitch()) > 0.02_deg || std::abs(modelPose.roll()) > 0.02_deg)
		{
			return {};
		}

		const size_t actualEdgeCount = oCyl->getSlicesCount();
		double actualRadius = std::max<double>(oCyl->getTopRadius(), oCyl->getBottomRadius());

		return processCylinderLike(
			actualEdgeCount, actualRadius, zMin, zMax, modelPose, modelScale);
	}

	if (auto oSph = dynamic_cast<const mrpt::opengl::CSphere*>(&obj); oSph)
	{
		// ===============================
		// Sphere
		// ===============================
		const size_t actualEdgeCount = oSph->getNumberOfSegments();

		double actualRadius = oSph->getRadius();

		return processCylinderLike(
			actualEdgeCount, actualRadius, zMin, zMax, modelPose, modelScale);
	}

	if (auto oBox = dynamic_cast<const mrpt::opengl::CBox*>(&obj); oBox)
	{
		// ===============================
		// Box
		// ===============================
		// If the object is not upright, skip and go for the generic algorithm
		if (std::abs(modelPose.pitch()) > 0.02_deg || std::abs(modelPose.roll()) > 0.02_deg)
		{
			return {};
		}

		mrpt::math::TPoint3D p1, p2;
		oBox->getBoxCorners(p1, p2);
		p1 *= modelScale;
		p2 *= modelScale;

		const mrpt::math::TPoint3D corners[4] = {
			modelPose.composePoint({p1.x, p1.y, 0}), modelPose.composePoint({p1.x, p2.y, 0}),
			modelPose.composePoint({p2.x, p2.y, 0}), modelPose.composePoint({p2.x, p1.y, 0})};

		mrpt::math::TPolygon2D contour;
		for (int i = 0; i < 4; i++)
		{
			contour.emplace_back(corners[i].x, corners[i].y);
		}

		Shape2p5 s;
		s.setShapeManual(contour, zMin, zMax);
		return {s};
	}

	// unknown:
	return {};
}

Shape2p5 CollisionShapeCache::processGenericGeometry(
	mrpt::opengl::CRenderizable& obj, float zMin, float zMax, const mrpt::poses::CPose3D& modelPose,
	const float modelScale)
{
	Shape2p5 ret;

	// Make sure the points and vertices buffers are up to date, so we can
	// access them:
	auto* oAssimp = dynamic_cast<mrpt::opengl::CAssimpModel*>(&obj);
	if (oAssimp)
	{
		oAssimp->onUpdateBuffers_all();
	}
	auto* oRSWF = dynamic_cast<mrpt::opengl::CRenderizableShaderWireFrame*>(&obj);
	if (oRSWF)
	{
		oRSWF->onUpdateBuffers_Wireframe();
	}
	auto* oRST = dynamic_cast<mrpt::opengl::CRenderizableShaderTriangles*>(&obj);
	if (oRST)
	{
		oRST->onUpdateBuffers_Triangles();
	}
	auto* oRSTT = dynamic_cast<mrpt::opengl::CRenderizableShaderTexturedTriangles*>(&obj);
	if (oRSTT)
	{
		oRSTT->onUpdateBuffers_TexturedTriangles();
	}
	auto* oRP = dynamic_cast<mrpt::opengl::CRenderizableShaderPoints*>(&obj);
	if (oRP)
	{
		oRP->onUpdateBuffers_Points();
	}

	// Slice bbox in z up to a given relevant height:
	size_t numTotalPts = 0, numPassedPts = 0;

	auto rawBB = obj.getBoundingBox();
	// transform and scale BBox too:
	rawBB.max *= modelScale;
	rawBB.min *= modelScale;
	const auto coarseBB = rawBB.compose(modelPose);
	ret.buildInit(
		mrpt::math::TPoint2Df(coarseBB.min.x, coarseBB.min.y),
		mrpt::math::TPoint2Df(coarseBB.max.x, coarseBB.max.y));

	auto lambdaUpdatePt = [&](const mrpt::math::TPoint3Df& orgPt)
	{
		numTotalPts++;
		auto pt = modelPose.composePoint(orgPt * modelScale);
		if (pt.z < zMin || pt.z > zMax)
		{
			return;	 // skip
		}
		ret.buildAddPoint(pt);
		numPassedPts++;
	};
	auto lambdaUpdateTri = [&](const mrpt::opengl::TTriangle& tri)
	{
		numTotalPts += 3;
		// transform the whole triangle, then compare with [z,z] limits:
		mrpt::opengl::TTriangle t = tri;
		for (int i = 0; i < 3; i++) t.vertex(i) = modelPose.composePoint(t.vertex(i) * modelScale);

		// does any of the point lie within the valid Z range, or is the
		// triangle going all the way down to top?
		bool outDown = false, outUp = false, anyIn = false;
		for (int i = 0; i < 3; i++)
		{
			const auto& p = t.vertex(i);
			if (p.z >= zMin && p.z <= zMax)
			{
				anyIn = true;
				break;
			}
			if (p.z > zMax)
			{
				outUp = true;
			}
			if (p.z < zMin)
			{
				outDown = true;
			}
		}
		if (!(anyIn || (outUp && outDown)))
		{
			return;	 // skip triangle
		}

		ret.buildAddTriangle(t);
		numPassedPts += 3;
	};

	if (oRST)
	{
		auto lck = mrpt::lockHelper(oRST->shaderTrianglesBufferMutex().data);
		const auto& tris = oRST->shaderTrianglesBuffer();
		for (const auto& tri : tris)
		{
			lambdaUpdateTri(tri);
		}
	}
	if (oRSTT)
	{
		auto lck = mrpt::lockHelper(oRSTT->shaderTexturedTrianglesBufferMutex().data);
		const auto& tris = oRSTT->shaderTexturedTrianglesBuffer();
		for (const auto& tri : tris)
		{
			lambdaUpdateTri(tri);
		}
	}
	if (oRP)
	{
		auto lck = mrpt::lockHelper(oRP->shaderPointsBuffersMutex().data);
		const auto& pts = oRP->shaderPointsVertexPointBuffer();
		for (const auto& pt : pts)
		{
			lambdaUpdatePt(pt);
		}
	}
	if (oRSWF)
	{
		auto lck = mrpt::lockHelper(oRSWF->shaderWireframeBuffersMutex().data);
		const auto& pts = oRSWF->shaderWireframeVertexPointBuffer();
		for (const auto& pt : pts)
		{
			lambdaUpdatePt(pt);
		}
	}

	if (oAssimp)
	{
		const auto& txtrdObjs = oAssimp->texturedObjects();	 // mrpt>=2.6.0
		for (const auto& o : txtrdObjs)
		{
			if (!o)
			{
				continue;
			}

			auto lck = mrpt::lockHelper(o->shaderTexturedTrianglesBufferMutex().data);
			const auto& tris = o->shaderTexturedTrianglesBuffer();
			for (const auto& tri : tris)
			{
				lambdaUpdateTri(tri);
			}
		}
	}

	// Convert all points into an actual 2.5D volume:
	// ---------------------------------------------------------
	ret.clipZMin(zMin);
	ret.clipZMax(zMax);

	ret.getContour();  // evalute it now

	return ret;
}

Shape2p5 CollisionShapeCache::processCylinderLike(
	const size_t actualEdgeCount, double actualRadius, float zMin, float zMax,
	const mrpt::poses::CPose3D& modelPose, const float modelScale)
{
	const size_t maxEdges = b2_maxPolygonVertices;

	const auto nFaces = std::min<size_t>(maxEdges, actualEdgeCount);

	const bool isApprox = actualEdgeCount > maxEdges;
	if (isApprox)
	{
		const int i = mrpt::round(nFaces / 4);
		double newR = actualRadius;
		for (int j = -1; j <= 1; j++)
		{
			const double ang = (i + j) * 2 * M_PI / nFaces;
			const double angp1 = (i + j + 1) * 2 * M_PI / nFaces;
			const mrpt::math::TPoint2D pt0 = {cos(ang), sin(ang)};
			const mrpt::math::TPoint2D pt1 = {cos(angp1), sin(angp1)};

			const double midDist = ((pt0 + pt1) * 0.5).norm();

			newR = std::max(newR, actualRadius * (1.0 / midDist));
		}
		actualRadius = newR;
	}

	mrpt::math::TPolygon2D contour;
	for (size_t i = 0; i < nFaces; i++)
	{
		const double ang = i * 2 * M_PI / nFaces;
		const mrpt::math::TPoint3D localPt = {cos(ang) * actualRadius, sin(ang) * actualRadius, .0};
		const auto pt = modelPose.composePoint(localPt * modelScale);
		contour.emplace_back(pt.x, pt.y);
	}

	Shape2p5 s;
	s.setShapeManual(contour, zMin, zMax);
	return {s};
}
