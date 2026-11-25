/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2026  Jose Luis Blanco Claraco                       |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

// mvsim-dataset-gen cameras, end to end: images are rendered at the exact
// times of the trajectory (a known landmark is where its projection says,
// with a global and a rolling shutter), depths match the geometry and the
// depth error model, the ground truth describes them, and cameras do not
// change the other streams. Requires off-screen OpenGL rendering (skipped if
// not available).

#include <mrpt/containers/yaml.h>
#include <mrpt/img/CImage.h>
#include <mrpt/io/CFileGZInputStream.h>
#include <mrpt/obs/CObservation3DRangeScan.h>
#include <mrpt/obs/CObservationImage.h>
#include <mrpt/obs/CObservationOdometry.h>
#include <mrpt/obs/CObservationPointCloud.h>
#include <mrpt/opengl/CFBORender.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/version.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <map>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

#include "test_utils.h"

int g_failures = 0;

namespace
{
constexpr double kT0 = 100.0;
constexpr double kSpeed = 1.0;	//!< Lateral (+Y) speed of the vehicle [m/s]
const mrpt::math::TPoint3D kLandmark(5.0, 0.3, 1.2);
constexpr double kWallX = 8.0;
constexpr double kCamX = 0.2;  //!< Camera positions on the vehicle

std::string tempPath(const std::string& suffix)
{
	return mrpt::system::getTempFileName() + "_" + suffix;
}

/** A flat world with a wall facing the cameras and a thin red disk (the landmark,
 * 20 cm wide, centered at kLandmark, facing the cameras and uniformly lit),
 * with an RGB camera (20 Hz) and an RGB-D camera (10 Hz). */
std::string writeWorld(const std::string& cameraTags, const std::string& rgbdTags)
{
	const std::string path = tempPath("cameras.world.xml");
	std::ofstream f(path);
	f << R"(<mvsim_world version="1.0">
  <element class="horizontal_plane"><x_min>-20</x_min><x_max>20</x_max><y_min>-20</y_min><y_max>20</y_max><z>0</z><color>#808080</color></element>
  <block name="wall"><static>true</static><color>#4060a0</color><zmin>0</zmin><zmax>4</zmax>
    <shape><pt>8 -10</pt><pt>8.5 -10</pt><pt>8.5 10</pt><pt>8 10</pt></shape><init_pose>0 0 0</init_pose></block>
  <block name="landmark"><static>true</static><skip_elevation_adjust/><color>#ff0000</color>
    <geometry type="cylinder" radius="0.1" length="0.001" vertex_count="64" />
    <init_pose3d>5 0.3 1.2 0 90 0</init_pose3d></block>
  <vehicle name="r1">
    <dynamics class="differential">
      <l_wheel pos="0  0.3" mass="1.0" width="0.05" diameter="0.20" />
      <r_wheel pos="0 -0.3" mass="1.0" width="0.05" diameter="0.20" />
      <chassis mass="10.0" zmin="0.05" zmax="0.25" />
    </dynamics>
    <init_pose>0 0 0</init_pose>
    <sensor class="lidar3d" name="lidar1"><pose_3d>0 0 0.75 0 0 0</pose_3d><vert_nrays>8</vert_nrays>
      <vert_fov_degrees>30</vert_fov_degrees><horz_nrays>360</horz_nrays><sensor_period>0.1</sensor_period>
      <range_std_noise>0.01</range_std_noise></sensor>
    <sensor class="camera" name="cam1">
      <pose_3d>0.2 0 1.2 -90 0 -90</pose_3d>
      <sensor_period>0.05</sensor_period>
      <ncols>640</ncols><nrows>480</nrows><cx>320</cx><cy>240</cy><fx>500</fx><fy>500</fy>
      <clip_min>0.05</clip_min><clip_max>200</clip_max>
      )"
	  << cameraTags << R"(
    </sensor>
    <sensor class="rgbd_camera" name="rgbd1">
      <pose_3d>0.2 0 1.0 0 0 0</pose_3d>
      <relativePoseIntensityWRTDepth>0 0 0 -90 0 -90</relativePoseIntensityWRTDepth>
      <sensor_period>0.1</sensor_period>
      <depth_ncols>320</depth_ncols><depth_nrows>240</depth_nrows><depth_cx>160</depth_cx><depth_cy>120</depth_cy>
      <depth_fx>250</depth_fx><depth_fy>250</depth_fy>
      <rgb_ncols>320</rgb_ncols><rgb_nrows>240</rgb_nrows><rgb_cx>160</rgb_cx><rgb_cy>120</rgb_cy>
      <rgb_fx>250</rgb_fx><rgb_fy>250</rgb_fy>
      <depth_clip_min>0.1</depth_clip_min><depth_clip_max>15</depth_clip_max>
      <rgb_clip_min>0.1</rgb_clip_min><rgb_clip_max>200</rgb_clip_max>
      <depth_resolution>1e-3</depth_resolution><depth_noise_sigma>0</depth_noise_sigma>
      )"
	  << rgbdTags << R"(
    </sensor>
  </vehicle>
</mvsim_world>
)";
	return path;
}

/** The vehicle moves along +Y at `speed`, from y0 at kT0. */
std::string writeTrajectory(double speed = kSpeed, double y0 = 0)
{
	const std::string path = tempPath("cameras.tum");
	std::ofstream f(path);
	f << std::fixed;
	for (int i = 0; i <= 40; i++)
	{
		const double t = 0.1 * i;
		f << kT0 + t << " 0 " << y0 + speed * t << " 0 0 0 0 1\n";
	}
	return path;
}

int runDatasetGen(
	const std::string& world, const std::string& traj, const std::string& out,
	const std::string& extraArgs)
{
	std::ostringstream cmd;
	cmd << "\"" << MVSIM_DATASET_GEN_EXE_PATH << "\" \"" << world << "\" --trajectory \"" << traj
		<< "\" -o \"" << out << "\" " << extraArgs << " > /dev/null 2>&1";
	return std::system(cmd.str().c_str());
}

std::string imagesDirOf(const std::string& rawlog)
{
	return mrpt::system::pathJoin(
		{mrpt::system::extractFileDirectory(rawlog),
		 mrpt::system::extractFileName(rawlog) + "_Images"});
}

void removeOutputs(const std::string& rawlog)
{
	for (const char* ext : {"gt.tum", "lidar1.gt.tum", "gt.yaml"})
	{
		std::remove(mrpt::system::fileNameChangeExtension(rawlog, ext).c_str());
	}
	std::remove(rawlog.c_str());
	mrpt::system::deleteFilesInDirectory(imagesDirOf(rawlog), true);
}

std::vector<mrpt::obs::CObservation::Ptr> loadObservations(const std::string& rawlogPath)
{
	std::vector<mrpt::obs::CObservation::Ptr> out;
	mrpt::io::CFileGZInputStream f(rawlogPath);
	auto arch = mrpt::serialization::archiveFrom(f);
	for (;;)
	{
		mrpt::serialization::CSerializable::Ptr obj;
		try
		{
			arch >> obj;
		}
		catch (const std::exception&)
		{
			break;	// EOF
		}
		if (auto obs = std::dynamic_pointer_cast<mrpt::obs::CObservation>(obj); obs)
		{
			out.push_back(obs);
		}
		else
		{
			break;
		}
	}
	return out;
}

/** Centroid of the red pixels of an image, if any. */
std::optional<mrpt::math::TPoint2D> redBlob(const mrpt::img::CImage& img)
{
	double su = 0;
	double sv = 0;
	size_t n = 0;
	const size_t nCh = img.channels();
	for (int v = 0; v < static_cast<int>(img.getHeight()); v++)
	{
		const uint8_t* row = img.ptrLine<uint8_t>(v);
		for (int u = 0; u < static_cast<int>(img.getWidth()); u++)
		{
			// RGB order:
			const int R = row[u * nCh];
			const int G = row[u * nCh + 1];
			const int B = row[u * nCh + 2];
			if (R > 50 && R > 2 * std::max(G, B) + 20)
			{
				su += u;
				sv += v;
				n++;
			}
		}
	}
	if (n < 10)
	{
		return std::nullopt;
	}
	return mrpt::math::TPoint2D(su / n, sv / n);
}

/** Projection of the landmark by a pinhole camera of focal f and principal
 * point (cx, cy), at height camZ on the vehicle at time t (see
 * writeTrajectory()), optionally with plumb_bob distortion (OpenCV
 * convention: the center of pixel (u,v) at (u,v)). */
mrpt::math::TPoint2D project(
	double t, double camZ, double f, double cx, double cy, double speed = kSpeed, double y0 = 0,
	const std::vector<double>& dist = {})
{
	const double depth = kLandmark.x - kCamX;
	double x = -(kLandmark.y - (y0 + speed * (t - kT0))) / depth;
	double y = -(kLandmark.z - camZ) / depth;
	if (!dist.empty())
	{
		const double k1 = dist[0];
		const double k2 = dist[1];
		const double p1 = dist[2];
		const double p2 = dist[3];
		const double r2 = x * x + y * y;
		const double radial = 1 + k1 * r2 + k2 * r2 * r2;
		const double xd = x * radial + 2 * p1 * x * y + p2 * (r2 + 2 * x * x);
		const double yd = y * radial + p1 * (r2 + 2 * y * y) + 2 * p2 * x * y;
		x = xd;
		y = yd;
	}
	return {cx + f * x, cy + f * y};
}

/** Max over the RGB camera frames of the error between the landmark seen
 * and its projection at the row time (`lineTime` per row; 0: the frame
 * stamp), and the number of frames. */
std::pair<double, int> landmarkError(
	const std::vector<mrpt::obs::CObservation::Ptr>& obs, double clockOffset, double lineTime)
{
	double maxErr = 0;
	int n = 0;
	for (const auto& o : obs)
	{
		auto im = std::dynamic_pointer_cast<mrpt::obs::CObservationImage>(o);
		if (!im)
		{
			continue;
		}
		im->load();
		const auto blob = redBlob(im->image);
		if (!blob)
		{
			continue;
		}
		const double t = mrpt::Clock::toDouble(im->timestamp) - clockOffset;
		// The row of the landmark tells its exposure time:
		const auto p0 = project(t, 1.2, 500, 320, 240);
		const auto p = project(t + p0.y * lineTime, 1.2, 500, 320, 240);
		maxErr = std::max(maxErr, std::hypot(blob->x - p.x, blob->y - p.y));
		n++;
	}
	return {maxErr, n};
}

void test_frame_times_and_depth()
{
	const std::string world = writeWorld("", "");
	const std::string traj = writeTrajectory();
	const std::string out = tempPath("cams.rawlog");
	EXPECT_TRUE(
		runDatasetGen(world, traj, out, "--duration 1.5 --seed 3 --clock-offset cam1=0.01") == 0);

	mrpt::img::CImage::setImagesPathBase(imagesDirOf(out));
	const auto obs = loadObservations(out);

	// The image time is the trajectory time: a frame off (5 cm at 4.8 m)
	// would move the landmark 5 px.
	const auto [err, nFrames] = landmarkError(obs, 0.01, 0);
	EXPECT_TRUE(nFrames == 31);
	EXPECT_LT(err, 0.3);
	std::cout << "Landmark reprojection error (global shutter): " << err << " px in " << nFrames
			  << " frames\n";

	// Depth of the wall (fronto-parallel, x = 8 m) and of the landmark:
	double maxWallErr = 0;
	int nDepth = 0;
	for (const auto& o : obs)
	{
		auto d = std::dynamic_pointer_cast<mrpt::obs::CObservation3DRangeScan>(o);
		if (!d)
		{
			continue;
		}
		d->load();
		EXPECT_TRUE(d->hasRangeImage && d->hasIntensityImage);
		EXPECT_TRUE(d->rangeImage.cols() == 320 && d->rangeImage.rows() == 240);
		nDepth++;
		// Rows 30-60 see the wall only (below its top, above the landmark):
		for (int r = 30; r < 60; r++)
		{
			for (int c = 0; c < 320; c++)
			{
				const double z = d->rangeImage(r, c) * d->rangeUnits;
				maxWallErr = std::max(maxWallErr, std::abs(z - (kWallX - kCamX)));
			}
		}
		// The RGB image of the RGB-D camera, at 1.0 m:
		const auto blob = redBlob(d->intensityImage);
		const double t = mrpt::Clock::toDouble(d->timestamp);
		const auto p = project(t, 1.0, 250, 160, 120);
		EXPECT_TRUE(blob.has_value());
		if (blob)
		{
			EXPECT_LT(std::hypot(blob->x - p.x, blob->y - p.y), 0.5);
		}
	}
	EXPECT_TRUE(nDepth == 16);
	EXPECT_LT(maxWallErr, 0.0006);
	std::cout << "Wall depth max error: " << maxWallErr * 1e3 << " mm\n";

	// Ground truth:
	const auto gt =
		mrpt::containers::yaml::FromFile(mrpt::system::fileNameChangeExtension(out, "gt.yaml"));
	const auto& cam = gt["sensors"]["cam1"];
	EXPECT_TRUE(cam["type"].as<std::string>() == "camera");
	EXPECT_NEAR(cam["clock_offset_s"].as<double>(), 0.01, 1e-12);
	EXPECT_NEAR(cam["rate_hz"].as<double>(), 20.0, 1e-9);
	EXPECT_NEAR(cam["intrinsics"]["fx"].as<double>(), 500.0, 1e-12);
	EXPECT_TRUE(cam["intrinsics"]["width"].as<int>() == 640);
	EXPECT_TRUE(cam["intrinsics"]["distortion"]["model"].as<std::string>() == "none");
	EXPECT_NEAR(cam["pose_in_vehicle"]["xyz"](2).as<double>(), 1.2, 1e-12);
	const auto& rgbd = gt["sensors"]["rgbd1"];
	EXPECT_TRUE(rgbd["type"].as<std::string>() == "rgbd_camera");
	EXPECT_NEAR(rgbd["intrinsics"]["depth"]["depth_unit_m"].as<double>(), 1e-3, 1e-9);
	EXPECT_NEAR(rgbd["intrinsics"]["rgb"]["pose_in_vehicle"]["xyz"](2).as<double>(), 1.0, 1e-12);
	// Both optical frames look along +X of the vehicle:
	EXPECT_NEAR(rgbd["pose_in_vehicle"]["ypr_deg"](0).as<double>(), -90.0, 1e-9);

	// Cameras do not change the other streams:
	const std::string outNoCam = tempPath("nocams.rawlog");
	EXPECT_TRUE(runDatasetGen(world, traj, outNoCam, "--duration 1.5 --seed 3 --no-cameras") == 0);
	std::vector<mrpt::obs::CObservation::Ptr> others;
	for (const auto& o : obs)
	{
		if (!std::dynamic_pointer_cast<mrpt::obs::CObservationImage>(o) &&
			!std::dynamic_pointer_cast<mrpt::obs::CObservation3DRangeScan>(o))
		{
			others.push_back(o);
		}
	}
	const auto noCam = loadObservations(outNoCam);
	EXPECT_TRUE(others.size() == noCam.size());
	bool same = others.size() == noCam.size();
	for (size_t i = 0; same && i < others.size(); i++)
	{
		same = others[i]->timestamp == noCam[i]->timestamp;
		auto a = std::dynamic_pointer_cast<mrpt::obs::CObservationPointCloud>(others[i]);
		auto b = std::dynamic_pointer_cast<mrpt::obs::CObservationPointCloud>(noCam[i]);
		if (a && b)
		{
			same = same && a->pointcloud->size() == b->pointcloud->size();
			for (size_t k = 0; same && k < a->pointcloud->size(); k++)
			{
				float ax;
				float ay;
				float az;
				float bx;
				float by;
				float bz;
				a->pointcloud->getPoint(k, ax, ay, az);
				b->pointcloud->getPoint(k, bx, by, bz);
				same = ax == bx && ay == by && az == bz;
			}
		}
	}
	EXPECT_TRUE(same);

	removeOutputs(out);
	removeOutputs(outNoCam);
	std::remove(world.c_str());
	std::remove(traj.c_str());
}

void test_rolling_shutter()
{
	// 0.1 ms per row: the middle row is exposed 24 ms after the first one,
	// when the landmark has moved 2.5 px.
	const std::string world = writeWorld(
		"<rolling_shutter_line_time>1e-4</rolling_shutter_line_time>"
		"<rolling_shutter_bands>48</rolling_shutter_bands>",
		"");
	const std::string traj = writeTrajectory();
	const std::string out = tempPath("rs.rawlog");
	EXPECT_TRUE(runDatasetGen(world, traj, out, "--duration 1.0 --seed 3") == 0);

	mrpt::img::CImage::setImagesPathBase(imagesDirOf(out));
	const auto obs = loadObservations(out);
	const auto [errRs, n] = landmarkError(obs, 0, 1e-4);
	const auto [errGlobal, n2] = landmarkError(obs, 0, 0);
	EXPECT_TRUE(n == 21 && n2 == 21);
	EXPECT_LT(errRs, 0.3);
	EXPECT_GT(errGlobal, 1.5);
	std::cout << "Landmark error with a rolling shutter: " << errRs
			  << " px (global shutter model: " << errGlobal << " px)\n";

	const auto gt =
		mrpt::containers::yaml::FromFile(mrpt::system::fileNameChangeExtension(out, "gt.yaml"));
	EXPECT_NEAR(
		gt["sensors"]["cam1"]["intrinsics"]["rolling_shutter_line_time_s"].as<double>(), 1e-4,
		1e-15);

	removeOutputs(out);
	std::remove(world.c_str());
	std::remove(traj.c_str());
}

void test_lens_distortion()
{
#if MRPT_VERSION >= 0x030600
	// The landmark crosses the image from u = 70 to u = 590 (at 2 m/s):
	const std::vector<double> dist = {-0.2, 0.05, 0.001, -0.001};
	const std::string world = writeWorld(
		"<distortion_model>plumb_bob</distortion_model><k1>-0.2</k1><k2>0.05</k2>"
		"<p1>0.001</p1><p2>-0.001</p2>",
		"");
	const std::string traj = writeTrajectory(2.0, -2.1);
	const std::string out = tempPath("dist.rawlog");
	EXPECT_TRUE(runDatasetGen(world, traj, out, "--duration 1.25 --seed 3") == 0);

	mrpt::img::CImage::setImagesPathBase(imagesDirOf(out));
	double maxErr = 0;
	double maxDistortion = 0;
	int n = 0;
	for (const auto& o : loadObservations(out))
	{
		auto im = std::dynamic_pointer_cast<mrpt::obs::CObservationImage>(o);
		if (!im)
		{
			continue;
		}
		im->load();
		const auto blob = redBlob(im->image);
		if (!blob)
		{
			continue;
		}
		const double t = mrpt::Clock::toDouble(im->timestamp);
		const auto p = project(t, 1.2, 500, 320, 240, 2.0, -2.1, dist);
		const auto p0 = project(t, 1.2, 500, 320, 240, 2.0, -2.1);
		maxErr = std::max(maxErr, std::hypot(blob->x - p.x, blob->y - p.y));
		maxDistortion = std::max(maxDistortion, std::hypot(p.x - p0.x, p.y - p0.y));
		n++;
	}
	EXPECT_GT(n, 20);
	EXPECT_GT(maxDistortion, 5.0);
	// The distorted image is resampled, so its edges blend: noisier for this
	// binary detector (no trend with the distortion).
	EXPECT_LT(maxErr, 0.5);
	std::cout << "Landmark error with lens distortion: " << maxErr << " px in " << n
			  << " frames (distortion up to " << maxDistortion << " px)\n";

	const auto gt =
		mrpt::containers::yaml::FromFile(mrpt::system::fileNameChangeExtension(out, "gt.yaml"));
	const auto& d = gt["sensors"]["cam1"]["intrinsics"]["distortion"];
	EXPECT_TRUE(d["model"].as<std::string>() == "plumb_bob");
	EXPECT_NEAR(d["coeffs"](0).as<double>(), -0.2, 1e-12);

	removeOutputs(out);
	std::remove(world.c_str());
	std::remove(traj.c_str());
#endif
}

void test_depth_errors()
{
	// d -> d (1.02 + 0.01 r^2) + 0.05:
	const std::string world = writeWorld(
		"",
		"<depth_error_scale>1.02</depth_error_scale><depth_error_offset>0.05</depth_error_offset>"
		"<depth_error_radial_k1>0.01</depth_error_radial_k1>");
	const std::string traj = writeTrajectory();
	const std::string out = tempPath("deptherr.rawlog");
	EXPECT_TRUE(runDatasetGen(world, traj, out, "--duration 0.5 --seed 3") == 0);

	mrpt::img::CImage::setImagesPathBase(imagesDirOf(out));
	double maxErr = 0;
	int n = 0;
	for (const auto& o : loadObservations(out))
	{
		auto d = std::dynamic_pointer_cast<mrpt::obs::CObservation3DRangeScan>(o);
		if (!d)
		{
			continue;
		}
		d->load();
		n++;
		for (int r = 30; r < 60; r++)
		{
			for (int c = 0; c < 320; c += 7)
			{
				const double x = (c - 160.0) / 250.0;
				const double y = (r - 120.0) / 250.0;
				const double expected = (kWallX - kCamX) * (1.02 + 0.01 * (x * x + y * y)) + 0.05;
				maxErr = std::max(maxErr, std::abs(d->rangeImage(r, c) * d->rangeUnits - expected));
			}
		}
	}
	EXPECT_TRUE(n == 6);
	EXPECT_LT(maxErr, 0.0006);
	std::cout << "Depth error model max deviation: " << maxErr * 1e3 << " mm\n";

	const auto gt =
		mrpt::containers::yaml::FromFile(mrpt::system::fileNameChangeExtension(out, "gt.yaml"));
	const auto& e = gt["sensors"]["rgbd1"]["intrinsics"]["depth"]["depth_errors"];
	EXPECT_NEAR(e["scale"].as<double>(), 1.02, 1e-12);
	EXPECT_NEAR(e["offset_m"].as<double>(), 0.05, 1e-12);
	EXPECT_NEAR(e["radial"](0).as<double>(), 0.01, 1e-12);

	removeOutputs(out);
	std::remove(world.c_str());
	std::remove(traj.c_str());
}
}  // namespace

int main()
{
	try
	{
		// Skip only if off-screen rendering is not available at all:
		try
		{
			mrpt::opengl::CFBORender probe(16, 16);
		}
		catch (const std::exception& e)
		{
			std::cout << "[SKIPPED] No OpenGL rendering available: " << e.what() << "\n";
			return 0;
		}

		test_frame_times_and_depth();
		test_rolling_shutter();
		test_lens_distortion();
		test_depth_errors();
	}
	catch (const std::exception& e)
	{
		std::cerr << "Exception: " << e.what() << "\n";
		return 1;
	}

	if (g_failures == 0)
	{
		std::printf("All mvsim-dataset-gen camera tests passed.\n");
	}
	else
	{
		std::fprintf(stderr, "%d test(s) FAILED.\n", g_failures);
	}
	return g_failures == 0 ? 0 : 1;
}
