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
#include <mvsim/Sensors/DepthCameraSensor.h>

#include <algorithm>
#include <array>
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
 * with an RGB camera (20 Hz, at height camZ) and an RGB-D camera (10 Hz). */
std::string writeWorld(
	const std::string& cameraTags, const std::string& rgbdTags, double camZ = kLandmark.z)
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
      <pose_3d>0.2 0 )"
	  << camZ << R"( -90 0 -90</pose_3d>
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
	const std::vector<mrpt::obs::CObservation::Ptr>& obs, double clockOffset, double lineTime,
	double camZ = kLandmark.z)
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
		const auto p0 = project(t, camZ, 500, 320, 240);
		const auto p = project(t + p0.y * lineTime, camZ, 500, 320, 240);
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
	// Values the sensors hold in single precision, written as given:
	EXPECT_TRUE(rgbd["intrinsics"]["depth"]["depth_unit_m"].as<double>() == 1e-3);
	EXPECT_TRUE(rgbd["intrinsics"]["depth"]["clip_m"](0).as<double>() == 0.1);
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
		auto oa = std::dynamic_pointer_cast<mrpt::obs::CObservationOdometry>(others[i]);
		auto ob = std::dynamic_pointer_cast<mrpt::obs::CObservationOdometry>(noCam[i]);
		if (oa || ob)
		{
			same = same && oa && ob && oa->odometry == ob->odometry;
		}
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
	// 0.1 ms per row. The camera is 1 m above the landmark, which it sees at
	// row 344: exposed 34 ms after the first row (3.6 px of motion), and 21 ms
	// later than if the rows were read bottom-up.
	constexpr double camZ = 2.2;
	const std::string world = writeWorld(
		"<rolling_shutter_line_time>1e-4</rolling_shutter_line_time>"
		"<rolling_shutter_bands>48</rolling_shutter_bands>",
		"", camZ);
	const std::string traj = writeTrajectory();
	const std::string out = tempPath("rs.rawlog");
	EXPECT_TRUE(runDatasetGen(world, traj, out, "--duration 1.0 --seed 3") == 0);

	mrpt::img::CImage::setImagesPathBase(imagesDirOf(out));
	const auto obs = loadObservations(out);
	const auto [errRs, n] = landmarkError(obs, 0, 1e-4, camZ);
	const auto [errGlobal, n2] = landmarkError(obs, 0, 0, camZ);
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

/** Red pixels centroid in a window of the image, if any. */
std::optional<mrpt::math::TPoint2D> redBlobNear(
	const mrpt::img::CImage& img, const mrpt::math::TPoint2D& p, int halfWidth)
{
	double su = 0;
	double sv = 0;
	size_t n = 0;
	const int W = static_cast<int>(img.getWidth());
	const int H = static_cast<int>(img.getHeight());
	for (int v = std::max(0, static_cast<int>(p.y) - halfWidth);
		 v <= std::min(H - 1, static_cast<int>(p.y) + halfWidth); v++)
	{
		const uint8_t* row = img.ptrLine<uint8_t>(v);
		for (int u = std::max(0, static_cast<int>(p.x) - halfWidth);
			 u <= std::min(W - 1, static_cast<int>(p.x) + halfWidth); u++)
		{
			const int R = row[3 * u];
			const int G = row[3 * u + 1];
			const int B = row[3 * u + 2];
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

void test_fisheye()
{
#if MRPT_VERSION >= 0x030600
	// A fisheye camera with a 200 deg image circle, at rest, sees red disks
	// (30 cm wide, 3 m away, facing it, above the floor) at 0 to 97 deg from
	// its optical axis: each is where the Kannala-Brandt projection of its
	// center says.
	constexpr double fx = 180;
	constexpr double fy = 181;
	constexpr double cx = 321.3;
	constexpr double cy = 238.7;
	const std::array<double, 4> k = {0.02, -0.004, 0.0005, -0.00005};
	const mrpt::math::TPoint3D cam(kCamX, 0, 1.2);	// on the vehicle (and world)
	// Directions (deg): angle from the optical axis, azimuth in the image:
	const std::vector<std::pair<double, double>> dirs = {
		{0, 0}, {40, 180}, {70, -30}, {85, 170}, {97, -35}};

	const auto opticalToWorld = [](const mrpt::math::TPoint3D& d)
	{
		// The optical frame (+Z forward, +X right, +Y down) of a camera
		// looking along +X of the vehicle:
		return mrpt::math::TPoint3D(d.z, -d.x, -d.y);
	};
	const auto projectKB = [&](const mrpt::math::TPoint3D& p)
	{
		const double r = std::hypot(p.x, p.y);
		const double th = std::atan2(r, p.z);
		const double t2 = th * th;
		const double td = th * (1 + t2 * (k[0] + t2 * (k[1] + t2 * (k[2] + t2 * k[3]))));
		const double s = r > 1e-12 ? td / r : 1.0;
		return mrpt::math::TPoint2D(cx + fx * s * p.x, cy + fy * s * p.y);
	};

	std::ostringstream disks;
	std::vector<mrpt::math::TPoint2D> expected;
	for (size_t i = 0; i < dirs.size(); i++)
	{
		const double th = mrpt::DEG2RAD(dirs[i].first);
		const double ph = mrpt::DEG2RAD(dirs[i].second);
		const mrpt::math::TPoint3D d(
			std::sin(th) * std::cos(ph), std::sin(th) * std::sin(ph), std::cos(th));
		expected.push_back(projectKB(d));
		const auto w = opticalToWorld(d);
		const auto pos = cam + w * 3.0;
		// The disk axis (local +Z) along the viewing ray:
		const double yaw = mrpt::RAD2DEG(std::atan2(w.y, w.x));
		const double pitch = mrpt::RAD2DEG(std::acos(std::clamp(w.z, -1.0, 1.0)));
		disks
			<< "<block name=\"disk" << i
			<< "\"><static>true</static><skip_elevation_adjust/><color>#ff0000</color>"
			   "<geometry type=\"cylinder\" radius=\"0.15\" length=\"0.001\" vertex_count=\"64\" />"
			   "<init_pose3d>"
			<< pos.x << " " << pos.y << " " << pos.z << " " << yaw << " " << pitch
			<< " 0</init_pose3d></block>\n";
	}

	const std::string world = tempPath("fisheye.world.xml");
	{
		std::ofstream f(world);
		f << R"(<mvsim_world version="1.0">
  <lights><light_ambient>1.0</light_ambient><enable_shadows>false</enable_shadows></lights>
  <element class="horizontal_plane"><x_min>-20</x_min><x_max>20</x_max><y_min>-20</y_min><y_max>20</y_max><z>0</z><color>#808080</color></element>
)" << disks.str()
		  << R"(  <vehicle name="r1">
    <dynamics class="differential">
      <l_wheel pos="0  0.3" mass="1.0" width="0.05" diameter="0.20" />
      <r_wheel pos="0 -0.3" mass="1.0" width="0.05" diameter="0.20" />
      <chassis mass="10.0" zmin="0.05" zmax="0.25" />
    </dynamics>
    <init_pose>0 0 0</init_pose>
    <sensor class="lidar3d" name="lidar1"><pose_3d>0 0 0.75 0 0 0</pose_3d><vert_nrays>8</vert_nrays>
      <vert_fov_degrees>30</vert_fov_degrees><horz_nrays>360</horz_nrays><sensor_period>0.1</sensor_period></sensor>
    <sensor class="camera" name="cam1">
      <pose_3d>)"
		  << kCamX << R"( 0 1.2 -90 0 -90</pose_3d>
      <sensor_period>0.05</sensor_period>
      <ncols>640</ncols><nrows>480</nrows><cx>321.3</cx><cy>238.7</cy><fx>180</fx><fy>181</fy>
      <clip_min>0.05</clip_min><clip_max>200</clip_max>
      <distortion_model>kannala_brandt</distortion_model>
      <k1>0.02</k1><k2>-0.004</k2><k3>0.0005</k3><k4>-0.00005</k4>
      <image_circle_fov_deg>200</image_circle_fov_deg>
    </sensor>
  </vehicle>
</mvsim_world>
)";
	}
	const std::string traj = writeTrajectory(0.0);
	const std::string out = tempPath("fisheye.rawlog");
	EXPECT_TRUE(runDatasetGen(world, traj, out, "--duration 0.1 --seed 3") == 0);

	mrpt::img::CImage::setImagesPathBase(imagesDirOf(out));
	double maxErr = 0;
	int n = 0;
	int nFound = 0;
	for (const auto& o : loadObservations(out))
	{
		auto im = std::dynamic_pointer_cast<mrpt::obs::CObservationImage>(o);
		if (!im)
		{
			continue;
		}
		im->load();
		n++;
		for (const auto& p : expected)
		{
			const auto blob = redBlobNear(im->image, p, 30);
			if (!blob)
			{
				continue;
			}
			nFound++;
			maxErr = std::max(maxErr, std::hypot(blob->x - p.x, blob->y - p.y));
		}
		// Outside the image circle (100 deg: 325 px from the center), black:
		const uint8_t* top = im->image.ptrLine<uint8_t>(0);
		const uint8_t* bottom = im->image.ptrLine<uint8_t>(479);
		EXPECT_TRUE(top[0] == 0 && top[3 * 639] == 0 && bottom[0] == 0 && bottom[3 * 639] == 0);
		// ...and the floor inside it (60 deg below the optical axis):
		const uint8_t* floorRow = im->image.ptrLine<uint8_t>(431);
		EXPECT_TRUE(floorRow[3 * 321] > 30);
	}
	EXPECT_TRUE(n == 3);
	EXPECT_TRUE(nFound == 3 * static_cast<int>(expected.size()));
	EXPECT_LT(maxErr, 0.5);
	std::cout << "Fisheye landmark error: " << maxErr << " px (" << nFound
			  << " landmarks, up to 97 deg off-axis)\n";

	const auto gt =
		mrpt::containers::yaml::FromFile(mrpt::system::fileNameChangeExtension(out, "gt.yaml"));
	const auto& in = gt["sensors"]["cam1"]["intrinsics"];
	EXPECT_TRUE(in["distortion"]["model"].as<std::string>() == "kannala_brandt");
	EXPECT_NEAR(in["distortion"]["coeffs"](3).as<double>(), -0.00005, 1e-15);
	EXPECT_NEAR(in["image_circle_fov_deg"].as<double>(), 200.0, 1e-12);

	// No rolling shutter with this lens model:
	const std::string worldRs = tempPath("fisheye_rs.world.xml");
	{
		std::ifstream fin(world);
		std::stringstream ss;
		ss << fin.rdbuf();
		std::string xml = ss.str();
		const std::string tag = "<image_circle_fov_deg>";
		xml.insert(xml.find(tag), "<rolling_shutter_line_time>1e-5</rolling_shutter_line_time>");
		std::ofstream fout(worldRs);
		fout << xml;
	}
	const std::string outRs = tempPath("fisheye_rs.rawlog");
	EXPECT_TRUE(runDatasetGen(worldRs, traj, outRs, "--duration 0.1 --seed 3") != 0);

	removeOutputs(out);
	removeOutputs(outRs);
	std::remove(world.c_str());
	std::remove(worldRs.c_str());
	std::remove(traj.c_str());
#endif
}

void test_determinism()
{
	// The same seed gives the same images, with depth and pixel noise:
#if MRPT_VERSION >= 0x030600
	const std::string rgbNoise = "<rgb_image_noise_std>3</rgb_image_noise_std>";
#else
	const std::string rgbNoise;
#endif
	const std::string world =
		writeWorld("", "<depth_noise_sigma>0.003</depth_noise_sigma>" + rgbNoise);
	const std::string traj = writeTrajectory();
	const std::string outA = tempPath("detA.rawlog");
	const std::string outB = tempPath("detB.rawlog");
	EXPECT_TRUE(runDatasetGen(world, traj, outA, "--duration 0.3 --seed 5") == 0);
	EXPECT_TRUE(runDatasetGen(world, traj, outB, "--duration 0.3 --seed 5") == 0);

	const auto load = [](const std::string& out)
	{
		mrpt::img::CImage::setImagesPathBase(imagesDirOf(out));
		std::vector<mrpt::obs::CObservation3DRangeScan::Ptr> frames;
		for (const auto& o : loadObservations(out))
		{
			if (auto d = std::dynamic_pointer_cast<mrpt::obs::CObservation3DRangeScan>(o); d)
			{
				d->load();
				frames.push_back(d);
			}
		}
		return frames;
	};
	const auto a = load(outA);
	const auto b = load(outB);
	EXPECT_TRUE(a.size() == 4 && a.size() == b.size());
	bool same = a.size() == b.size();
	bool noisy = false;
	for (size_t i = 0; same && i < a.size(); i++)
	{
		same = a[i]->rangeImage == b[i]->rangeImage;
		const auto& ia = a[i]->intensityImage;
		const auto& ib = b[i]->intensityImage;
		const size_t rowBytes = ia.getWidth() * ia.channels();
		for (int r = 0; same && r < static_cast<int>(ia.getHeight()); r++)
		{
			same = std::equal(
				ia.ptrLine<uint8_t>(r), ia.ptrLine<uint8_t>(r) + rowBytes, ib.ptrLine<uint8_t>(r));
		}
		// The wall (fronto-parallel) has noisy depths:
		noisy = noisy || a[i]->rangeImage(40, 100) != a[i]->rangeImage(40, 101);
	}
	EXPECT_TRUE(same);
	EXPECT_TRUE(noisy);

	removeOutputs(outA);
	removeOutputs(outB);
	std::remove(world.c_str());
	std::remove(traj.c_str());
}

void test_depth_error_model()
{
	// Depths beyond the range stay out of it, those that become non-positive
	// are no return, and r is measured from the pixel centers (u, v):
	mvsim::DepthErrorModel e;
	e.scale = 0.8;
	e.offset = -0.5;
	e.radialK1 = 0.1;
	mrpt::img::TCamera cam;
	cam.ncols = 4;
	cam.nrows = 1;
	cam.fx(2.0);
	cam.fy(2.0);
	cam.cx(1.0);
	cam.cy(0.0);
	mrpt::math::CMatrixFloat d(1, 4);
	d(0, 0) = 0.5f;	 // 0.5 (0.8 + 0.1 * 0.25) - 0.5 < 0
	d(0, 1) = 4.0f;	 // At the principal point
	d(0, 2) = 6.0f;	 // Beyond 5 m, although it would become 4.45 m
	d(0, 3) = 3.0f;	 // r = 1
	e.apply(d, cam, 5.0);
	EXPECT_NEAR(d(0, 0), 0.0, 1e-9);
	EXPECT_NEAR(d(0, 1), 4.0 * 0.8 - 0.5, 1e-6);
	EXPECT_NEAR(d(0, 2), 0.0, 1e-9);
	EXPECT_NEAR(d(0, 3), 3.0 * 0.9 - 0.5, 1e-6);
}

void test_depth_errors()
{
	// A depth-only camera (rendered on its own), d -> d (1.02 + 0.01 r^2) + 0.05:
	const std::string world = writeWorld(
		"",
		"<sense_rgb>false</sense_rgb>"
		"<depth_error_scale>1.02</depth_error_scale><depth_error_offset>0.05</depth_error_offset>"
		"<depth_error_radial_k1>0.01</depth_error_radial_k1>");
	const std::string traj = writeTrajectory();
	const std::string out = tempPath("deptherr.rawlog");
	EXPECT_TRUE(runDatasetGen(world, traj, out, "--duration 0.5 --seed 3") == 0);

	mrpt::img::CImage::setImagesPathBase(imagesDirOf(out));
	double maxWallErr = 0;
	double maxFloorErr = 0;
	int n = 0;
	for (const auto& o : loadObservations(out))
	{
		auto d = std::dynamic_pointer_cast<mrpt::obs::CObservation3DRangeScan>(o);
		if (!d)
		{
			continue;
		}
		d->load();
		EXPECT_TRUE(d->hasRangeImage && !d->hasIntensityImage);
		n++;
		// The wall (rows 30-60), and the floor (rows 160-240, the camera is
		// 1 m above it): the depth of the floor changes 2 to 6 cm per row, so
		// it also checks the pixel centers of the depth image.
		for (int r = 30; r < 240; r++)
		{
			if (r >= 60 && r < 160)
			{
				continue;
			}
			const double y = (r - 120.0) / 250.0;
			const double z = r < 60 ? kWallX - kCamX : 1.0 / y;
			double& maxErr = r < 60 ? maxWallErr : maxFloorErr;
			for (int c = 0; c < 320; c += 7)
			{
				const double x = (c - 160.0) / 250.0;
				const double expected = z * (1.02 + 0.01 * (x * x + y * y)) + 0.05;
				maxErr = std::max(maxErr, std::abs(d->rangeImage(r, c) * d->rangeUnits - expected));
			}
		}
	}
	EXPECT_TRUE(n == 6);
	// Rounding to 1 mm units, and the depth buffer resolution on the floor:
	EXPECT_LT(maxWallErr, 0.0006);
	EXPECT_LT(maxFloorErr, 0.001);
	std::cout << "Depth error model max deviation: " << maxWallErr * 1e3 << " mm (wall), "
			  << maxFloorErr * 1e3 << " mm (floor)\n";

	const auto gt =
		mrpt::containers::yaml::FromFile(mrpt::system::fileNameChangeExtension(out, "gt.yaml"));
	const auto& cam = gt["sensors"]["rgbd1"];
	EXPECT_TRUE(cam["type"].as<std::string>() == "depth_camera");
	const auto& e = cam["intrinsics"]["depth_errors"];
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
		test_depth_error_model();

		// Skip the rest only if off-screen rendering is not available at all:
		try
		{
			mrpt::opengl::CFBORender probe(16, 16);
		}
		catch (const std::exception& e)
		{
			std::cout << "[SKIPPED] No OpenGL rendering available: " << e.what() << "\n";
			return g_failures == 0 ? 0 : 1;
		}

		test_frame_times_and_depth();
		test_rolling_shutter();
		test_lens_distortion();
		test_fisheye();
		test_determinism();
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
