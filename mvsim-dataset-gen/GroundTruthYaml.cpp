/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2026  Jose Luis Blanco Claraco                       |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#include "GroundTruthYaml.h"

#include <mrpt/core/bits_math.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/math/CQuaternion.h>
#include <mvsim/Sensors/GNSS.h>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <initializer_list>
#include <limits>
#include <sstream>
#include <vector>

using namespace mvsim_dataset_gen;

namespace
{
/** Shortest decimal form of `v` that reads back exactly, without exponent
 * for ordinary magnitudes (e.g. timestamps). Exponents always get a mantissa
 * with a dot, which YAML 1.1 readers need to see a float. */
std::string num(double v)
{
	if (!std::isfinite(v))
	{
		return ".nan";
	}
	if (v == 0)
	{
		return "0";	 // also for -0
	}
	char buf[64];
	if (std::abs(v) >= 1e-3 && std::abs(v) < 1e15)
	{
		for (int decimals = 0; decimals <= 20; decimals++)
		{
			std::snprintf(buf, sizeof(buf), "%.*f", decimals, v);
			if (std::strtod(buf, nullptr) == v)
			{
				return buf;
			}
		}
	}
	for (int prec = 6; prec <= 17; prec++)
	{
		std::snprintf(buf, sizeof(buf), "%.*g", prec, v);
		if (std::strtod(buf, nullptr) == v)
		{
			break;
		}
	}
	std::string s = buf;
	const size_t e = s.find('e');
	if (e != std::string::npos && s.find('.') == std::string::npos)
	{
		s.insert(e, ".0");
	}
	return s;
}

/** num() of a value that a sensor holds in single precision: the shortest
 * decimal that reads back as the same float (0.1, not 0.10000000149011612).
 * Values that are not exactly a float are written in full. */
std::string numf(double v)
{
	const auto f = static_cast<float>(v);
	if (!std::isfinite(f) || static_cast<double>(f) != v)
	{
		return num(v);
	}
	char buf[64];
	for (int prec = 1; prec < 9; prec++)
	{
		std::snprintf(buf, sizeof(buf), "%.*g", prec, static_cast<double>(f));
		if (std::strtof(buf, nullptr) == f)
		{
			return num(std::strtod(buf, nullptr));
		}
	}
	return num(v);
}

/** A string as a YAML scalar: plain if it starts with a letter, '_' or '/',
 * has only letters, digits and "_-./", and is not a word that YAML 1.1
 * readers take for a boolean or null; else double-quoted and escaped. */
std::string str(const std::string& v)
{
	const auto isAlpha = [](char c) { return std::isalpha(static_cast<unsigned char>(c)) != 0; };
	bool plain = !v.empty() && (isAlpha(v.front()) || v.front() == '_' || v.front() == '/');
	for (const char c : v)
	{
		plain = plain && (std::isalnum(static_cast<unsigned char>(c)) || c == '_' || c == '-' ||
						  c == '.' || c == '/');
	}
	if (plain)
	{
		std::string low;
		for (const char c : v)
		{
			low += static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
		}
		for (const char* w : {"true", "false", "yes", "no", "on", "off", "y", "n", "null"})
		{
			plain = plain && low != w;
		}
	}
	if (plain)
	{
		return v;
	}
	std::string s = "\"";
	for (const char c : v)
	{
		if (c == '"' || c == '\\')
		{
			s += '\\';
			s += c;
		}
		else if (static_cast<unsigned char>(c) < 0x20)
		{
			char buf[8];
			std::snprintf(buf, sizeof(buf), "\\x%02x", static_cast<unsigned char>(c));
			s += buf;
		}
		else
		{
			s += c;
		}
	}
	return s + "\"";
}

std::string vec(std::initializer_list<double> values)
{
	std::string s = "[";
	bool first = true;
	for (const double v : values)
	{
		s += first ? "" : ", ";
		s += num(v);
		first = false;
	}
	return s + "]";
}

std::string vec3(const mrpt::math::TVector3D& v) { return vec({v.x, v.y, v.z}); }

std::string rateHz(double period) { return num(period > 0 ? 1.0 / period : 0.0); }

/** The keys common to every sensor: type, frame, pose, clock offset, rate. */
void writeSensorHeader(
	std::ostream& o, const std::string& label, const std::string& type,
	const mrpt::poses::CPose3D& p, double clockOffset, double period)
{
	mrpt::math::CQuaternionDouble q;
	p.getAsQuaternion(q);
	o << "  " << str(label) << ":\n";
	o << "    type: " << type << "\n";
	o << "    frame_id: " << str(label) << "\n";
	o << "    pose_in_vehicle:\n";
	o << "      xyz: " << vec({p.x(), p.y(), p.z()}) << "\n";
	o << "      quat_xyzw: " << vec({q.x(), q.y(), q.z(), q.r()}) << "\n";
	o << "      ypr_deg: "
	  << vec({mrpt::RAD2DEG(p.yaw()), mrpt::RAD2DEG(p.pitch()), mrpt::RAD2DEG(p.roll())}) << "\n";
	o << "    clock_offset_s: " << num(clockOffset) << "\n";
	o << "    rate_hz: " << rateHz(period) << "\n";
	o << "    intrinsics:\n";
}

void writeImuChannel(
	std::ostream& o, const std::string& name, const mvsim::ImuNoiseModel::ChannelParams& c)
{
	const auto& m = c.misalignment;
	const auto T = c.sensitivityMatrix();
	o << "      " << name << ":\n";
	o << "        bias: " << vec3(c.initial_bias) << "\n";
	o << "        scale: " << vec3(c.scale) << "\n";
	o << "        misalignment:  # M, full off-diagonal [rad]\n";
	o << "          - " << vec({0.0, m[0], m[1]}) << "\n";
	o << "          - " << vec({m[2], 0.0, m[3]}) << "\n";
	o << "          - " << vec({m[4], m[5], 0.0}) << "\n";
	o << "        sensitivity_matrix:  # diag(scale) * (I + M)\n";
	for (int r = 0; r < 3; r++)
	{
		o << "          - " << vec({T(r, 0), T(r, 1), T(r, 2)}) << "\n";
	}
	o << "        noise: {white_noise_std: " << num(c.white_noise_std)
	  << ", random_walk_std: " << num(c.random_walk_std) << "}\n";
}

void writeLidar(std::ostream& o, const LidarGroundTruth& l)
{
	const auto& p = l.model->params();
	const auto& opts = *l.opts;
	const bool calibrated = !opts.beamRangeBias.empty();
	writeSensorHeader(o, l.label, "lidar3d", l.pose, l.clockOffset, p.sensorPeriod);
	o << "      rings: " << l.model->vertRays() << "\n";
	o << "      horz_nrays: " << l.model->horzRays() << "\n";
	o << "      min_range: " << num(p.minRange) << "\n";
	o << "      max_range: " << num(p.maxRange) << "\n";
	o << "      elevations_deg:  # nominal, ring 0 first (ascending)\n";
	o << "        [";
	for (int ring = 0; ring < l.model->vertRays(); ring++)
	{
		o << (ring > 0 ? ", " : "") << num(mrpt::RAD2DEG(l.model->ringElevationRad(ring)));
	}
	o << "]\n";
	o << "      noise:\n";
	o << "        range_std_noise: " << num(opts.rangeStdNoise) << "\n";
	o << "        range_std_noise_per_meter: " << num(opts.rangeStdNoisePerMeter) << "\n";
	o << "        beam_divergence_deg: " << num(opts.beamDivergenceDeg) << "\n";
	o << "        max_incidence_deg: " << num(opts.maxIncidenceDeg) << "\n";
	o << "        mixed_pixel_probability: " << num(opts.mixedPixelProbability) << "\n";
	o << "        echo_separation: " << num(opts.echoSeparation) << "\n";
	o << "        detection_range: " << num(opts.detectionRange) << "\n";
	o << "      calibration_seed: " << l.calibrationSeed << "\n";
	o << "      beam_error_std: {elevation_deg: " << num(l.beamElevationStdDeg)
	  << ", azimuth_deg: " << num(l.beamAzimuthStdDeg) << ", range_m: " << num(l.beamRangeBiasStd)
	  << "}\n";
	o << "      # d_elevation_deg, d_azimuth_deg: perturbed (sampled) ray minus the\n"
		 "      # nominal (reported) one. d_range_m: reported range minus true range.\n";
	o << "      beam_errors:\n";
	for (int ring = 0; ring < l.model->vertRays(); ring++)
	{
		const double dEl = calibrated ? opts.beamElevOffset.at(ring) : 0.0;
		const double dAz = calibrated ? opts.beamAzimOffset.at(ring) : 0.0;
		const double bias = calibrated ? opts.beamRangeBias.at(ring) : 0.0;
		o << "        - {ring: " << ring << ", d_elevation_deg: " << num(mrpt::RAD2DEG(dEl))
		  << ", d_azimuth_deg: " << num(mrpt::RAD2DEG(dAz)) << ", d_range_m: " << num(bias)
		  << "}\n";
	}
}

void writeImu(std::ostream& o, const ImuGroundTruth& imu)
{
	writeSensorHeader(o, imu.label, "imu", imu.pose, imu.clockOffset, imu.period);
	o << "      # reading = diag(scale) * (I + M) * true + bias(t) + white noise; bias(t)\n"
		 "      # starts at `bias` and follows the random walk.\n";
	writeImuChannel(o, "gyroscope", imu.gyroscope);
	writeImuChannel(o, "accelerometer", imu.accelerometer);
	if (!imu.biasFile.empty())
	{
		o << "      bias_file: " << str(imu.biasFile) << "\n";
	}
}

void writeGnss(std::ostream& o, const GnssGroundTruth& g, double tStart)
{
	const mvsim::GNSS& s = *g.sensor;
	writeSensorHeader(
		o, g.label, "gnss", s.sensorPoseOnVehicle(), g.clockOffset, s.sensor_period());
	o << "      # The antenna is at pose_in_vehicle.xyz.\n";
	o << "      quality: " << mvsim::GNSS::fixTypeName(s.fixType()) << "\n";
	o << "      horizontal_std: " << num(s.horizontalStdNoise()) << "\n";
	o << "      vertical_std: " << num(s.verticalStdNoise()) << "\n";
	o << "      noise_scale: " << num(g.noiseScale) << "\n";
	o << "      # Intervals [t_start, t_end) whose quality, reported sigmas or injected\n"
		 "      # ENU error [m] (with rtk_fixed, a false fix) differ from the above: the\n"
		 "      # values in effect, from all the <event> tags covering them.\n";

	// The quality is constant between consecutive event boundaries:
	std::vector<double> cuts;
	for (const auto& ev : s.events())
	{
		cuts.push_back(ev.start);
		cuts.push_back(ev.end);
	}
	std::sort(cuts.begin(), cuts.end());
	cuts.erase(std::unique(cuts.begin(), cuts.end()), cuts.end());
	const mvsim::GNSS::Quality nominal = s.qualityAt(-std::numeric_limits<double>::infinity());
	struct Episode
	{
		double start;
		double end;
		mvsim::GNSS::Quality q;
	};
	std::vector<Episode> episodes;
	for (size_t i = 0; i + 1 < cuts.size(); i++)
	{
		const auto q = s.qualityAt(cuts[i]);
		if (q == nominal)
		{
			continue;
		}
		if (!episodes.empty() && episodes.back().end == cuts[i] && episodes.back().q == q)
		{
			episodes.back().end = cuts[i + 1];
		}
		else
		{
			episodes.push_back({cuts[i], cuts[i + 1], q});
		}
	}
	o << "      episodes:" << (episodes.empty() ? " []" : "") << "\n";
	for (const auto& e : episodes)
	{
		o << "        - {t_start: " << num(tStart + e.start) << ", t_end: " << num(tStart + e.end)
		  << ", quality: "
		  << (e.q.outage ? std::string("outage") : mvsim::GNSS::fixTypeName(e.q.fix_type))
		  << ", horizontal_std: " << num(e.q.horizontal_std_noise)
		  << ", vertical_std: " << num(e.q.vertical_std_noise)
		  << ", injected_error_m: " << vec3(e.q.offset * g.noiseScale) << "}\n";
	}
}

/** The intrinsics of an image stream, at `indent`, optionally with its
 * pose (for the sub-blocks of RGB-D cameras). */
void writeCameraImage(
	std::ostream& o, const std::string& indent, const CameraImageGroundTruth& im, bool withPose)
{
	const auto& c = im.camera;
	if (withPose)
	{
		mrpt::math::CQuaternionDouble q;
		im.pose.getAsQuaternion(q);
		o << indent << "pose_in_vehicle:\n";
		o << indent << "  xyz: " << vec({im.pose.x(), im.pose.y(), im.pose.z()}) << "\n";
		o << indent << "  quat_xyzw: " << vec({q.x(), q.y(), q.z(), q.r()}) << "\n";
		o << indent << "  ypr_deg: "
		  << vec(
				 {mrpt::RAD2DEG(im.pose.yaw()), mrpt::RAD2DEG(im.pose.pitch()),
				  mrpt::RAD2DEG(im.pose.roll())})
		  << "\n";
	}
	o << indent << "width: " << c.ncols << "\n";
	o << indent << "height: " << c.nrows << "\n";
	o << indent << "fx: " << num(c.fx()) << "\n";
	o << indent << "fy: " << num(c.fy()) << "\n";
	o << indent << "cx: " << num(c.cx()) << "\n";
	o << indent << "cy: " << num(c.cy()) << "\n";
	if (c.distortion == mrpt::img::DistortionModel::plumb_bob)
	{
		o << indent << "distortion: {model: plumb_bob, coeffs: "
		  << vec({c.k1(), c.k2(), c.p1(), c.p2(), c.k3()}) << "}  # k1 k2 p1 p2 k3\n";
	}
	else
	{
		o << indent << "distortion: {model: none, coeffs: []}\n";
	}
	o << indent << "clip_m: [" << numf(im.clipMin) << ", " << numf(im.clipMax) << "]\n";
	if (im.isDepth)
	{
		o << indent << "depth_unit_m: " << numf(im.depthUnit) << "\n";
		o << indent << "depth_noise_std_m: " << numf(im.noiseStd) << "\n";
		const auto& e = im.depthErrors;
		o << indent << "depth_errors: {scale: " << num(e.scale) << ", offset_m: " << num(e.offset)
		  << ", radial: " << vec({e.radialK1, e.radialK2}) << "}\n";
	}
	else
	{
		o << indent << "rolling_shutter_line_time_s: " << num(im.rollingShutterLineTime) << "\n";
		if (im.rollingShutterLineTime > 0)
		{
			o << indent << "rolling_shutter_bands: " << im.rollingShutterBands << "\n";
		}
		o << indent << "image_noise_std: " << num(im.noiseStd) << "\n";
	}
	o << indent << "files: " << str(im.files) << "\n";
}

void writeCamera(std::ostream& o, const CameraGroundTruth& cam)
{
	const auto& main = cam.depth ? *cam.depth : *cam.rgb;
	const char* type =
		cam.rgb && cam.depth ? "rgbd_camera" : (cam.depth ? "depth_camera" : "camera");
	writeSensorHeader(o, cam.label, type, main.pose, cam.clockOffset, cam.period);
	if (cam.rgb && cam.depth)
	{
		o << "      rgb:\n";
		writeCameraImage(o, "        ", *cam.rgb, true);
		o << "      depth:\n";
		writeCameraImage(o, "        ", *cam.depth, true);
	}
	else
	{
		writeCameraImage(o, "      ", main, false);
	}
}

void writeOdometry(std::ostream& o, const OdometryGroundTruth& od)
{
	writeSensorHeader(o, od.label, "odometry", {}, od.clockOffset, od.period);
	o << "      trans_noise_rel: " << num(od.transNoiseRel) << "\n";
	o << "      rot_noise_rel: " << num(od.rotNoiseRel) << "\n";
}
}  // namespace

void mvsim_dataset_gen::writeGroundTruthYaml(const std::string& path, const DatasetGroundTruth& gt)
{
	std::ostringstream o;
	o << "# Sensor ground truth of a dataset written by mvsim-dataset-gen.\n"
		 "# Poses: of each sensor frame in the vehicle frame (T_vehicle_sensor).\n"
		 "# Times: seconds, in the clock of the trajectory. Each observation is\n"
		 "# stamped at its true sampling time plus clock_offset_s.\n";
	o << "format: mvsim/dataset_gt/1\n";
	o << "world: " << str(gt.worldFile) << "\n";
	o << "vehicle: " << str(gt.vehicle) << "\n";
	o << "base_frame: " << str(gt.baseFrame) << "\n";
	o << "rawlog: " << str(gt.rawlogFile) << "\n";
	o << "vehicle_ground_truth: " << str(gt.vehicleGtFile) << "\n";
	o << "time: {start: " << num(gt.tStart) << ", end: " << num(gt.tEnd) << "}\n";
	o << "lidar_shutter: " << gt.shutter << "\n";
	o << "seed: " << gt.seed << "\n";
	o << "noiseless: " << (gt.noiseless ? "true" : "false") << "\n";
	o << "noise_scale: " << num(gt.noiseScale) << "\n";
	if (gt.georef)
	{
		const auto& g = *gt.georef;
		o << "georeference:\n";
		o << "  latitude: " << num(g.georefCoord.lat.decimal_value) << "\n";
		o << "  longitude: " << num(g.georefCoord.lon.decimal_value) << "\n";
		o << "  height: " << num(g.georefCoord.height) << "\n";
		o << "  world_to_enu_rotation_deg: " << num(mrpt::RAD2DEG(g.world_to_enu_rotation)) << "\n";
		o << "  world_is_utm: " << (g.world_is_utm ? "true" : "false") << "\n";
	}
	o << "sensors:\n";
	for (const auto& l : gt.lidars)
	{
		writeLidar(o, l);
	}
	for (const auto& imu : gt.imus)
	{
		writeImu(o, imu);
	}
	for (const auto& g : gt.gnss)
	{
		writeGnss(o, g, gt.tStart);
	}
	for (const auto& od : gt.odometry)
	{
		writeOdometry(o, od);
	}
	for (const auto& cam : gt.cameras)
	{
		writeCamera(o, cam);
	}

	std::ofstream f(path);
	if (!f.is_open())
	{
		THROW_EXCEPTION("Cannot write ground-truth file '" + path + "'");
	}
	f << o.str();
	if (!f.good())
	{
		THROW_EXCEPTION("Error writing ground-truth file '" + path + "'");
	}
}
