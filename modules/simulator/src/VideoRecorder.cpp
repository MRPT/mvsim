/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2026  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#include <mrpt/core/exceptions.h>
#include <mrpt/core/format.h>
#include <mrpt/img/CImage.h>
#include <mrpt/opengl/CFBORender.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/system/string_utils.h>
#include <mvsim/TParameterDefinitions.h>
#include <mvsim/VideoRecorder.h>

#include <atomic>
#include <cmath>
#include <condition_variable>
#include <ctime>
#include <deque>
#include <iostream>
#include <thread>

#include "xml_utils.h"

#if defined(_WIN32)
#include <cstdio>
#else
#include <fcntl.h>
#include <signal.h>
#include <spawn.h>
#include <sys/wait.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>

extern char** environ;
#endif

using namespace mvsim;

namespace
{
// Frames waiting to be encoded. When full, the rendering (and so the
// simulation) waits for the encoder:
constexpr size_t MAX_QUEUED_FRAMES = 16;

bool is_image_sequence(const std::string& outputFile)
{
	return outputFile.find('%') != std::string::npos;
}

std::string default_output_file()
{
	const std::time_t t = std::time(nullptr);
	std::tm tm{};
#if defined(_WIN32)
	localtime_s(&tm, &t);
#else
	localtime_r(&t, &tm);
#endif
	char buf[64];
	std::strftime(buf, sizeof(buf), "mvsim_video_%Y%m%d_%H%M%S.mp4", &tm);
	return buf;
}
}  // namespace

// ----------------------------------------------------------------------------
// Encoder: a thread writing the frames to ffmpeg (through a pipe to its
// stdin), or to image files.
// ----------------------------------------------------------------------------
class VideoRecorder::Encoder
{
   public:
	explicit Encoder(const Options& o) : opts_(o)
	{
		if (!is_image_sequence(opts_.output_file))
		{
			launchFfmpeg();
		}
		thread_ = std::thread(&Encoder::run, this);
	}

	~Encoder() { finish(); }

	Encoder(const Encoder&) = delete;
	Encoder& operator=(const Encoder&) = delete;

	/** Queues a frame, to be written "repeats" times. It waits while the
	 * queue is full. Returns false if the encoder is closed. */
	bool push(mrpt::img::CImage&& img, size_t repeats)
	{
		std::unique_lock<std::mutex> lck(mtx_);
		cvNotFull_.wait(lck, [this]() { return closing_ || queue_.size() < MAX_QUEUED_FRAMES; });
		if (closing_)
		{
			return false;
		}
		queue_.emplace_back(std::move(img), repeats);
		cvNotEmpty_.notify_one();
		return true;
	}

	/** Writes the queued frames, then closes the output (and waits for ffmpeg
	 *  to complete the file). */
	void finish()
	{
		{
			std::lock_guard<std::mutex> lck(mtx_);
			if (closing_ && !thread_.joinable())
			{
				return;
			}
			closing_ = true;
		}
		cvNotEmpty_.notify_all();
		cvNotFull_.notify_all();
		if (thread_.joinable())
		{
			thread_.join();
		}
		closeFfmpeg();
	}

	size_t framesWritten() const { return framesWritten_; }

	std::string error() const
	{
		std::lock_guard<std::mutex> lck(mtx_);
		return error_;
	}

   private:
	const Options opts_;

	mutable std::mutex mtx_;
	std::condition_variable cvNotEmpty_, cvNotFull_;
	std::deque<std::pair<mrpt::img::CImage, size_t>> queue_;
	bool closing_ = false;
	std::string error_;
	std::atomic<size_t> framesWritten_{0};
	std::thread thread_;

#if defined(_WIN32)
	FILE* pipe_ = nullptr;
#else
	int pipeFd_ = -1;
	pid_t pid_ = -1;
#endif

	void setError(const std::string& e)
	{
		std::lock_guard<std::mutex> lck(mtx_);
		if (error_.empty())
		{
			error_ = e;
		}
	}

	std::vector<std::string> ffmpegArgs() const
	{
		std::vector<std::string> args = {
			"ffmpeg",	 "-hide_banner",
			"-loglevel", "error",
			"-nostdin",	 "-y",
			"-f",		 "rawvideo",
			"-pix_fmt",	 "rgb24",
			"-s",		 mrpt::format("%ux%u", opts_.width, opts_.height),
			"-r",		 mrpt::format("%g", opts_.fps),
			"-i",		 "-"};
		std::vector<std::string> extra;
		mrpt::system::tokenize(opts_.ffmpeg_options, " \t", extra);
		args.insert(args.end(), extra.begin(), extra.end());
		args.push_back(opts_.output_file);
		return args;
	}

#if defined(_WIN32)
	void launchFfmpeg()
	{
		std::string cmd;
		for (const auto& a : ffmpegArgs())
		{
			cmd += (a.find(' ') != std::string::npos) ? "\"" + a + "\" " : a + " ";
		}
		pipe_ = _popen(cmd.c_str(), "wb");
		if (!pipe_)
		{
			THROW_EXCEPTION_FMT("Could not launch: %s", cmd.c_str());
		}
	}

	bool writeBytes(const uint8_t* data, size_t len)
	{
		return pipe_ && std::fwrite(data, 1, len, pipe_) == len;
	}

	void closeFfmpeg()
	{
		if (pipe_)
		{
			const int ret = _pclose(pipe_);
			pipe_ = nullptr;
			if (ret != 0)
			{
				setError(mrpt::format("ffmpeg exited with code %d", ret));
			}
		}
	}
#else
	void launchFfmpeg()
	{
		int fds[2];
		if (::pipe2(fds, O_CLOEXEC) != 0)
		{
			THROW_EXCEPTION_FMT("pipe2() failed: %s", std::strerror(errno));
		}

		posix_spawn_file_actions_t actions;
		posix_spawn_file_actions_init(&actions);
		posix_spawn_file_actions_adddup2(&actions, fds[0], STDIN_FILENO);

		// In its own process group, so Ctrl+C in the terminal does not stop
		// it before the video file is complete:
		posix_spawnattr_t attr;
		posix_spawnattr_init(&attr);
		posix_spawnattr_setflags(&attr, POSIX_SPAWN_SETPGROUP);
		posix_spawnattr_setpgroup(&attr, 0);

		const auto args = ffmpegArgs();
		std::vector<char*> argv;
		for (const auto& a : args)
		{
			argv.push_back(const_cast<char*>(a.c_str()));
		}
		argv.push_back(nullptr);

		const int ret = posix_spawnp(&pid_, argv[0], &actions, &attr, argv.data(), environ);

		posix_spawn_file_actions_destroy(&actions);
		posix_spawnattr_destroy(&attr);
		::close(fds[0]);

		if (ret != 0)
		{
			::close(fds[1]);
			pid_ = -1;
			THROW_EXCEPTION_FMT(
				"Could not launch 'ffmpeg' (%s). Install it, or save the frames as images with an "
				"output file name like 'frames/img_%%06d.png'.",
				std::strerror(ret));
		}
		pipeFd_ = fds[1];
	}

	bool writeBytes(const uint8_t* data, size_t len)
	{
		while (len > 0 && pipeFd_ >= 0)
		{
			const ssize_t n = ::write(pipeFd_, data, len);
			if (n < 0)
			{
				if (errno == EINTR)
				{
					continue;
				}
				return false;
			}
			data += n;
			len -= static_cast<size_t>(n);
		}
		return len == 0;
	}

	void closeFfmpeg()
	{
		if (pipeFd_ >= 0)
		{
			::close(pipeFd_);  // EOF: ffmpeg completes the file and exits
			pipeFd_ = -1;
		}
		if (pid_ > 0)
		{
			int status = 0;
			while (::waitpid(pid_, &status, 0) < 0 && errno == EINTR)
			{
			}
			pid_ = -1;
			if (!WIFEXITED(status) || WEXITSTATUS(status) != 0)
			{
				setError(mrpt::format(
					"ffmpeg failed (exit status %d). Check its options and the output file '%s'",
					WIFEXITED(status) ? WEXITSTATUS(status) : -1, opts_.output_file.c_str()));
			}
		}
	}
#endif

	void run()
	{
#if !defined(_WIN32)
		// If ffmpeg dies, writes must fail (EPIPE) instead of killing the
		// whole process with SIGPIPE:
		sigset_t set;
		sigemptyset(&set);
		sigaddset(&set, SIGPIPE);
		pthread_sigmask(SIG_BLOCK, &set, nullptr);
#endif
		const bool images = is_image_sequence(opts_.output_file);
		const size_t frameBytes = static_cast<size_t>(opts_.width) * opts_.height * 3;
		bool failed = false;

		for (;;)
		{
			std::pair<mrpt::img::CImage, size_t> frame;
			{
				std::unique_lock<std::mutex> lck(mtx_);
				cvNotEmpty_.wait(lck, [this]() { return closing_ || !queue_.empty(); });
				if (queue_.empty())
				{
					break;	// closing, and all frames written
				}
				frame = std::move(queue_.front());
				queue_.pop_front();
			}
			cvNotFull_.notify_one();

			const auto& img = frame.first;
			if (failed || img.getWidth() != opts_.width || img.getHeight() != opts_.height)
			{
				continue;
			}

			for (size_t i = 0; i < frame.second && !failed; i++)
			{
				if (images)
				{
					const auto file = mrpt::format(
						opts_.output_file.c_str(), static_cast<int>(framesWritten_.load()));
					if (!img.saveToFile(file))
					{
						setError("Could not write image file: " + file);
						failed = true;
					}
				}
				else if (!writeBytes(img.ptrLine<uint8_t>(0), frameBytes))
				{
					setError("Could not write to ffmpeg: it exited prematurely");
					failed = true;
				}
				if (!failed)
				{
					framesWritten_++;
				}
			}
		}
	}
};

// ----------------------------------------------------------------------------
// VideoRecorder
// ----------------------------------------------------------------------------

bool VideoRecorder::Options::loadFromXML(
	const rapidxml::xml_node<char>& node, const std::map<std::string, std::string>& vars)
{
	bool enabled = false;
	const TParameterDefinitions attribs = {{"enabled", {"%bool", &enabled}}};
	parse_xmlnode_attribs(node, attribs, vars, "[VideoRecorder]");

	const TParameterDefinitions params = {
		{"output_file", {"%s", &output_file}},
		{"fps", {"%lf", &fps}},
		{"width", {"%u", &width}},
		{"height", {"%u", &height}},
		{"start_time", {"%lf", &start_time}},
		{"duration", {"%lf", &duration}},
		{"close_simulator_when_done", {"%bool", &close_simulator_when_done}},
		{"ffmpeg_options", {"%s", &ffmpeg_options}},
		{"camera", {"%s", &camera}},
		{"follow_vehicle", {"%s", &follow_vehicle}},
		{"follow_heading", {"%bool", &follow_heading}},
		{"cam_distance", {"%lf", &cam_distance}},
		{"cam_azimuth", {"%lf", &cam_azimuth}},
		{"cam_elevation", {"%lf", &cam_elevation}},
		{"cam_point_to", {"%point3d", &cam_point_to}},
		{"fov_deg", {"%lf", &fov_deg}},
		{"ortho", {"%bool", &ortho}},
	};
	parse_xmlnode_children_as_param(node, params, vars, "[VideoRecorder]");
	return enabled;
}

VideoRecorder::VideoRecorder() = default;

VideoRecorder::~VideoRecorder()
{
	std::lock_guard<std::mutex> lck(mtx_);
	internalStop();
}

void VideoRecorder::start(const Options& opts)
{
	Options o = opts;
	if (o.output_file.empty())
	{
		o.output_file = default_output_file();
	}
	ASSERTMSG_(o.fps > 0 && std::isfinite(o.fps), "Video fps must be a positive number");
	ASSERTMSG_(o.width >= 16 && o.height >= 16, "Video width and height must be >= 16");
	ASSERTMSG_(
		std::isfinite(o.duration) && o.duration >= 0, "Video duration must be a number >= 0");
	ASSERTMSG_(
		o.camera == "gui" || o.camera == "orbit", "Video camera must be either 'gui' or 'orbit'");
	if (!is_image_sequence(o.output_file))
	{
		// Most video codecs (e.g. yuv420p) require even sizes:
		o.width &= ~1U;
		o.height &= ~1U;
	}

	if (const auto dir = mrpt::system::extractFileDirectory(o.output_file);
		!dir.empty() && !mrpt::system::directoryExists(dir))
	{
		mrpt::system::createDirectory(dir);
	}

	// Launch the encoder before touching the current recording, in case it
	// fails:
	auto enc = std::make_shared<Encoder>(o);

	std::lock_guard<std::mutex> lck(mtx_);
	internalStop();
	encoder_ = std::move(enc);
	opts_ = std::move(o);
	startTime_.reset();
	nextFrameTime_ = 0;
	pendingFrameRepeats_ = 0;
	framesRequested_ = 0;
	last_ = Status();
}

VideoRecorder::Status VideoRecorder::stop()
{
	std::lock_guard<std::mutex> lck(mtx_);
	return internalStop();
}

VideoRecorder::Status VideoRecorder::internalStop()
{
	if (!opts_)
	{
		return internalStatus();
	}
	if (encoder_)
	{
		encoder_->finish();
	}
	auto st = internalStatus();
	st.recording = false;
	last_ = st;
	opts_.reset();
	encoder_.reset();
	pendingFrameRepeats_ = 0;
	return st;
}

VideoRecorder::Status VideoRecorder::status() const
{
	std::lock_guard<std::mutex> lck(mtx_);
	return internalStatus();
}

VideoRecorder::Status VideoRecorder::internalStatus() const
{
	if (!opts_)
	{
		return last_;
	}
	Status st;
	st.recording = true;
	if (encoder_)
	{
		st.output_file = opts_->output_file;
		st.frames = encoder_->framesWritten();
		st.duration = static_cast<double>(st.frames) / opts_->fps;
		st.error = encoder_->error();
	}
	return st;
}

bool VideoRecorder::isRecording() const
{
	std::lock_guard<std::mutex> lck(mtx_);
	return opts_.has_value();
}

std::optional<VideoRecorder::Options> VideoRecorder::currentOptions() const
{
	std::lock_guard<std::mutex> lck(mtx_);
	return opts_;
}

void VideoRecorder::setDefaultOptions(const Options& opts, bool autostart)
{
	std::lock_guard<std::mutex> lck(mtx_);
	defaultOpts_ = opts;
	autostartPending_ = autostart;
}

VideoRecorder::Options VideoRecorder::defaultOptions() const
{
	std::lock_guard<std::mutex> lck(mtx_);
	return defaultOpts_;
}

bool VideoRecorder::onSimulationStep(double simTime, bool& closeSimulator)
{
	closeSimulator = false;
	constexpr double eps = 1e-9;

	std::optional<Options> toStart;
	{
		std::lock_guard<std::mutex> lck(mtx_);
		if (!opts_ && autostartPending_ && simTime + eps >= defaultOpts_.start_time)
		{
			autostartPending_ = false;
			toStart = defaultOpts_;
		}
	}
	if (toStart)
	{
		try
		{
			start(*toStart);
		}
		catch (const std::exception& e)
		{
			std::lock_guard<std::mutex> lck(mtx_);
			last_ = Status();
			last_.output_file = toStart->output_file;
			last_.error = mrpt::exception_to_str(e);
			std::cerr << "[mvsim::VideoRecorder] Could not start recording: " << last_.error
					  << std::endl;
		}
	}

	std::lock_guard<std::mutex> lck(mtx_);
	if (!opts_)
	{
		return false;
	}

	const double T = 1.0 / opts_->fps;
	const size_t totalFrames =
		opts_->duration > 0 ? static_cast<size_t>(std::lround(opts_->duration * opts_->fps)) : 0;

	if (!startTime_)
	{
		startTime_ = simTime;
		nextFrameTime_ = simTime;
	}

	if (totalFrames > 0 && framesRequested_ >= totalFrames && pendingFrameRepeats_ == 0)
	{
		closeSimulator = opts_->close_simulator_when_done;
		internalStop();
		return false;
	}

	if (simTime + eps >= nextFrameTime_)
	{
		// If the time step is longer than the frame period, the frame is
		// repeated, so the video still plays at real speed:
		size_t n = 1 + static_cast<size_t>(std::floor((simTime - nextFrameTime_) / T + eps));
		if (totalFrames > 0)
		{
			n = std::min(n, totalFrames - framesRequested_);
		}
		pendingFrameRepeats_ += n;
		framesRequested_ += n;
		nextFrameTime_ += static_cast<double>(n) * T;
	}
	return pendingFrameRepeats_ > 0;
}

std::optional<double> VideoRecorder::nextFrameTime() const
{
	std::lock_guard<std::mutex> lck(mtx_);
	if (!opts_)
	{
		return std::nullopt;
	}
	return startTime_ ? nextFrameTime_ : 0.0;
}

bool VideoRecorder::hasPendingFrame() const
{
	std::lock_guard<std::mutex> lck(mtx_);
	return opts_.has_value() && pendingFrameRepeats_ > 0;
}

void VideoRecorder::renderPendingFrame(
	const mrpt::viz::Scene& scene, const mrpt::viz::CCamera& camera,
	const std::function<bool()>& hasToCreateEGLContext)
{
	std::shared_ptr<Encoder> enc;
	size_t repeats = 0;
	unsigned int w = 0;
	unsigned int h = 0;
	{
		std::lock_guard<std::mutex> lck(mtx_);
		if (!opts_ || !encoder_ || pendingFrameRepeats_ == 0)
		{
			return;
		}
		enc = encoder_;
		repeats = pendingFrameRepeats_;
		pendingFrameRepeats_ = 0;
		w = opts_->width;
		h = opts_->height;
	}

	std::shared_ptr<mrpt::opengl::CFBORender> fbo;
	for (const auto& f : fbos_)
	{
		if (f->width() == w && f->height() == h)
		{
			fbo = f;
		}
	}
	if (!fbo)
	{
		mrpt::opengl::CFBORender::Parameters p;
		p.width = w;
		p.height = h;
		p.create_EGL_context = hasToCreateEGLContext();
		fbo = std::make_shared<mrpt::opengl::CFBORender>(p);
		fbos_.push_back(fbo);
	}

	fbo->setCamera(camera);

	mrpt::img::CImage img;	// a new one per frame: images are shallow copies
	fbo->render_RGB(scene, img);

	// It waits here if the encoder is slower:
	enc->push(std::move(img), repeats);
}

void VideoRecorder::freeOpenGLResources() { fbos_.clear(); }
