/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2020  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#include <mrpt/core/exceptions.h>
#include <mvsim/Server/Server.h>

using namespace mvsim;

Server::Server() : mrpt::system::COutputLogger("mvsim::Server") {}

Server::~Server() { shutdown(); }

void Server::start()
{
	ASSERTMSG_(world_ != nullptr, "Server is not attached to any mvsim::World");

	ASSERTMSG_(!mainThread_.joinable(), "Server is already running.");

	mainThreadMustExit_ = false;
	mainThread_ = std::thread(&Server::internalServerThread, this);
}

void Server::shutdown() noexcept
{
	try
	{
		MRPT_LOG_INFO_STREAM("Waiting for the thread to quit.");
		mainThreadMustExit_ = true;
		if (mainThread_.joinable()) mainThread_.join();
	}
	catch (const std::exception& e)
	{
		MRPT_LOG_ERROR_STREAM(
			"shutdown: Exception: " << mrpt::exception_to_str(e));
	}
}

void Server::internalServerThread()
{
	try
	{
		MRPT_LOG_INFO_STREAM("Server thread started.");

		while (!mainThreadMustExit_)
		{
			std::this_thread::sleep_for(std::chrono::milliseconds(100));
		}
	}
	catch (const std::exception& e)
	{
		MRPT_LOG_ERROR_STREAM(
			"internalServerThread: Exception: " << mrpt::exception_to_str(e));
	}
	MRPT_LOG_INFO_STREAM("Server thread quitted.");
}
