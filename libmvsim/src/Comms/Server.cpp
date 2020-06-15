/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2020  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#include <mrpt/core/exceptions.h>
#include <mvsim/Comms/Server.h>
#include <mvsim/Comms/common.h>
#include <mvsim/Comms/ports.h>

#if defined(MVSIM_HAS_ZMQ) && defined(MVSIM_HAS_PROTOBUF)
#include <zmq.hpp>

#include "RegisterNodeAnswer.pb.h"
#include "RegisterNodeRequest.pb.h"

#endif

using namespace mvsim;

Server::Server() : mrpt::system::COutputLogger("mvsim::Server") {}

Server::~Server() { shutdown(); }

void Server::start()
{
	ASSERTMSG_(!mainThread_.joinable(), "Server is already running.");

#if defined(MVSIM_HAS_ZMQ) && defined(MVSIM_HAS_PROTOBUF)
	requestMainThreadTermination();
	mainThread_ = std::thread(&Server::internalServerThread, this);
#else
	THROW_EXCEPTION(
		"MVSIM needs building with ZMQ and PROTOBUF to enable client/server");
#endif
}

void Server::shutdown() noexcept
{
	try
	{
		MRPT_LOG_DEBUG_STREAM("Waiting for the thread to quit.");
		requestMainThreadTermination();

		if (mainThread_.joinable()) mainThread_.join();

		MRPT_LOG_DEBUG_STREAM("Joined thread.");
	}
	catch (const std::exception& e)
	{
		MRPT_LOG_ERROR_STREAM(
			"shutdown: Exception: " << mrpt::exception_to_str(e));
	}
}

void Server::internalServerThread()
{
	using namespace std::string_literals;

#if defined(MVSIM_HAS_ZMQ) && defined(MVSIM_HAS_PROTOBUF)
	try
	{
		MRPT_LOG_INFO_STREAM("Server thread started.");

		zmq::context_t context(1);
		mainThreadZMQcontext_ = &context;

		zmq::socket_t mainRepSocket(context, ZMQ_REP);
		mainRepSocket.bind("tcp://*:"s + std::to_string(MVSIM_PORTNO_MAIN_REP));

		for (;;)
		{
			zmq::message_t request;

			//  Wait for next request from client
#if ZMQ_VERSION >= ZMQ_MAKE_VERSION(4, 4, 0)
			std::optional<size_t> reqSize = mainRepSocket.recv(request);
			ASSERT_(reqSize.has_value());
#else
			mainRepSocket.recv(&request);
#endif

#if ZMQ_VERSION >= ZMQ_MAKE_VERSION(4, 4, 0)
			MRPT_LOG_DEBUG_STREAM("Received: " << request.str());
#endif

			MRPT_TODO("Actual dispatch");

			//  Send reply back to client
			mvsim_msgs::RegisterNodeAnswer rna;
			rna.set_success(true);
			mvsim::sendMessage(rna, mainRepSocket);
		}
	}
	catch (const zmq::error_t& e)
	{
		if (e.num() == ETERM)
		{
			// This simply means someone called requestMainThreadTermination().
			// Just exit silently.
			MRPT_LOG_INFO_STREAM(
				"Server thread about to exit for ZMQ term signal.");
		}
		else
		{
			MRPT_LOG_ERROR_STREAM(
				"internalServerThread: ZMQ error: " << e.what());
		}
	}
	catch (const std::exception& e)
	{
		MRPT_LOG_ERROR_STREAM(
			"internalServerThread: Exception: " << mrpt::exception_to_str(e));
	}
	MRPT_LOG_DEBUG_STREAM("Server thread quitted.");

	mainThreadZMQcontext_ = nullptr;
#endif
}

void Server::requestMainThreadTermination()
{
#if defined(MVSIM_HAS_ZMQ) && defined(MVSIM_HAS_PROTOBUF)
	zmq::context_t* ctx = mainThreadZMQcontext_;
	if (ctx)
	{
#if ZMQ_VERSION >= ZMQ_MAKE_VERSION(4, 4, 0)
		ctx->shutdown();
#else
		// Missing shutdown() in older versions:
		zmq_ctx_shutdown(ctx->operator void*());
#endif
	}
#endif
}
