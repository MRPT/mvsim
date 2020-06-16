/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2020  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#include <mrpt/core/exceptions.h>
#include <mrpt/version.h>
#include <mvsim/Comms/Client.h>
#include <mvsim/Comms/common.h>
#include <mvsim/Comms/ports.h>
#if MRPT_VERSION >= 0x204
#include <mrpt/system/thread_name.h>
#endif

#if defined(MVSIM_HAS_ZMQ) && defined(MVSIM_HAS_PROTOBUF)

#include <google/protobuf/text_format.h>

#include <zmq.hpp>

#include "RegisterNodeAnswer.pb.h"
#include "RegisterNodeRequest.pb.h"

#endif

using namespace mvsim;

Client::Client(const std::string& nodeName)
	: mrpt::system::COutputLogger("mvsim::Client"), nodeName_(nodeName)
{
}
Client::~Client() { shutdown(); }

void Client::connect()
{
	ASSERTMSG_(!mainThread_.joinable(), "Client is already running.");

#if defined(MVSIM_HAS_ZMQ) && defined(MVSIM_HAS_PROTOBUF)
	requestMainThreadTermination();
	mainThread_ = std::thread(&Client::internalClientThread, this);

#if MRPT_VERSION >= 0x204
	mrpt::system::thread_name("clientMain", mainThread_);
#endif

#else
	THROW_EXCEPTION(
		"MVSIM needs building with ZMQ and PROTOBUF to enable "
		"client/server");
#endif
}

void Client::shutdown() noexcept
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

void Client::internalClientThread()
{
	using namespace std::string_literals;

#if defined(MVSIM_HAS_ZMQ) && defined(MVSIM_HAS_PROTOBUF)
	try
	{
		MRPT_LOG_INFO_STREAM("Client thread started.");

		zmq::context_t context(1);
		mainThreadZMQcontext_ = &context;

		zmq::socket_t mainReqSocket(context, ZMQ_REQ);
		mainReqSocket.connect(
			"tcp://"s + serverHostAddress_ + ":"s +
			std::to_string(MVSIM_PORTNO_MAIN_REP));

		// Let the server know about this new node:
		//------------------------------------------
		mvsim_msgs::RegisterNodeRequest rnq;
		rnq.set_nodename(nodeName_);
		mvsim::sendMessage(rnq, mainReqSocket);

		//  Get the reply.
		zmq::message_t reply;
#if ZMQ_VERSION >= ZMQ_MAKE_VERSION(4, 3, 1)
		std::optional<size_t> msgSize = mainReqSocket.recv(reply);
		ASSERT_(msgSize.has_value());
#else
		mainReqSocket.recv(&reply);
#endif

		mvsim_msgs::RegisterNodeAnswer rna;
		mvsim::parseMessage(reply, rna);

		std::string s;
		google::protobuf::TextFormat::PrintToString(rna, &s);
		MRPT_LOG_DEBUG_STREAM("Recv as text:\n" << s);

		if (!rna.success())
		{
			THROW_EXCEPTION_FMT(
				"Server did not allow registering node: %s",
				rna.errormessage().c_str());
		}
		MRPT_LOG_DEBUG("Successfully registered in the server.");
	}
	catch (const zmq::error_t& e)
	{
		if (e.num() == ETERM)
		{
			// This simply means someone called requestMainThreadTermination().
			// Just exit silently.
			MRPT_LOG_INFO_STREAM(
				"Client thread about to exit for ZMQ term signal.");
		}
		else
		{
			MRPT_LOG_ERROR_STREAM(
				"internalClientThread: ZMQ error: " << e.what());
		}
	}
	catch (const std::exception& e)
	{
		MRPT_LOG_ERROR_STREAM(
			"internalClientThread: Exception: " << mrpt::exception_to_str(e));
	}
	MRPT_LOG_INFO_STREAM("Client thread quitted.");

	mainThreadZMQcontext_ = nullptr;
#endif
}

void Client::requestMainThreadTermination()
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
