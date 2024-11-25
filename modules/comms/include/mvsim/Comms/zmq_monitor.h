/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2024  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/core/format.h>

#include <atomic>
#include <iostream>
#include <mutex>
#include <thread>

#if defined(MVSIM_HAS_ZMQ) && defined(MVSIM_HAS_PROTOBUF)
#include <zmq.hpp>

namespace mvsim
{
/** \ingroup mvsim_comms_module
 * @{ */

class SocketMonitor : public zmq::monitor_t
{
   public:
	SocketMonitor() = default;
	~SocketMonitor()
	{
		zmq::monitor_t::abort();
		if (runningMonitor_.joinable()) runningMonitor_.join();
	}

	void monitor(zmq::socket_t& s)
	{
		static std::atomic_int nonce = 1000;
		const int v = nonce++;

		const std::string endpoint =
			mrpt::format("inproc://monitor%i_%p.req", v, s.operator void*());

		runningMonitor_ = std::thread(
			[&, endpoint]()
			{
				try
				{
					zmq::monitor_t::monitor(s, endpoint, ZMQ_EVENT_ALL);
				}
				catch (const std::exception& e)
				{
					if (zmq_errno() == ETERM)
					{
						// Not a real error, just we are shutting down.
					}
					else
					{
						std::cerr << "[MySocketMonitor] Error: " << e.what()
								  << " (zmq_errno=" << zmq_errno() << ")\n";
					}
				}
			});
	}

	void setConnected(bool v)
	{
		std::lock_guard<std::mutex> lck(connectedMtx_);
		connected_ = v;
	}

	void on_event_disconnected(
		[[maybe_unused]] const zmq_event_t& event_, [[maybe_unused]] const char* addr_) override
	{
		setConnected(false);
	}

	void on_event_connected(
		[[maybe_unused]] const zmq_event_t& event_, [[maybe_unused]] const char* addr_) override
	{
		setConnected(true);
	}

	bool connected() const
	{
		std::lock_guard<std::mutex> lck(connectedMtx_);
		return connected_;
	}

   private:
	bool connected_ = false;
	mutable std::mutex connectedMtx_;

	std::thread runningMonitor_;
};

/** @} */

}  // namespace mvsim

#endif
