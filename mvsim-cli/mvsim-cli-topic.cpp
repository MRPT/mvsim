/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2024  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#include <mrpt/core/exceptions.h>
#include <mrpt/math/distributions.h>
#include <mrpt/math/ops_containers.h>
#include <mrpt/system/datetime.h>

#if defined(MVSIM_HAS_ZMQ) && defined(MVSIM_HAS_PROTOBUF)
#include <mvsim/Comms/Client.h>
#include <mvsim/mvsim-msgs/ObservationLidar2D.pb.h>
#include <mvsim/mvsim-msgs/TimeStampedPose.pb.h>
#endif

#include "mvsim-cli.h"

static int printCommandsTopic(bool showErrorMsg);
static int topicList();
static int topicEcho();
static int topicHz();

const std::map<std::string, cmd_t> cliTopicCommands = {
	{"list", cmd_t(&topicList)},
	{"echo", cmd_t(&topicEcho)},
	{"hz", cmd_t(&topicHz)},
};

int commandTopic()
{
	const auto& lstCmds = cli->argCmd.getValue();
	if (cli->argHelp.isSet()) return printCommandsTopic(false);
	if (lstCmds.size() != 2 && lstCmds.size() != 3) return printCommandsTopic(true);

	// Take second unlabeled argument:
	const std::string subcommand = lstCmds.at(1);
	auto itSubcmd = cliTopicCommands.find(subcommand);

	if (itSubcmd == cliTopicCommands.end()) return printCommandsTopic(true);

	// Execute command:
	return (itSubcmd->second)();

	return 0;
}

int topicList()
{
#if defined(MVSIM_HAS_ZMQ) && defined(MVSIM_HAS_PROTOBUF)
	mvsim::Client client;

	client.setMinLoggingLevel(mrpt::typemeta::TEnumType<mrpt::system::VerbosityLevel>::name2value(
		cli->argVerbosity.getValue()));

	std::cout << "# Connecting to server...\n";
	client.connect();
	std::cout << "# Connected.\n";
	std::cout << "# Querying list of topics to server...\n";
	const auto lstTopics = client.requestListOfTopics();

	std::cout << "# Done. Found " << lstTopics.size() << " topics:\n";

	for (const auto& n : lstTopics)
	{
		if (cli->argDetails.isSet())
		{
			std::cout << "- name: \"" << n.name << "\"\n"
					  << "  type: \"" << n.type << "\"\n"
					  << "  publishers:\n";

			ASSERT_EQUAL_(n.endpoints.size(), n.publishers.size());

			for (size_t i = 0; i < n.endpoints.size(); i++)
			{
				std::cout << "    - endpoint: \"" << n.endpoints[i] << "\"\n"
						  << "    - publisherNode: \"" << n.publishers[i] << "\"\n";
			}
		}
		else
		{
			std::cout << n.name << " [" << n.type << "] from node ";
			if (n.publishers.size() != 1)
				std::cout << n.publishers.size() << " publishers.\n";
			else
				std::cout << n.publishers.at(0) << "\n";
		}
	}
#endif
	return 0;
}

#if defined(MVSIM_HAS_ZMQ) && defined(MVSIM_HAS_PROTOBUF)
static void echo_TimeStampedPose(const std::string& data)
{
	mvsim_msgs::TimeStampedPose out;
	if (bool ok = out.ParseFromString(data); !ok)
	{
		std::cerr << "ERROR: Protobuf could not parse message.\n";
		return;
	}
	out.PrintDebugString();
}
static void echo_ObservationLidar2D(const std::string& data)
{
	mvsim_msgs::ObservationLidar2D out;
	if (bool ok = out.ParseFromString(data); !ok)
	{
		std::cerr << "ERROR: Protobuf could not parse message.\n";
		return;
	}
	out.PrintDebugString();
}

static void callbackSubscribeTopicGeneric(const zmq::message_t& msg)
{
	const auto [typeName, serializedData] = mvsim::internal::parseMessageToParts(msg);
	std::cout << "[" << mrpt::system::dateTimeLocalToString(mrpt::Clock::now())
			  << "] Received data : \n";
	std::cout << " - typeName: " << typeName << "\n";
	std::cout << " - data: " << serializedData.size() << " bytes\n";

	if (typeName == mvsim_msgs::TimeStampedPose().GetTypeName())
		echo_TimeStampedPose(serializedData);
	else if (typeName == mvsim_msgs::ObservationLidar2D().GetTypeName())
		echo_ObservationLidar2D(serializedData);

	std::cout << std::endl;
}
#endif

int topicEcho()
{
#if defined(MVSIM_HAS_ZMQ) && defined(MVSIM_HAS_PROTOBUF)
	mvsim::Client client;

	client.setMinLoggingLevel(mrpt::typemeta::TEnumType<mrpt::system::VerbosityLevel>::name2value(
		cli->argVerbosity.getValue()));

	const auto& lstCmds = cli->argCmd.getValue();
	if (lstCmds.size() != 3) return printCommandsTopic(true);

	const auto& topicName = lstCmds.at(2);

	std::cout << "# Connecting to server...\n";
	client.connect();
	std::cout << "# Connected.\n";

	std::cout << "# Subscribing to topic '" << topicName << "'. Press CTRL+C to stop.\n";
	client.subscribe_topic_raw(topicName, &callbackSubscribeTopicGeneric);

	// loop until user does a CTRL+C
	for (;;)
	{
	}

#endif
	return 0;
}

int topicHz()
{
#if defined(MVSIM_HAS_ZMQ) && defined(MVSIM_HAS_PROTOBUF)
	mvsim::Client client;

	client.setMinLoggingLevel(mrpt::typemeta::TEnumType<mrpt::system::VerbosityLevel>::name2value(
		cli->argVerbosity.getValue()));

	const auto& lstCmds = cli->argCmd.getValue();
	if (lstCmds.size() != 3) return printCommandsTopic(true);

	const auto& topicName = lstCmds.at(2);

	std::cout << "# Connecting to server...\n";
	client.connect();
	std::cout << "# Connected.\n";

	const double WAIT_SECONDS = 5.0;

	std::cout << "# Subscribing to topic '" << topicName << "'. Will listen for " << WAIT_SECONDS
			  << " seconds...\n";

	int numMsgs = 0;
	std::optional<double> lastMsgTim;
	std::vector<double> measuredPeriods;

	client.subscribe_topic_raw(
		topicName,
		[&](const zmq::message_t&)
		{
			numMsgs++;
			const double t = mrpt::Clock::nowDouble();
			if (lastMsgTim)
			{
				const double dt = t - lastMsgTim.value();
				measuredPeriods.push_back(dt);
			}
			lastMsgTim = t;
		});

	std::this_thread::sleep_for(
		std::chrono::milliseconds(static_cast<size_t>(WAIT_SECONDS * 1000)));

	const double rate = numMsgs / WAIT_SECONDS;

	std::cout << std::endl;
	std::cout << "- ReceivedMsgs: " << numMsgs << std::endl;
	std::cout << "- Rate: " << rate << " # Hz" << std::endl;

	if (numMsgs > 0)
	{
		double periodMean = 0, periodStd = 0;
		mrpt::math::meanAndStd(measuredPeriods, periodMean, periodStd);

		std::cout << "- MeanPeriod: " << periodMean << " # [sec] 1/T = " << 1.0 / periodMean
				  << " Hz" << std::endl;

		std::cout << "- PeriodStdDev: " << periodStd << " # [sec]" << std::endl;

		double periodMin = 0, periodMax = 0;
		mrpt::math::minimum_maximum(measuredPeriods, periodMin, periodMax);

		std::cout << "- PeriodMin: " << periodMin << " # [sec]" << std::endl;
		std::cout << "- PeriodMax: " << periodMax << " # [sec]" << std::endl;

		const double conf = 0.05;
		double tMean, tLow, tHigh;

		mrpt::math::CVectorDouble x;
		for (size_t i = 0; i < measuredPeriods.size(); i++) x.push_back(measuredPeriods[i]);

		mrpt::math::confidenceIntervals(x, tMean, tLow, tHigh, conf, 100);

		std::cout << "- Period_05percent: " << tLow << " # [sec]" << std::endl;
		std::cout << "- Period_95percent: " << tHigh << " # [sec]" << std::endl;
	}

#endif
	return 0;
}

int printCommandsTopic(bool showErrorMsg)
{
	if (showErrorMsg)
	{
		setConsoleErrorColor();
		std::cerr << "Error: missing or unknown subcommand.\n";
		setConsoleNormalColor();
	}

	fprintf(
		stderr,
		R"XXX(Usage:

    mvsim topic --help            Show this help
    mvsim topic list [--details]  List all advertised topics in the server
    mvsim topic echo <topicName>  Subscribe and print a topic
    mvsim topic hz <topicName>    Estimate topic publication rate (in Hz)

)XXX");

	return showErrorMsg ? 1 : 0;
}
