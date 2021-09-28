/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2020  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#include <mrpt/core/exceptions.h>
#include <mrpt/system/datetime.h>
#include <mvsim/Comms/Client.h>
#include <mvsim/mvsim-msgs/TimeStampedPose.pb.h>

#include "mvsim-cli.h"

static int printCommandsTopic(bool showErrorMsg);
static int topicList();
static int topicEcho();

static const std::map<std::string, cmd_t> cliTopicCommands = {
	{"list", cmd_t(&topicList)},
	{"echo", cmd_t(&topicEcho)},
};

int commandTopic()
{
	const auto& lstCmds = argCmd.getValue();
	if (argHelp.isSet()) return printCommandsTopic(false);
	if (lstCmds.size() != 2 && lstCmds.size() != 3)
		return printCommandsTopic(true);

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
	mvsim::Client client;

	client.setMinLoggingLevel(
		mrpt::typemeta::TEnumType<mrpt::system::VerbosityLevel>::name2value(
			argVerbosity.getValue()));

	std::cout << "# Connecting to server...\n";
	client.connect();
	std::cout << "# Connected.\n";
	std::cout << "# Querying list of topics to server...\n";
	const auto lstTopics = client.requestListOfTopics();

	std::cout << "# Done. Found " << lstTopics.size() << " topics:\n";

	for (const auto& n : lstTopics)
	{
		if (argDetails.isSet())
		{
			std::cout << "- name: \"" << n.name << "\"\n"
					  << "  type: \"" << n.type << "\"\n"
					  << "  publishers:\n";

			ASSERT_EQUAL_(n.endpoints.size(), n.publishers.size());

			for (size_t i = 0; i < n.endpoints.size(); i++)
			{
				std::cout << "    - endpoint: \"" << n.endpoints[i] << "\"\n"
						  << "    - publisherNode: \"" << n.publishers[i]
						  << "\"\n";
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
	return 0;
}

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

static void callbackSubscribeTopicGeneric(const zmq::message_t& msg)
{
	const auto [typeName, serializedData] =
		mvsim::internal::parseMessageToParts(msg);
	std::cout << "[" << mrpt::system::dateTimeLocalToString(mrpt::Clock::now())
			  << "] Received data : \n ";
	std::cout << " - typeName: " << typeName << "\n";
	std::cout << " - data: " << serializedData.size() << " bytes\n";

	if (typeName == mvsim_msgs::TimeStampedPose().GetTypeName())
		echo_TimeStampedPose(serializedData);

	std::cout << std::endl;
}

int topicEcho()
{
	mvsim::Client client;

	client.setMinLoggingLevel(
		mrpt::typemeta::TEnumType<mrpt::system::VerbosityLevel>::name2value(
			argVerbosity.getValue()));

	const auto& lstCmds = argCmd.getValue();
	if (lstCmds.size() != 3) return printCommandsTopic(true);

	const auto& topicName = lstCmds.at(2);

	std::cout << "# Connecting to server...\n";
	client.connect();
	std::cout << "# Connected.\n";

	std::cout << "# Subscribing to topic '" << topicName
			  << "'. Press CTRL+C to stop.\n";
	client.subscribe_topic_raw(topicName, &callbackSubscribeTopicGeneric);

	// loop until user does a CTRL+C
	for (;;)
	{
	}

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
    mvsim topic list [--details]  List all advertised topics in the server.
    mvsim topic echo <topicName>  Subscribe and print a topic.

)XXX");

	return showErrorMsg ? 1 : 0;
}
