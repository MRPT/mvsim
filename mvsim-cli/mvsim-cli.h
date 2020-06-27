/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2020  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/3rdparty/tclap/CmdLine.h>

extern TCLAP::CmdLine cmd;
extern TCLAP::UnlabeledMultiArg<std::string> argCmd;
extern TCLAP::ValueArg<std::string> argVerbosity;
extern TCLAP::ValueArg<int> argPort;
extern TCLAP::SwitchArg argHelp;

int printListCommands();  // "help"
int launchStandAloneServer();  // "server"
int launchSimulation();	 // "launch"
int commandNode();	// "node"
int commandTopic();	 // "topic"

void commonLaunchServer();
