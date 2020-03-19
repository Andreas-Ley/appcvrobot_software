/*
 * <one line to give the program's name and a brief idea of what it does.>
 * Copyright (C) 2020  <copyright holder> <email>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "config.h"

#if 0
const char *logFile = CMAKE_INSTALL_PREFIX"/var/log/robothardwarecontroller.log";
const char *socketFile = CMAKE_INSTALL_PREFIX"/var/run/robothardwarecontroller.socket";
const char *configFile = CMAKE_INSTALL_PREFIX"/etc/robothardwarecontroller/config.txt";
const char *stateFile = CMAKE_INSTALL_PREFIX"/var/robothardwarecontroller/state.txt";

unsigned logLevel = 0;
#else
const char *logFile = "robothardwarecontroller.log";
const char *socketFile = "robothardwarecontroller.socket";
const char *configFile = "config.txt";
const char *stateFile = "state.txt";

unsigned logLevel = 2;
#endif
