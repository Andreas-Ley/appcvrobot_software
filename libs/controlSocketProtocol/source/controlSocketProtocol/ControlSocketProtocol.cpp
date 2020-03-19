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

#include "ControlSocketProtocol.h"

#include <stdexcept>

namespace robot {   
namespace hardwareSocket {

void RequestBodyLCDSetText::setLine(unsigned line, const std::string &src)
{
    if (line >= NUM_LINES)
        throw std::runtime_error("Invalid line!");
    
    // because strncpy_s is not available in c++
    
    unsigned len = std::min<std::size_t>(src.size(), LINE_LENGTH);
    for (unsigned i = 0; i < len; i++)
        lines[line][i] = src[i];
    lines[line][len] = 0;
}

}
}
