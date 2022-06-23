#include "Request.h"

const std::map<ramulator::Request::Type, string> ramulator::Request::name_type = {
    {Type::READ, "READ"}, {Type::WRITE, "WRITE"}, {Type::RNG, "RNG"},
    {Type::REFRESH, "REFRESH"}, {Type::POWERDOWN, "POWERDOWN"},
    {Type::SELFREFRESH, "SELFREFRESH"}, {Type::EXTENSION, "EXTENSION"}
};
//seungwoo: for debugging