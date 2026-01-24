#include "mower_controller/wheel.hpp"

Wheel::Wheel()
{
}

Wheel::~Wheel()
{
}

void Wheel::setup(const std::string &wheel_name, int counts_per_rev)
{
    name = wheel_name;
    // counts_per_rev = counts_per_rev;
}

