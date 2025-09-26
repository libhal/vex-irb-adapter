/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:                                                                 */
/*    Created:                                                                */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#include <cstdio>

#include "irb_adapter.h"
#include "vex.h"

using namespace vex;

vex::brain Brain;

int main()
{
  // Adapter board is plugged into smart port 1
  irb_adapter::adapter board(1);
}
