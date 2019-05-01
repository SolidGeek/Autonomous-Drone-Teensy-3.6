#include "Settings.h"

Settings::Settings( void ){}

void Settings::load()
{
  // Load settings from EEPROM to this
  EEPROM.get(0, *this);
}

void Settings::save()
{
  EEPROM.put(0, *this);
}
