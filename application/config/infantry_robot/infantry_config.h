#ifndef INFANTRY_CONFIG_H
#define INFANTRY_CONFIG_H

#include "config.h"

#include "referee_UI.h"

Robot_Config_s *InfantryConfigInit(uint8_t robot_id);
void InfantryStaticUI(referee_info_t *referee_recv_info);
void InfantryDynamciscUI(referee_info_t *referee_recv_info, Referee_Interactive_info_t *_Interactive_data);

#endif