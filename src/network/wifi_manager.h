#pragma once

#include <sMQTTBroker.h>

bool wifi_connect_from_list();
void wifi_start_offline_services();
void wifi_stop_offline_services();
void wifi_update_connectivity();
void wifi_sync_ntp();
bool wifi_is_offline_active();
bool wifi_is_local_broker_started();
sMQTTBroker& wifi_get_local_broker();
