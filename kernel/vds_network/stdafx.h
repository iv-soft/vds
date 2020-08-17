#ifndef __VDS_NETWORK_STDAFX_H_
#define __VDS_NETWORK_STDAFX_H_

/*
Copyright (c) 2017, Vadim Malyshev, lboss75@gmail.com
All rights reserved
*/

#include <set>

#include "vds_core.h"
#include "vds_network.h"

#ifdef _WIN32

#include <Ws2tcpip.h>
#include <iphlpapi.h>
#pragma comment(lib, "IPHLPAPI.lib")

#else

#include <sys/socket.h>
#include <sys/types.h>
#include <netdb.h>
#include <ifaddrs.h>
#include <net/if.h>

#endif

#endif//__VDS_NETWORK_STDAFX_H_

