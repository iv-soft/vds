#ifndef __VDS_DB_MODEL_CHUNK_REPLICA_MAP_DBO_H_
#define __VDS_DB_MODEL_CHUNK_REPLICA_MAP_DBO_H_
/*
Copyright (c) 2017, Vadim Malyshev, lboss75@gmail.com
All rights reserved
*/
#include <chrono>
#include "database_orm.h"

namespace vds {
	namespace orm {
		class chunk_replica_map_dbo : public database_table {
		public:
			chunk_replica_map_dbo()
				: database_table("chunk_replica_map"),
				object_id(this, "object_id"),
				replica(this, "replica"),
				node(this, "node"),
        last_access(this, "last_access") {
			}

			database_column<std::string> object_id;
			database_column<int> replica;
      database_column<std::string> node;
      database_column<std::chrono::system_clock::time_point> last_access;
		};
	}
}

#endif //__VDS_DB_MODEL_CHUNK_REPLICA_MAP_DBO_H_
