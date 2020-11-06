#ifndef __VDS_DB_MODEL_CHUNK_TMP_DATA_DBO_H_
#define __VDS_DB_MODEL_CHUNK_TMP_DATA_DBO_H_

/*
Copyright (c) 2017, Vadim Malyshev, lboss75@gmail.com
All rights reserved
*/

#include "database_orm.h"

namespace vds {
  namespace orm {
    class chunk_tmp_data_dbo : public database_table {
    public:
      chunk_tmp_data_dbo()
          : database_table("chunk_tmp_data"),
            object_id(this, "object_id"),
            last_sync(this, "last_sync"),
            replica_data(this, "replica_data") {
      }

      database_column<const_data_buffer, std::string> object_id;
      database_column<std::chrono::system_clock::time_point> last_sync;
      database_column<const_data_buffer> replica_data;

      static constexpr const char* create_table =
        "CREATE TABLE chunk_tmp_data (\
			  object_id VARCHAR(64) PRIMARY KEY NOT NULL,\
			  last_sync INTEGER NOT NULL, \
        replica_data BLOB NOT NULL)";
    };
  }
}

#endif //__VDS_DB_MODEL_CHUNK_TMP_DATA_DBO_H_
