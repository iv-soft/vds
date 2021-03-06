#ifndef __VDS_CORE_VDS_EXCEPTIONS_H_
#define __VDS_CORE_VDS_EXCEPTIONS_H_

/*
Copyright (c) 2017, Vadim Malyshev, lboss75@gmail.com
All rights reserved
*/
#include <stdexcept>

namespace vds {
  namespace vds_exceptions {
    class not_found : public std::runtime_error {
    public:
      not_found();

    };

    class invalid_operation : public std::runtime_error {
    public:
      invalid_operation();
      invalid_operation(const std::string& message);

    };

	class signature_validate_error : public std::runtime_error {
	public:
		signature_validate_error();

	};
		class access_denied_error : public std::runtime_error {
		public:
			access_denied_error(const std::string & message);

		};
		class shooting_down_exception : public std::runtime_error {
		public:
			shooting_down_exception();

		};
  };
}


#endif //__VDS_CORE_VDS_EXCEPTIONS_H_
