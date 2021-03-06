#ifndef __VDS_CORE_APP_H_
#define __VDS_CORE_APP_H_

/*
Copyright (c) 2017, Vadim Malyshev, lboss75@gmail.com
All rights reserved
*/

#include <string>
#include <exception>
#include <iostream>

#ifndef _WIN32
#include <sys/resource.h> 
#include <signal.h>
#include <sys/stat.h>
#include <wait.h>
#include "barrier.h"
/* an idea from 'Advanced Programming in the Unix Environment'
  Stevens 1993 - see BecomeDaemonProcess() */

#define OPEN_MAX_GUESS 256
#define CHILD_NEED_TERMINATE 9

#endif

#include "service_provider.h"
#include "logger.h"
#include "command_line.h"
#include "filename.h"
#include "persistence.h"
#include "file.h"
#include "vds_exceptions.h"

namespace vds{
  
  class app
  {
  public:
    static const version product_version();
    static const char * brunch();
    virtual std::string app_name() const;

    virtual std::string app_description() const;

    virtual std::string app_version() const;

    virtual expected<void> main(const service_provider * sp) = 0;

    app();

    int run(int argc, const char** argv);

    expected<int> run_app(int argc, const char** argv);

  protected:
      file_logger logger_;
      command_line_value log_level_;
      command_line_value log_modules_;
      command_line_value root_folder_;
      const command_line_set *current_command_set_;
      filename current_process_;

    expected<void> start(vds::service_registrator& registrator);

    virtual void register_services(service_registrator& registrator);

    virtual expected<void> start_services(service_registrator& registrator, service_provider* /*sp*/);

    virtual expected<void> before_main(service_provider* /*sp*/);

    virtual expected<void> prepare(service_provider* /*sp*/);

    virtual void register_command_line(command_line& cmd_line);

    virtual void register_common_parameters(command_line& cmd_line);

    void process_common_parameters();

    virtual bool need_demonize();

    static app* the_app_;

#ifdef _WIN32

    virtual TCHAR* service_name() const;

    expected<int> demonize(const foldername & root_folder);

    static VOID WINAPI SvcMain(DWORD dwArgc, LPTSTR* lpszArgv);

      SERVICE_STATUS          gSvcStatus;
      SERVICE_STATUS_HANDLE   gSvcStatusHandle;
      HANDLE                  ghSvcStopEvent;
      static BOOL WINAPI CtrlHandler(DWORD fdwCtrlType);

    void service_main();

    void SvcInit();

    VOID ReportSvcStatus(DWORD dwCurrentState,
                         DWORD dwWin32ExitCode,
                         DWORD dwWaitHint);

    static VOID WINAPI SvcCtrlHandler(DWORD dwCtrl);
#else//_WIN32

    barrier stop_barrier_;
static expected<void> kill_prev(const foldername & root_folder, const std::string & process_name);
static expected<void> demonize(const foldername & root_folder, const std::string & process_name);

        expected<int> demonize(const foldername & root_folder);


      static void signalHandler(int /*signum*/);

#endif // _WIN32
      void waiting_stop_signal(bool is_service);

  private:
      command_line_set help_cmd_set_;
      command_line_switch help_cmd_switch_;
  };
  
  class console_app : public app
  {
  public:
    console_app()
    {
    }
  };
}

#endif // __VDS_CORE_APP_H_
