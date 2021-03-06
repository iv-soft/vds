#ifndef __VDS_HTTP_HTTPS_PIPELINE_H_
#define __VDS_HTTP_HTTPS_PIPELINE_H_

/*
Copyright (c) 2017, Vadim Malyshev, lboss75@gmail.com
All rights reserved
*
namespace vds {
  class https_pipeline
  {
  public:
    https_pipeline(
      const std::string & address,
      int port,
      certificate * client_certificate,
      asymmetric_private_key * client_private_key);
    ~https_pipeline();
    
    void connect();
   
    const std::string & address() const;
    int port() const;
   
  protected:
    friend class _https_pipeline;
    
    virtual void on_connected();
    virtual void on_connection_closed();
    virtual void on_error( const std::shared_ptr<std::exception> & error);
    
    virtual bool on_response( json_value * response) = 0;
    virtual void get_commands() = 0;

    void run( const std::string & body);
    
  private
    const std::unique_ptr<class _https_pipeline> impl_;
  };
}
*/

#endif // __VDS_HTTP_HTTPS_PIPELINE_H_
