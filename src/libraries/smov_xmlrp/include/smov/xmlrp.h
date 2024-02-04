//
// Created by ros on 2/3/24.
//

#ifndef XMLRP_H_
#define XMLRP_H_

#include <xmlrpcpp/XmlRpc.h>
#include <rclcpp/logging.hpp>

namespace smov {

class Xmlrp {
 public:
  static std::string get_string_param(XmlRpc::XmlRpcValue obj, std::string param_name);

  static int get_int_param(XmlRpc::XmlRpcValue obj, std::string param_name);

  static float get_float_param(XmlRpc::XmlRpcValue obj, std::string param_name);

  static double get_double_param(XmlRpc::XmlRpcValue obj, std::string param_name);

};

} // namespace smov


#endif // XMLRP_H_
