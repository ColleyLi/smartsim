#include "app/simulator/vtd_integration/workflow_bev.hpp"
#include "app/simulator/vtd_integration/vtd_dependencies/common/vtd_integration_gflags.h"


using namespace smartsil::app::vtd_integration;
int main (int argc, char** argv) {
  if (argc <= 1) {
    std::cerr << "Usage: ./vtd_simulation NODE" << std::endl;;
    std::cerr << "    [BFW]    short for bev module" << std::endl;
    std::cerr << "    [ALL]    short for all module" << std::endl;
    exit(-1);
  }

  std::string deployment = argv[1];

  Workflow wf;
  WorkflowLoader wfl(deployment);

  while (true) {
    wfl.init(&wf);
  }
  
  return 0;
}
