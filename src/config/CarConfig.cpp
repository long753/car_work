#include "CarConfig.hpp"


CarConfig::CarConfig(){
this->load();
this->print_params();
}

CarConfig::~CarConfig(){


}

void CarConfig::load(){
    std::string jsonPath = "../src/config/motion.json";
    std::ifstream config_is(jsonPath);
    if (!config_is.good()) {
      std::cout << "Error: Params file path:[" << jsonPath << "] not find .\n";
      exit(-1);
    }

    nlohmann::json js_value;
    config_is >> js_value;

    try {
      shared_params = std::make_shared<Params>(std::move(js_value.get<Params>()));
    } catch (const nlohmann::detail::exception &e) {
      std::cerr << "Json Params Parse failed :" << e.what() << '\n';
      exit(-1);
    }

    
}
void CarConfig::print_params(){
    std::cout << "--- turnP:" << shared_params->turnP 
        << " | turnD:" << shared_params->turnD << std::endl;
    std::cout << "--- speedLow:" << shared_params->speedLow
         << "m/s  |  speedHigh:" << shared_params->speedHigh << "m/s" << std::endl;
}
RingConfig::RingConfig(){
  this->ring_config_load();  
}

void RingConfig::ring_config_load(){
  std::string ring_config_path = "../src/config/ring.json";
  std::ifstream config_istream(ring_config_path);
  if (!config_istream.good()) {
      std::cout << "Error:Ring Params file path:[" << ring_config_path << "] not find .\n";
      exit(-1);
    }
    nlohmann::json js_value;
    config_istream>>js_value;
    try{
      ring_params = std::make_shared<RingParams>(std::move(js_value.get<RingParams>()));
    }catch(const nlohmann::detail::exception &e) {
      std::cerr << "Json Params Parse failed :" << e.what() << '\n';
      exit(-1);
    }

}

AIConfig::AIConfig(){
  this->ai_config_load();  
}

void AIConfig::ai_config_load(){
  std::string ai_config_path = "../src/config/ai.json";
  std::ifstream config_istream(ai_config_path);
  if (!config_istream.good()) {
      std::cout << "Error:Ai Params file path:[" << ai_config_path << "] not find .\n";
      exit(-1);
    }
    nlohmann::json js_value;
    config_istream>>js_value;
    try{
      ai_params = std::make_shared<AIParams>(std::move(js_value.get<AIParams>()));
    }catch(const nlohmann::detail::exception &e) {
      std::cerr << "Json Params Parse failed :" << e.what() << '\n';
      exit(-1);
    }

}