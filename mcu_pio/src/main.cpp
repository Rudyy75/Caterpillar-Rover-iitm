#include "include_all.cpp"

void setup(){

    // serial begin
    Serial.begin(115200);
}

void loop() {

    // recieves data in every loop 
    std::vector<uint8_t> payload = receive_data();

    // stores data, called every loop
    store_data(payload);
    
    // spin the callbacks
    t1.spin();
}
