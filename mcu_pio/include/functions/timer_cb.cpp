void timer_cb(){

    // count_response.id = 44;
    count_response.a ++;
    count_response.b ++;

    send_data(pack_data<Counter>(count_response, COUNTER));

    // std::string data = "";

    // uint8_t num = 3;
    // float num2 = 4.567;
    
    // data = "data sending " + std::to_string(num) + " " + std::to_string(num2);
    // debug_state(data);
    
}

BlitzTimer t1(timer_cb, 100);
