uint64 timestamp        # timestamp pomiaru, na podstawie ktorego policzono predykcje
bool valid               # false, gdy brak predykcji (krazek stoi, oddala sie, brak detekcji itp.)
float32 x                # przewidziane x wejscia w strefe obrony (mm)
float32 y                # przewidziane y wejscia w strefe obrony (mm)
float32 confidence        # z KalmanFilter::getVelocityConfidence(), 0-1
float32 time_to_entry     # sekundy od momentu pomiaru (timestamp) do przewidywanego wejscia w strefe;
                          # ujemne/0, gdy valid=false

float32 puck_x
float32 puck_y
float32 vx
float32 vy
bool puck_in_defense_zone
int8 defense_zone_index