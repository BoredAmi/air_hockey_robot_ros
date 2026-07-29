# Wynik predykcji trajektorii - publikowany przez trajectory_node
# do konsumpcji przez movement_node

uint64 timestamp        # timestamp pomiaru, na podstawie ktorego policzono predykcje
bool valid               # false, gdy brak predykcji (krazek stoi, oddala sie, brak detekcji itp.)
float32 x                # przewidziane x wejscia w strefe obrony (mm)
float32 y                # przewidziane y wejscia w strefe obrony (mm)
float32 confidence        # z KalmanFilter::getVelocityConfidence(), 0-1
float32 time_to_entry     # sekundy od momentu pomiaru (timestamp) do przewidywanego wejscia w strefe;
                          # ujemne/0, gdy valid=false

# Ponizsze pola pozwalaja movement_node podjac decyzje bez subskrybowania
# dodatkowo /puck/detection - wszystko liczone raz, tutaj, na podstawie stanu Kalmana.
float32 puck_x            # aktualna wygladzona pozycja X krazka (mm), state(0) filtru Kalmana
float32 puck_y            # aktualna wygladzona pozycja Y krazka (mm), state(1)
float32 vx                # predkosc X (mm/s), state(2)
float32 vy                # predkosc Y (mm/s), state(3)
bool puck_in_defense_zone  # czy aktualna (nie przewidziana) pozycja juz jest w strefie obrony
int8 defense_zone_index    # ktora strefa jest aktywna (0=lewa,1=prawa,2=gorna,3=dolna) - config_.WHERE_DEFENSE_ZONE
