#ifndef MAP_H_
#define MAP_H_

class Map {
public:
  struct single_landmark_s{
    int id_i ;
    float x_f;  // x-position in the map (global coordinates)
    float y_f;  // y-position in the map (global coordinates)
  };

  std::vector<single_landmark_s> landmark_list;
};

#endif
