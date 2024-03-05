
struct version {
    int color; // int corresponding to a color for the id flash
        // 0 is red, beta
        // 1 is v1.X.X
    int flash; // number of flashes
};

struct version beta = {0, 1};
struct version v1_0_3 = {1, 1};
