
struct version {
    int color; // int corresponding to a color for the id flash
        // 0 is red, beta
        // 1 is v1.X.X
    int flash; // number of flashes
};

struct version beta = {0, 1};

// The structs listed below are official releases of Photon
// When prepping for a formal release, create a new struct named the scematic version number of the release with . replaced with _
// The first element in the struct should equal the MAJOR revision number of the version. This indicates the color.
// The second element indicates the number of flashes. Increment this from the previous MAJOR version's value, or if a new MAJOR version, start with 1.
// Save, commit, and push this change before attempting to make a release. It will fail compilation if this step is not completed.

struct version v1_0_3 = {1, 1};
