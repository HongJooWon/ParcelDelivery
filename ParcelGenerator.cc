// #include <iostream>
// #include <fstream>
// #include <vector>
// #include <random>

// using namespace std;

// struct Coord {
//     int x, y;
// };

// struct parcel {
//     int parcelID;
//     int weight;
//     Coord parceldest;
// };

// void generateFixedParcels(int nparcels, const std::string& filename) {
    

//     vector<parcel> parcel_depot;

//     for (int i = 0; i < nparcels; i++) {
//         parcel tmpparcel;
//         tmpparcel.parcelID = i;
//         tmpparcel.weight = disWeight(gen);
//         tmpparcel.parceldest.x = disX(gen);
//         tmpparcel.parceldest.y = disY(gen);
//         parcel_depot.push_back(tmpparcel);
//     }

//     // 파일에 저장
//     std::ofstream outFile(filename);
//     if (!outFile) {
//         std::cerr << "Can't open file " << filename << std::endl;
//         return;
//     }

//     for (const auto& p : parcel_depot) {
//         outFile << p.parcelID << " " << p.weight << " "
//                 << p.parceldest.x << " " << p.parceldest.y << "\n";
//     }

//     outFile.close();
// }

// int main(int argc, char* argv[]) {

//     if (argc != 4) {
//         std::cerr << "사용법: " << argv[0] << " <width> <height> <parcel_count>" << std::endl;
//         return 1;
//     }

//     int width = std::stoi(argv[1]);
//     int height = std::stoi(argv[2]);
//     int count = std::stoi(argv[3]);

//     //file name = x_y_parcelnum.txt
//     string fileName = to_string(width) + "_" + to_string(height) + "_" + to_string(count) + ".txt";
//     generateFixedParcels(count, fileName);

//     return 0;
// }